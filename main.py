import threading
import time
from motor_controller import MotorPIDController
from recieve import ZMQArrowReceiver
from send_images import ZMQCameraPublisher

# For IMU complementary filter (you need your implementation here)
from complementary_filter_imu import ComplementaryFilterIMU  # Assuming you have this

class RobotController:
    def __init__(self):
        self.motor_controller = MotorPIDController()
        self.receive_arrows = ZMQArrowReceiver(pc_ip="10.29.0.1", port=5566)
        self.send_images = ZMQCameraPublisher()
        self.imu = ComplementaryFilterIMU()

        self.arrow_spotted = False
        self.current_angle = 0
        self.current_distance = 0
        self.current_direction = None

        self.running = True

    def camera_thread(self):
        print("Starting camera publishing thread...")
        self.send_images.publish_frames()

    def control_thread(self):
        print("Starting control thread...")
        while self.running:
            angle, distance, direction = self.receive_arrows.receive_arrow_data()
            if angle is not None:
                self.arrow_spotted = True
                self.current_angle = angle
                self.current_distance = distance
                self.current_direction = direction

                # PID control for angle correction
                self.motor_controller.compute_pwm(angle)

                print(f"Angle: {angle:.2f}, Distance: {distance:.2f}, Direction: {direction}")

            else:
                # No arrow spotted, move straight using IMU yaw
                self.arrow_spotted = False
                yaw = self.imu.get_yaw()
                # Here you can implement straight movement logic based on yaw
                # For simplicity, just stop motors or send zero commands
                self.motor_controller.stop()

            time.sleep(0.05)  # Small delay to avoid hogging CPU

    def run(self):
        cam_thread = threading.Thread(target=self.camera_thread, daemon=True)
        control_thread = threading.Thread(target=self.control_thread, daemon=True)

        cam_thread.start()
        control_thread.start()

        print("Robot controller started.")

        try:
            while True:
                time.sleep(1)  # Keep main thread alive
        except KeyboardInterrupt:
            print("Stopping robot controller...")
            self.running = False
            self.motor_controller.stop()

if __name__ == "__main__":
    robot = RobotController()
    robot.run()
