import threading
import time
import zmq
import serial
import cv2
from picamera2 import Picamera2
import smbus2
import math

# ---------------- Complementary Filter IMU ----------------

class ComplementaryFilterIMU:
    def __init__(self, bus_num=1, addr=0x68):
        self.bus_num = bus_num
        self.addr = addr
        self.init_imu()

        self.gyro_scale = 131.0
        self.accel_scale = 16384.0

        self.angle = 0.0
        self.alpha = 0.98
        self.dt = 0.01  # 10 ms loop time

        self.last_time = time.time()

    def init_imu(self):
        try:
            self.bus = smbus2.SMBus(self.bus_num)
            # Wake up MPU6500
            self.bus.write_byte_data(self.addr, 0x6B, 0)
        except Exception as e:
            print(f"[IMU INIT ERROR] {e}")
            self.bus = None

    def read_raw_data(self, reg):
        if not self.bus:
            raise IOError("I2C bus not initialized")
        high = self.bus.read_byte_data(self.addr, reg)
        low = self.bus.read_byte_data(self.addr, reg + 1)
        value = (high << 8) | low
        if value > 32768:
            value -= 65536
        return value

    def get_yaw(self):
        try:
            current_time = time.time()
            dt = current_time - self.last_time
            if dt <= 0:
                dt = self.dt
            self.last_time = current_time

            # Gyro Z axis raw data
            gz = self.read_raw_data(0x43 + 4)  # GYRO_ZOUT_H = 0x47
            gz_rate = gz / self.gyro_scale  # degrees per second

            # Integrate gyro
            gyro_angle = self.angle + gz_rate * dt

            # Complementary filter (mostly gyro for yaw)
            self.angle = self.alpha * gyro_angle + (1 - self.alpha) * self.angle

            return self.angle
        except Exception as e:
            print(f"[IMU ERROR] {e}, attempting to reinitialize IMU...")
            self.init_imu()
            # On error, return last known angle to avoid crash
            return self.angle

    def __init__(self, bus_num=1, addr=0x68):
        self.bus = smbus2.SMBus(bus_num)
        self.addr = addr
        # MPU6500 registers
        self.PWR_MGMT_1 = 0x6B
        self.GYRO_XOUT_H = 0x43
        self.ACCEL_XOUT_H = 0x3B

        # Wake up MPU6500
        self.bus.write_byte_data(self.addr, self.PWR_MGMT_1, 0)

        self.gyro_scale = 131.0
        self.accel_scale = 16384.0

        self.angle = 0.0
        self.alpha = 0.98
        self.dt = 0.01  # 10 ms loop time

        self.last_time = time.time()

    def read_raw_data(self, reg):
        high = self.bus.read_byte_data(self.addr, reg)
        low = self.bus.read_byte_data(self.addr, reg + 1)
        value = (high << 8) | low
        if value > 32768:
            value -= 65536
        return value

    def get_yaw(self):
        current_time = time.time()
        dt = current_time - self.last_time
        if dt <= 0:
            dt = self.dt
        self.last_time = current_time

        # Gyro Z axis raw data
        gz = self.read_raw_data(self.GYRO_XOUT_H + 4)  # GYRO_ZOUT_H
        gz_rate = gz / self.gyro_scale  # degrees per second

        # Integrate gyro
        gyro_angle = self.angle + gz_rate * dt

        # Accelerometer angle (yaw not directly measurable by accel, so fallback)
        # Here we only use gyro since accel can't give yaw. Complementary filter mostly gyro integration.
        self.angle = self.alpha * gyro_angle + (1 - self.alpha) * self.angle

        return self.angle


# ---------------- PID Motor Controller ----------------

class MotorPIDController:
    def __init__(self, port='/dev/serial0', baudrate=9600, kp=7.0, ki=-0.02, kd=0.05):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.serial = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Stabilize serial port

    def compute_pwm(self, target_angle):
        error = target_angle
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        output = max(min(output, 255), -255)

        base_speed = 200

        left_speed = base_speed + output
        right_speed = base_speed - output

        def clamp_direction(speed):
            direction = 1 if speed >= 0 else 0
            pwm = abs(int(speed))
            pwm = max(60, min(pwm, 255))
            return pwm, direction

        left_pwm, left_dir = clamp_direction(left_speed)
        right_pwm, right_dir = clamp_direction(right_speed)

        data = f"{left_pwm},{left_dir},{right_pwm},{right_dir}\n"
        self.serial.write(data.encode())
        print(f"[PID] Angle: {target_angle:.2f} | L_PWM: {left_pwm} Dir: {left_dir} | R_PWM: {right_pwm} Dir: {right_dir}")

    def stop(self):
        self.serial.write(b"0,1,0,1\n")  # Stop motors


# ---------------- ZMQ Camera Publisher ----------------

class ZMQCameraPublisher(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://*:5555")

        self.picam2 = Picamera2()
        self.picam2.start()

        self.running = True

    def run(self):
        while self.running:
            frame = self.picam2.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            _, buffer = cv2.imencode('.jpg', frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            self.socket.send(buffer.tobytes())
            time.sleep(0.1)

    def stop(self):
        self.running = False


# ---------------- ZMQ Arrow Receiver ----------------

class ZMQArrowReceiver(threading.Thread):
    def __init__(self, pc_ip="10.29.0.1", port=5566):
        threading.Thread.__init__(self)
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{pc_ip}:{port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, '')
        self.angle = None
        self.distance = None
        self.direction = None
        self.running = True

    def run(self):
        while self.running:
            try:
                msg = self.socket.recv_json(flags=zmq.NOBLOCK)
                self.angle = msg.get("angle", 0.0)
                self.distance = msg.get("distance", 1000.0)
                self.direction = msg.get("arrow_dir", "none")
            except zmq.Again:
                # no message received
                time.sleep(0.01)
            except Exception as e:
                print("Error receiving arrow data:", e)

    def stop(self):
        self.running = False


# ---------------- Main Robot Controller ----------------

class RobotController:
    def __init__(self):
        self.imu = ComplementaryFilterIMU()
        self.motor = MotorPIDController()
        self.camera_pub = ZMQCameraPublisher()
        self.arrow_receiver = ZMQArrowReceiver()

        self.turning_90 = False
        self.turn_start_time = None
        self.turn_direction = None
        self.turn_duration = 1.1  # 1100 ms for 90 degree turn
        self.turn_pwm = 200

    def start(self):
        print("Starting camera publisher...")
        self.camera_pub.start()
        print("Starting arrow receiver...")
        self.arrow_receiver.start()
        print("Robot controller started.")

        try:
            while True:
                self.control_loop()
                time.sleep(0.01)
        except KeyboardInterrupt:
            print("Stopping robot...")
            self.camera_pub.stop()
            self.arrow_receiver.stop()
            self.motor.stop()
            self.camera_pub.join()
            self.arrow_receiver.join()
            print("Stopped.")

    def control_loop(self):
        angle = self.arrow_receiver.angle
        distance = self.arrow_receiver.distance
        direction = self.arrow_receiver.direction
        yaw = self.imu.get_yaw()

        if angle is None or distance is None or direction is None:
            # No data received yet, move straight
            self.motor.compute_pwm(0)
            return

        # If currently turning 90 degrees, override normal PID
        if self.turning_90:
            elapsed = time.time() - self.turn_start_time
            if elapsed < self.turn_duration:
                # Run motors opposite direction for turning in place
                if self.turn_direction == "left":
                    # Left turn: right motor forward, left motor backward
                    self.motor.serial.write(f"200,1,200,0\n".encode())
                elif self.turn_direction == "right":
                    # Right turn: right motor backward, left motor forward
                    self.motor.serial.write(f"200,0,200,1\n".encode())
                else:
                    self.motor.stop()
            else:
                self.turning_90 = False
                self.motor.stop()
                print("90 degree turn complete.")
            return

        # Check distance, if less than 30 cm trigger 90 degree turn
        if distance < 30:
            self.turning_90 = True
            self.turn_start_time = time.time()
            self.turn_direction = direction
            print(f"Starting 90 degree turn {direction} at distance {distance:.2f}cm")
            return

        # Normal PID control for angle alignment
        self.motor.compute_pwm(angle)


if __name__ == "__main__":
    controller = RobotController()
    controller.start()
