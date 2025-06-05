import threading
import time
import math
import serial
import smbus2  # For I2C communication with IMU
import time

from motor_controller import MotorPIDController
from recieve import ZMQArrowReceiver
from send_images import ZMQCameraPublisher

# IMU complementary filter class for yaw estimation
class ComplementaryFilterIMU:
    def __init__(self, bus_num=1, imu_addr=0x68):
        self.bus = smbus2.SMBus(bus_num)
        self.addr = imu_addr

        # MPU6050 registers
        self.PWR_MGMT_1 = 0x6B
        self.GYRO_CONFIG = 0x1B
        self.ACCEL_CONFIG = 0x1C
        self.ACCEL_XOUT_H = 0x3B
        self.GYRO_XOUT_H = 0x43

        # Wake up the MPU6050
        self.bus.write_byte_data(self.addr, self.PWR_MGMT_1, 0)

        self.gyro_scale = 131.0
        self.accel_scale = 16384.0

        self.last_time = time.time()
        self.yaw = 0.0  # Integrated yaw angle
        self.alpha = 0.98  # Complementary filter coefficient

    def read_word(self, reg):
        high = self.bus.read_byte_data(self.addr, reg)
        low = self.bus.read_byte_data(self.addr, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            val = -((65535 - val) + 1)
        return val

    def get_gyro_z(self):
        return self.read_word(self.GYRO_XOUT_H + 4) / self.gyro_scale  # gyro z axis

    def update(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        gyro_z = self.get_gyro_z()

        # Integrate gyro z to get delta angle
        delta_yaw = gyro_z * dt

        # Complementary filter: no magnetometer, so use gyro integration only (simplified)
        self.yaw += delta_yaw

        # Normalize yaw to -180 to 180
        if self.yaw > 180:
            self.yaw -= 360
        elif self.yaw < -180:
            self.yaw += 360

        return self.yaw

class RobotController:
    def __init__(self):
        self.motor_controller = MotorPIDController()
        self.arrow_receiver = ZMQArrowReceiver(pc_ip="10.29.0.1")  # replace with PC IP

        self.imu = ComplementaryFilterIMU()
        self.turn_duration_ms = 1100
        self.turn_speed_pwm = 200

        self.is_turning = False

        self.target_yaw = None  # Will be set on startup

    def turn_90_degrees(self, direction):
        print(f"Turning 90 degrees {direction}")

        if direction == "right":
            left_pwm = self.turn_speed_pwm
            left_dir = 0  # backward
            right_pwm = self.turn_speed_pwm
            right_dir = 1  # forward
        elif direction == "left":
            left_pwm = self.turn_speed_pwm
            left_dir = 1  # forward
            right_pwm = self.turn_speed_pwm
            right_dir = 0  # backward
        else:
            print(f"Unknown turn direction: {direction}")
            return

        data = f"{left_pwm},{left_dir},{right_pwm},{right_dir}\n"
        self.motor_controller.serial.write(data.encode())

        time.sleep(self.turn_duration_ms / 1000)

        self.motor_controller.stop()
        print("Turn complete, motors stopped")

        # After turn, reset target yaw to current IMU yaw
        self.target_yaw = self.imu.yaw

    def start(self):
        # Set initial target yaw as current IMU reading
        print("Calibrating IMU for initial yaw...")
        time.sleep(1)
        self.target_yaw = self.imu.update()
        print(f"Initial target yaw set to: {self.target_yaw:.2f}")

        print("Robot controller started")
        while True:
            angle, distance, direction = self.arrow_receiver.receive_arrow_data()

            # Update IMU yaw reading every loop
            current_yaw = self.imu.update()

            if angle is None or distance is None:
                # No arrow detected, try to maintain straight path using IMU yaw
                error_yaw = self.target_yaw - current_yaw

                # Normalize error_yaw to [-180,180]
                if error_yaw > 180:
                    error_yaw -= 360
                elif error_yaw < -180:
                    error_yaw += 360

                if not self.is_turning:
                    print(f"No arrow detected. Maintaining straight yaw. Yaw error: {error_yaw:.2f}")
                    self.motor_controller.compute_pwm(error_yaw)
                continue

            print(f"Arrow detected - Direction: {direction}, Angle: {angle:.2f}, Distance: {distance:.2f} cm")

            if self.is_turning:
                # Currently turning, ignore commands
                continue

            if distance < 30:
                self.is_turning = True
                self.motor_controller.stop()
                self.turn_90_degrees(direction)
                self.is_turning = False
            else:
                # Use arrow angle to align
                self.motor_controller.compute_pwm(angle)


if __name__ == "__main__":
    robot = RobotController()
    robot.start()
