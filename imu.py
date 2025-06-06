from smbus2 import SMBus
import time
import json
import os
import threading

MPU_ADDR = 0x68
CALIB_FILE = "calib.json"

class IMU:
    def __init__(self, dt=0.05):
        self.dt = dt  # loop delay in seconds
        self.yaw = 0.0
        self.running = False
        self.bus = None
        self.gyro_bias = 0.0
        self.thread = None

    def init_imu(self):
        try:
            self.bus = SMBus(1)
            self.bus.write_byte_data(MPU_ADDR, 0x6B, 0)  # Wake up MPU
            self.bus.write_byte_data(MPU_ADDR, 0x19, 9)  # Sample rate divider
            print("MPU connected")
            return True
        except Exception as e:
            print(f"MPU init error: {e}")
            return False

    def read(self, reg):
        high = self.bus.read_byte_data(MPU_ADDR, reg)
        low = self.bus.read_byte_data(MPU_ADDR, reg + 1)
        value = (high << 8) + low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value

    def calibrate(self, samples=500):
        print("Calibrating gyro... keep MPU still.")
        total = 0.0
        for _ in range(samples):
            total += self.read(0x47)
            time.sleep(0.01)
        self.gyro_bias = total / samples
        with open(CALIB_FILE, "w") as f:
            json.dump({"gyro_bias": self.gyro_bias}, f)
        print("Calibration saved.")

    def load_calibration(self):
        if os.path.exists(CALIB_FILE):
            try:
                with open(CALIB_FILE, "r") as f:
                    self.gyro_bias = json.load(f)["gyro_bias"]
                    return
            except:
                print("Corrupted calibration file. Recalibrating...")
        self.calibrate()

    def yaw_loop(self):
        while self.running:
            try:
                raw = self.read(0x47)
                gyro_z = (raw - self.gyro_bias) / 131.0
                self.yaw += gyro_z * self.dt
            except OSError as e:
                print(f"I2C error: {e}, reconnecting...")
                try:
                    self.bus.close()
                except:
                    pass
                time.sleep(0.2)
                while not self.init_imu():
                    time.sleep(0.2)
                self.load_calibration()
            time.sleep(self.dt)

    def start(self):
        while not self.init_imu():
            print("Retrying IMU...")
            time.sleep(0.2)
        self.load_calibration()
        self.running = True
        self.thread = threading.Thread(target=self.yaw_loop)
        self.thread.daemon = True
        self.thread.start()

    def restart(self):
        self.stop()
        self.yaw=0.0
        while not self.init_imu():
            print("Retrying IMU...")
            self.init_imu()
            time.sleep(0.2)
        self.load_calibration()
        self.running = True
        self.thread = threading.Thread(target=self.yaw_loop)
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
        if self.bus:
            self.bus.close()

    def get_yaw(self):
        return self.yaw
