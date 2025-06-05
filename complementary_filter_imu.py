import smbus2
import time
import math

class ComplementaryFilterIMU:
    def __init__(self, bus_num=1, addr=0x68, alpha=0.98, calibrate_samples=500):
        self.bus = smbus2.SMBus(bus_num)
        self.addr = addr
        self.alpha = alpha  # complementary filter coefficient (between 0 and 1)

        # MPU registers
        self.PWR_MGMT_1 = 0x6B
        self.SMPLRT_DIV = 0x19
        self.CONFIG = 0x1A
        self.GYRO_CONFIG = 0x1B
        self.ACCEL_CONFIG = 0x1C
        self.GYRO_XOUT_H = 0x43
        self.ACCEL_XOUT_H = 0x3B

        # Wake up MPU (clear sleep)
        self.bus.write_byte_data(self.addr, self.PWR_MGMT_1, 0)
        time.sleep(0.1)

        # Configure gyro and accel full scale ranges (±250 deg/s, ±2g)
        self.bus.write_byte_data(self.addr, self.GYRO_CONFIG, 0x00)  # gyro ±250dps
        self.bus.write_byte_data(self.addr, self.ACCEL_CONFIG, 0x00) # accel ±2g

        # Scale factors for raw data to physical units
        self.gyro_scale = 131.0      # LSB/deg/s for ±250dps
        self.accel_scale = 16384.0   # LSB/g for ±2g

        # Calibration offsets
        self.gyro_z_offset = 0.0

        # Yaw angle in degrees
        self.yaw = 0.0
        self.last_time = time.time()

        # Calibrate gyro to find offset bias
        self.calibrate(calibrate_samples)

    def read_word_2c(self, reg):
        high = self.bus.read_byte_data(self.addr, reg)
        low = self.bus.read_byte_data(self.addr, reg + 1)
        val = (high << 8) + low
        if val >= 0x8000:
            val = -((65535 - val) + 1)
        return val

    def get_gyro_z(self):
        raw_gyro_z = self.read_word_2c(self.GYRO_XOUT_H + 4)  # GYRO_ZOUT_H
        # Convert to deg/s and apply calibration offset
        return (raw_gyro_z / self.gyro_scale) - self.gyro_z_offset

    def get_accel_angles(self):
        ax = self.read_word_2c(self.ACCEL_XOUT_H)
        ay = self.read_word_2c(self.ACCEL_XOUT_H + 2)
        az = self.read_word_2c(self.ACCEL_XOUT_H + 4)

        # Convert raw to g's
        ax /= self.accel_scale
        ay /= self.accel_scale
        az /= self.accel_scale

        # Calculate pitch and roll from accelerometer (degrees)
        pitch = math.degrees(math.atan2(ax, math.sqrt(ay * ay + az * az)))
        roll = math.degrees(math.atan2(ay, math.sqrt(ax * ax + az * az)))

        return pitch, roll

    def calibrate(self, samples=500):
        print("Calibrating gyro offset...")
        total = 0.0
        for _ in range(samples):
            total += self.read_word_2c(self.GYRO_XOUT_H + 4) / self.gyro_scale
            time.sleep(0.005)
        self.gyro_z_offset = total / samples
        print(f"Gyro Z offset: {self.gyro_z_offset:.4f} deg/s")

    def get_yaw(self):
        current_time = time.time()
        dt = current_time - self.last_time
        self.last_time = current_time

        gyro_z = self.get_gyro_z()  # deg/s

        # Integrate gyro z for delta yaw
        delta_yaw = gyro_z * dt

        # Complementary filter update
        self.yaw = self.alpha * (self.yaw + delta_yaw) + (1 - self.alpha) * self.get_yaw_accel_estimate()

        # Normalize yaw to -180 to 180
        if self.yaw > 180:
            self.yaw -= 360
        elif self.yaw < -180:
            self.yaw += 360

        return self.yaw

    def get_yaw_accel_estimate(self):
        # Without magnetometer, yaw cannot be derived reliably from accelerometer alone
        # Return previous yaw as no absolute reference
        return self.yaw
