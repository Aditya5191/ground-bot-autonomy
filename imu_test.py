import smbus2
import time

class IMUTest:
    def __init__(self, bus_num=1, addr=0x68):
        self.bus_num = bus_num
        self.addr = addr
        self.bus = smbus2.SMBus(bus_num)
        self.init_imu()

    def init_imu(self):
        # Wake up the MPU-6050 / MPU-6500 (PWR_MGMT_1 register = 0)
        self.bus.write_byte_data(self.addr, 0x6B, 0)
        time.sleep(0.1)

    def read_raw_data(self, reg):
        # Read 2 bytes of data from register `reg` and combine them
        high = self.bus.read_byte_data(self.addr, reg)
        low = self.bus.read_byte_data(self.addr, reg + 1)
        value = (high << 8) | low
        if value > 32767:
            value -= 65536
        return value

    def get_all_data(self):
        # Read accelerometer raw values
        acc_x = self.read_raw_data(0x3B)
        acc_y = self.read_raw_data(0x3D)
        acc_z = self.read_raw_data(0x3F)

        # Read gyro raw values
        gyro_x = self.read_raw_data(0x43)
        gyro_y = self.read_raw_data(0x45)
        gyro_z = self.read_raw_data(0x47)

        return acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z

if __name__ == "__main__":
    imu = IMUTest()
    print("Reading IMU data... Press Ctrl+C to stop")
    try:
        while True:
            acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z = imu.get_all_data()
            print(f"Accel: X={acc_x}, Y={acc_y}, Z={acc_z} | Gyro: X={gyro_x}, Y={gyro_y}, Z={gyro_z}")
            time.sleep(0.1)  # 10 Hz update
    except KeyboardInterrupt:
        print("\nExiting")
