from smbus2 import SMBus
import time
import json
import os
MPU_ADDR = 0x68
CALIB_FILE = "calib.json"

class IMU:
    def init_imu(self):
        global bus
        try: 
            bus = SMBus(1)
            bus.write_byte_data(MPU_ADDR, 0x6B, 0) #set the register value to wake up IMU
            bus.write_byte_data(MPU_ADDR, 0x19, 9) #rate clk/(divider+1)  80kz/(9+1) = 800hz
            print("MPU connected")
            return True 
        except Exception as e:
            print(f"MPU init error: {e}")
            return False

    def calib_mpu(self, samples=200):
        gyro_bias = 0.0
        print("Calibrating gyro... keep MPU still.")

        for _ in range(samples):
            gyro_bias += self.read(0x47)
            time.sleep(0.01)

        gyro_bias = gyro_bias / samples

        data = {"gyro_bias": gyro_bias}
        with open(CALIB_FILE, "w") as f:
            json.dump(data, f)

        print("Calibration saved.")
        return data
    
    def load_calib(self):
        if os.path.exists(CALIB_FILE):
            try:
                with open(CALIB_FILE, "r") as f:
                    return json.load(f)
            except:
                print("Corrupted calibration. Recalibrating...")
        return self.calib_mpu()

    def read(self,reg):
        high = bus.read_byte_data(MPU_ADDR, reg)
        low = bus.read_byte_data(MPU_ADDR, reg+1)
        value = (high << 8) + low
        if value >= 0x8000:
            value = -((65535 - value) + 1)
        return value

imu = IMU()
while not imu.init_imu():
    print("Retrying IMU..")
    time.sleep(0.2)

calib_data = imu.calib_mpu()

yaw = 0.0
dt = 0.05

try:
    while True:
        try:
            gyro_z_raw =  imu.read(0x47) - calib_data["gyro_bias"]
            gyro_z = gyro_z_raw / 131.0 

            yaw += gyro_z *dt
            print(f"yaw: {yaw}")
        except OSError as e:
            try: bus.close()
            except: pass
            time.sleep(0.2)
            while not imu.init_imu():
                print("Retrying IMU..")
                time.sleep(0.2)
            calib_data = imu.load_calib()
        time.sleep(dt)
except:
    pass


