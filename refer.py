from smbus2 import SMBus
import time
import serial
import json
import os

MPU_ADDR = 0x68
CALIB_FILE = "calib.json"

# UART setup
ser = serial.Serial("/dev/serial0", 9600, timeout=1)
time.sleep(2)
ser.reset_input_buffer()
ser.reset_output_buffer()

def init_mpu():
    global bus
    try:
        bus = SMBus(1)
        bus.write_byte_data(MPU_ADDR, 0x6B, 0)  # Wake up MPU
        bus.write_byte_data(MPU_ADDR, 0x19, 9)  # Sample rate divider
        print("MPU connected.")
        return True
    except Exception as e:
        print(f"MPU init error: {e}")
        return False

def read_word(reg):
    high = bus.read_byte_data(MPU_ADDR, reg)
    low = bus.read_byte_data(MPU_ADDR, reg + 1)
    value = (high << 8) + low
    if value >= 0x8000:
        value = -((65535 - value) + 1)
    return value

def calibrate_mpu(samples=200):
    gyro_bias = [0.0, 0.0, 0.0]
    print("Calibrating gyro... keep MPU still.")

    for _ in range(samples):
        gyro_bias[0] += read_word(0x43)
        gyro_bias[1] += read_word(0x45)
        gyro_bias[2] += read_word(0x47)
        time.sleep(0.01)

    gyro_bias = [x / samples for x in gyro_bias]

    data = {"gyro_bias": gyro_bias}
    with open(CALIB_FILE, "w") as f:
        json.dump(data, f)

    print("Calibration saved.")
    return data

def load_calibration():
    if os.path.exists(CALIB_FILE):
        try:
            with open(CALIB_FILE, "r") as f:
                return json.load(f)
        except:
            print("Corrupted calibration. Recalibrating...")
    return calibrate_mpu()

# === Initial setup ===
while not init_mpu():
    print("Retrying MPU...")
    time.sleep(0.2)

calib_data = calibrate_mpu()

# PID Control variables
yaw = 0.0
error_sum = 0.0
last_error = 0.0
Kp = 4.0
Ki = -0.1
Kd = 0.03
RIGHT_PWM = 200
dt = 0.05  # 50 ms loop

print("Running PID control loop...")

try:
    while True:
        try:
            gyro_z_raw = read_word(0x47) - calib_data["gyro_bias"][2]
            gyro_z = gyro_z_raw / 131.0  # degrees/sec

            # Integrate yaw
            yaw += gyro_z * dt

            # PID error calculations
            error = yaw
            error_sum += error * dt
            d_error = (error - last_error) / dt
            last_error = error

            # PID control
            control = Kp * error + Ki * error_sum + Kd * d_error

            # Adjust left motor based on yaw
            left_pwm = int(RIGHT_PWM + control)
            left_pwm = max(0, min(255, left_pwm))

            # Send to Arduino
            ser.write(bytes([left_pwm, RIGHT_PWM]))

            # Debug print
            print(f"Yaw: {yaw:.2f}Â°, Left PWM: {left_pwm}, Right PWM: {RIGHT_PWM}")

        except OSError as e:
            print(f"I2C error: {e}. Reconnecting...")
            try: bus.close()
            except: pass
            time.sleep(0.2)
            while not init_mpu():
                time.sleep(0.2)
            calib_data = load_calibration()

        time.sleep(dt)

except KeyboardInterrupt:
    print("Stopped by user.")
finally:
    try:
        bus.close()
        ser.close()
    except:
        pass
