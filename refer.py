# ===================================
# Pi Code (PiCamera2 + IMU + Control)
# ===================================

import time
import serial
import threading
import socket
import json
import cv2
import zmq
import base64
from smbus2 import SMBus
import numpy as np
from picamera2 import Picamera2

# UART to Arduino
ser = serial.Serial('/dev/serial0', 9600)

# Kalman Filter Class
class KalmanFilter:
    def __init__(self):
        self.angle = 0.0
        self.bias = 0.0
        self.P = [[0, 0], [0, 0]]
        self.Q_angle = 0.001
        self.Q_bias = 0.003
        self.R_measure = 0.03

    def update(self, rate, measured_angle, dt):
        self.rate = rate - self.bias
        self.angle += dt * self.rate

        self.P[0][0] += dt * (dt*self.P[1][1] - self.P[0][1] - self.P[1][0] + self.Q_angle)
        self.P[0][1] -= dt * self.P[1][1]
        self.P[1][0] -= dt * self.P[1][1]
        self.P[1][1] += self.Q_bias * dt

        S = self.P[0][0] + self.R_measure
        K = [self.P[0][0]/S, self.P[1][0]/S]

        y = measured_angle - self.angle

        self.angle += K[0] * y
        self.bias += K[1] * y

        P00_temp = self.P[0][0]
        P01_temp = self.P[0][1]

        self.P[0][0] -= K[0] * P00_temp
        self.P[0][1] -= K[0] * P01_temp
        self.P[1][0] -= K[1] * P00_temp
        self.P[1][1] -= K[1] * P01_temp

        return self.angle

# Globals
Kp, Ki, Kd = 2, -0.1, 0.3
integral = 0
last_error = 0
last_time = time.time()
kf = KalmanFilter()
target_angle = 0
distance = 999
direction = "none"
yaw_filtered = 0.0
imu_dt = 0.05  # Default/fallback value

# IMU Thread using raw I2C access (MPU-6500 with I2C address 0x68)
def imu_thread():
    global yaw_filtered, last_time, imu_dt
    MPU_ADDR = 0x68
    PWR_MGMT_1 = 0x6B
    GYRO_ZOUT_H = 0x47
    bus = None

    while True:
        try:
            if bus is None:
                bus = SMBus(1)
                bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)

            now = time.time()
            imu_dt = now - last_time
            last_time = now

            high = bus.read_byte_data(MPU_ADDR, GYRO_ZOUT_H)
            low = bus.read_byte_data(MPU_ADDR, GYRO_ZOUT_H + 1)
            z_raw = (high << 8) | low
            if z_raw > 32767:
                z_raw -= 65536

            gyro_z = z_raw / 131.0
            yaw_filtered = kf.update(gyro_z, 0.0, imu_dt)
            time.sleep(0.01)

        except Exception as e:
            print("[IMU] Error detected:", e)
            print("[IMU] Resetting connection...")
            try:
                if bus:
                    bus.close()
            except:
                pass
            bus = None
            time.sleep(0.05)  # quick recovery

# UDP Receive Thread
def udp_receiver():
    global target_angle, distance, direction
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('', 8888))
    while True:
        data, _ = sock.recvfrom(1024)
        msg = json.loads(data.decode())
        target_angle = msg["angle"]
        distance = msg["distance"]
        direction = msg["arrow_dir"]

# Control Thread
def control_loop():
    global integral, last_error, target_angle, yaw_filtered, distance, direction
    while True:
        if True: #changed from if distance > 40:
            error = -target_angle + yaw_filtered
        else:
            if direction == 'left':
                target_angle = yaw_filtered - 90
            else:
                target_angle = yaw_filtered + 90
            error = target_angle - yaw_filtered

        dt = imu_dt
        integral += error * dt
        derivative = (error - last_error) / dt
        correction = Kp * error - Ki * integral + Kd * derivative
        last_error = error

        right_motor = 200
        left_motor = max(0, min(255, int(200 - correction)))

        print(f"Yaw: {yaw_filtered:.2f}, L: {left_motor}, R: {right_motor}, Target: {target_angle:.2f}, Dist: {distance:.1f}, Dir: {direction}")
        ser.write(f"{right_motor},{left_motor}\n".encode())
        time.sleep(dt)

# Camera Sender Thread using Picamera2

def camera_sender():
    picam2 = Picamera2()
    picam2.configure(picam2.create_video_configuration(
        main={"size": (640, 480), "format": "RGB888"},
        controls={"FrameDurationLimits": (200000, 200000)}
    ))
    picam2.start()
    time.sleep(1)

    context = zmq.Context()
    socket_pub = context.socket(zmq.PUB)
    socket_pub.bind("tcp://*:5555")

    while True:
        frame = picam2.capture_array()
        frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        _, buffer = cv2.imencode('.jpg', frame_bgr)
        jpg_as_text = base64.b64encode(buffer)
        socket_pub.send(jpg_as_text)
        time.sleep(0.2)

# Launch threads
threading.Thread(target=imu_thread, daemon=True).start()
threading.Thread(target=udp_receiver, daemon=True).start()
threading.Thread(target=camera_sender, daemon=True).start()
control_loop()
