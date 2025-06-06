import serial
import time

class Motor_ctrl:
    def __init__(self):
        self.ser = serial.Serial("/dev/serial0", 9600, timeout=1)
        time.sleep(2)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()   

    def write_pwm(self, left_pwm, RIGHT_PWM):
        self.ser.write(bytes([left_pwm, RIGHT_PWM]))

    def turn_left(self):
        self.ser.write(bytes([0, 255]))
        time.sleep(0.9)
        self.stop()

    def turn_right(self):
        self.ser.write(bytes([255, 0]))
        time.sleep(0.9)
        self.stop()
    
    def stop(self):
        self.ser.write(bytes([0, 0]))