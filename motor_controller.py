import serial
import time

class MotorPIDController:
    def __init__(self, kp=7.0, ki=-0.02, kd=0.05, port='/dev/serial0', baudrate=9600):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.integral = 0
        self.serial = serial.Serial(port, baudrate, timeout=1)
        time.sleep(2)  # Let serial port stabilize

    def compute_pwm(self, target_angle):
        error = target_angle
        self.integral += error
        derivative = error - self.prev_error
        output = self.kp * error + self.ki * self.integral + self.kd * derivative
        self.prev_error = error

        # Clamp output to -255 to +255
        output = max(min(output, 255), -255)

        # Determine motor speeds and directions
        base_speed = 150  # Adjustable central speed

        left_speed = base_speed + output
        right_speed = base_speed - output

        # Clamp and determine direction
        def clamp_direction(speed):
            direction = 1 if speed >= 0 else 0  # 1 = forward, 0 = backward
            pwm = abs(int(speed))
            pwm = max(60, min(pwm, 255))  # Avoid very low PWM
            return pwm, direction

        left_pwm, left_dir = clamp_direction(left_speed)
        right_pwm, right_dir = clamp_direction(right_speed)

        # Format and send
        data = f"{left_pwm},{left_dir},{right_pwm},{right_dir}\n"
        self.serial.write(data.encode())

        print(f"[PID] Angle: {target_angle:.2f} | L_PWM: {left_pwm} Dir: {left_dir} | R_PWM: {right_pwm} Dir: {right_dir}")

    def stop(self):
        self.serial.write(b"0,1,0,1\n")  # Stop both motors

