import time

class TurnController:
    def __init__(self, imu, ser, calib_data, dt=0.05, pwm_value=255, tolerance=3):
        """
        Initialize the TurnController with IMU, serial, calibration data, and parameters.

        Args:
            imu: IMU object with .read() method.
            ser: serial.Serial object for motor control.
            calib_data: Calibration data dictionary with "gyro_bias" key.
            dt: Time step for yaw integration (seconds).
            pwm_value: PWM value for turning wheel (0-255).
            tolerance: Allowed error in degrees for stopping the turn.
        """
        self.imu = imu
        self.ser = ser
        self.calib_data = calib_data
        self.dt = dt
        self.pwm_value = pwm_value
        self.tolerance = tolerance

    def send_pwm(self, left_pwm, right_pwm):
        """
        Send PWM values to Arduino over serial.
        Args:
            left_pwm: PWM value for left motor (0-255).
            right_pwm: PWM value for right motor (0-255).
        """
        self.ser.write(bytes([left_pwm, right_pwm]))

    def _turn(self, direction, angle):
        """
        Internal method to perform the turn.
        Args:
            direction: "left" or "right".
            angle: Angle in degrees to turn.
        """
        # Initialize yaw by integrating gyro reading over one dt interval.
        yaw = 0.0
        try:
            gyro_z_raw = self.imu.read(0x47) - self.calib_data["gyro_bias"]
            gyro_z = gyro_z_raw / 131.0
            yaw += gyro_z * self.dt
        except Exception as e:
            print(f"IMU read error at start of turn: {e}")

        initial_yaw = yaw
        print(f"[TURN] Starting {direction} turn for {angle} degrees from yaw {initial_yaw:.2f}")

        turning = True
        try:
            while turning:
                try:
                    # Read gyro (z-axis) and update yaw
                    gyro_z_raw = self.imu.read(0x47) - self.calib_data["gyro_bias"]
                    gyro_z = gyro_z_raw / 131.0
                    yaw += gyro_z * self.dt

                    # Calculate how much we've turned from the starting yaw
                    delta = abs(yaw - initial_yaw)

                    if direction == "right":
                        self.send_pwm(self.pwm_value, 0)
                        print(f"[TURN] Turning RIGHT... yaw: {yaw:.2f}, turned: {delta:.2f} deg")
                        if delta >= angle - self.tolerance:
                            self.send_pwm(0, 0)
                            print("[TURN] Right turn complete.")
                            turning = False
                    elif direction == "left":
                        self.send_pwm(0, self.pwm_value)
                        print(f"[TURN] Turning LEFT... yaw: {yaw:.2f}, turned: {delta:.2f} deg")
                        if delta >= angle - self.tolerance:
                            self.send_pwm(0, 0)
                            print("[TURN] Left turn complete.")
                            turning = False
                    else:
                        print("[TURN] Unknown direction. Stopping motors for safety.")
                        self.send_pwm(0, 0)
                        turning = False

                except OSError as e:
                    # Handle I2C errors (e.g., IMU disconnects) robustly
                    print(f"[TURN] I2C error during turn: {e}. Retrying IMU initialization...")
                    try:
                        self.imu.bus.close()
                    except:
                        pass
                    time.sleep(0.2)
                    while not self.imu.init_imu():
                        print("[TURN] Retrying IMU initialization...")
                        time.sleep(0.2)
                    self.calib_data = self.imu.load_calib()
                time.sleep(self.dt)
        except Exception as e:
            print(f"[TURN] Error during turn: {e}")
            self.send_pwm(0, 0)

    def left(self, angle):
        """
        Turn left by the specified angle in degrees.
        Args:
            angle: Angle in degrees to turn left.
        """
        self._turn("left", angle)

    def right(self, angle):
        """
        Turn right by the specified angle in degrees.
        Args:
            angle: Angle in degrees to turn right.
        """
        self._turn("right", angle)
