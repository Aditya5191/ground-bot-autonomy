import time

def send_pwm(ser, left_pwm, right_pwm):
    """
    Send PWM values to Arduino over serial.
    Args:
        ser: serial.Serial object for UART communication.
        left_pwm: PWM value for the left motor (0-255).
        right_pwm: PWM value for the right motor (0-255).
    """
    ser.write(bytes([left_pwm, right_pwm]))

def turn_robot(imu, ser, direction, angle, calib_data, dt=0.05, pwm_value=255, tolerance=3):
    """
    Turns the robot left or right by a specified angle using IMU feedback.
    This function blocks until the turn is complete.

    Args:
        imu: IMU object with .read() method for reading sensor data.
        ser: serial.Serial object for sending PWM commands to Arduino.
        direction: "left" or "right" (string), direction to turn.
        angle: Target turn angle in degrees (positive number).
        calib_data: Calibration data dictionary with "gyro_bias" key.
        dt: Time step for yaw integration (seconds).
        pwm_value: PWM value for the turning wheel (default: 255, max speed).
        tolerance: Allowed error in degrees for stopping the turn (default: 3).
    """

    # Initialize yaw by integrating gyro reading over one dt interval.
    yaw = 0.0
    try:
        gyro_z_raw = imu.read(0x47) - calib_data["gyro_bias"]
        gyro_z = gyro_z_raw / 131.0  # Convert raw gyro value to deg/sec
        yaw += gyro_z * dt
    except Exception as e:
        print(f"IMU read error at start of turn: {e}")

    initial_yaw = yaw
    print(f"[TURN] Starting {direction} turn for {angle} degrees from yaw {initial_yaw:.2f}")

    turning = True
    try:
        while turning:
            try:
                # Read gyro (z-axis) and update yaw
                gyro_z_raw = imu.read(0x47) - calib_data["gyro_bias"]
                gyro_z = gyro_z_raw / 131.0  # deg/sec
                yaw += gyro_z * dt

                # Calculate how much we've turned from the starting yaw
                delta = abs(yaw - initial_yaw)

                if direction == "right":
                    send_pwm(ser, pwm_value, 0)  # Left wheel moves, right wheel stopped
                    print(f"[TURN] Turning RIGHT... yaw: {yaw:.2f}, turned: {delta:.2f} deg")
                    # Stop when desired turn (within tolerance) is reached
                    if delta >= angle - tolerance:
                        send_pwm(ser, 0, 0)
                        print("[TURN] Right turn complete.")
                        turning = False
                elif direction == "left":
                    send_pwm(ser, 0, pwm_value)  # Right wheel moves, left wheel stopped
                    print(f"[TURN] Turning LEFT... yaw: {yaw:.2f}, turned: {delta:.2f} deg")
                    if delta >= angle - tolerance:
                        send_pwm(ser, 0, 0)
                        print("[TURN] Left turn complete.")
                        turning = False
                else:
                    print("[TURN] Unknown direction. Stopping motors for safety.")
                    send_pwm(ser, 0, 0)
                    turning = False

            except OSError as e:
                # Handle I2C errors (e.g., IMU disconnects) robustly
                print(f"[TURN] I2C error during turn: {e}. Retrying IMU initialization...")
                try:
                    imu.bus.close()
                except:
                    pass
                time.sleep(0.2)
                while not imu.init_imu():
                    print("[TURN] Retrying IMU initialization...")
                    time.sleep(0.2)
                calib_data = imu.load_calib()
            time.sleep(dt)
    except Exception as e:
        print(f"[TURN] Error during turn: {e}")
        send_pwm(ser, 0, 0)  # Stop motors on any unexpected error

