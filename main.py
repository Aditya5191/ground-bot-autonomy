import time
from imu import IMU 
from send_images import ZMQCameraPublisher
from recieve import ZMQArrowReceiver
from motor_controller import Motor_ctrl
from datetime import datetime
import csv

PC_IP= "10.19.61.158"

if __name__=="__main__":
 #---------- Innitialize Objects -----------#
    imu = IMU()
    camera = ZMQCameraPublisher()
    recv = ZMQArrowReceiver(PC_IP)
    motor = Motor_ctrl()

 #---------- start the loops -----------#
    camera.start()
    imu.start()
    recv.start()

 #----------- Variables ----------------#
    dt = 0.05
    error_sum = 0.0
    last_error = 0.0
    Kp = 6.0
    Ki = 2.0
    Kd = 0.5
    RIGHT_PWM = 150
    x=1

    # CSV logging setup
    logfile = open("yaw_log.csv", "w", newline='')
    csv_writer = csv.writer(logfile)
    csv_writer.writerow(["Time", "Yaw", "Left_PWM", "Right_PWM"])

 #------------- main loop -----------------#
    try:
        while True:
            yaw=imu.yaw
            angle,distance,direction = recv.get_latest()
            
            if direction == None and angle == None:
                error = yaw
                error_sum += error * (dt*x)
                d_error = (error - last_error) / (dt*x)
                last_error = error

                # PID control
                control = Kp * error + Ki * error_sum + Kd * d_error

                # Adjust left motor based on yaw
                left_pwm = int(RIGHT_PWM + control)
                left_pwm = max(0, min(255, left_pwm))
                motor.write_pwm(left_pwm,RIGHT_PWM)

                # Time in HH:MM:SS.mmm format
                now = datetime.now()
                timestamp = now.strftime("%H:%M:%S.%f")[:-3]

                print(f"[STRAIGHT] {timestamp} yaw: {yaw}, left: {left_pwm}, right: {RIGHT_PWM}")
                csv_writer.writerow([timestamp, round(yaw, 2), left_pwm, RIGHT_PWM])

            elif distance != None and angle != None:
                if not distance <=30:
                    error = angle
                    error_sum += error * (dt*x)
                    d_error = (error - last_error) / (dt*x)
                    last_error = error

                    # PID control
                    control = Kp * error + Ki * error_sum + Kd * d_error

                    # Adjust left motor based on error
                    left_pwm = int(RIGHT_PWM + control)
                    left_pwm = max(0, min(255, left_pwm))
                    motor.write_pwm(left_pwm,RIGHT_PWM)
                    print(f"[ALIGNING] angle: {angle}, left: {left_pwm}, right: {RIGHT_PWM}")

                else:
                    if direction == "left":
                        print(f"[LEFT] left: 0 right: 225")
                        motor.turn_left()
                        recv.set_latest(None,None,None)
                        imu.restart()
                        time.sleep(2)

                    elif direction == "right":
                        print(f"[RIGHT] left: 225 right: 0")
                        motor.turn_right()
                        recv.set_latest(None,None,None)
                        imu.restart()
                        time.sleep(2)

            time.sleep(dt)

    except Exception as e:
        print(e)
        camera.stop()
        imu.stop()
        recv.stop()
        logfile.close()
