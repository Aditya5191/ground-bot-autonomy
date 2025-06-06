
import time
from imu import IMU 
from send_images import ZMQCameraPublisher
from recieve import ZMQArrowReceiver

PC_IP= "10.19.0.1"



if __name__=="__main__":
 #---------- Innitialize Objects -----------#
    imu = IMU()
    camera = ZMQCameraPublisher()
    recv = ZMQArrowReceiver(PC_IP)




 #---------- start the loops -----------#
    camera.start()
    imu.start()
    recv.start()




 #----------- Variables ----------------#
    calib_data = imu.calib_mpu()
    yaw = 0.0
    dt = 0.05


 #------------- main loop -----------------#
    try:
        while True:
            yaw=imu.yaw
            distance = recv.distance
            angle = recv.angle
            direction = recv.direction

            print(f"yaw: {yaw}, distance: {distance}, angle: {angle}, direction: {direction}")

    except Exception as e:
        print(e)
        camera.stop()
        imu.stop()
        recv.stop()

