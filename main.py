
import time
from imu import IMU 
from send_images import ZMQCameraPublisher



if __name__=="__main__":
 #---------- Innitialize Objects -----------#
    imu = IMU()
    camera = ZMQCameraPublisher()




 #---------- start the loops -----------#
    




 #----------- Variables ----------------#
    calib_data = imu.calib_mpu()
    yaw = 0.0
    dt = 0.05


 #------------- main loop -----------------#



