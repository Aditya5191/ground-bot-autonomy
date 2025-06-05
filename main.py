from send_images import ZMQCameraPublisher
from recieve import ZMQArrowReceiver
import threading

angle=0
distance=0
direction=""

camera= ZMQCameraPublisher()
pc_response =  ZMQArrowReceiver()
 

def get_response():
    while True:
        angle, distance, direction = pc_response.receive_arrow_data()
        print(angle,distance,direction)

camera_thread = threading.Thread(target=camera.publish_frames)
response_thread = threading.Thread(target=get_response)

camera_thread.start()
response_thread.start()
