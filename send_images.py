import zmq
import cv2
import base64
import time
from picamera2 import Picamera2

class ZMQCameraClient:
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUSH)
        self.socket.connect("tcp://192.168.1.100:5555")  # Replace with server IP and port

        self.picam2 = Picamera2()
        self.picam2.start()

    def camera_sender(self):
        while True:
            frame = self.picam2.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            _, buffer = cv2.imencode('.jpg', frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 70])  # optional: lower quality
            self.socket.send(buffer.tobytes())
            
            time.sleep(0.1)  # 10 FPS


# To run
# client = ZMQCameraClient()
# client.camera_sender()
