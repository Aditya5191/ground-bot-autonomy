import zmq
import cv2
import time
from picamera2 import Picamera2

class ZMQCameraPublisher:
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://*:5555")  # Bind to all interfaces on port 5555

        self.picam2 = Picamera2()
        self.picam2.start()

    def publish_frames(self):
        while True:
            frame = self.picam2.capture_array()
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            _, buffer = cv2.imencode('.jpg', frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
            self.socket.send(buffer.tobytes())  # Send raw JPEG bytes
            
            time.sleep(0.1)  # 10 FPS