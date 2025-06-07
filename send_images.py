import zmq
import cv2
import time
from picamera2 import Picamera2
import threading


class ZMQCameraPublisher:
    def __init__(self):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind("tcp://*:5555")  # Publisher socket

        self.picam2 = Picamera2()
        self.picam2.start()

        # Thread control
        self.running = True
        self.thread = threading.Thread(target=self.publish_frames, daemon=True)
    
    def start(self):
        """Start the camera thread"""
        self.thread.start()
    
    def publish_frames(self):
        while self.running:
            try:
                frame = self.picam2.capture_array()
                frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
                _, buffer = cv2.imencode('.jpg', frame_bgr, [int(cv2.IMWRITE_JPEG_QUALITY), 70])
                self.socket.send(buffer.tobytes())
                time.sleep(0.2)  # ~10 FPS
            except Exception as e:
                print(f"Camera error: {e}")
                break

    def stop(self):
        """Stop the thread safely"""
        self.running = False
        self.thread.join()
        self.picam2.stop()
        self.socket.close()
        self.context.term()
        print("Camera publisher stopped.")