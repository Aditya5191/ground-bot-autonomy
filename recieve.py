import zmq
import json
import threading
import time

class ZMQArrowReceiver:
    def __init__(self, pc_ip="10.29.0.1", port=5566):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{pc_ip}:{port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, '')

        self.angle = None
        self.distance = None
        self.direction = None

        self.running = False
        self.thread = None

    def _listen_loop(self):
        while self.running:
            try:
                msg = self.socket.recv_json(flags=zmq.NOBLOCK)
                self.angle = msg.get("angle")
                self.distance = msg.get("distance")
                self.direction = msg.get("arrow_dir")
            except zmq.Again:
                time.sleep(0.01)  # No data, wait a bit
            except Exception as e:
                print("Error receiving arrow data:", e)

    def start(self):
        if not self.running:
            self.running = True
            self.thread = threading.Thread(target=self._listen_loop)
            self.thread.daemon = True
            self.thread.start()

    def stop(self):
        self.running = False
        if self.thread:
            self.thread.join()
        self.socket.close()
        self.context.term()

    def get_latest(self):
        return self.angle, self.distance, self.direction
    
    def set_latest(self, angle ,distance, direction):
        self.angle = angle
        self.distance = distance
        self.direction = direction

