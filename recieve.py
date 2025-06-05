import zmq
import json

class ZMQArrowReceiver:
    def __init__(self, pc_ip="10.29.0.1", port=5566):
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{pc_ip}:{port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, '')
        print(f"📡 Connected to tcp://{pc_ip}:{port} to receive arrow data")

    def receive_arrow_data(self):
        try:
            msg = self.socket.recv_json()
            angle = msg["angle"]
            distance = msg["distance"]
            direction = msg["arrow_dir"]
            return angle, distance, direction
        except Exception as e:
            print("⚠️ Error receiving arrow data:", e)
            return None, None, None

# Example usage:
# receiver = ZMQArrowReceiver("192.168.1.100")  # Replace with actual PC IP
# while True:
#     angle, distance, direction = receiver.receive_arrow_data()
#     if angle is not None:
#         print(f"➡️  Direction: {direction}, Angle: {angle:.2f}, Distance: {distance:.2f} cm")
