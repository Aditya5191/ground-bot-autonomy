import cv2
import numpy as np
import zmq
from ultralytics import YOLO
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import PolynomialFeatures
import json

# Load YOLO model
model = YOLO("C:/Users/adity/OneDrive/Desktop/ground bot autonomy/PC _SIDE/best.pt")

# IP of the Raspberry Pi that is streaming frames
pi_IP = "10.29.15.54"

# Calibration: heights vs distances
X = np.array([154, 81, 47, 28, 24, 17, 18, 13, 12, 10]).reshape(-1, 1)
distances = np.array([10, 30, 50, 90, 120, 155, 160, 190, 220, 257]).reshape(-1, 1)

# Polynomial regression model
poly_features = PolynomialFeatures(degree=5, include_bias=False)
X_poly = poly_features.fit_transform(1 / X)
regressor = LinearRegression()
regressor.fit(X_poly, distances)

# ZMQ Context and sockets
context = zmq.Context()

# SUB socket to receive frames from Pi
frame_receiver = context.socket(zmq.SUB)
frame_receiver.connect(f"tcp://{pi_IP}:5555")
frame_receiver.setsockopt_string(zmq.SUBSCRIBE, '')

# PUB socket to send results back to Pi
result_sender = context.socket(zmq.PUB)
result_sender.bind("tcp://*:5566")  # Pi should connect to this

# Distance estimation

def get_distance(h):
    h_inv = 1 / h
    h_poly = poly_features.transform([[h_inv]])
    d = regressor.predict(h_poly)[0][0]
    return d - (d / 10)

# Main processing loop
while True:
    jpg_data = frame_receiver.recv()
    npimg = np.frombuffer(jpg_data, dtype=np.uint8)
    frame = cv2.imdecode(npimg, cv2.IMREAD_COLOR)

    if frame is None:
        continue

    rgb_frame = frame
    results = model(rgb_frame)[0]

    for box in results.boxes:
        conf = float(box.conf[0])
        if conf >= 0.6:
            x1, y1, x2, y2 = map(int, box.xyxy[0])
            cls_id = int(box.cls[0])
            label = model.names[cls_id]
            height = y2 - y1

            if height <= 0:
                continue

            distance = get_distance(height)
            box_center = (x1 + x2) // 2
            frame_center = frame.shape[1] // 2
            angle_offset = (box_center - frame_center) * (60 / frame.shape[1])

            direction = "left" if label.lower() == "left" else "right"

            msg = {
                "angle": angle_offset,
                "distance": distance,
                "arrow_dir": direction
            }

            result_sender.send_json(msg)

            cv2.rectangle(rgb_frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.putText(rgb_frame, f"{label} {distance:.2f}cm", (x1, y1 - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    cv2.imshow("YOLO + Distance", rgb_frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
