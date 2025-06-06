# test_receiver.py
import zmq
import json

context = zmq.Context()
socket = context.socket(zmq.SUB)
socket.connect("tcp://10.19.0.1:5566")  # Make sure this matches sender IP
socket.setsockopt_string(zmq.SUBSCRIBE, '')

print("Listening...")

while True:
    try:
        msg = socket.recv_json(flags=0)
        print("Received:", msg)
    except Exception as e:
        print("Error:", e)