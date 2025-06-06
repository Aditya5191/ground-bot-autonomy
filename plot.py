import csv
import matplotlib.pyplot as plt
from datetime import datetime

# Load data
timestamps = []
yaw_values = []

with open("yaw_log.csv", "r") as file:
    reader = csv.DictReader(file)
    for row in reader:
        # Parse time (HH:MM:SS.mmm) into datetime object
        time_obj = datetime.strptime(row["Time"], "%H:%M:%S.%f")
        timestamps.append(time_obj)
        yaw_values.append(float(row["Yaw"]))

# Plot yaw vs time
plt.figure(figsize=(10, 5))
plt.plot(timestamps, yaw_values, label="Yaw (deg)", color="blue")
plt.xlabel("Time")
plt.ylabel("Yaw")
plt.title("Yaw vs Time")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()
