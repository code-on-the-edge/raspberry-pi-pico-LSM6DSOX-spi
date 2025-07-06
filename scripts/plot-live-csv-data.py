import matplotlib
matplotlib.use("TkAgg")  # Force interactive backend (can also try "Qt5Agg")

import matplotlib.pyplot as plt
import csv
import time
import os

# === CONFIGURATION ===
csv_file_path = '/home/mg/Documents/raspberry-pi-pico-projects/raspberry-pi-pico-LSM6DSOX/scripts/imu_data.csv'
update_interval = 0.001  # seconds
max_points = 500       # number of samples to display

# === INITIALIZE DATA ===
x_vals, y_vals, z_vals = [], [], []

plt.ion()  # Turn on interactive plotting
fig, ax = plt.subplots()
line_x, = ax.plot([], [], 'r-', label='X')
line_y, = ax.plot([], [], 'g-', label='Y')
line_z, = ax.plot([], [], 'b-', label='Z')
ax.set_title("Live IMU Data (X, Y, Z)")
ax.set_xlabel("Sample")
ax.set_ylabel("Value")
ax.legend()
ax.set_xlim(0, max_points)
ax.set_ylim(-200, 200)

# === MAIN LOOP ===
try:
    print("Plotting live data... Press Ctrl+C to stop.")
    last_line_count = 0

    while True:
        if not os.path.exists(csv_file_path):
            print("Waiting for CSV file...")
            time.sleep(update_interval)
            continue

        with open(csv_file_path, 'r') as f:
            lines = f.readlines()[1:]  # Skip header

        new_lines = lines[last_line_count:]
        last_line_count = len(lines)

        for line in new_lines:
            try:
                x, y, z = map(float, line.strip().split(','))
                x_vals.append(x)
                y_vals.append(y)
                z_vals.append(z)
            except ValueError:
                continue  # Skip malformed lines

        # Keep only the last max_points
        x_vals = x_vals[-max_points:]
        y_vals = y_vals[-max_points:]
        z_vals = z_vals[-max_points:]

        samples = list(range(len(x_vals)))
        line_x.set_data(samples, x_vals)
        line_y.set_data(samples, y_vals)
        line_z.set_data(samples, z_vals)

        ax.set_xlim(max(0, len(samples) - max_points), len(samples))
        all_vals = x_vals + y_vals + z_vals
        if all_vals:
            ax.set_ylim(min(all_vals) - 10, max(all_vals) + 10)

        plt.draw()
        plt.pause(update_interval)

except KeyboardInterrupt:
    print("\nLive plotting stopped.")
    plt.ioff()
    plt.show()
