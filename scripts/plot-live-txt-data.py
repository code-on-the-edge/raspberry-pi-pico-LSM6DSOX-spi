import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

# File containing the magnetometer data
data_file = '/home/mg/Documents/raspberry-pi-pico-projects/raspberry-pi-pico-LSM6DSOX/scripts/imu_data.csv'

# Initialize empty lists for data
time_data = []
x_data = []
y_data = []
z_data = []

# Create the figure and axes
fig, ax = plt.subplots()
ax.set_title("Live Magnetometer Data vs Time")
ax.set_xlabel("Time")
ax.set_ylabel("Magnetometer Value")

# Initialize lines for X, Y, Z data
line_x, = ax.plot([], [], label="X-axis", color="r")
line_y, = ax.plot([], [], label="Y-axis", color="g")
line_z, = ax.plot([], [], label="Z-axis", color="b")

# Set up the legend
ax.legend()

# Set up dynamic plot limits
ax.set_xlim(0, 10)  # Start with a small time window
ax.set_ylim(-100, 100)  # Update dynamically based on data range

# Function to read new data
def read_new_data():
    global time_data, x_data, y_data, z_data

    try:
        with open(data_file, "r") as f:
            lines = f.readlines()
            for i, line in enumerate(lines[len(time_data):]):
                values = list(map(float, line.strip().split(",")))
                time_data.append(len(time_data))  # Append incremental time value
                x_data.append(values[0])
                y_data.append(values[1])
                z_data.append(values[2])
    except Exception as e:
        print(f"Error reading data: {e}")

# Update function for animation
def update(frame):
    read_new_data()

    # Update plot data
    line_x.set_data(time_data, x_data)
    line_y.set_data(time_data, y_data)
    line_z.set_data(time_data, z_data)

    # Dynamically adjust the x-axis limits
    ax.set_xlim(0, len(time_data))

    # Dynamically adjust the y-axis limits based on data range
    if len(x_data) > 0:
        min_y = min(min(x_data), min(y_data), min(z_data))
        max_y = max(max(x_data), max(y_data), max(z_data))
        ax.set_ylim(min_y - 10, max_y + 10)

    return line_x, line_y, line_z

# Animate the plot
ani = FuncAnimation(fig, update, interval=1000, blit=True)

# Show the plot
plt.show()
