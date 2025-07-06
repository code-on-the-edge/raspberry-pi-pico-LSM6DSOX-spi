import csv
import time
import serial as sr

# Serial port configuration
s = sr.Serial('/dev/ttyACM0', 115200)

# CSV file path
csv_file_path = '/home/mg/Documents/raspberry-pi-pico-projects/raspberry-pi-pico-LSM6DSOX/scripts/imu_data.csv'

# CSV header
fieldnames = ["x", "y", "z"]

# Write header once at the start
with open(csv_file_path, 'w') as csv_file:
    csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)
    csv_writer.writeheader()

try:
    while True:
        with open(csv_file_path, 'a') as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames=fieldnames)

            # Read line from serial port
            line = s.readline().decode().strip()
            parts = line.split(',')

            try:
                if len(parts) == 3:
                    x = float(parts[0])
                    y = float(parts[1])
                    z = float(parts[2])

                    # Write to CSV
                    info = {"x": x, "y": y, "z": z}
                    csv_writer.writerow(info)
                    print(x, y, z)
            except ValueError:
                print("Invalid input:", line)

        time.sleep(0.004)  # 4 ms delay

except KeyboardInterrupt:
    print("Stopped logging.")
    s.close()
