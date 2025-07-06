import csv
import time
import serial as sr

# Serial port configuration
s = sr.Serial('/dev/ttyACM0', 115200)

# Text file configuration
txt_file_path = '/home/mg/Documents/raspberry-pi-pico-projects/raspberry-pi-pico-LSM6DSOX/scripts/imu_data.txt'
# txt_file_path = '/home/mg/Documents/diy-drone/raspberry-pi-drone/raw_data/imu_data_cal.txt'

# Fieldnames for the text file
fieldnames = ["mag_x", "mag_y", "mag_z"]

# Initialize and write the header (tab-delimited)
with open(txt_file_path, 'w', newline='') as txt_file:
    csv_writer = csv.DictWriter(txt_file, fieldnames=fieldnames, delimiter='\t')
    # csv_writer.writeheader()

# Main loop to read data and write to text file
while True:
    with open(txt_file_path, 'a', newline='') as txt_file:
        csv_writer = csv.DictWriter(txt_file, fieldnames=fieldnames, delimiter='\t')

        # Read a line from the serial port
        line = s.readline().decode().strip()
        parts = line.split(',')

        try:
            # Check if there are exactly 3 elements in the line
            if len(parts) == 3:
                # Parse mag_x, mag_y, mag_z values
                mag_x = float(parts[0])
                mag_y = float(parts[1])
                mag_z = float(parts[2])

                # Prepare data to write to the text file
                info = {
                    "mag_x": mag_x,
                    "mag_y": mag_y,
                    "mag_z": mag_z
                }
                # Write data to the text file
                csv_writer.writerow(info)
                print(mag_x, mag_y, mag_z)

        except ValueError:
            # Ignore invalid lines that can't be parsed as floats
            print("Invalid input:", line)

    # Add a delay to match the data sampling rate (4 ms)
    time.sleep(0.004)
