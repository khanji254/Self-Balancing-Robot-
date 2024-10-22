import serial
import csv
import time
import matplotlib.pyplot as plt
from collections import deque

# Replace 'COM_PORT' with your actual COM port (e.g., 'COM3' on Windows, '/dev/rfcomm0' on Linux)
COM_PORT = 'COM17'  # Update this to your Bluetooth module's port
BAUD_RATE = 38400

# Initialize serial connection
ser = serial.Serial(COM_PORT, BAUD_RATE, timeout=1)

# Data storage
data_buffer = deque(maxlen=100)  # Buffer for the last 100 readings
csv_file = open('mpu6050_data.csv', mode='w', newline='')
csv_writer = csv.writer(csv_file)
csv_writer.writerow(['Time', 'Temperature', 'Acc_X', 'Acc_Y', 'Acc_Z', 'Gyro_X', 'Gyro_Y', 'Gyro_Z', 'Acc_Angle_X', 'Acc_Angle_Y', 'Angle_X', 'Angle_Y', 'Angle_Z'])

# Plotting setup
plt.ion()  # Interactive mode on
fig, axs = plt.subplots(3, 1, figsize=(10, 8))

# Initialize lists for plotting
time_data = []
temp_data = []
acc_x_data = []
acc_y_data = []
acc_z_data = []
gyro_x_data = []
gyro_y_data = []
gyro_z_data = []

try:
    while True:
        line = ser.readline().decode('utf-8').strip()  # Read line from serial
        if line:
            # Split the data and convert to appropriate types
            values = line.split(',')
            temp = float(values[0])
            acc_x = float(values[1])
            acc_y = float(values[2])
            acc_z = float(values[3])
            gyro_x = float(values[4])
            gyro_y = float(values[5])
            gyro_z = float(values[6])
            acc_angle_x = float(values[7])
            acc_angle_y = float(values[8])
            angle_x = float(values[9])
            angle_y = float(values[10])
            angle_z = float(values[11])
            
            # Get current time
            current_time = time.time()
            data_buffer.append([current_time, temp, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, acc_angle_x, acc_angle_y, angle_x, angle_y, angle_z])
            csv_writer.writerow([current_time, temp, acc_x, acc_y, acc_z, gyro_x, gyro_y, gyro_z, acc_angle_x, acc_angle_y, angle_x, angle_y, angle_z])
            csv_file.flush()  # Write to CSV file immediately
            
            # Update plotting data
            time_data.append(current_time)
            temp_data.append(temp)
            acc_x_data.append(acc_x)
            acc_y_data.append(acc_y)
            acc_z_data.append(acc_z)
            gyro_x_data.append(gyro_x)
            gyro_y_data.append(gyro_y)
            gyro_z_data.append(gyro_z)

            # Clear the previous plots
            axs[0].cla()
            axs[1].cla()
            axs[2].cla()

            # Plot Temperature
            axs[0].plot(time_data, temp_data, label='Temperature (C)')
            axs[0].set_title('Temperature over Time')
            axs[0].set_xlabel('Time (s)')
            axs[0].set_ylabel('Temperature (C)')
            axs[0].legend()

            # Plot Accelerometer Data
            axs[1].plot(time_data, acc_x_data, label='Acc X')
            axs[1].plot(time_data, acc_y_data, label='Acc Y')
            axs[1].plot(time_data, acc_z_data, label='Acc Z')
            axs[1].set_title('Accelerometer Data over Time')
            axs[1].set_xlabel('Time (s)')
            axs[1].set_ylabel('Acceleration (g)')
            axs[1].legend()

            # Plot Gyroscope Data
            axs[2].plot(time_data, gyro_x_data, label='Gyro X')
            axs[2].plot(time_data, gyro_y_data, label='Gyro Y')
            axs[2].plot(time_data, gyro_z_data, label='Gyro Z')
            axs[2].set_title('Gyroscope Data over Time')
            axs[2].set_xlabel('Time (s)')
            axs[2].set_ylabel('Gyro (deg/s)')
            axs[2].legend()

            plt.pause(0.1)  # Pause to allow the plot to update

except KeyboardInterrupt:
    print("Exiting...")
finally:
    ser.close()
    csv_file.close()
