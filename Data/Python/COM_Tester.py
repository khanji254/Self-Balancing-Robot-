import serial

try:
    ser = serial.Serial('COM17', 9600)  # Update COM port
    print("COM port opened successfully")
    ser.close()
except serial.SerialException as e:
    print(f"Error opening COM port: {e}")
