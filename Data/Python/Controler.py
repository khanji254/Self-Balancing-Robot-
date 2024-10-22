import serial
import keyboard
import time

# Replace 'COM_PORT' with your actual COM port (e.g., 'COM3' on Windows, '/dev/rfcomm0' on Linux)
COM_PORT = 'COM17'  # Make sure this is the correct port for your HC-05
BAUD_RATE = 38400  # Baud rate for HC-05

def connect_to_hc05():
    # Try to connect to the HC-05 Bluetooth module
    attempts = 0
    max_attempts = 5  # Number of attempts to connect

    while attempts < max_attempts:
        try:
            bt_serial = serial.Serial(COM_PORT, BAUD_RATE)
            print(f"Connected to {COM_PORT} at {BAUD_RATE} baud rate.")
            return bt_serial
        except Exception as e:
            print(f"Attempt {attempts + 1}: Failed to connect: {e}")
            attempts += 1
            time.sleep(2)  # Wait before retrying

    print("Max connection attempts reached. Exiting...")
    return None

def main():
    bt_serial = connect_to_hc05()
    if bt_serial is None:
        return  # Exit if connection was not successful

    print("Press W to move forward, S to move backward, A to turn left, D to turn right, X to stop.")

    try:
        while True:
            if keyboard.is_pressed('w'):  # Move Forward
                bt_serial.write(b'F')  # Send 'F' command
                print("Moving Forward")
                time.sleep(0.1)  # Debounce time
            elif keyboard.is_pressed('s'):  # Move Backward
                bt_serial.write(b'B')  # Send 'B' command
                print("Moving Backward")
                time.sleep(0.1)  # Debounce time
            elif keyboard.is_pressed('a'):  # Turn Left
                bt_serial.write(b'L')  # Send 'L' command
                print("Turning Left")
                time.sleep(0.1)  # Debounce time
            elif keyboard.is_pressed('d'):  # Turn Right
                bt_serial.write(b'R')  # Send 'R' command
                print("Turning Right")
                time.sleep(0.1)  # Debounce time
            elif keyboard.is_pressed('x'):  # Stop
                bt_serial.write(b'S')  # Send 'S' command
                print("Stopping")
                time.sleep(0.1)  # Debounce time

            time.sleep(0.1)  # Delay for a short period to avoid overwhelming the serial buffer
    except KeyboardInterrupt:
        print("Exiting...")
    finally:
        bt_serial.close()  # Close the Bluetooth connection

if __name__ == "__main__":
    main()
