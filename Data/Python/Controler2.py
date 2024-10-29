import serial
import time

# Replace with the correct COM port (e.g., COM3, COM4) and baud rate
com_port = "COM7"
baud_rate = 38400  # Match the baud rate set in the Arduino sketch
timeout = 1  # 1 second timeout for the connection

# Create a connection to the COM port
try:
    ser = serial.Serial(com_port, baud_rate, timeout=timeout)
    print(f"Connected to {com_port} at {baud_rate} baud rate.")
except serial.SerialException as e:
    print(f"Failed to connect to {com_port}: {e}")
    exit()

# Define available commands
commands = {
    'F': "Move Forward",
    'B': "Move Backward",
    'L': "Turn Left",
    'R': "Turn Right",
    'Z': "Rotate Left",
    'X': "Rotate Right",
    'S': "Stop Motors"
}

def send_command(command):
    """Send a command to the bot and wait for response."""
    if command in commands:
        ser.write(command.encode())  # Send the command
        print(f"Sent command: {commands[command]}")

        # Wait for feedback from the bot (optional)
        time.sleep(0.5)
        while ser.in_waiting:
            response = ser.readline().decode().strip()
            if response:
                print(f"Response: {response}")
    else:
        print("Invalid command. Use one of the available commands: ", list(commands.keys()))

def close_connection():
    """Close the serial connection."""
    if ser.is_open:
        ser.close()
        print("Connection closed.")

# Main loop to send commands based on user input
try:
    while True:
        print("\nAvailable commands:")
        for key, value in commands.items():
            print(f" {key}: {value}")
        command = input("\nEnter a command (or 'q' to quit): ").upper()

        if command == 'Q':
            break
        else:
            send_command(command)

except KeyboardInterrupt:
    print("Exiting the program.")

finally:
    close_connection()
