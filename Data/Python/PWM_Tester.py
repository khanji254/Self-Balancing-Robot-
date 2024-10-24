import serial
import time
import keyboard  # Requires 'keyboard' module, install using `pip install keyboard`

ser = serial.Serial('COM17', 38400)  # Update COM port for your Arduino

def send_command(command):
    ser.write(command.encode())
    time.sleep(0.1)

print("Use Left (L), Right (R), Forward (F), and Back (B) buttons to control")

try:
    while True:
        if keyboard.is_pressed('left'):
            send_command('L')
            print("Left wheel selected")
            time.sleep(0.3)
        elif keyboard.is_pressed('right'):
            send_command('R')
            print("Right wheel selected")
            time.sleep(0.3)
        elif keyboard.is_pressed('up'):
            send_command('F')
            print("Increasing speed")
            time.sleep(0.3)
        elif keyboard.is_pressed('down'):
            send_command('B')
            print("Decreasing speed")
            time.sleep(0.3)

except KeyboardInterrupt:
    print("Exiting")
    ser.close()
