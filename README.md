# Self-Balancing 2WD Robot

## Overview

This project focuses on creating a **self-balancing two-wheel robot** (2WD) using an Arduino, MPU6050 sensor, and an HC-05 Bluetooth module for wireless control. The robot utilizes Proportional-Integral-Derivative (PID) control to maintain balance and execute movements like forward, backward, left and right turns, as well as rotations in place. This system is designed to keep the robot upright by adjusting motor speeds based on real-time sensor data.

## Components

1. **Arduino (Uno or Nano)**: The microcontroller that processes sensor data and controls the motors.
2. **MPU6050 (6-axis Gyro/Accelerometer)**: Provides yaw, pitch, and roll angles to the Arduino for maintaining balance.
3. **HC-05 Bluetooth Module**: Enables wireless control from a mobile device.
4. **L298N Motor Driver**: Drives the two DC motors responsible for the movement of the robot.
5. **DC Motors (x2)**: Power the robot's wheels.
6. **Power Supply**: A battery pack to power the Arduino, motor driver, and motors.
7. **Wheels (x2)**: Attached to the motors to create the movement and balance.

## Working Principle

The self-balancing robot uses the MPU6050 sensor to continuously measure the robot’s orientation. The sensor provides the yaw, pitch, and roll angles, which are processed by a PID controller to calculate the necessary motor speeds to maintain balance. 

The Bluetooth module allows remote control of the robot, letting it move forward, backward, rotate, or turn based on received commands.

### Main Features:
- **PID Control**: Adjusts motor speed to keep the robot balanced.
- **Wireless Control**: Controlled through the HC-05 module using a mobile app or Bluetooth device.
- **Real-Time Feedback**: The robot constantly adjusts its position to remain upright using data from the MPU6050.

## Code Breakdown

### 1. **MPU6050 and Serial Communication**
The Arduino receives orientation data (yaw, pitch, and roll) from the MPU6050 sensor via I2C communication. The data is sent to the robot’s control system, which uses PID control logic to ensure the robot remains upright.

The relevant part of the code reads and processes sensor data:
```cpp
String data = String("ypr,") + 
              String(ypr[0] * 180/M_PI) + "," +
              String(ypr[1] * 180/M_PI) + "," +
              String(ypr[2] * 180/M_PI);
BTSerial.println(data);
```

### 2. **Motor Control**
The robot has two DC motors controlled using the L298N motor driver. The Arduino sends PWM signals to the motor driver to adjust the speed and direction of the motors.

The motor control is handled by the following functions:
- **Move_Forward()**: Moves the robot forward by driving both motors forward.
- **Move_Backward()**: Moves the robot backward by reversing both motors.
- **Turn_Left() and Turn_Right()**: Adjusts motor speeds for left and right turns.
- **Rotate_Left() and Rotate_Right()**: Spins the robot on the spot by rotating the motors in opposite directions.

### 3. **PID Control**
The PID controller ensures that the robot remains balanced by adjusting the motor speeds based on the difference between the current angle and the desired setpoint (usually vertical). 

The loop in the code where the angles are read and adjustments are made:
```cpp
mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
// Calculate and adjust motor speed using PID logic
```

### 4. **Bluetooth Control**
The HC-05 Bluetooth module receives commands from a mobile device. These commands dictate whether the robot should move forward, backward, or rotate. The Bluetooth input is processed in the loop, and the corresponding motor control functions are called.

### Commands:
- **F**: Move forward.
- **B**: Move backward.
- **L**: Turn left.
- **R**: Turn right.
- **Z**: Rotate left.
- **X**: Rotate right.
- **S**: Stop all movements.

## How to Use

### Hardware Setup:
1. Connect the MPU6050 to the Arduino using the I2C protocol.
2. Wire the L298N motor driver to the Arduino and DC motors.
3. Attach the HC-05 Bluetooth module to the Arduino for wireless control.
4. Connect the power supply (battery pack) to power the Arduino and motors.

### Software:
1. Upload the provided Arduino sketch to your Arduino board.
2. Pair your Bluetooth device with the HC-05 module.
3. Use a Bluetooth terminal app or custom controller app to send commands (F, B, L, R, Z, X, S) to the robot.

### Calibration:
The PID parameters need to be tuned to ensure the robot remains upright. Adjust the proportional, integral, and derivative values based on your hardware setup and the robot's behavior during operation.

## Future Improvements
- **Improved Stabilization**: Fine-tune the PID controller for better stability.
- **Obstacle Avoidance**: Integrate ultrasonic sensors for autonomous movement.
- **Advanced Control**: Implement custom apps for easier wireless control.

## Conclusion
This self-balancing 2WD robot uses sensor data, PID control, and Bluetooth communication to perform tasks while maintaining balance. It demonstrates how real-time feedback systems work in robotics, making it an excellent project for learning about dynamic systems, control theory, and embedded electronics.
