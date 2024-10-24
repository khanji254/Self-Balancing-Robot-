#include <SoftwareSerial.h>
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"

// Bluetooth Serial settings
SoftwareSerial BTSerial(10, 11); // RX, TX pins for HC-05

// Define the Variables for Motor Control
int ENA = 4;   // Enable pin for right motor (PWM pin)
int IN1 = 5;   // Input pin for right motor (control pin 1)
int IN2 = 6;   // Input pin for right motor (control pin 2)
int IN3 = 7;   // Input pin for left motor (control pin 1)
int IN4 = 8;   // Input pin for left motor (control pin 2)
int ENB = 9;   // Enable pin for left motor (PWM pin)

// Variables for Bluetooth Communication
char command; // to store incoming Bluetooth command

// PID Variables
float setPoint = 0; // Desired angle (straight ahead)
float input;        // Current angle from MPU6050
float output;       // Motor speed adjustment
float Kp = 2.0;     // Proportional gain
float Ki = 0.0;     // Integral gain
float Kd = 1.0;     // Derivative gain
float lastInput = 0; // Last input for derivative calculation
float integral = 0;  // Integral sum

// MPU6050 Setup
MPU6050 mpu;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
Quaternion q;           // quaternion container
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container

void setup() {
  // Initialize motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  // Initialize Serial communication for HC-05 Bluetooth module
  BTSerial.begin(38400); // Start Bluetooth Serial
  Serial.begin(9600);   // Start Serial for debugging

  // MPU6050 initialization
  Wire.begin();
  mpu.initialize();

  // Check MPU6050 connection
  if (mpu.testConnection()) {
    Serial.println("MPU6050 connection successful");
  } else {
    Serial.println("MPU6050 connection failed");
  }

  // Load and configure the DMP
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    dmpReady = true;
  }
}

void loop() {
  if (!dmpReady) return;

  // Get current yaw, pitch, and roll data from MPU6050
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) {
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    input = ypr[0] * 180 / M_PI; // Convert to degrees

    // PID control logic
    float error = setPoint - input; // Calculate error
    integral += error;               // Integral term
    float derivative = input - lastInput; // Derivative term
    output = Kp * error + Ki * integral - Kd * derivative; // PID output

    lastInput = input; // Save input for next loop

    // Adjust motor speeds based on PID output
    adjustMotors(output);

    // Handle Bluetooth commands
    if (BTSerial.available()) {
      command = BTSerial.read();
      Serial.print("Received command: ");
      Serial.println(command);
      // Control the bot based on the command received
      switch (command) {
        case 'F': Move_Forward(); break;
        case 'B': Move_Backward(); break;
        case 'L': Turn_Left(); break;
        case 'R': Turn_Right(); break;
        case 'S': Stop_Motors(); break;
        case 'Z': Rotate_Left(); break;
        case 'X': Rotate_Right(); break;
      }
    }
  }
}

// Function to stop all motor movements
void Stop_Motors() {
  analogWrite(ENA, 0); // Stop right motor
  analogWrite(ENB, 0); // Stop left motor
  Serial.println("Motors stopped.");
}

// Function to adjust motors based on PID output
void adjustMotors(float adjustment) {
  int baseSpeed = 200; // Base speed for forward movement
  int leftSpeed = baseSpeed + adjustment;
  int rightSpeed = baseSpeed - adjustment;

  // Ensure speed values are within the valid range (0-255)
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);

  analogWrite(ENA, rightSpeed); // Right motor
  analogWrite(ENB, leftSpeed);   // Left motor

  Serial.print("Left Speed: ");
  Serial.print(leftSpeed);
  Serial.print(" | Right Speed: ");
  Serial.println(rightSpeed);
}

// Function to move the bot forward
void Move_Forward() {
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, HIGH);  // Right wheel forward
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, HIGH);  // Left wheel forward
  Serial.println("Moving forward.");
}

// Function to move the bot backward
void Move_Backward() {
  digitalWrite(IN1, HIGH);
  digitalWrite(IN2, LOW);  // Right wheel backward
  digitalWrite(IN3, HIGH);
  digitalWrite(IN4, LOW);  // Left wheel backward
  Serial.println("Moving backward.");
}

// Function to turn the bot left
void Turn_Left() {
  analogWrite(ENA, 255); // Right motor at full speed
  analogWrite(ENB, 100); // Left motor at lower speed
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);  // Right wheel forward
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH);  // Left wheel forward
  Serial.println("Turning left.");
}

// Function to turn the bot right
void Turn_Right() {
  analogWrite(ENA, 100); // Right motor at lower speed
  analogWrite(ENB, 255); // Left motor at full speed
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);  // Right wheel forward
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH);  // Left wheel forward
  Serial.println("Turning right.");
}


// Function to rotate the bot left (pivot in place)
void Rotate_Left() {
  analogWrite(ENA, 255); // Right motor at full speed
  analogWrite(ENB, 255); // Left motor at full speed
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);  // Right wheel forward
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);   // Left wheel backward
  Serial.println("Rotating left.");
}

// Function to rotate the bot right (pivot in place)
void Rotate_Right() {
  analogWrite(ENA, 255); // Right motor at full speed
  analogWrite(ENB, 255); // Left motor at full speed
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);   // Right wheel backward
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH);  // Left wheel forward
  Serial.println("Rotating right.");
}

