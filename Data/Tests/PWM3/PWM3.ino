#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include <SoftwareSerial.h>

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

// Default PID constants
//float Kp = 0.11;
//float Ki = 0.0009;
//float Kd = 0.010;
float Kp = 10;
float Ki =0;
float Kd =0;

float lastInput = 0; // Last input for derivative calculation
float integral = 0;  // Integral sum


#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

MPU6050 mpu;

#define OUTPUT_READABLE_YAWPITCHROLL

#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}

// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================

void setup() {

   // Initialize motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);



    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
        Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    BTSerial.begin(38400); // Start Bluetooth Serial (38400 baud rate)
    Serial.begin(115200);  // Start USB Serial for debugging
    
    while (!Serial); // Wait for Leonardo or other boards to be ready

    // initialize device
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    pinMode(INTERRUPT_PIN, INPUT);

    // verify connection
    Serial.println(F("Testing device connections..."));
    Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

    // wait for 3 seconds before starting DMP initialization
    delay(3000); // Wait for 3 seconds

    // load and configure the DMP
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXGyroOffset(220);
    mpu.setYGyroOffset(76);
    mpu.setZGyroOffset(-85);
    mpu.setZAccelOffset(1788); // 1688 factory default for my test chip

    if (devStatus == 0) {
        mpu.CalibrateAccel(6);
        mpu.CalibrateGyro(6);
        mpu.PrintActiveOffsets();

        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
        Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
        Serial.println(F(")..."));
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;

        packetSize = mpu.dmpGetFIFOPacketSize();
    } else {
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }

    pinMode(LED_PIN, OUTPUT);
}

// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================
// Function to adjust motors based on PID output
void adjustMotors(float adjustment) {
  int baseSpeed = 200; // Base speed for forward movement
  int leftSpeed = baseSpeed - adjustment;
  int rightSpeed = baseSpeed + adjustment;

  // Ensure speed values are within the valid range (0-255)
  leftSpeed = constrain(leftSpeed, 120, 255);
  rightSpeed = constrain(rightSpeed, 120, 255);

  analogWrite(ENA, rightSpeed); // Right motor
  analogWrite(ENB, leftSpeed);   // Left motor

  Serial.print("Left Speed: ");
  Serial.print(leftSpeed);
  Serial.print(" | Right Speed: ");
  Serial.println(rightSpeed);
}


// Function to stop all motor movements
void Stop_Motors() {
  analogWrite(ENA, 0); // Stop right motor
  analogWrite(ENB, 0); // Stop left motor
  Serial.println("Motors stopped.");
}


void Move_Forward( ) {
    // Move forward with the calculated adjustment
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH); // Right wheel forward
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH); // Left wheel forward
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
  

  
  // Update the setPoint with the current yaw angle
  setPoint = ypr[0] * 180 / M_PI;  // Update setPoint to the new yaw after the turn
  Serial.println("Updated setPoint after left turn: " + String(setPoint));
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
  

  
  // Update the setPoint with the current yaw angle
  setPoint = ypr[0] * 180 / M_PI;  // Update setPoint to the new yaw after the turn
  Serial.println("Updated setPoint after right turn: " + String(setPoint));
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
  

  
  // Update the setPoint with the current yaw angle
  setPoint = ypr[0] * 180 / M_PI;  // Update setPoint to the new yaw after the rotation
  Serial.println("Updated setPoint after left rotation: " + String(setPoint));
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
  

  
  // Update the setPoint with the current yaw angle
  setPoint = ypr[0] * 180 / M_PI;  // Update setPoint to the new yaw after the rotation
  Serial.println("Updated setPoint after right rotation: " + String(setPoint));
}


// Function to update costants via bluetooth or software serial(Ps i didn't know how to write the function while typing this. I will think about it tho. hmmm....this comment is awfully long:())
void Update_Constantz() {
  // Stop the bot first
  Stop_Motors();
  Serial.println("Bot stopped. Waiting for new PID constants...");

  String input = "";
  bool receivedConstants = false;

  // Wait for the new constants in the format (Kp,Ki,Kd)
  while (!receivedConstants) {
    if (BTSerial.available()) {  // Change to 'Serial.available()' if using USB serial
      char received = BTSerial.read();  // Change to 'Serial.read()' if using USB serial
      if (received == '\n') {
        // Process input when new line is detected
        input.trim();  // Remove any extra whitespace
        if (input.startsWith("(") && input.endsWith(")")) {
          // Remove the parentheses
          input = input.substring(1, input.length() - 1);

          // Split the values by commas
          int comma1 = input.indexOf(',');
          int comma2 = input.lastIndexOf(',');

          if (comma1 != -1 && comma2 != -1 && comma1 != comma2) {
            // Extract Kp, Ki, and Kd values as substrings
            String kpString = input.substring(0, comma1);
            String kiString = input.substring(comma1 + 1, comma2);
            String kdString = input.substring(comma2 + 1);

            // Convert the strings to floats
            float newKp = kpString.toFloat();
            float newKi = kiString.toFloat();
            float newKd = kdString.toFloat();

            // Update the PID constants
            Kp = newKp;
            Ki = newKi;
            Kd = newKd;

            Serial.println("New PID constants received:");
            Serial.print("Kp = ");
            Serial.println(Kp);
            Serial.print("Ki = ");
            Serial.println(Ki);
            Serial.print("Kd = ");
            Serial.println(Kd);

            // Exit the loop after constants are updated
            receivedConstants = true;
          } else {
            Serial.println("Error: Invalid input format. Please enter (Kp,Ki,Kd).");
          }
        } else {
          Serial.println("Error: Input must be in the format (Kp,Ki,Kd).");
        }

        input = "";  // Reset the input string after processing
      } else {
        input += received;  // Accumulate the input
      }
    }
  }

  // Resume the normal loop after receiving the constants
  Serial.println("Resuming normal operation...");
}





void loop() {
    if (!dmpReady) return;

    if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet 
        #ifdef OUTPUT_READABLE_YAWPITCHROLL
            mpu.dmpGetQuaternion(&q, fifoBuffer);
            mpu.dmpGetGravity(&gravity, &q);
            mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);

    // The MPU6050 part 
    input = ypr[0] * 180 / M_PI; // Convert to degrees
    Serial.println("The Input is: " + String(input));
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
        case 'p': Update_Constantz();break;

            // Prepare data string for both Serial and Bluetooth output
            String data = String("ypr,") + 
                          String(ypr[0] * 180/M_PI) + "," +
                          String(ypr[1] * 180/M_PI) + "," +
                          String(ypr[2] * 180/M_PI);

            // Send data over Bluetooth
            BTSerial.println(data);  
            delay(10); // Ensure HC-05 has time to process and send the data
            
            // Optional: Print to USB Serial for debugging
            Serial.print("ypr\t");
            Serial.print(ypr[0] * 180/M_PI);
            Serial.print("\t");
            Serial.print(ypr[1] * 180/M_PI);
            Serial.print("\t");
            Serial.println(ypr[2] * 180/M_PI);
        #endif

        // Toggle LED to indicate activity
        blinkState = !blinkState;
        digitalWrite(LED_PIN, blinkState);
    }
}
}
}
