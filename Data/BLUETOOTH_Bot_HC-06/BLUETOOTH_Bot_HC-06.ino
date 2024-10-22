// Define the Variables for Motor Control
int ENA = 4;   // Enable pin for right motor (PWM pin)
int IN1 = 5;   // Input pin for right motor (control pin 1)
int IN2 = 6;   // Input pin for right motor (control pin 2)
int IN3 = 7;   // Input pin for left motor (control pin 1)
int IN4 = 8;   // Input pin for left motor (control pin 2)
int ENB = 9;   // Enable pin for left motor (PWM pin)

// Variables for Bluetooth Communication
char command; // to store incoming Bluetooth command

void setup() {
  // Initialize motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  
  // Initialize motor speeds to zero (stop)
  analogWrite(ENA, 0); // Right motor speed = 0
  analogWrite(ENB, 0); // Left motor speed = 0

  // Initialize Serial communication for HC-05 Bluetooth module
  Serial.begin(9600); // HC-05 default baud rate is 9600
}

void loop() {
  // Check if there is any command received from Bluetooth
  if (Serial.available() > 0) {
    command = Serial.read(); // Read the command sent via Bluetooth
    
    // Control the bot based on the command received
    if (command == 'F') {
      Move_Forward();   // Move Forward
    }
    else if (command == 'B') {
      Move_Backward();  // Move Backward
    }
    else if (command == 'L') {
      Turn_Left();      // Turn Left
    }
    else if (command == 'R') {
      Turn_Right();     // Turn Right
    }
    else if (command == 'Z') {
      Rotate_Left();    // Rotate Left
    }
    else if (command == 'X') {
      Rotate_Right();   // Rotate Right
    }
    else if (command == 'S') {
      Stop_Motors();    // Stop the bot
    }
  }
}

// Function to stop all motor movements
void Stop_Motors() {
  analogWrite(ENA, 0); // Stop right motor
  analogWrite(ENB, 0); // Stop left motor
  Serial.println("Motors stopped.");
}

// Function to move the bot forward
void Move_Forward() {
  analogWrite(ENA, 255); // Right motor at full speed
  analogWrite(ENB, 255); // Left motor at full speed
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);  // Right wheel forward
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH);  // Left wheel forward
  Serial.println("Moving forward.");
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

// Function to move the bot backward
void Move_Backward() {
  analogWrite(ENA, 255); // Right motor at full speed
  analogWrite(ENB, 255); // Left motor at full speed
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);  // Right wheel backward
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);  // Left wheel backward
  Serial.println("Moving backward.");
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
