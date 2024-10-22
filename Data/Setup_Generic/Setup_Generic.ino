// Define the Variables for Motor Control
int ENA = 4;   // Enable pin for right motor (PWM pin)
int IN1 = 5;   // Input pin for right motor (control pin 1)
int IN2 = 6;   // Input pin for right motor (control pin 2)
int IN3 = 7;   // Input pin for left motor (control pin 1)
int IN4 = 8;   // Input pin for left motor (control pin 2)
int ENB = 9;   // Enable pin for left motor (PWM pin)

/* 
  Motor Wiring:
  Right Motor:
    IN1 = black_right_wheel
    IN2 = red_right_wheel
  Left Motor:
    IN3 = red_left_wheel
    IN4 = black_left_wheel

  Function Descriptions:
  ================================
  Move Forward: 
    Right and left wheels both move forward.
    AnalogWrite(IN1, LOW); // Stop the right motor
    AnalogWrite(IN2, HIGH); // Start right motor forward
    AnalogWrite(IN3, LOW);  // Stop the left motor
    AnalogWrite(IN4, HIGH); // Start left motor forward
  ================================
  Turn Left:
    Right wheel moves forward faster while the left wheel moves slower or is stationary.
  ================================
  Turn Right:
    Left wheel moves forward faster while the right wheel moves slower or is stationary.
  ================================
  Rotate Left:
    Right wheel moves forward, left wheel moves backward for in-place left rotation.
  ================================
  Rotate Right:
    Left wheel moves forward, right wheel moves backward for in-place right rotation.
  ================================
*/

void setup() {
  // Set motor control pins as outputs
  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);
  // Initialize motor speeds to zero (stop)
  analogWrite(ENA, 0); // Right motor speed = 0
  analogWrite(ENB, 0); // Left motor speed = 0
}

void loop() {
  // Placeholder loop. Functions are called individually to move the bot.
  Move_Forward();


}

// Function to move the bot forward
void Move_Forward() {
  // Move both wheels forward
  analogWrite(ENA, 255); // Right motor at full speed
  analogWrite(ENB, 255); // Left motor at full speed
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);  // Right wheel forward
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH);  // Left wheel forward
}

// Function to turn the bot left
void Turn_Left() {
  // Slow left wheel, full speed right wheel
  analogWrite(ENA, 255); // Right motor at full speed
  analogWrite(ENB, 100); // Left motor at lower speed
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);  // Right wheel forward
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH);  // Left wheel forward
}

// Function to turn the bot right
void Turn_Right() {
  // Slow right wheel, full speed left wheel
  analogWrite(ENA, 100); // Right motor at lower speed
  analogWrite(ENB, 255); // Left motor at full speed
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);  // Right wheel forward
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH);  // Left wheel forward
}

// Function to move the bot backward
void Move_Backward() {
  // Move both wheels backward
  analogWrite(ENA, 255); // Right motor at full speed
  analogWrite(ENB, 255); // Left motor at full speed
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);  // Right wheel backward
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);  // Left wheel backward
}

// Function to rotate the bot left (pivot in place)
void Rotate_Left() {
  // Right wheel forward, left wheel backward
  analogWrite(ENA, 255); // Right motor at full speed
  analogWrite(ENB, 255); // Left motor at full speed
  digitalWrite(IN1, LOW); 
  digitalWrite(IN2, HIGH);  // Right wheel forward
  digitalWrite(IN3, HIGH); 
  digitalWrite(IN4, LOW);   // Left wheel backward
}

// Function to rotate the bot right (pivot in place)
void Rotate_Right() {
  // Left wheel forward, right wheel backward
  analogWrite(ENA, 255); // Right motor at full speed
  analogWrite(ENB, 255); // Left motor at full speed
  digitalWrite(IN1, HIGH); 
  digitalWrite(IN2, LOW);   // Right wheel backward
  digitalWrite(IN3, LOW); 
  digitalWrite(IN4, HIGH);  // Left wheel forward
}
