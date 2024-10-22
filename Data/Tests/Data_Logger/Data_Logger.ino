#include <Wire.h>
#include <MPU6050_light.h>
#include <SoftwareSerial.h>

MPU6050 mpu(Wire);

// Define software serial for Bluetooth communication
SoftwareSerial BTSerial(10, 11); // RX, TX pins for HC-05

long timer = 0;

void setup() {
  Serial.begin(9600);
  BTSerial.begin(38400); // Set baud rate for Bluetooth
  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  mpu.calcOffsets(true,true); // gyro and accelero
  Serial.println("Done!\n");
}

void loop() {
  mpu.update();

  if(millis() - timer > 100){ // print data every second
    // Prepare data string
    String data = String(mpu.getTemp()) + "," +
                  String(mpu.getAccX()) + "," +
                  String(mpu.getAccY()) + "," +
                  String(mpu.getAccZ()) + "," +
                  String(mpu.getGyroX()) + "," +
                  String(mpu.getGyroY()) + "," +
                  String(mpu.getGyroZ()) + "," +
                  String(mpu.getAccAngleX()) + "," +
                  String(mpu.getAccAngleY()) + "," +
                  String(mpu.getAngleX()) + "," +
                  String(mpu.getAngleY()) + "," +
                  String(mpu.getAngleZ());
    
    // Send data over Bluetooth
    BTSerial.println(data);
    
    // Also print to Serial Monitor for debugging
    Serial.println(data);
    
    timer = millis();
  }
}
