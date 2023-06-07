#include "Wire.h" // This library allows you to communicate with I2C devices.
#include <stdio.h>
#include <stdlib.h>

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ; //gyro raw data, gyro roll angle
float rotX, rotY, rotZ, gyroRotX, timer; //angular velocities, timer vars

float rotXOffset, rotYOffset, rotZOffset;
float sensitivity = 500.0 / 32767.0;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  delay(250);
  Wire.begin();
  setupMPU();
  delay(250);

  Serial.println("Start calculating offsets");

  for( int i = 0; i < 10000; i++){
    recordGyroRegisters();
    processGyroData();
    rotXOffset += rotX;
    rotYOffset += rotY;
    rotZOffset += rotZ;
    delay(3);
  }

  rotXOffset /= 10000;
  rotYOffset /= 10000;
  rotZOffset /= 10000;

  Serial.println("Finished offsets");

  Serial.println("RotX");
  Serial.println(rotXOffset);
  Serial.println("RotY");
  Serial.println(rotYOffset);
  Serial.println("RotZ");
  Serial.println(rotZOffset);

}

void loop() {
  // put your main code here, to run repeatedly:

}

void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000001); //Setting the gyro to full scale +/- 500deg./s 
  Wire.endTransmission(); 
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1C); //Accessing the register 1C - Acccelerometer Configuration (Sec. 4.5) 
  Wire.write(0x00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 
}


void recordAccelRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  accelY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  accelZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
}

void processAccelData(){
  gForceX = accelX / 16384.0;
  gForceY = accelY / 16384.0; 
  gForceZ = accelZ / 16384.0;
}

void recordGyroRegisters() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = Wire.read()<<8|Wire.read(); //Store first two bytes into accelX
  gyroY = Wire.read()<<8|Wire.read(); //Store middle two bytes into accelY
  gyroZ = Wire.read()<<8|Wire.read(); //Store last two bytes into accelZ
}

void processGyroData() {
  
  rotX = gyroX * sensitivity;
  rotY = gyroY * sensitivity; 
  rotZ = gyroZ * sensitivity;
}
