#include "Wire.h" // This library allows you to communicate with I2C devices.
#include <stdio.h>
#include <stdlib.h>

/********* Connections **********/
// Motor connections
#define   IN1   3
#define   IN2   9
#define   IN3   10
#define   IN4   11
#define   ENA   5
#define   ENB   6
/******** Variables ********/
// Direction variables
#define   F     1
#define   FR    2
#define   R     3
#define   BR    4
#define   B     5
#define   BL    6
#define   L     7
#define   FL    8
#define   STOP  0
#define   SPEED_CONSTANT  1

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ; //gyro raw data, gyro roll angle
float rotX, rotY, rotZ, gyroRotX, timer; //angular velocities, timer vars

float rotXOffset = -7.82;
float rotYOffset = -3.21;
float rotZOffset = 3.04;

float accAngle, currentAngle=-88.5, prevAngle=-88.5, error, error_prev=0, error_sum=0, error_div=0;
float stuuractie;
float setpoint = -88.5;
float sensitivity = 500.0 / 32767.0;

int16_t count = 0;

class waggie{
private:
  float Kp = 6.2;
  float Ki = 0.001;
  float Kd = 0.02;
  float dt = 0.02;
  char data;
  float tmp;
  bool go = true;
  float max = 0;

public:

  void setupWaggie() {
    // LED for bluetooth PID
    pinMode(13, OUTPUT);
    digitalWrite(13,LOW);

    delay(250);
    Wire.begin();
    setupMPU();
    delay(250);

    Serial.println("Starting");
  }
  

  void startUp(){
    int s = 0;
    for ( s; s < 80; s++){
        // Left motor forward with full speed
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, s);
      // Right motor forward with full speed
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, s);
      delay(5);
    }
    delay(80);
    s = 0;
    digitalWrite(IN1, LOW);
    digitalWrite(IN2, HIGH);
    analogWrite(ENA, s);
    // Right motor forward with full speed
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, s);
    delay(170);
  
    analogWrite(ENA, 0);
    analogWrite(ENB, 0);
  
  }


  void zend(float waarde){
    ///zend de stuurwaarde naar de motorcontrol
    if(waarde > 1){
      motor(B, waarde + 25);
    }
    if(waarde < -1){
      float waardePos = -waarde;
      motor(F, waardePos + 25);
    }
  }

  void pidLoop(){
    if (Serial.available() > 0) {
      data = Serial.read();
      if ( data == 's' ){
          digitalWrite(13,HIGH);
          Serial.println("P waarde /1000:");
          setValue(Kp);
          Serial.println("I waarde /1000:");
          setValue(Ki);
          Serial.println("D waarde /1000:");
          setValue(Kd);
          Serial.println("Values:");
          Serial.println(Kp);
          Serial.println(Ki);
          Serial.println(Kd);
          go = true;
        }
      
      if (data == 't'){
        digitalWrite(13,LOW);
        go = false;
        motor(STOP, 0);
      }      
    }

    if ( go ){
      recordAccelRegisters();  //check accelerometer readings
      processAccelData(); //process the data

      double elapsedTimeInSeconds = ((double)(micros()-timer)/1000000);
      timer = micros();

      recordGyroRegisters();
      processGyroData();
      accAngle = atan2(accelY, accelZ)*RAD_TO_DEG;
      gyroRotX = ((rotX - rotXOffset) * elapsedTimeInSeconds); // kp   angle of roll adding with an angular velocity and its corresponding time
      currentAngle = 0.96 * (prevAngle + gyroRotX) + 0.04*(accAngle);

      // Serial.print("currentAngle:");
      // Serial.print(currentAngle);
      // Serial.print(",");
      // Serial.print("GyroRotX:");
      // Serial.print(gyroRotX);
      // Serial.print(",");
      // Serial.print("AccAngle:");
      // Serial.println(accAngle);

      
      error = currentAngle - setpoint;
      error_sum = error_sum + error * elapsedTimeInSeconds;
      error_div = (error - error_prev) / elapsedTimeInSeconds;
      stuuractie = Kp * error + Ki * error_sum + Kd * error_div;
      zend(stuuractie);
      error_prev = error;
      prevAngle = currentAngle;
      delay(25);
      // count++;
      // if(count == 50)  {
      //   count = 0;
      //   digitalWrite(13, !digitalRead(13));
      // }
    }
  }


  void blink(int time){
    digitalWrite(13,LOW);
    delay(time);
    digitalWrite(13,HIGH);
    delay(time);
  }


  void setValue(float & value){
    value = -1;
    while (value == -1){
      tmp = Serial.parseFloat();
      if (tmp == 9999){
        value = 0;
        blink(200);
        blink(200);
        break;
      }
      if(tmp != 0){
        value = tmp/10000;
      }
    }
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

  // Motor function ============================================================================================================
  /*
    dir:
    F- Forward
    FR- Forward Right
    R- Right
    BR- Backward Right
    B- Backward
    BL- Backward Left
    L- Left
    FL- Forward Left
    STOP- Stop
    speed:
    min: 0, max: 255
  */
  void motor(int dir, int speed) {
    if (dir == F) {
      // Left motor forward with full speed
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, speed * SPEED_CONSTANT);
      // Right motor forward with full speed
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, speed * SPEED_CONSTANT);
    } else if (dir == FR) {
      // Left motor forward with full speed
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, speed);
      // Right motor forward with speed*SPEED_CONSTANT
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, speed * SPEED_CONSTANT);
    } else if (dir == R) {
      // Left motor forward with full speed
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, speed);
      // Right motor backward with full speed
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, speed);
    } else if (dir == BR) {
      // Left motor backward with full speed
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, speed);
      // Right motor backward with speed*SPEED_CONSTANT
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, speed * SPEED_CONSTANT);
    } else if (dir == B) {
      // Left motor backward with full speed
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, speed * SPEED_CONSTANT);
      // Right motor backward with full speed
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, speed * SPEED_CONSTANT);
    } else if (dir == BL) {
      // Left motor backward with speed*SPEED_CONSTANT
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, speed * SPEED_CONSTANT);
      // Right motor backward with full speed
      digitalWrite(IN3, LOW);
      digitalWrite(IN4, HIGH);
      analogWrite(ENB, speed);
    } else if (dir == L) {
      // Left motor forward with full speed
      digitalWrite(IN1, LOW);
      digitalWrite(IN2, HIGH);
      analogWrite(ENA, speed);
      // Right motor backward with full speed
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, speed);
    } else if (dir == FL) {
      // Left motor forward with speed*SPEED_CONSTANT
      digitalWrite(IN1, HIGH);
      digitalWrite(IN2, LOW);
      analogWrite(ENA, speed * SPEED_CONSTANT);
      // Right motor forward with full speed
      digitalWrite(IN3, HIGH);
      digitalWrite(IN4, LOW);
      analogWrite(ENB, speed);
    } else if (dir == STOP) {
      // Left motor stop
      analogWrite(ENA, 0);
      // Right motor stop
      analogWrite(ENB, 0);
    }
  }
};


