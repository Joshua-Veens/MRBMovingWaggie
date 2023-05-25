#include "Wire.h" // This library allows you to communicate with I2C devices.


/********* Connections **********/
// Motor connections
#define   IN1   4
#define   IN2   7
#define   IN3   9
#define   IN4   8
#define   ENA   6
#define   ENB   5
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
#define   SPEED_CONSTANT  0.2

const int MPU_ADDR = 0x68; // I2C address of the MPU-6050. If AD0 pin is set to HIGH, the I2C address will be 0x69.

int16_t accelerometer_x, accelerometer_y, accelerometer_z; // variables for accelerometer raw data
int16_t gyro_x, gyro_y, gyro_z; // variables for gyro raw data
int16_t temperature; // variables for temperature data

char tmp_str[7]; // temporary variable used in convert function

char* convert_int16_to_str(int16_t i) { // converts int16 to string. Moreover, resulting strings will have the same length in the debug monitor.
  sprintf(tmp_str, "%6d", i);
  return tmp_str;
}

long accelX, accelY, accelZ;
float gForceX, gForceY, gForceZ;

long gyroX, gyroY, gyroZ, gyroRotZ; //gyro raw data, gyro roll angle
float rotX, rotY, rotZ, timer; //angular velocities, timer vars

float accAngle, currentAngle, prevAngle=0, error, error_prev=0, error_sum=0, error_div=0;
int stuuractie;
float setpoint = -88;

float Kp = 5;
float Ki = 1;
float Kd = 0.00000;
float dt = 0.01;


void setup() {
  pinMode(10, OUTPUT);
  digitalWrite(10, HIGH);
  delay(250);
  Serial.begin(9600);
  Wire.begin();
  setupMPU();

}


void zend(int waarde){
  ///zend de stuurwaarde naar de motorcontrol
//  Serial.println(waarde);
  
  if(waarde  < 1){
    motor(F, waarde);
  }
  if(waarde > -1){
    motor(B, waarde);
  }
}

void loop() {
  recordAccelRegisters();  //check accelerometer readings
  // processAccelData(); //process the data

  double elapsedTimeInSeconds = ((double)(micros()-timer)/1000000);
  recordGyroRegisters();
  processGyroData();
  accAngle = atan2(accelY, accelZ)*RAD_TO_DEG;
  // gyroRotZ += rotZ * elapsedTimeInSeconds; //angle of roll adding with an angular velocity and its corresponding time
  // currentAngle = 0.9934*(prevAngle + gyroRotZ) + 0.0066*(accAngle);
  
  error = accAngle - setpoint;
  error_sum = error_sum + error * dt;
  error_div = (accAngle - prevAngle) / dt;
  stuuractie = Kp * error + Ki * error_sum + Kd * error_div;
  zend(stuuractie);
  error_prev = error;
  
}


void setupMPU(){
  Wire.beginTransmission(0b1101000); //This is the I2C address of the MPU (b1101000/b1101001 for AC0 low/high datasheet sec. 9.2)
  Wire.write(0x6B); //Accessing the register 6B - Power Management (Sec. 4.28)
  Wire.write(0b00000000); //Setting SLEEP register to 0. (Required; see Note on p. 9)
  Wire.endTransmission();  
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x1B); //Accessing the register 1B - Gyroscope Configuration (Sec. 4.4) 
  Wire.write(0x00000000); //Setting the gyro to full scale +/- 250deg./s 
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
  processAccelData();
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
  processGyroData();
}

void processGyroData() {
  rotX = gyroX / 131.0;
  rotY = gyroY / 131.0; 
  rotZ = gyroZ / 131.0;
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
    analogWrite(ENA, speed);
    // Right motor forward with full speed
    digitalWrite(IN3, HIGH);
    digitalWrite(IN4, LOW);
    analogWrite(ENB, speed);
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
    analogWrite(ENA, speed);
    // Right motor backward with full speed
    digitalWrite(IN3, LOW);
    digitalWrite(IN4, HIGH);
    analogWrite(ENB, speed);
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
