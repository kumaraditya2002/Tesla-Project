#include <PID_v1.h>
#include <Wire.h>

long accelX, accelY, accelZ;
float accAngleY=0;

long gyroX, gyroY, gyroZ;
float gyroAngleY=0;

float prevTime=0,currTime=0,elapsedTime=0;

float pitch;
double input,output;
double setpoint = 178;
double Kp;
double Kd;
double Ki;

PID pid(&input,&output,&setpoint,Kp,Ki,Kd,DIRECT);

void setup() {
  Serial.begin(9600);
  Wire.begin();
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
  Wire.write(0b00000000); //Setting the accel to +/- 2g
  Wire.endTransmission(); 

  //pid setup
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255,255);

  //motor pin setup
  pinMode(6,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(11,OUTPUT);

  analogWrite(6,0);
  analogWrite(9,0);
  analogWrite(10,0);
  analogWrite(11,0);
}

void loop() {
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x3B); //Starting register for Accel Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Accel Registers (3B - 40)
  while(Wire.available() < 6);
  accelX = (Wire.read()<<8)|Wire.read()/ 16384; //Store first two bytes into accelX
  accelY = (Wire.read()<<8)|Wire.read()/ 16384; //Store middle two bytes into accelY
  accelZ = (Wire.read()<<8)|Wire.read()/ 16384; //Store last two bytes into accelZ
  accAngleY = atan(-1 * accelX / sqrt(pow(accelY,2) + pow(accelZ,2))) * 180/PI;
  
  prevTime = currTime;
  currTime = millis();
  elapsedTime = (currTime - prevTime) / 1000;
  Wire.beginTransmission(0b1101000); //I2C address of the MPU
  Wire.write(0x43); //Starting register for Gyro Readings
  Wire.endTransmission();
  Wire.requestFrom(0b1101000,6); //Request Gyro Registers (43 - 48)
  while(Wire.available() < 6);
  gyroX = (Wire.read()<<8)|Wire.read()/ 131; //Store first two bytes into accelX
  gyroY = (Wire.read()<<8)|Wire.read()/ 131; //Store middle two bytes into accelY
  gyroZ = (Wire.read()<<8)|Wire.read()/ 131; //Store last two bytes into accelZ
  gyroAngleY = gyroAngleY + gyroY * elapsedTime;

  //complemetry filter
  pitch = 0.96*gyroAngleY + 0.04*accAngleY;
  input = pitch + 180;

  pid.compute();
  if(input>150 && input<200) //bot is falling
  {
    if(output>0) //falling towards front
      Forward(); //move forward
    else if(output<0) //falling towards back
      Reverse(); //move back
  }
  else
    Stop(); // stop
  delay(20);
}
void Forward() //Code to rotate the wheel forward 
{
    analogWrite(6,output);
    analogWrite(9,0);
    analogWrite(10,output);
    analogWrite(11,0);
    Serial.print("F"); //Debugging information 
}
void Reverse() //Code to rotate the wheel Backward  
{
    analogWrite(6,0);
    analogWrite(9,output*-1);
    analogWrite(10,0);
    analogWrite(11,output*-1); 
    Serial.print("R");
}
void Stop() //Code to stop both the wheels
{
    analogWrite(6,0);
    analogWrite(9,0);
    analogWrite(10,0);
    analogWrite(11,0); 
    Serial.print("S");
}
