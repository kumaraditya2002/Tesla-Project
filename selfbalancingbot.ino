#include <PID_v1.h>
#include <math.h>


const int x_out = A1;
const int y_out = A2; 
const int z_out = A3; 


float pitch;
double input,output;
double setpoint = 94;
double Kp=1;
double Kd=2;
double Ki=3;

PID pid(&input,&output,&setpoint,Kp,Ki,Kd,DIRECT);

void setup() {
  Serial.begin(9600); 
  //pid setup
  pid.SetMode(AUTOMATIC);
  pid.SetSampleTime(10);
  pid.SetOutputLimits(-255,255);
}

void loop() {
  int x_adc_value, y_adc_value, z_adc_value; 
  double x_g_value, y_g_value, z_g_value;
  double roll, pitch, yaw;
  x_adc_value = analogRead(x_out); /* Digital value of voltage on x_out pin */ 
  y_adc_value = analogRead(y_out); /* Digital value of voltage on y_out pin */ 
  z_adc_value = analogRead(z_out); /* Digital value of voltage on z_out pin */ 
  
  
  x_g_value = ( ( ( (double)(x_adc_value * 5)/1024) - 1.65 ) / 0.330 ); /* Acceleration in x-direction in g units */ 
  y_g_value = ( ( ( (double)(y_adc_value * 5)/1024) - 1.65 ) / 0.330 ); /* Acceleration in y-direction in g units */ 
  z_g_value = ( ( ( (double)(z_adc_value * 5)/1024) - 1.80 ) / 0.330 ); /* Acceleration in z-direction in g units */ 

  roll = ( ( (atan2(y_g_value,z_g_value) * 180) / 3.14 ) + 180 ); /* Formula for roll */
  pitch = ( ( (atan2(z_g_value,x_g_value) * 180) / 3.14 ) + 180 ); /* Formula for pitch */
 

  Serial.print("Roll = ");
  Serial.print(roll);
  Serial.print("\t");
  Serial.print("Pitch = ");
  Serial.print(pitch);
  input=pitch;
  pid.Compute();
  Serial.print("Output = ");
  Serial.print(output);
  Serial.print("\n\n");
  
  
//  Serial.print(output);
//  if(input<90 && input>0) //bot is falling
//  {
//    if(output>0) //falling towards front
//      Forward(); //move forward
//    else if(output<0) //falling towards back
//      Reverse(); //move back
//  }
//  else
//    Stop(); // stop
  delay(1000);
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
