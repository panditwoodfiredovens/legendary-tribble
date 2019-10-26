#include <QTRSensors.h>

#define MaxSpeed 255// max speed of the robot
#define BaseSpeed 200

#define NUM_SENSORS             8  
#define NUM_SAMPLES_PER_SENSOR  4 
#define EMITTER_PIN             2  

const float Kp=5;
const float Ki=0;
const float Kd=16;

const int LeftMotorForward = 9;
const int LeftMotorReverse = 3;

const int RightMotorForward = 6;
const int RightMotorReverse = 10;

void callibrate();
void calcPID();
void motorControl();

// sensors 0 through 7 are connected to analog inputs 0 through 7, respectively
QTRSensorsRC qtra((unsigned char[]) {A0, A1, A2, A3, A4, A5 ,11 ,12},  NUM_SENSORS, NUM_SAMPLES_PER_SENSOR, EMITTER_PIN);
unsigned int sensorValues[NUM_SENSORS];

int position = 0 ;
int bgcolor = 1; //use 1 when background is black
int linecentre = 3500; //default is 3500
float error = 0 , P=0, I=0, D=0, PID_value=0, previous_error=0;

void setup()
{
  
  pinMode(LeftMotorForward, OUTPUT);  
  pinMode(RightMotorForward, OUTPUT);  
  pinMode(LeftMotorReverse, OUTPUT);  
  pinMode(RightMotorReverse, OUTPUT);
  pinMode(4,INPUT);
  //setSpeeds(-100,100);
  delay(500);
  Serial.begin(9600);
  callibrate();
  setSpeeds(0,0);
 
 
  int buttonState = 0;         // variable for reading the pushbutton status
  //while(buttonState != HIGH){
    //buttonState = digitalRead(4);
  //}   
}


void loop()
{    
  position = qtra.readLine(sensorValues, QTR_EMITTERS_ON ,bgcolor )-linecentre;
  error=position/100;
  Serial.println(error);
  //calcPID();
  //motorControl();
}



void callibrate(){
  ////Serial.begin(9600);
  Serial.println("Callibrating...");
     // turn on Arduino's LED to indicate we are in calibration mode
     digitalWrite(13, HIGH); 
  for (int i = 0; i < 400; i++)  // make the calibration take about 10 seconds
  {
    qtra.calibrate();       // reads all sensors 10 times at 2.5 ms per six sensors (i.e. ~25 ms per call)
  }
  digitalWrite(13, LOW);     // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMinimumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  
  // print the calibration maximum values measured when emitters were on
  for (int i = 0; i < NUM_SENSORS; i++)
  {
    Serial.print(qtra.calibratedMaximumOn[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
  Serial.println("Callibrated");
  Serial.println();
}
void calcPID(){
  P = error;  
  I = I + error;
  D = error - previous_error;
  
  PID_value = (Kp*P) + (Ki*I) + (Kd*D);
    
  previous_error = error;
  Serial.println(PID_value);
  
} 

void motorControl(){
  int rightMotorSpeed = BaseSpeed + PID_value;
  int leftMotorSpeed = BaseSpeed - PID_value;
  
  if (rightMotorSpeed > MaxSpeed ) rightMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (leftMotorSpeed > MaxSpeed ) leftMotorSpeed = MaxSpeed; // prevent the motor from going beyond max speed
  if (rightMotorSpeed < -MaxSpeed ) rightMotorSpeed = -MaxSpeed; // prevent the motor from going beyond min speed
  if (leftMotorSpeed < -MaxSpeed ) leftMotorSpeed = -MaxSpeed; // prevent the motor from going beyond min speed
 setSpeeds(rightMotorSpeed,leftMotorSpeed);  
}

void stopAll(){
  analogWrite(RightMotorForward, 0);
  analogWrite(LeftMotorForward, 0); 
  analogWrite(RightMotorReverse, 0);  
  analogWrite(LeftMotorReverse, 0);   
}
void setSpeeds(int a, int b){
  stopAll();
  if(a<0){
    analogWrite(RightMotorReverse, a);   
    analogWrite(RightMotorForward, 0);
  }
  else {
    analogWrite(RightMotorForward, a); 
    analogWrite(RightMotorReverse, 0);
   }
  
 
  if(b<0){
    analogWrite(LeftMotorReverse, -b);
    analogWrite(LeftMotorForward, 0);
  }
  else {
    analogWrite(LeftMotorForward, b);
    analogWrite(LeftMotorReverse, 0);}
  
}
