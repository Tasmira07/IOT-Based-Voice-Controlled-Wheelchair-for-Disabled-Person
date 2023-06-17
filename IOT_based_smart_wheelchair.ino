#include <Servo.h>          //Servo motor library. This is standard library
#include <NewPing.h>        //Ultrasonic sensor function library. You must install this library
#include <SoftwareSerial.h>
SoftwareSerial bluetooth(0,1);

//our L298N control pins
#define enA 3
#define LeftMotorForward 7//in1 pin
#define LeftMotorBackward 6//in2 pin
#define enB 9
#define RightMotorForward 5//in3 in
#define RightMotorBackward 4//in4 pin
#define sw 8
int motorSpeedA = 0;
int motorSpeedB = 0;

//sensor pins
#define trig_pin A0 //analog input 1
#define echo_pin A1 //analog input 2

#define maximum_distance 200
//boolean goesForward = false;
int distance = 100;

NewPing sonar(trig_pin, echo_pin, maximum_distance); //sensor function
Servo servo_motor; //our servo name
String t;

void setup(){
//Serial.begin(9600);
  bluetooth.begin(9600);
  pinMode(RightMotorForward, OUTPUT);
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(sw,INPUT_PULLUP);// setting pin sw as input
 
  servo_motor.attach(11); //our servo pin

  servo_motor.write(115);
  delay(2000);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
  distance = readPing();
  delay(100);
}

void loop(){
  int distanceRight = 0;
  int distanceLeft = 0;
  delay(50);

if(bluetooth.available()>0)
{
  t = bluetooth.readString();
  Serial.println(t);
}
if(t=="go forward")
  {
      if (distance <= 35){
      moveStop();
      delay(200);
      moveBackward();
      delay(400);
      moveStop();
      delay(300);
      distanceRight = lookRight();
      delay(300);
      distanceLeft = lookLeft();
      delay(300);

      if (distance >= distanceLeft){
        turnRight();
        moveStop();
      }
      else{
        turnLeft();
        moveStop();
      }
    }
    else{
       moveForward();
    }
      distance = readPing();
    }
    else if(t=="go backward"){
      moveBackward();
      delay(500);
      }
    else if(t=="left")
        {
          analogWrite(enA,255);
          analogWrite(enB,255);
          digitalWrite(LeftMotorBackward, HIGH);
          digitalWrite(RightMotorForward, HIGH);
          digitalWrite(LeftMotorForward, LOW);
          digitalWrite(RightMotorBackward, LOW);
          delay(250);
          digitalWrite(LeftMotorBackward, LOW);
          digitalWrite(RightMotorForward, LOW);
        }
        else if(t=="right")
        {
          analogWrite(enA,255);
          analogWrite(enB,255);
          digitalWrite(LeftMotorBackward, LOW);
          digitalWrite(RightMotorForward, LOW);
          digitalWrite(LeftMotorForward, HIGH);
          digitalWrite(RightMotorBackward, HIGH);
          delay(250);
          digitalWrite(LeftMotorForward, LOW);
          digitalWrite(RightMotorBackward, LOW);
        }
        else if(t=="stop"){
          moveStop();
        }

    //Joystick control
    int xAxis = analogRead(A2); // Read Joysticks X-axis
    int yAxis = analogRead(A3); // Read Joysticks Y-axis
    int buttonState = digitalRead(sw);
    // Y-axis used for forward and backward control
  if (yAxis < 470) {
    moveBackward();
    // Convert the declining Y-axis readings for going backward from 470 to 0 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(yAxis, 470, 0, 0, 255);
    motorSpeedB = map(yAxis, 470, 0, 0, 255);
  }
  else if (yAxis > 550) {
   moveForward();
    // Convert the increasing Y-axis readings for going forward from 550 to 1023 into 0 to 255 value for the PWM signal for increasing the motor speed
    motorSpeedA = map(yAxis, 550, 1023, 0, 255);
    motorSpeedB = map(yAxis, 550, 1023, 0, 255);
  }
  // If joystick stays in middle the motors are not moving
    else {
      motorSpeedA = 0;
      motorSpeedB = 0;
    }

    // X-axis used for left and right control
  if (xAxis < 470) {
    // Convert the declining X-axis readings from 470 to 0 into increasing 0 to 255 value
    int xMapped = map(xAxis, 470, 0, 0, 255);
    // Move to left - decrease left motor speed, increase right motor speed
    motorSpeedA = motorSpeedA - xMapped;
    motorSpeedB = motorSpeedB + xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedA < 0) {
      motorSpeedA = 0;
    }
    if (motorSpeedB > 255) {
      motorSpeedB = 255;
      }
    }
    if (xAxis > 550) {
    // Convert the increasing X-axis readings from 550 to 1023 into 0 to 255 value
    int xMapped = map(xAxis, 550, 1023, 0, 255);
    // Move right - decrease right motor speed, increase left motor speed
    motorSpeedA = motorSpeedA + xMapped;
    motorSpeedB = motorSpeedB - xMapped;
    // Confine the range from 0 to 255
    if (motorSpeedA > 255) {
      motorSpeedA = 255;
    }
    if (motorSpeedB < 0) {
      motorSpeedB = 0;
      }
    }
     // Prevent buzzing at low speeds (Adjust according to your motors. My motors couldn't start moving if PWM value was below value of 70)
  if (motorSpeedA < 70) {
    motorSpeedA = 0;
  }
  if (motorSpeedB < 70) {
    motorSpeedB = 0;
  }
  analogWrite(enA, motorSpeedA); // Send PWM signal to motor A
  analogWrite(enB, motorSpeedB); // Send PWM signal to motor B
 }

int lookRight(){  
  servo_motor.write(50);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
}

int lookLeft(){
  servo_motor.write(170);
  delay(500);
  int distance = readPing();
  delay(100);
  servo_motor.write(115);
  return distance;
  delay(100);
}

int readPing(){
  delay(70);
  int cm = sonar.ping_cm();
  if (cm==0){
    cm=250;
  }
  return cm;
}

void moveStop(){
  analogWrite(enA,0);
  analogWrite(enB,0);
  digitalWrite(RightMotorForward, LOW);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);
  digitalWrite(LeftMotorBackward, LOW);
}

void moveForward(){

//  if(!goesForward){
//
//    goesForward=true;
    analogWrite(enA,255);
    analogWrite(enB,255);
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);
    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW);
//  }
}

void moveBackward(){

//  goesForward=false;
  analogWrite(enA,255);
  analogWrite(enB,255);
  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorForward, LOW);
}

void turnRight(){
  analogWrite(enA,255);
  analogWrite(enB,255);
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorBackward, HIGH);
 
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorForward, LOW);
 
  delay(250);
 
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
 
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
 
}

void turnLeft(){
    analogWrite(enA,255);
    analogWrite(enB,255);
  digitalWrite(LeftMotorBackward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
 
  digitalWrite(LeftMotorForward, LOW);
  digitalWrite(RightMotorBackward, LOW);

  delay(250);
 
  digitalWrite(LeftMotorForward, HIGH);
  digitalWrite(RightMotorForward, HIGH);
 
  digitalWrite(LeftMotorBackward, LOW);
  digitalWrite(RightMotorBackward, LOW);
}
