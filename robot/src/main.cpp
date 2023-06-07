#include <Arduino.h>
#include "Preferences.h"
#include "MobilePlatform.h"
#include "motor_encoder.h"

MobilePlatform platform;
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed);
void setMotorState(int motorPin1, int motorPin2, int motorSpeed);

#include <motor_encoder.h>

// Define pins connected the encoders A & B
#define RIGHT_ENCA 18 // Yellow
#define RIGHT_ENCB 19 // Green
#define LEFT_ENCA 2   // Yellow
#define LEFT_ENCB 3   // Green

// Motor Encoder Pulses per Rotation
#define ENCODERPPR 66


// More motor variables
int motorSpeed = 220;
int leftEncoderPosition = 0;
int rightEncoderPosition = 0;

//-------- Distance-related variables ---------
long left_current_millis = 0;
long left_previous_millis = 0;
long right_previous_millis = 0;
long right_current_millis = 0;


// PID-related variables
long prevT = 0;
float eprev = 0;
float eintegral = 0;

// Function prototypes
void readLeftEncoder();
void readRightEncoder();

void setup() {
  platform.setup();
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCA), readLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCA), readRightEncoder, RISING);
  
  delay(10000);
}
void loop() {
  // platform.loop();
  rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
  int leftDistance = getDistance(leftEncoderPosition, left_current_millis, left_previous_millis);
  int rightDistance = getDistance(rightEncoderPosition, right_current_millis, right_previous_millis);

  Serial.print("Left: " + String(leftDistance));
  Serial.println(" Right: " + String(rightDistance));
  delay(5000);

  
  // if (analogRead(FAR_RIGHT) > 700 || analogRead(FAR_LEFT) > 800) {
  //   rotateMotor(0, 0);
  //   delay(100);
  //   if (analogRead(FAR_RIGHT) > 800 || analogRead(FAR_LEFT) > 700)
  //   {
  //     rotateMotor(0, 0);
  //     while (true)
  //       delay(1000);
  //   }
  // }
// Serial.println("Left: " + String(analogRead(LEFT_SENSOR)));
// Serial.println("Middle: " + String(analogRead(MIDDLE_SENSOR)));
// Serial.println("Right: " + String(analogRead(RIGHT_SENSOR)));
// // Serial.println("Far Left: " + String(analogRead(FAR_LEFT)));
// // Serial.println("Far Right: " + String(analogRead(FAR_RIGHT)));
// // Serial.println(analogRead(FAR_RIGHT) > 700 || analogRead(FAR_LEFT) > 700);
// Serial.println("***************");

// delay(10000);
}


void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  setMotorState(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, rightMotorSpeed);
  setMotorState(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, leftMotorSpeed);
  
  analogWrite(ENABLE_RIGHT_MOTOR, abs(rightMotorSpeed));
  analogWrite(ENABLE_LEFT_MOTOR, abs(leftMotorSpeed));
}

void setMotorState(int motorPin1, int motorPin2, int motorSpeed) {
  if (motorSpeed < 0) {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, HIGH);
  } else if (motorSpeed > 0) {
    digitalWrite(motorPin1, HIGH);
    digitalWrite(motorPin2, LOW);
  } else {
    digitalWrite(motorPin1, LOW);
    digitalWrite(motorPin2, LOW);
  }
}



void readLeftEncoder()
{
  int b = digitalRead(LEFT_ENCB);

  //? WARNING: left motor is inverted (mirror image of right motor)
  if (b > 0)
  {
    leftEncoderPosition--;
  }
  else
  {
    leftEncoderPosition++;
  }
}

void readRightEncoder()
{
  int b = digitalRead(RIGHT_ENCB);

  if (b > 0)
  {
    rightEncoderPosition++;
  }
  else
  {
    rightEncoderPosition--;
  }
}