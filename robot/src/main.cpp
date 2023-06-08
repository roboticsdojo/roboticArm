#include <Arduino.h>

#include "MobilePlatform.h"
#include "Preferences.h"

double Kp = 1.0; // Proportional gain
double Ki = 0.0; // Integral gain
double Kd = 5.0; // Derivative gain

double previous_error = 0.0;
double integral = 0.0;

MobilePlatform platform;

// prototype
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed);
void setMotorState(int motorPin1, int motorPin2, int motorSpeed);

//Encoder stuff
#include <Encoder.h>

// Encoder Pins
#define RIGHT_ENCODER_A_PIN 18
#define RIGHT_ENCODER_B_PIN 19
#define LEFT_ENCODER_A_PIN 2
#define LEFT_ENCODER_B_PIN 3

int counter = 0;

// Motor Parameters
#define PPR 360.0              // Pulses per revolution for the encoder
#define wheelDiameter 0.1      // Diameter of the wheel in meters (adjust this to match your setup)
#define wheelCircumference wheelDiameter * PI

// Encoder
Encoder rightEnc(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN);
Encoder leftEnc(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN);
long oldPositionRight  = -999;
long oldPositionLeft  = -999;

double distanceRight = 0;
double distanceLeft = 0;


// Function Prototypes
void getDistanceRight();
void getDistanceLeft();

void setup() {
  platform.setup();
  Serial.begin(9600);
  
  delay(5000);
}
void loop() {
  // platform.loop();
  double sensorValues[] = {
    analogRead(LEFT_SENSOR),
    analogRead(MIDDLE_SENSOR),
    analogRead(RIGHT_SENSOR)
  };

  double position = sensorValues[0] - sensorValues[2];
  // Calculate the proportional term
  double P = Kp * position;
  integral += position;
  double I = Ki * integral;

  double D = Kd * (position - previous_error);

  double steering = P + I + D;

  
  rotateMotor(MOTOR_SPEED + steering, MOTOR_SPEED - steering);
  previous_error = position;

  // if (analogRead(FAR_RIGHT) > 700 || analogRead(FAR_LEFT) > 700) {
  //   rotateMotor(0, 0);
  //   while (true) {
  //     delay(1000);
  //   }
  // }

  


  // Serial.println("Left: " +String(analogRead(LEFT_SENSOR)));
  // Serial.println("Middle: " +String(analogRead(MIDDLE_SENSOR)));
  // Serial.println("Right: " +String(analogRead(RIGHT_SENSOR)));
  
  // // Serial.print("D_Left: " + String(distanceLeft));
//   // // Serial.println(" D_Right: " + String(distanceRight));
//   // // // Serial.println("**********************");

  // delay(10000);
}


void rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  if (rightMotorSpeed > 255) rightMotorSpeed = 255;
  if (rightMotorSpeed < -255) rightMotorSpeed = -255;
  if (leftMotorSpeed > 255) leftMotorSpeed = 255;
  if (leftMotorSpeed < -255) leftMotorSpeed = -255;
  if (rightMotorSpeed > -200 && rightMotorSpeed <= 0) rightMotorSpeed = -220;
  if (rightMotorSpeed < 200 && rightMotorSpeed >= 0) rightMotorSpeed = 220;
  if (leftMotorSpeed > -200 && leftMotorSpeed <= 0) leftMotorSpeed = -220;
  if (leftMotorSpeed < 200 && leftMotorSpeed >= 0) leftMotorSpeed = 220;
  
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

void getDistanceRight() {
  long newPosition = rightEnc.read();
  
  if (newPosition != oldPositionRight) {
    oldPositionRight = newPosition;
    
    double wheelRotations = newPosition / PPR;
    double distanceTravelled = wheelRotations * wheelCircumference;
    
    // Serial.print("Distance Travelled Right (m): ");
    // Serial.println(distanceTravelled, 3);
    distanceRight = distanceTravelled;
  }
}

void getDistanceLeft() {
  long newPosition = leftEnc.read();
  
  if (newPosition != oldPositionLeft) {
    oldPositionLeft = newPosition;
    
    double wheelRotations = newPosition / PPR;
    double distanceTravelled = wheelRotations * wheelCircumference;
    
    // Serial.print("Distance Travelled Left (m): ");
    // Serial.println(distanceTravelled, 3);
    distanceLeft = distanceTravelled;
  }
}