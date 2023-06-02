#include <Arduino.h>
#include <motor_encoder.h>

// Define pins connected the encoders A & B
#define RIGHT_ENCA 18 // Yellow
#define RIGHT_ENCB 19 // Green
#define LEFT_ENCA 2   // Yellow
#define LEFT_ENCB 3   // Green

// Motor Encoder Pulses per Rotation
#define ENCODERPPR 66

// Motor A connections
int enA = 4;
int in1 = 8;
int in2 = 9;

// Motor B connections
int enB = 5;
int in3 = 10;
int in4 = 11;

// More motor variables
int motorSpeed = 220;
int leftEncoderPosition = 0;
int rightEncoderPosition = 0;

//-------- Distance-related variables ---------
long current_millis = 0;
long previous_millis = 0;

// PID-related variables
long prevT = 0;
float eprev = 0;
float eintegral = 0;

// Function prototypes
void readLeftEncoder();
void readRightEncoder();

void setup()
{

  // Initialize the serial port
  Serial.begin(9600);

  // Initialize motor pins
  pinMode(LEFT_ENCA, INPUT);
  pinMode(LEFT_ENCB, INPUT);
  pinMode(RIGHT_ENCA, INPUT);
  pinMode(RIGHT_ENCB, INPUT);

  pinMode(enA, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  attachInterrupt(digitalPinToInterrupt(LEFT_ENCA), readLeftEncoder, RISING);
  attachInterrupt(digitalPinToInterrupt(RIGHT_ENCA), readRightEncoder, RISING);
}

void loop()
{

  setMotor(1, enA, motorSpeed, in1, in2);
  setMotor(1, enB, motorSpeed, in3, in4);

  PIDLoop(leftEncoderPosition, rightEncoderPosition, current_millis, previous_millis, prevT, eprev, eintegral, enA, enB, in1, in2, in3, in4);
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
