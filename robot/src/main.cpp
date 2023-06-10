#include <Arduino.h>

#include <NewPing.h>

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

unsigned long currentTime = 0;
unsigned long previousTime = 0;

unsigned long previousTimeRight = 0;
unsigned long currentTimeRight = 0;

unsigned long previousTimeLeft = 0;
unsigned long currentTimeLeft = 0;

unsigned current_time_distance = 0;
unsigned previous_time_distance = 0;


// Function Prototypes
void getDistanceRight();
void getDistanceLeft();
void stop();
void turn_right();
void turnRight(bool intercecrions);
void PIDLoop ();
int getDistance();

// Ultrasonic Sensor Stuff
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters). Maximum sensor distance is rated at 400-500cm.

NewPing sonar(TRIGPIN, ECHOPIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

unsigned int pingSpeed = 100; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.
volatile int distance = 400;
int distance_counter = 0;

// Prototypes
void echoCheck();



void setup() {
  platform.setup();
  Serial.begin(9600);

  pingTimer = millis(); // Start now.
  Serial.println("Starting...");
// pinMode(TRIGPIN, OUTPUT);
  // pinMode(ECHOPIN, INPUT);  
  while (true) {
    getDistanceLeft();
    getDistanceLeft();
    Serial.print("Left: " + String(distanceLeft));
    Serial.println(" Right: " + String(distanceRight));
  }
  
  
  delay(2000);
}
int count = 0;
int i = 0;
void loop() {
  // platform.loop();
 
  while(true) {
    int sensorValues[] = {
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

    // Serial.println(distance);
    if (distance <= 30 ) {
      distance_counter ++;
      if (distance_counter < 5)
        break;
      while (true)
        stop();
    }

    
    if (analogRead(FAR_RIGHT) > 750)
      break;

     if (millis() >= pingTimer) // pingSpeed milliseconds since last ping, do another ping.
      break;

    }
    // if (getDistance() == 30){
    //     stop();
    //     delay(5000);
    //   // spin 180deg
    //     rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
    //     delay(20000);

    //     // for (int i = 0; i < 10000; i++) {
    //     //   PIDLoop();
    //     // }
    //     // while (true){
    //     // // rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
    //     // stop();
    //     // // delay(3000);
    //     // }

        
    // }
    // if ((FAR_LEFT) >600 && (FAR_RIGHT) > 600){
    //   rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
    // }

    
    
    // previous_time_distance = current_time_distance;
    // current_time_distance = millis();

    // if ((current_time_distance - previous_time_distance) > 5000 && i == 0) {
    //   i++;
    //   if (getDistance() <= 30 && getDistance() != 13){
    //     while(true){
    //       stop();
    //     }
    //   }
    // }

    // if ((current_time_distance - previous_time_distance) > 1000 && i > 0) {
    //   i++;
    //   if (getDistance() == 30){
    //     while(true){
    //       stop();
    //     }
    //   }
    // }
    
  // }
  
  // previousTime = currentTime;
  // currentTime = millis();
  // if (currentTime - previousTime > 2000)
  //   count ++;

  pingTimer += pingSpeed;      // Set the next ping time.
  sonar.ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
  
  
  // Serial.println("count: " + String(count));

  // if (count == 1) {
  //     turn_right();
  // }
  // else if (count == 4) {
  //     turnRight(true);
  // }
  // else {
  //   PIDLoop();
  //   delay(100);
  // }
  

  // if (analogRead(FAR_LEFT) > 700 || analogRead(FAR_RIGHT) > 700) {
  //   stop();
  //   Serial.println("Here");
  //   while (true) {
  //     delay(1000);
  //     Serial.print("Far Left: " + String(analogRead(FAR_LEFT)));
  //     Serial.println("  Far Right: " + String(analogRead(FAR_RIGHT)));
  //   }
  // }


  // Serial.println("Left: " +String(analogRead(LEFT_SENSOR)));
  // Serial.println("Middle: " +String(analogRead(MIDDLE_SENSOR)));
  // Serial.println("Right: " +String(analogRead(RIGHT_SENSOR)));
  // Serial.print("Far Left: " + String(analogRead(FAR_LEFT)));
  // Serial.println("  Far Right: " + String(analogRead(FAR_RIGHT)));
  
  // // Serial.print("D_Left: " + String(distanceLeft));
  // // Serial.println(" D_Right: " + String(distanceRight));
  // // // Serial.println("**********************");

  // delay(10000);
}

void PIDLoop () {
  int sensorValues[] = {
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
}

void turn_right() {
  // rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
  // delay(4000);

  // while (analogRead(FAR_RIGHT) > 700) {
  //   PIDLoop();
  // }
  
  while(true) {
    rotateMotor(MOTOR_SPEED, -(MOTOR_SPEED - 20));
    if (analogRead(FAR_LEFT) > 720) {
      // rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
      break;
    }
  }

  while (analogRead(FAR_LEFT) > 600 || analogRead(FAR_RIGHT) > 600)
    rotateMotor(MOTOR_SPEED, -(MOTOR_SPEED - 20));

  // just stop
  // while(true)
  //   stop();

  // while(analogRead((FAR_LEFT)) > 600) {
  //   rotateMotor(MOTOR_SPEED+20, 200);
  //   if (analogRead(FAR_LEFT) < 500)
  //     break;
  // }
  // previousTimeRight = millis();

  // for (int i= 0; i < 10; i++)
  //   stop();

  // while (millis() - previousTimeRight < 1000) {
  //   // if (analogRead(FAR_LEFT) > 600 || analogRead(FAR_RIGHT) > 600)
  //   //   break;
  //   PIDLoop();
  // }

  // while (millis() - previousTimeRight < 4000) {
  //   if (analogRead(FAR_LEFT) > 600 || analogRead(FAR_RIGHT) > 600)
  //     break;
  //   PIDLoop();
  // }

  // while (true) {
  //   stop();
  // }

  // while(analogRead(RIGHT_SENSOR) > 700) {
  //   rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
  // }
  // rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
  // delay(4000);

  // while (true) {
  //   stop();
  //   delay(100);
  // }

  // for (int i = 0; i < 100; i++) {
  //   stop();
  //   delay(10);
  // }
  // while(true) {
  //   Serial.println(analogRead(RIGHT_SENSOR));
  //   stop();
  // }
}


void turnRight(bool intercecrion) {
  while(analogRead(FAR_LEFT) > 700) {
    rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
  }
  while(true) {
    rotateMotor(MOTOR_SPEED, -MOTOR_SPEED-80);
    if (analogRead(FAR_LEFT) > 700) {
      break;
    }
  }
  while(analogRead((RIGHT_SENSOR)) < 700) {
    rotateMotor(MOTOR_SPEED+20, MOTOR_SPEED+20);
  }

  // while(analogRead(RIGHT_SENSOR) > 700) {
  //   rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
  // }
  rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
  delay(4000);

  // for (int i = 0; i < 100; i++) {
  //   stop();
  //   delay(10);
  // }
  // while(true) {
  //   Serial.println(analogRead(RIGHT_SENSOR));
  //   stop();
  // }
}
void stop() {
  setMotorState(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, 0);
  setMotorState(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, 0);

  analogWrite(ENABLE_RIGHT_MOTOR, 0);
  analogWrite(ENABLE_LEFT_MOTOR,0);
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

int getDistance() {
  // Clears the trigPin
  digitalWrite(TRIGPIN, LOW);
  delayMicroseconds(2);
  // Sets the trigPin on HIGH state for 10 micro seconds
  digitalWrite(TRIGPIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGPIN, LOW);
  // Reads the echoPin, returns the sound wave travel time in microseconds
  int duration = pulseIn(ECHOPIN, HIGH);
  // Calculating the distance
  int distance = duration * 0.034 / 2;
  
  return distance;
}


void echoCheck() { // Timer2 interrupt calls this function every 24uS where you can check the ping status.
  // Don't do anything here!
  if (sonar.check_timer()) { // This is how you check to see if the ping was received.
    // // Here's where you can add code.
    // Serial.print("Ping: ");
    // Serial.print(sonar.ping_result / US_ROUNDTRIP_CM); // Ping returned, uS result in ping_result, convert to cm with US_ROUNDTRIP_CM.
    // Serial.println("cm");
    distance = sonar.ping_result / US_ROUNDTRIP_CM;
  }
  // Don't do anything here!
}
