#include <Arduino.h>

#include <NewPing.h>

#include "MobilePlatform.h"
#include "Preferences.h"

double Kp = 1.0; // Proportional gain
double Ki = 0.0; // Integral gain
double Kd = 5.0; // Derivative gain

double previous_error = 0.0;
double integral = 0.0;

// Straight Line
double KpS = 5.0; // Proportional gain
double KiS = 0.0; // Integral gain
double KdS = 1.0; // Derivative gain

double previous_error_s = 0.0;
double integral_s = 0.0;

MobilePlatform platform;

// prototype
void rotateMotor(int rightMotorSpeed, int leftMotorSpeed);
void setMotorState(int motorPin1, int motorPin2, int motorSpeed);

//Encoder stuff
#include <Encoder.h>

int counter = 0;

// Motor Parameters
#define PPR 274.63        // Pulses per revolution for the encoder
#define wheelDiameter 65.0     // Diameter of the wheel in mm (adjust this to match your setup)
#define wheelCircumference wheelDiameter * PI

// Encoder
Encoder rightEnc(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN);
Encoder leftEnc(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN);
long oldPositionRight  = -999;
long oldPositionLeft  = -999;

int deviation = 0.5;

int distanceRight = 0;

int distanceLeft = 0;

// unsigned long currentTime = 0;
// unsigned long previousTime = 0;

unsigned long previousTimeRight = 0;
unsigned long currentTimeRight = 0;

unsigned long previousTimeLeft = 0;
unsigned long currentTimeLeft = 0;

unsigned current_time_distance = 0;
unsigned previous_time_distance = 0;


// Function Prototypes
void setDistanceRight();
void setDistanceLeft();
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

void moveEncPIDLoop();

// gyro stuff
#include <Wire.h>

const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ; //linear acceleration
float GyroX, GyroY, GyroZ; //angular velocity
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; //used in void loop()
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

const int maxSpeed = 255; //max PWM value written to motor speed pin. It is typically 255.
const int minSpeed = 160; //min PWM value at which motor moves
float angle; //due to how I orientated my MPU6050 on my car, angle = roll
float targetAngle = 0;
int equilibriumSpeed = 248; //rough estimate of PWM at the speed pin of the stronger motor, while driving straight 
//and weaker motor at maxSpeed
int leftSpeedVal;
int rightSpeedVal;
bool isDriving = false; //it the car driving forward OR rotate/stationary
bool prevIsDriving = true; //equals isDriving in the previous iteration of void loop()
bool paused = false; //is the program paused

// prototypes
void calculateError();
void readAcceleration();
void readGyro();
void rotate();


double KpGyro = 30.0; // Proportional gain
double KiGyro = 0.05; // Integral gain
double KdGyro = 3.0; // Derivative gain

double previous_error_gyro = 0.0;
double integral_gyro = 0.0;

void forwardGyroPIDLoop();
void setGyroReadings();

// VARIABLES
unsigned long PREVIOUS_TIME = 0;
int COUNT = 0;
bool FIRST_MANOUVER_DONE = false;


void setup() {
  platform.setup();
  Serial.begin(9600);

  pingTimer = millis(); // Start now.

  //gyro
  Wire.begin();                      // Initialize comunication
  Wire.beginTransmission(MPU);       // Start communication with MPU6050 // MPU=0x68
  Wire.write(0x6B);                  // Talk to the register 6B
  Wire.write(0x00);                  // Make reset - place a 0 into the 6B register
  Wire.endTransmission(true);        //end the transmission
  // Call this function if you need to get the IMU error values for your module
  calculateError();
  delay(20);

  currentTime = micros();

  /**Step 1: Pick chasis**/ 
  while (true) {
    setDistanceLeft();
    setDistanceRight();
    setGyroReadings();
    if (distanceLeft >= (1300) || distanceRight >= (1300)) {
      break;
    }
    forwardGyroPIDLoop();
  }
  targetAngle = 30; //90 -> about turn
  while (true) {
    setGyroReadings();
    forwardGyroPIDLoop();
    if (abs(angle - targetAngle) < 2) {
      break;
    }
  }

  targetAngle = -115;
  while (true) {
    setGyroReadings();
    forwardGyroPIDLoop();

    if ((analogRead(LEFT_SENSOR) > 500 || analogRead(RIGHT_SENSOR) > 500)&& (millis() - PREVIOUS_TIME) > 3000) {
      PREVIOUS_TIME = millis();
      COUNT ++;
      if (COUNT >= 2){
        break;
      }
    }
  }

  // targetAngle = -90;
  // PREVIOUS_TIME = millis();
  // while (millis() - PREVIOUS_TIME < 3000) {
  //   setGyroReadings();
  //   forwardGyroPIDLoop();
  // }

  // while (true) 
  //   stop();

  PREVIOUS_TIME = millis();
  while (true) {
    // stop();
    setGyroReadings();

    PIDLoop();
    if ((millis() - PREVIOUS_TIME) > 2000)
      break;
  }
  
  //Rotate
  //Move in arc
  //Skip first line
  //PID to wheels
  /**Step 2: Wheels**/
  //pick wheels
  //arc to next line
  /**Step 3: Next**/
  //Pick next
  //Follow line to chasis
  /**Drop**/
  //Ramp
  // delay(2000);
}
int count = 0;
int i = 0;
void loop() {
    setGyroReadings();

  // platform.loop();
 
  while(true) {
    setGyroReadings();

    int sensorValues[] = {
    analogRead(LEFT_SENSOR),
    analogRead(MIDDLE_SENSOR),
    analogRead(RIGHT_SENSOR)
    };

    double position = (sensorValues[0] + 36) - sensorValues[2];
    // Calculate the proportional term
    double P = Kp * position;
    integral += position;
    double I = Ki * integral;

    double D = Kd * (position - previous_error);

    double steering = P + I + D;

    
    rotateMotor(MOTOR_SPEED + steering, MOTOR_SPEED - steering);
    previous_error = position;

    // Serial.println(distance);
    if (distance <= 50 && !FIRST_MANOUVER_DONE) {
      distance_counter ++;
      if (distance_counter < 5)
        break;
      targetAngle = 0;
      PREVIOUS_TIME = millis();
      while (true) {
        // stop();
        setGyroReadings();
        forwardGyroPIDLoop();

        if ((analogRead(LEFT_SENSOR) > 400 || analogRead(RIGHT_SENSOR) > 400) && (millis() - PREVIOUS_TIME) > 1000) {
          break;
        }
      }
      targetAngle = -70;
      while (true) {
        setGyroReadings();
        forwardGyroPIDLoop();

        if (abs(targetAngle - angle) < 2) {
          // stop(); Pick wheels
          break;
        }
      }

      targetAngle = 90;
      PREVIOUS_TIME = millis();
      pingTimer = millis();
      while (true) {
        setGyroReadings();
        forwardGyroPIDLoop();

        if (millis() >= pingTimer) {
          pingTimer += pingSpeed;      // Set the next ping time.
          sonar.ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
        }
        
        if (distance <= 50 && ((millis() - PREVIOUS_TIME) > 3000)) {
          while (true)
            stop();
        }
      }
    }

     if (millis() >= pingTimer) // pingSpeed milliseconds since last ping, do another ping.
      break;

    }

  pingTimer += pingSpeed;      // Set the next ping time.
  sonar.ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
}

void setGyroReadings () {
  // === Read accelerometer (on the MPU6050) data === //
    readAcceleration();
    // Calculating Roll and Pitch from the accelerometer data
    accAngleX = (atan(AccY / sqrt(pow(AccX, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorX; //AccErrorX is calculated in the calculateError() function
    accAngleY = (atan(-1 * AccX / sqrt(pow(AccY, 2) + pow(AccZ, 2))) * 180 / PI) - AccErrorY;
    
    
    // === Read gyroscope (on the MPU6050) data === //
    previousTime = currentTime;
    currentTime = micros();
    elapsedTime = (currentTime - previousTime) / 1000000; // Divide by 1000 to get seconds
    readGyro();
    // Correct the outputs with the calculated error values
    GyroX -= GyroErrorX; //GyroErrorX is calculated in the calculateError() function
    GyroY -= GyroErrorY;
    GyroZ -= GyroErrorZ;
    // Currently the raw values are in degrees per seconds, deg/s, so we need to multiply by sendonds (s) to get the angle in degrees
    gyroAngleX += GyroX * elapsedTime; // deg/s * s = deg
    gyroAngleY += GyroY * elapsedTime;
    yaw += GyroZ * elapsedTime;
    //combine accelerometer- and gyro-estimated angle values. 0.96 and 0.04 values are determined through trial and error by other people
    roll = 0.96 * gyroAngleX + 0.04 * accAngleX;
    pitch = 0.96 * gyroAngleY + 0.04 * accAngleY;
}

void forwardGyroPIDLoop() {
    angle = yaw; //if you mounted MPU6050 in a different orientation to me, angle may not = roll. It can roll, pitch, yaw or minus version of the three
    //for me, turning right reduces angle. Turning left increases angle.
    int angleError = targetAngle - angle;
    float P = KpGyro * angleError;
    integral_gyro += angleError;
    float I = KiGyro * integral_gyro;

    float derivative = angleError - previous_error_gyro;
    float D = KdGyro * derivative; // Derivative term

    float steering = P + I + D;

            
    rotateMotor(MOTOR_SPEED + steering, MOTOR_SPEED - steering);
    previous_error_gyro = angleError;
}

void PIDLoop () {
  int sensorValues[] = {
    analogRead(LEFT_SENSOR),
    analogRead(MIDDLE_SENSOR),
    analogRead(RIGHT_SENSOR)
    };

    double position = (sensorValues[0] - 100) - sensorValues[2];
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
  while(true) {
    rotateMotor(MOTOR_SPEED, -(MOTOR_SPEED - 20));
    if (analogRead(FAR_LEFT) > 720) {
      // rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
      break;
    }
  }

  while (analogRead(FAR_LEFT) > 600 || analogRead(FAR_RIGHT) > 600)
    rotateMotor(MOTOR_SPEED, -(MOTOR_SPEED - 20));
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
  rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
  delay(4000);
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

void setDistanceRight() {
  long newPosition = rightEnc.read();
  
  if (newPosition != oldPositionRight) {
    oldPositionRight = newPosition;
    
    double wheelRotations = newPosition / PPR;
    double distanceTravelled = wheelRotations * wheelCircumference;
    
    // Serial.print("Distance Travelled Right (mm): ");
    // Serial.println(distanceTravelled, 3);
    distanceRight = distanceTravelled;
  }
}

void setDistanceLeft() {
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

void moveEncPIDLoop() {
    double error_s = leftEnc.read() - rightEnc.read() - deviation;
    // Calculate the proportional term
    double P = KpS * error_s;
    integral_s += error_s;
    double I = KiS * integral_s;

    double D = Kd * (error_s - previous_error_s);

    double steering = P + I + D;

    
    rotateMotor(MOTOR_SPEED + steering, MOTOR_SPEED - steering);
    previous_error_s = error_s;
}


void calculateError() {
  //When this function is called, ensure the car is stationary. See Step 2 for more info
  
  // Read accelerometer values 200 times
  c = 0;
  while (c < 200) {
    readAcceleration();
    // Sum all readings
    AccErrorX += (atan((AccY) / sqrt(pow((AccX), 2) + pow((AccZ), 2))) * 180 / PI);
    AccErrorY += (atan(-1 * (AccX) / sqrt(pow((AccY), 2) + pow((AccZ), 2))) * 180 / PI);
    c++;
  }
  //Divide the sum by 200 to get the error value, since expected value of reading is zero
  AccErrorX = AccErrorX / 200;
  AccErrorY = AccErrorY / 200;
  c = 0;
  
  // Read gyro values 200 times
  while (c < 200) {
    readGyro();
    // Sum all readings
    GyroErrorX += GyroX;
    GyroErrorY += GyroY;
    GyroErrorZ += GyroZ;
    c++;
  }
  //Divide the sum by 200 to get the error value
  GyroErrorX = GyroErrorX / 200;
  GyroErrorY = GyroErrorY / 200;
  GyroErrorZ = GyroErrorZ / 200;
  Serial.println("The the gryoscope setting in MPU6050 has been calibrated");
}

void readAcceleration() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B); // Start with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // Read 6 registers total, each axis value is stored in 2 registers
  //For a range of +-2g, we need to divide the raw values by 16384, according to the MPU6050 datasheet
  AccX = (Wire.read() << 8 | Wire.read()) / 16384.0; // X-axis value
  AccY = (Wire.read() << 8 | Wire.read()) / 16384.0; // Y-axis value
  AccZ = (Wire.read() << 8 | Wire.read()) / 16384.0; // Z-axis value
}

void readGyro() {
  Wire.beginTransmission(MPU);
  Wire.write(0x43);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true);
  GyroX = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroY = (Wire.read() << 8 | Wire.read()) / 131.0;
  GyroZ = (Wire.read() << 8 | Wire.read()) / 131.0;
}