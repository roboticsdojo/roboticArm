#include <Arduino.h>

#include <NewPing.h>
#include "MobilePlatform.h"
#include "Preferences.h"
#include <Encoder.h>
#include <Wire.h>

MobilePlatform platform;

void moveCar(int speedLeft, int speedRight);
void rotateToAngle(float target_angle, float tolerance);


// Encoders
// Motor Parameters
#define PPR 274.63        // Pulses per revolution for the encoder
#define wheelDiameter 65.0     // Diameter of the wheel in mm (adjust this to match your setup)
#define wheelCircumference wheelDiameter * PI

Encoder rightEnc(RIGHT_ENCODER_A_PIN, RIGHT_ENCODER_B_PIN);
Encoder leftEnc(LEFT_ENCODER_A_PIN, LEFT_ENCODER_B_PIN);
long oldPositionRight  = -999;
long oldPositionLeft  = -999;
int deviation = 0.5;
int distanceRight = 0;
int distanceLeft = 0;


// Function Prototypes
void setDistanceRight();
void setDistanceLeft();
void moveDistance(double distance_mm, bool detectLine = false);

int minSpeed = 180; // Minimum motor speed
int maxSpeed = 240; // Maximum motor speed


// MPU 6050
const int MPU = 0x68; // MPU6050 I2C address
float AccX, AccY, AccZ; //linear acceleration
float GyroX, GyroY, GyroZ; //angular velocity
float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; //used in void loop()
float roll, pitch, yaw;
float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
float elapsedTime, currentTime, previousTime;
int c = 0;

float angle; //due to how I orientated my MPU6050 on my car, angle = roll
float targetAngle = 0;

// prototypes
void calculateError();
void readAcceleration();
void readGyro();
void setGyroReadings();

// Global variables
double Kp = 1.00, Ki = 0.001, Kd = 20.00; // Tune these values
double integral = 0, previous_error = 0;

double Kp_line = 1.0, Ki_line = 0.0, Kd_line = 5.0; // Tune these values
double integral_line = 0, previous_error_line = 0;

double PIDController(double target_heading);
void rotateToAngle(double target_angle);

void followLine(int stopDistance);

enum State {
      PICK_TRAILER,
      PICK_ENGINE,
      PICK_WHEELS,
      BACK_TO_CHASIS,
      PICK_CABIN,
      BACK_TO_START,
      STOP
    };
void customDelay(unsigned long milliseconds);
unsigned long PREVIOUS_TIME;
State state;
bool done;
float mapToMotorSpeed(float pid_output);

//Line following

double PIDControllerLine();

// Ultra sonic and distance stuff
#define MAX_DISTANCE 200 // Maximum distance we want to ping for (in centimeters).

NewPing sonar(TRIGPIN, ECHOPIN, MAX_DISTANCE); // NewPing setup of pins and maximum distance.

unsigned int pingSpeed = 100; // How frequently are we going to send out a ping (in milliseconds). 50ms would be 20 times a second.
unsigned long pingTimer;     // Holds the next ping time.
volatile int distance = 400;
int distance_counter = 0;
unsigned long previous_time_distance;

// Prototypes
void echoCheck();


void setup() {
  platform.setup ();
  Serial.begin(9600);

  // ultrasonic
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

  state = PICK_TRAILER;

  // Test the sensors
  // while (true) {
  //   Serial.print("LEFT_SENSOR: ");
  //   Serial.print(analogRead(LEFT_SENSOR));
  //   Serial.print("\t||\tRIGHT_SENSOR: ");
  //   Serial.println(analogRead(RIGHT_SENSOR));
  // }

  // followLine(50);
  // while (true) {
  //   Serial.println("stop");
  //   customDelay(1000);
  // }

void loop() {
  setGyroReadings();
  switch (state) {
    case PICK_TRAILER:
      moveDistance(1500);
      customDelay(200);
      rotateToAngle(90, 10);
      customDelay(5000);//wait for the trailer to be picked
      state = PICK_ENGINE;
      break;
    case PICK_ENGINE:
      fastRotate(-150);
      customDelay(1500);
      rotateToAngle(-150, 5);
      targetAngle = -(150);
      moveDistance(850);

      // Move and detect line
      targetAngle = -130;
      moveDistance(2000, true); // Set to true to move and detect line
      moveDistance(200);
      rotateToAngle(-80, 10);
      targetAngle = -80;
      moveDistance(1000, true);
      followLine(50);
      customDelay(2000); // wait to pick engine

      // Follow the  line and Stop when distance <= 50
      customDelay(5000); // Wait to pick wheels
      state = PICK_WHEELS;
      break;
    case PICK_WHEELS:
      rotateToAngle(0, 10); // Turn Left
      targetAngle = 0;
      moveDistance(1000, true); // Move Straight until meet line
      rotateToAngle(-90, 10); // Turn Right
      customDelay(2000);// Wait to pick wheels
      state = BACK_TO_CHASIS;
      break;
    case BACK_TO_CHASIS:
      rotateToAngle(90, 10); // Rotate 180
      followLine(50); // Follow Line to chasis and Stop at distance <= 50
      customDelay(2000); // Wait to drop staff
      state = PICK_CABIN;
      break;
    case PICK_CABIN:
      rotateToAngle(0, 10);
      targetAngle = 0;
      moveDistance(600);
      moveDistance(1000, true);
      // Climb ramp
      // Stop when distance <= 50
      // Wait to pick cabin
      // Rotate 180
      // Return Cabin to chasis
      // Wait to drop
      state = BACK_TO_START;
    case BACK_TO_START:
      // Go back to starting position
      state = STOP;
      break;
    case STOP:
      customDelay(5000);
  }
}

void customDelay(unsigned long milliseconds) {
  PREVIOUS_TIME = millis();
  while (abs(millis() - PREVIOUS_TIME) <= milliseconds)
        setGyroReadings();
}

void moveCar(int speedLeft, int speedRight) {
  if(speedLeft >= 0) {
    digitalWrite(LEFT_MOTOR_PIN1, HIGH); 
    digitalWrite(LEFT_MOTOR_PIN2, LOW);
  } else {
    digitalWrite(LEFT_MOTOR_PIN1, LOW);
    digitalWrite(LEFT_MOTOR_PIN2, HIGH);
    speedLeft = -speedLeft; // Ensure the speed is positive
  }
  
  if(speedRight >= 0) {
    digitalWrite(RIGHT_MOTOR_PIN1, HIGH);
    digitalWrite(RIGHT_MOTOR_PIN2, LOW);
  } else {
    digitalWrite(RIGHT_MOTOR_PIN1, LOW);
    digitalWrite(RIGHT_MOTOR_PIN2, HIGH);
    speedRight = -speedRight; // Ensure the speed is positive
  }

  analogWrite(ENABLE_LEFT_MOTOR, speedLeft); // control speed of left motor
  analogWrite(ENABLE_RIGHT_MOTOR, speedRight); // control speed of right motor
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

void moveDistance(double distance_mm, bool detectLine = false) {
  // Reset PID values
  integral = 0;
  previous_error = 0;

  setGyroReadings();
  // Reset the encoder readings
  rightEnc.write(0);
  leftEnc.write(0);

  // Initialize speeds
  int currentSpeedRight = minSpeed; // Start at minSpeed for right wheel
  int currentSpeedLeft = minSpeed; // Start at minSpeed for left wheel
  
  // Start the motors
  moveCar(currentSpeedLeft, currentSpeedRight);

  while (true) {
    // Update the current distances
    setDistanceLeft();
    setDistanceRight();
    // Serial.print("Distance Left: ");
    // Serial.print(distanceLeft);
    // Serial.print("\t||\tDistance Right: ");
    // Serial.println(distanceRight);

    // Get the PID controller output
    double pidOutput = PIDController(targetAngle);

    // Adjust the speeds
    currentSpeedRight = constrain(currentSpeedRight + pidOutput, minSpeed, maxSpeed);
    currentSpeedLeft = constrain(currentSpeedLeft - pidOutput, minSpeed, maxSpeed);

    // Apply the new speeds
    moveCar(currentSpeedLeft, currentSpeedRight);

    // If both wheels have traveled the desired distance, stop
    if (distanceLeft >= distance_mm && distanceRight >= distance_mm) {
      break;
    }

    if (detectLine && (analogRead(LEFT_SENSOR) > 650 || analogRead(RIGHT_SENSOR) > 650)) {
      break;
    }
  }
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);

  analogWrite(ENABLE_LEFT_MOTOR, 0); // control speed of left motor
  analogWrite(ENABLE_RIGHT_MOTOR, 0); // control speed of right motor
}

double PIDController(double target_heading) {
  setGyroReadings();
  
  // Calculate the error
  double error = target_heading - angle;

  // Proportional term
  double P = Kp * error;

  // Integral term
  integral += error;
  double I = Ki * integral;

  // Derivative term
  double derivative = error - previous_error;
  double D = Kd * derivative;

  // Remember the error for the next time
  previous_error = error;

  // The PID controller output
  return P + I + D;
}

double PIDControllerLine() {
    setGyroReadings();

    // Read the sensor values
    int sensorValues[] = {
      analogRead(LEFT_SENSOR),
      analogRead(MIDDLE_SENSOR),
      analogRead(RIGHT_SENSOR)
      };

    // Calculate the error
    double error = (sensorValues[0]) - ((sensorValues[2]));

     // Proportional term
    double P = Kp_line * error;

    // Integral term
    integral_line += error;
    double I = Ki_line * integral_line;

    // Derivative term
    double D = Kd_line * (error - previous_error_line);
    
    // Remember the error for the next time
    previous_error_line = error;

    // The PID controller output
    return P + I + D;
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
    angle = yaw;
    // if (angle < -180) angle = 0;
    // if (angle > 180) angle = 0;
    while(angle > 180) angle -= 360;
    while(angle < -180) angle += 360;
}

void calculateError() {
  //When this function is called, ensure the car is stationary.
  
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

void rotateToAngle(double target_angle) {
  // Reset the PID variables
  integral = 0;
  previous_error = 0;
  
  // Initialize speeds
  int currentSpeedRight = 250; // Start at minSpeed for right wheel
  int currentSpeedLeft = 250; // Start at minSpeed for left wheel // 220 instead of minspeed
  
  // Start the motors
  moveCar(currentSpeedLeft, currentSpeedRight);

  while (true) {
    // Get the PID controller output
    double pidOutput = PIDController(target_angle);

    // Adjust the speeds
    currentSpeedRight = constrain(currentSpeedRight + pidOutput, 250, 254);// 220 instead of
    currentSpeedLeft = constrain(currentSpeedLeft - pidOutput, 250, 254);  // minspeed

    // Apply the new speeds
    moveCar(currentSpeedLeft, currentSpeedRight);

    // If the error is below a small threshold, stop
    if (abs(target_angle - angle) <= 1.0) { // Adjust the threshold as needed
      break;
    }
  }
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);

  analogWrite(ENABLE_LEFT_MOTOR, 0); // control speed of left motor
  analogWrite(ENABLE_RIGHT_MOTOR, 0); // control speed of right motor
}

void rotateToAngle(float target_angle, float tolerance) {//tolerance = 2.0
  // float current_angle = 0.0; // initialize current angle
  float current_angle = angle;
  float error = target_angle - current_angle; // initialize error

  // float min_motor_speed = 220.0; // minimum speed required to move the car

  do {
    // calculate the current orientation of the car
    setGyroReadings();
    current_angle = angle;

    // calculate the error and PID output
    error = target_angle - current_angle;
    float pid_output = PIDController(target_angle);
    float motor_speed = mapToMotorSpeed(pid_output);
    
    // // apply the PID output to the motors (turn in place)
    // // check if pid_output is below minimum motor speed, and if so, set it to minimum motor speed
    // if (pid_output < min_motor_speed) {
    //   pid_output = min_motor_speed;
    // }

    // // note: we might need to adjust the sign of pid_output depending on setup
    // moveCar(pid_output, -pid_output);
    if (pid_output < 0) { // If PID output is negative, rotate in the opposite direction
      moveCar(-motor_speed, motor_speed);
    } else {
      moveCar(motor_speed, -motor_speed);
    }

  } while(abs(error) > tolerance); // continue rotating until we are close to the target angle
  
  // once we are close enough to the target angle, stop the car
  moveCar(0, 0);
}

void echoCheck() { // Timer2 interrupt calls this function every 24uS where you can check the ping status.
  // Don't do anything here!
  if (sonar.check_timer()) { // This is how you check to see if the ping was received.
    // // Here's where you can add code.
    Serial.print("Ping: ");
    Serial.print(sonar.ping_result / US_ROUNDTRIP_CM); // Ping returned, uS result in ping_result, convert to cm with US_ROUNDTRIP_CM.
    Serial.println("cm");
    distance = sonar.ping_result / US_ROUNDTRIP_CM;
  }
  // Don't do anything here!
}

float mapToMotorSpeed(float pid_output) {
  // Ensure pid_output is between 0 and 1
  pid_output = constrain(pid_output, 0.0, 1.0);
  // Map from PID output to motor speed
  return pid_output * (240 - 230) + 230;
}

void followLine(int stopDistance) {
  distance = 200;
  sonar.ping_timer(echoCheck);
  customDelay(200);
  // Initialize speeds
  // int currentSpeedRight = minSpeed; // Start at minSpeed for right wheel
  // int currentSpeedLeft = minSpeed; // Start at minSpeed for left wheel
  previous_time_distance = millis();

  while (true) {
    // Check obstacle distance
    if (millis() >= pingTimer) {
      pingTimer += pingSpeed;      // Set the next ping time.
      sonar.ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
    }

    // Keep updating angles: improves accuracy
    setGyroReadings();
    

    // Follow Line
     // Get the PID controller output
    double pidOutput = PIDControllerLine();

    // // Adjust the speeds
    // currentSpeedRight = constrain(currentSpeedRight + pidOutput, minSpeed, maxSpeed);
    // currentSpeedLeft = constrain(currentSpeedLeft - pidOutput, minSpeed, maxSpeed);

    // // Apply the new speeds
    // moveCar(currentSpeedLeft, currentSpeedRight);
    platform.rotateMotor(MOTOR_SPEED - pidOutput, MOTOR_SPEED + pidOutput);

    if (distance <= stopDistance && ((millis() - PREVIOUS_TIME) > 3000)) {
      break;
    }
  }

  // Stop the car
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);

  analogWrite(ENABLE_LEFT_MOTOR, 0); // control speed of left motor
  analogWrite(ENABLE_RIGHT_MOTOR, 0); // control speed of right motor
}

void followLine(int stopDistance) {
  sonar.ping_timer(echoCheck);
  customDelay(100);
  // Initialize speeds
  // int currentSpeedRight = minSpeed; // Start at minSpeed for right wheel
  // int currentSpeedLeft = minSpeed; // Start at minSpeed for left wheel
  previous_time_distance = millis();

  while (true) {
    // Check obstacle distance
    if (millis() >= pingTimer) {
      pingTimer += pingSpeed;      // Set the next ping time.
      sonar.ping_timer(echoCheck); // Send out the ping, calls "echoCheck" function every 24uS where you can check the ping status.
    }

    // Keep updating angles: improves accuracy
    setGyroReadings();
    

    // Follow Line
     // Get the PID controller output
    double pidOutput = PIDControllerLine();

    // // Adjust the speeds
    // currentSpeedRight = constrain(currentSpeedRight + pidOutput, minSpeed, maxSpeed);
    // currentSpeedLeft = constrain(currentSpeedLeft - pidOutput, minSpeed, maxSpeed);

    // // Apply the new speeds
    // moveCar(currentSpeedLeft, currentSpeedRight);
    platform.rotateMotor(MOTOR_SPEED - pidOutput, MOTOR_SPEED + pidOutput);

    if (distance <= 10 && ((millis() - PREVIOUS_TIME) > 3000)) {
      break;
    }
  }

  // Stop the car
  digitalWrite(LEFT_MOTOR_PIN1, LOW);
  digitalWrite(LEFT_MOTOR_PIN2, LOW);
  digitalWrite(RIGHT_MOTOR_PIN1, LOW);
  digitalWrite(RIGHT_MOTOR_PIN2, LOW);

  analogWrite(ENABLE_LEFT_MOTOR, 0); // control speed of left motor
  analogWrite(ENABLE_RIGHT_MOTOR, 0); // control speed of right motor
}
