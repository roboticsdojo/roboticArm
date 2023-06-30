#include <Arduino.h>
#include <Servo.h>
#include <Ramp.h>


// Define the servos for the robotic arm
Servo j1Servo;
Servo j2Servo;
Servo j3Servo;
Servo j4Servo;

#define JOINT_1 2
#define JOINT_2 3
#define JOINT_3 4
#define JOINT_4 5

// Initial angles
float initialTheta1 = 90.0;
float initialTheta2 = 90.0;
float initialTheta3 = 90.0;
float initialTheta4 = 0.0;

void moveServo(int angle, Servo servo);
void moveToPos(double x, double y, double z);

rampInt servo1Ramp;  
rampInt servo2Ramp;  
rampInt servo3Ramp;  
rampInt servo4Ramp;

int prevAngle1 = 0;
int prevAngle2 = 0;
int prevAngle3 = 0;
int prevAngle4 = 0;

void moveServos(int angle1, int angle2, int angle3, int angle4, uint32_t time);
void updateServo();
void rampToAngle(double s1, double s2, double s3, double s4, unsigned long t, bool init=false);
void moveToAngle(double s1, double s2, double s3, double s4);
void calculateInverseKinematics(int x_e, int y_e, int z_e, int phi_e);

void setup()
{
  Serial.begin(9600);
  j1Servo.attach(JOINT_1, 600, 2600);
  j2Servo.attach(JOINT_2, 400, 2700);
  j3Servo.attach(JOINT_3, 600, 2600);
  j4Servo.attach(JOINT_4, 600, 2600);

  j1Servo.write(90);
  j2Servo.write(90);
  j3Servo.write(90);
  j4Servo.write(90);
  rampToAngle(90, 90, 90, 90, 1000, true);


  // moveServos(90, 90, 90, 90, 50000);  // Move servos to initial position
  // rampToAngle(90, 90, 90, 90, 3000);
  delay(2000);  // Wait for movement to complete
  // moveServo(-90, j1Servo);

  // delay(3000);
  // //Sequence 1
  // moveServo(-80, j2Servo);
  // moveServo(20, j3Servo);
  // delay(3000);
  // moveServo(0, j2Servo);
  // moveServo(0, j3Servo);
  // j2Servo.writeMicroseconds(750);
  // delay(5000);
  // j2Servo.writeMicroseconds(2600);
  
}

void loop()
{
  // int z = 0;
  // for (z = 0; z <= 200; z+=50)
  // {
    // moveToPos(0, 100, 0);
    // delay(100);
  // }
  // moveServo(-90, j3Servo);
  // moveToPos(0, 200, 0);
  // delay(3000);
  // moveToPos(0,200,(200 / 2)); // devide by 2 to correct error
  // delay(6000);
  // delay(100);
  // moveServos(90, 90, 120, 0, 60000);  // Move servos to these angles in 2000 milliseconds
  // // delay(2000);  // Wait for movement to complete
  // // moveServos(0, 0, 0, 0, 2000);  // Move servos back to initial position in 2000 milliseconds
  // // delay(2000);  // Wait for movement to complete
  // updateServo();  
  // delay(10); 
  // 
  calculateInverseKinematics(300, 200, 0, 0);
  // calculateInverseKinematics(300, -400, 0, 0);

}

void moveToPos(double x, double y, double z)
{
  double b = atan2(x, y) * (180 / 3.1415); // base angle

  double l = sqrt(x * x + y * y); // x and y extension

  double h = sqrt(l * l + z * z);

  double phi = atan(z / l) * (180 / 3.1415);

  double theta = acos((h / 2) / 150) * (180 / 3.1415); // perfect

  double a1 = phi + theta; // angle for first part of the arm
  double a2 = phi - theta; // angle for second part of the arm

  // moveToAngle(b,a1,a2,g);
  Serial.print("b: ");
  Serial.println(b);
  Serial.print("a1: ");
  Serial.println(-(a1));
  Serial.print("a2: ");
  Serial.println(a2);
  Serial.println();
  // delay(3000);
  moveServo(b, j1Servo);
  // if (z == 0)
  moveServo(-(90 - a1), j2Servo);
  // else
    // moveServo((-(90 - a1)), j2Servo);
  if (z == 0)
    moveServo(90 - (180 - (a1 * 2)), j3Servo);
  else
    moveServo(-90 + (180 - (a1 * 2)), j3Servo);

}

void moveServo(int angle, Servo servo)
{

  // int servoAngle = 90 - angle;
  int servoAngle = map(angle, 90, -90, 700, 2500);
  // Serial.println(servoAngle);

  // if (servoAngle > 180 || servoAngle < 0)
    // Serial.println("Angle out of range");

  // constrain the angle to the range 0-180
  // servoAngle = constrain(servoAngle, 0, 180);

  // move the servo to the specified angle
  // servo.write(servoAngle);
  servo.writeMicroseconds(servoAngle);
}

void updateServo() {
  j1Servo.write(servo1Ramp.update());
  j2Servo.write(servo2Ramp.update());
  j3Servo.write(servo3Ramp.update());
  j4Servo.write(servo4Ramp.update());
}

void moveServos(int angle1, int angle2, int angle3, int angle4, uint32_t time) {
  uint32_t duration1 = abs(angle1 - prevAngle1);
  uint32_t duration2 = abs(angle2 - prevAngle2);
  uint32_t duration3 = abs(angle3 - prevAngle3);
  uint32_t duration4 = abs(angle4 - prevAngle4);

  uint32_t maxDuration = max(max(duration1, duration2), max(duration3, duration4));

  // Calculate step for each servo
  float step1 = (maxDuration == 0) ? 0 : (float)duration1 / (time / 10);
  float step2 = (maxDuration == 0) ? 0 : (float)duration2 / (time / 10);
  float step3 = (maxDuration == 0) ? 0 : (float)duration3 / (time / 10);
  float step4 = (maxDuration == 0) ? 0 : (float)duration4 / (time / 10);

  servo1Ramp.go(angle1, step1);
  servo2Ramp.go(angle2, step2);
  servo3Ramp.go(angle3, step3);
  servo4Ramp.go(angle4, step4);

  prevAngle1 = angle1;
  prevAngle2 = angle2;
  prevAngle3 = angle3;
  prevAngle4 = angle4;
}

void moveToAngle(double s1, double s2, double s3, double s4) {
  j1Servo.write(s1);
  j2Servo.write(s2);
  j3Servo.write(s3);
  j4Servo.write(s4);
}

void rampToAngle(double s1, double s2, double s3, double s4, unsigned long t, bool init) {
  servo1Ramp.go(s1,t, LINEAR, ONCEFORWARD);
  servo2Ramp.go(s2,t, LINEAR, ONCEFORWARD);
  servo3Ramp.go(s3,t, LINEAR, ONCEFORWARD);
  servo4Ramp.go(s4,t, LINEAR, ONCEFORWARD);
  
  while (servo1Ramp.isRunning()) {
    if (init) {
      servo1Ramp.update();
      servo2Ramp.update();
      servo3Ramp.update();
      servo4Ramp.update();
    }
    else
      moveToAngle(servo1Ramp.update(),servo2Ramp.update(),servo3Ramp.update(),servo4Ramp.update());
  }
}

void calculateInverseKinematics(int x_e, int y_e, int z_e, int phi_e) {
  double l1 = 180;
  double l2 = 115;
  double l3 = 170;

  double x_w = x_e - (l3 * cos(phi_e));
  double y_w = y_e - (l3 * sin(phi_e));


  double theta2 = PI - acos((sq(l1) + sq(l2) - sq(x_w) - sq(y_w)) / (2 * l1 * l2));
  double theta1 = (atan(y_w / x_w)) - acos((sq(x_w) + sq(y_w) + sq(l1) - sq(l2)) / (2 *l1 * sqrt(sq(x_w) + sq(y_w))));
  double theta3 = phi_e - theta1 - theta2;

  // convert to degrees
  theta1 = (theta1 * (180 / PI)); // remains
  theta2 = theta2 * (180 / PI) + 90;
  theta3 = theta3 * (180 / PI) - 90;

  // theta2 = (180 - (theta2 > 90 ? theta2 - 90 : theta2 + 90));

  // theta1 = map(theta1, -90, 90, 0, 180);

  rampToAngle(90, abs(theta1), abs(theta2), abs(theta3), 2000);
  Serial.print("theta 1: "); Serial.print(theta1);
  Serial.print("\ttheta 2: "); Serial.print(theta2);
  Serial.print("\ttheta 3: "); Serial.println(theta3);
}

