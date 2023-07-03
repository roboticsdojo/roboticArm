
#include <Arduino.h>
#include <Servo.h>
#include <math.h>

// Define the servos for the robotic arm
Servo j1Servo;
Servo j2Servo;
Servo j3Servo;
Servo j4Servo;
Servo j5Servo;

#define JOINT_1 2
#define JOINT_2 3
#define JOINT_3 4
#define JOINT_4 5
#define JOINT_5 6

// Initial angles
float initialTheta1 = 00.0;
float initialTheta2 = 0.0;
float initialTheta3 = 0.0;
float initialTheta4 = 0.0;
float initialTheta5 = 0.0;



void moveServo(int angle, Servo servo);
void snail(double angle, Servo servo);
void moveToPos(double x, double y, double z);

void homePosition();
void   positionZero();
void grasp(double angle, Servo servo, int Gdelay);
void setup()
{
  pinMode(JOINT_1, OUTPUT);
  pinMode(JOINT_2, OUTPUT);
  pinMode(JOINT_3, OUTPUT);
  pinMode(JOINT_4, OUTPUT);
   pinMode(JOINT_5, OUTPUT);


  Serial.begin(9600);
  j1Servo.attach(JOINT_1,750, 2600);
  j2Servo.attach(JOINT_2, 500, 3000);
  j3Servo.attach(JOINT_3, 750, 2600);
  j4Servo.attach(JOINT_4, 750, 2600);
  j5Servo.attach(JOINT_5, 750, 2600);

  snail(180, j4Servo);

  moveToPos(-150, 150, 150);
//    delay(2000);
//   moveToPos(0, 150, 100);
//  delay(2000);
//   moveToPos(0, 150, 150);
//   delay(2000);
  
 }

void loop()
{
  int z = 0;
  
   
}
//x and y are distances in x and y plane of the destinaation
//H distance to the final position from 0,0 [in z and y plane]

void moveToPos(double x, double y, double z)
{
  Serial.println("Values of x , y , z :");
   
 
   Serial.print(x);
   Serial.print(" ");
   Serial.print(y);
    Serial.print(" ");
   Serial.println(z);
    Serial.print(" ");

   
    double pi = 3.141592653589793238462643383279502884197;

     // z= z  50; 
    // y = y - 80;
  double l1 = 180.00; 
  double l2 = 120.00;
  double Ls = 350;
  //cosine rule 
  double q2 = round(180 -  acos ( ( (l1 * l1) + (l2 * l2 ) - (x * x ) - (y * y) ) / (2 * l1 * l2) )); //* 180 / 3.142;
 double q21 = round(acos ( ((x *x ) + (y * y ) - (l1 * l1) - (l2 * l2)) / (2 * l1 * l2 ))); 

  double l = round(sqrt(x * x + y * y)); // x and y extension

  //total length of the link 
  //l = 350; 


  // angle q1 
  double q1 =  degrees(atan2(z , l )) ;//- degrees(atan( (l2 * degrees(sin (q21)) / (l1 + l2 * degrees(cos (q21))))));
  Serial.print("Value of q1 is :");
  Serial.print(q1);
  Serial.print("\n\n");
  q1 = round(q1);


  double b = atan2(x, y) * (180 / pi); // base angle



  Serial.print(" BOTH VALUES OF Q2 ARE "); 
  Serial.print(q2);
  Serial.print("  ");
  Serial.println(q21);

  double h = round(sqrt(l * l + z * z));
  /*
  Since our link lengths are in the ratio of 18 cm by 12 cm 
  we divide h by 2.5 to 
  */
  //double theta = acos((h / 2.5) / 150) * (180 / 3.1415); // perfect
  double theta = round(atan2( y, x) * 180/ pi);  // atan2 to identify coordinate frame
  theta = 180 - theta -8; 
  Serial.print("value of theta is :");
  Serial.print("value of q2 is :");
  Serial.println(q2);
  snail(q2, j3Servo);
  //moveServo(q2, j3Servo);
  Serial.println(theta);
  snail(theta,j1Servo);
 // moveServo(theta, j1Servo);
  delay(200);
  // if (z == 0)
  //moveServo(-(90 - a1), j2Servo);

  Serial.println("value of q1 is :");
  Serial.print(q1);
  Serial.println();
  q1 = q1 + 13;
  snail(q1, j2Servo);
 // moveServo(q1, j2Servo);
  delay(200);


  

   

}

void moveServo(int angle, Servo servo)
{
  Serial.print("Final Angle is  : ");
  Serial.println(angle);
  snail(angle, servo);

  delay(600);
}

void homePosition(){
  Serial.println("Home position of the arm robot");
   delay(1000);

  
  
  j2Servo.write(0);
   delay(500);
  j3Servo.write(0);
   delay(500);
  j4Servo.write(0);
  j5Servo.write(0);
 
  j1Servo.write(0);
  delay(1000);

 
}
void positionZero(){
   Serial.println("zero position of the arm robot");
   delay(1000);

  
  j1Servo.write(0);
  j2Servo.write(0);
  j3Servo.write(20);
  j4Servo.write(0);
  j5Servo.write(0);
  delay(1000);
}
void grasp(double angle, Servo servo, int Gdelay){
  //opening the gripper
  //servo.write(angle);
  snail(angle, servo);
  Serial.println("Gripper is fully open");

  delay(Gdelay);
  Serial.println("closing the gripper");
  servo.write(0);
  snail(0, servo);

 
 
}
void snail(double angle, Servo servo){
  Serial.println("executing Snail ");
  double i =0.0; 
  while (i <= angle){
    servo.write(i);
    delay(10);
    // Serial.print("Inside the loop : ");
    // Serial.println(i);
    i = i + 1;
  }
}


