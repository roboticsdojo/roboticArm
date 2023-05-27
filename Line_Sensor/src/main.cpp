#include <Arduino.h>
// Define IR sensor pins
const int sensor1 = 22; // Grey
const int sensor2 = 21; //Green
// const int sensor3 = 43;

// Enable pins
int EnablePinA = 4;
int EnablePinB =5;


// Define motor driver pins
// motor A
int leftMotor1pin1 = 8;
int leftMotor1pin2 = 9;
//motor B
int leftMotor2pin3 = 10;
int leftMotor2pin4 = 11;


// Define motor speed
const int motorSpeed = 70;

void setup() {
  // // Configure sensor pins as inputs
  // pinMode(sensor1, INPUT);
  // pinMode(sensor2, INPUT);
  // pinMode(sensor3, INPUT);
  
  Serial.begin(9600);

  // // Configure motor driver pins as outputs
  pinMode(EnablePinA, OUTPUT);
  pinMode(EnablePinB, OUTPUT);

  pinMode(leftMotor1pin1, OUTPUT);
  pinMode(leftMotor1pin2, OUTPUT);

  pinMode(leftMotor2pin3, OUTPUT);
  pinMode(leftMotor2pin4, OUTPUT);


  delay(20);

  // Set initial motor direction
  digitalWrite(leftMotor1pin1, HIGH);
  digitalWrite(leftMotor1pin2, LOW);
  
  digitalWrite(leftMotor2pin3, HIGH);
  digitalWrite(leftMotor2pin4, LOW);
  
  analogWrite(EnablePinA, motorSpeed);
  analogWrite(EnablePinB, motorSpeed);
}

void loop() {
  // // Read sensor values
  int sensor1Value = digitalRead(sensor1);
  int sensor2Value = digitalRead(sensor2);

  Serial.println(sensor1Value);
  Serial.println(sensor2Value);
  delay(3000);

  // if (sensor1Value == LOW && sensor2Value == LOW)
  // {
  //   //Stop
  //   // digitalWrite()
  // }
  // else if (sensor1Value == LOW && sensor2Value == HIGH)
  // {
  //   //Move right
  //   Serial.println("Move right");
  //   digitalWrite(leftMotor1pin1, LOW);
  //   digitalWrite(leftMotor1pin2, HIGH);
    
  //   digitalWrite(leftMotor2pin3, HIGH);
  //   digitalWrite(leftMotor2pin4, LOW);
  // }

  // else if (sensor1Value == HIGH && sensor2Value == LOW)
  // {
  //   // Move left
  //   Serial.println("Move left");
  //   digitalWrite(leftMotor1pin1, HIGH);
  //   digitalWrite(leftMotor1pin2, LOW);
    
  //   digitalWrite(leftMotor2pin3, LOW);
  //   digitalWrite(leftMotor2pin4, HIGH);
  
  // }

  // else if (sensor1Value == HIGH && sensor2Value == HIGH)
  // {
  //   //Move forward
  //   Serial.println("Move forward");
  //   digitalWrite(leftMotor1pin1, LOW);
  //   digitalWrite(leftMotor1pin2, HIGH);
    
  //   digitalWrite(leftMotor2pin3, LOW);
  //   digitalWrite(leftMotor2pin4, HIGH);
    
  // }
  
  
}