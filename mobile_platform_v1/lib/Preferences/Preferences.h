#ifndef PREFERENCES_H
#define PREFERENCES_H

// Mobile Platform
#define MOTOR_SPEED 245
#define RIGHT_SENSOR A3//
#define MIDDLE_SENSOR A2//
#define LEFT_SENSOR A4//
#define FAR_RIGHT A0//
#define FAR_LEFT A1//
#define ENABLE_LEFT_MOTOR 45 //3 //6//
#define ENABLE_RIGHT_MOTOR 46 //5//
#define LEFT_MOTOR_PIN1 8//
#define LEFT_MOTOR_PIN2 9//
#define RIGHT_MOTOR_PIN1 10//
#define RIGHT_MOTOR_PIN2 4//

#define TRIGPIN 14//12 //4
#define ECHOPIN 15//11 //7

#define CORRECTION 100

#define RIGHT_ENCODER_A_PIN 18 //18 //2 (2 and 20 for left)//
#define RIGHT_ENCODER_B_PIN 19 //19 //3 (18 and 19 for right)//

// ENCODERS
#define LEFT_ENCODER_A_PIN 3//20 //18 //2 (2 and 20 for left)//
#define LEFT_ENCODER_B_PIN 2 //19 //3 (18 and 19 for right)//

// // Motor Parameters
// #define PPR 360.0              // Pulses per revolution for the encoder
// #define wheelDiameter 0.1      // Diameter of the wheel in meters (adjust this to match your setup)
// #define wheelCircumference wheelDiameter * PI
// #define WHEELBASE 250



#endif //PREFERENCES_H
