#include <Arduino.h>

#include "MobilePlatform.h"
#include "Preferences.h"



MobilePlatform platform;

// // Encoder-Related Variables
// int leftEncoderPosition = 0;
// int rightEncoderPosition = 0;

// //-------- Distance-related variables ---------
// long current_millis = 0;
// long previous_millis = 0;

// // PID-related variables
// long prevT = 0;
// float eprev = 0;
// float eintegral = 0;

// // Function prototypes
// void readLeftEncoder();
// void readRightEncoder();

void setup()
{
  platform.setup();
  Serial.begin(9600);
  
  delay(2000);

//   // Motor Encoders
//   pinMode(LEFT_ENCA, INPUT);
//   pinMode(LEFT_ENCB, INPUT);
//   pinMode(RIGHT_ENCA, INPUT);
//   pinMode(RIGHT_ENCB, INPUT);

//   attachInterrupt(digitalPinToInterrupt(LEFT_ENCA), readLeftEncoder, RISING);
//   attachInterrupt(digitalPinToInterrupt(RIGHT_ENCA), readRightEncoder, RISING);
}

void loop()
{
  platform.loop();

  // Serial.println(platform.getDistance());
  // delay(5000);
  // Serial.print(" Left: ");
  // Serial.println(digitalRead(LEFT_SENSOR));
  // Serial.print(" Middle: ");
  // Serial.println(digitalRead(MIDDLE_SENSOR));
  // Serial.print(" Right: ");
  // Serial.println(digitalRead(RIGHT_SENSOR));
  // Serial.print(" Far Left: ");
  // Serial.println(digitalRead(FAR_LEFT));
  // Serial.print(" Far Right: ");
  // Serial.println(digitalRead(FAR_RIGHT));
  // Serial.println("***********************");
  // Serial.println("");
  // delay(5000);
}

//   // ------------------------- Motor Controller PID LOOP -------------------------
//   PIDLoop(leftEncoderPosition, rightEncoderPosition, current_millis, previous_millis, prevT, eprev, eintegral, enableLeftMotor, enableRightMotor, leftMotorPin1, leftMotorPin2, rightMotorPin1, rightMotorPin2);



// void readLeftEncoder()
// {
//   int b = digitalRead(LEFT_ENCB);

//   //? WARNING: left motor is inverted (mirror image of right motor)
//   if (b > 0)
//   {
//     leftEncoderPosition--;
//   }
//   else
//   {
//     leftEncoderPosition++;
//   }
// }

// void readRightEncoder()
// {
//   int b = digitalRead(RIGHT_ENCB);

//   if (b > 0)
//   {
//     rightEncoderPosition++;
//   }
//   else
//   {
//     rightEncoderPosition--;
//   }
// }
