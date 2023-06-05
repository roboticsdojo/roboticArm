#include "MobilePlatform.h"

MobilePlatform::MobilePlatform() {
  turning = false;
  stop = false;
}

void MobilePlatform::setup() {
  configurePins();
  configurePWMFrequency();
  rotateMotor(0, 0);
}
int MobilePlatform:: getDistance() {
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


void MobilePlatform::configurePins() {
  pinMode(RIGHT_SENSOR, INPUT);
  pinMode(MIDDLE_SENSOR, INPUT);
  pinMode(LEFT_SENSOR, INPUT);

  pinMode(FAR_RIGHT, INPUT);
  pinMode(FAR_LEFT, INPUT);

  pinMode(ENABLE_LEFT_MOTOR, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN1, OUTPUT);
  pinMode(RIGHT_MOTOR_PIN2, OUTPUT);
  
  pinMode(ENABLE_RIGHT_MOTOR, OUTPUT);
  pinMode(LEFT_MOTOR_PIN1, OUTPUT);
  pinMode(LEFT_MOTOR_PIN2, OUTPUT);

  pinMode(TRIGPIN, OUTPUT);
  pinMode(ECHOPIN, INPUT);  
}

void MobilePlatform::configurePWMFrequency() {
  // Change the frequency of PWM signal on pins D5 and D6 to 7812.5 Hz.
  TCCR0B = TCCR0B & (B11111000 | B00000010);
}

int turn = 0;
unsigned long current = 0;
void MobilePlatform::loop() {
  if (turn == 0) {
    stop = true;
    handleFarTurnValues();
  }
  handleSensorValues();

  if (stop && getDistance() == 25) {
    rotateMotor(0, 0);
    
    // Pick  stuff
      delay(5000);


    // Turn 180 degrees
      rotateMotor(-(220), 220);
      delay(10000); //Move right
      
      turn = 0;
      // current = millis();
  }

  // if (turn == 10 && millis() - current >= 2000)
  //   turn = 0;
}

void MobilePlatform::handleFarTurnValues() {
  int farTurnValues[] = {digitalRead(FAR_RIGHT), digitalRead(FAR_LEFT)};

  if (farTurnValues[0]) {//Far right
    delay(100);
    
    farTurnValues[0] = digitalRead(FAR_RIGHT);
    if (farTurnValues[0])
    {
      turn = 100;
      rotateMotor(-(220), 220);
      delay(4000);
      rotateMotor(0, 0);
      delay(1000);
    }
    // if (farTurnValues[0])
    // {
    //   while (true)
    //   {
    //     rotateMotor(0, 0);
    //     delay(4500);
    //   }
    }

    
    
    
    // while(!digitalRead(LEFT_SENSOR) && !digitalRead(MIDDLE_SENSOR) && !digitalRead(RIGHT_SENSOR))
    //   continue;

    // handleSensorValues();
  }

  // if (farTurnValues[1])
  // {
  //   while (true)
  //   {
  //     Serial.println(farTurnValues[1]);
  //     delay(3000);
  //   }
    
  // }
// }

void MobilePlatform::handleSensorValues() {
  int sensorValues[] = {
    digitalRead(LEFT_SENSOR),
    digitalRead(MIDDLE_SENSOR),
    digitalRead(RIGHT_SENSOR)
  };
  if (!turning) {
    if (!sensorValues[0] && !sensorValues[1] && !sensorValues[2])//0 0 0
    {
      // Serial.println("All white");
      rotateMotor(MOTOR_SPEED, MOTOR_SPEED);
    }
    else if (!sensorValues[0] && !sensorValues[1] && sensorValues[2])//0 0 1
    {
      // Serial.println("Correction: turn right");
      rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);

    }
    else if (!sensorValues[0] && sensorValues[1] && !sensorValues[2])//0 1 0
    {
      // Serial.println("Forward");
      rotateMotor(MOTOR_SPEED, MOTOR_SPEED);// Right  Left
    }
    else if (!sensorValues[0] && sensorValues[1] && sensorValues[2])//0 1 1
    {
      // Serial.println("90 deg right Turn");
      rotateMotor(MOTOR_SPEED, -MOTOR_SPEED);
    }
    else if (sensorValues[0] && !sensorValues[1] && !sensorValues[2])//1 0 0
    {
      // Serial.println("Correction: turn left");
      rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);

    }
    else if (sensorValues[0] && !sensorValues[1] && sensorValues[2])//1 0 1
    {
      // Serial.println("Impossible");
      rotateMotor(0, 0);
    }
    else if (sensorValues[0] && sensorValues[1] && !sensorValues[2])//1 1 0
    {
      // Serial.println("90 deg left Turn");
      rotateMotor(-MOTOR_SPEED, MOTOR_SPEED);
      // delay(3000);
    }
    else if (sensorValues[0] && sensorValues[1] && sensorValues[2])//1 1 1
    {
      // Serial.println("Sharp Left/Sharp Right");
      rotateMotor(MOTOR_SPEED+20, MOTOR_SPEED+20);
    }
  }
}

void MobilePlatform::rotateMotor(int rightMotorSpeed, int leftMotorSpeed) {
  setMotorState(RIGHT_MOTOR_PIN1, RIGHT_MOTOR_PIN2, rightMotorSpeed);
  setMotorState(LEFT_MOTOR_PIN1, LEFT_MOTOR_PIN2, leftMotorSpeed);
  
  analogWrite(ENABLE_RIGHT_MOTOR, abs(rightMotorSpeed));
  analogWrite(ENABLE_LEFT_MOTOR, abs(leftMotorSpeed));
}

void MobilePlatform::setMotorState(int motorPin1, int motorPin2, int motorSpeed) {
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
