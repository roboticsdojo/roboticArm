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

void MobilePlatform::loop() {
  setState();
  Serial.println(currentState);
  delay(6000);
  if (currentState == MOVING_FORWARD){
    //Move forward
    handleSensorValues();
  }
  if (currentState == TURNING_RIGHT) {
    //Turn Right
    rotateMotor(-(220), 220);
    delay(4000);
    rotateMotor(0, 0);
    delay(1000);
  }
  if (currentState == TURNING_LEFT) {
    //Turn Left
    rotateMotor(220, -220);
    delay(4000);
    rotateMotor(0, 0);
    delay(1000);
  }
  if (currentState == TURNING_180) {
    //turn 180
    rotateMotor(-(220), 220);
    delay(10000);
    rotateMotor(0, 0);
    delay(1000);
  }
  if (currentState == STOP) {
    //stop
    rotateMotor(0, 0);
    //pick stuff
    delay(5000);
    currentState = TURNING_180;
  }
}

void MobilePlatform::setState() {
  if (handleFarTurnValues() == 0) {
    currentState = MOVING_FORWARD;
    // return;
  }
  if (handleFarTurnValues() == 1) {
    // intercection detected
    //Decide left or right or 180
    int turn = turns[counter];
    if (turn == 1)
      currentState = TURNING_RIGHT;
    else if (turn == -1)
      currentState = TURNING_LEFT;
    else if (turn == 0)
      currentState = MOVING_FORWARD;
    counter++;
  }

  if (getDistance() <= 30) {
    currentState = STOP;
    // return;
  }
}

int MobilePlatform::handleFarTurnValues() {
  int farTurnValues[] = {digitalRead(FAR_RIGHT), digitalRead(FAR_LEFT)};

  if (farTurnValues[0] || farTurnValues[1]) {//Far right
    delay(100);
    
    farTurnValues[0] = digitalRead(FAR_RIGHT);
    farTurnValues[1] = digitalRead(FAR_LEFT);

    if (farTurnValues[0] || farTurnValues[1])
    {
      // // turn = 100;
      // 
      
      // decide where to turn
      return 1;
    }
    }
    return (0);
  }


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
