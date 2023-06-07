
// Define IR sensor pins
const int leftSensor = 22;
const int rightSensor = 21;
//const int sensor3 = 43;

//L298N motor control pins
const int LeftMotorForward = 8;
const int LeftMotorBackward = 9;
const int leftEnable = 4;

const int RightMotorForward = 10;
const int RightMotorBackward = 11;
const int rightEnable = 5;


// Define motor speed
const int motorSpeed = 70;

void setup() {
  // Configure sensor pins as inputs
  pinMode(leftSensor, INPUT);
  pinMode(rightSensor, INPUT);
  //pinMode(sensor3, INPUT);
  
  Serial.begin(9600);

  // Configure motor driver pins as outputs
  pinMode(LeftMotorForward, OUTPUT);
  pinMode(LeftMotorBackward, OUTPUT);
  pinMode(leftEnable, OUTPUT);
  pinMode(RightMotorBackward, OUTPUT);
  pinMode(RightMotorForward, OUTPUT);
  pinMode(rightEnable, OUTPUT);

  //Start motor
  analogWrite(LeftMotorForward, motorSpeed);
  analogWrite(LeftMotorBackward, motorSpeed);
  
  analogWrite(RightMotorForward, motorSpeed);
  analogWrite(RightMotorBackward, motorSpeed);
}

void loop() {
  // Read sensor values
  int leftSensorValue = digitalRead(leftSensor);
  int rightSensorValue = digitalRead(rightSensor);
  //int sensor3Value = digitalRead(sensor3);
  
  // Line following logic
  if (leftSensorValue == LOW && rightSensorValue == LOW) {
    // Move forward if the middle sensor detects the line
    Serial.println("Forward");
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorForward, HIGH);

    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorBackward, LOW);
    delay(500);
  }
  else if (leftSensorValue == LOW && rightSensorValue == HIGH) {
    // Move forward if the middle sensor detects the line
    Serial.println("move right");
    digitalWrite(LeftMotorForward, HIGH);
    digitalWrite(RightMotorBackward, HIGH);

    digitalWrite(LeftMotorBackward, LOW);
    digitalWrite(RightMotorForward, LOW);
    delay(500);
    
  } else if (leftSensorValue == HIGH && rightSensorValue == LOW) {
    // Turn left if the left sensor detects the line
    Serial.println("move left");
    digitalWrite(LeftMotorBackward, HIGH);
    digitalWrite(RightMotorForward, HIGH);

    digitalWrite(LeftMotorForward, LOW);
    digitalWrite(RightMotorBackward, LOW);
    delay(500);
     
  } //else if (leftSensorValue == LOW && rightSensorValue == LOW) {
    // Turn right if the right sensor detects the line
   // Serial.println("Right");
//}
}
