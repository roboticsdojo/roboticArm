// Define IR sensor pins
const int sensor1 = 47;
const int sensor2 = 45;
const int sensor3 = 43;

// Define motor driver pins
// motor 1
int leftMotor1pin1 = 2;
int leftMotor1pin2 = 3;
int leftEnablePinA = 4;
//motor 2
int leftMotor2pin3 = 5;
int leftMotor2pin4 = 6;
int leftEnablePinB =7;
//motor 3
int rightMotor3pin1 = 9;
int rightMotor3pin2 = 10;
int rightEnablePinA =8;
//motor 4
int rightMotor4pin3 = 11;
int rightMotor4pin4 = 12;
int rightEnablePinB =13;

// Define motor speed
const int motorSpeed = 70;

void setup() {
  // Configure sensor pins as inputs
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  
  Serial.begin(9600);

  // Configure motor driver pins as outputs
  pinMode(leftMotor1pin1, OUTPUT);
  pinMode(leftMotor1pin2, OUTPUT);
  pinMode(leftEnablePinA, OUTPUT);
  
  pinMode(leftMotor2pin3, OUTPUT);
  pinMode(leftMotor2pin4, OUTPUT);
  pinMode(leftEnablePinB, OUTPUT);

  pinMode(rightMotor3pin1, OUTPUT);
  pinMode(rightMotor3pin2, OUTPUT);
  pinMode(rightEnablePinA, OUTPUT);

  pinMode(rightMotor4pin3, OUTPUT);
  pinMode(rightMotor4pin4, OUTPUT);
  pinMode(rightEnablePinB, OUTPUT);
  
  // Set initial motor direction
  digitalWrite(leftMotor1pin1, HIGH);
  digitalWrite(leftMotor1pin2, LOW);
  
  digitalWrite(leftMotor2pin3, HIGH);
  digitalWrite(leftMotor2pin4, LOW);

  digitalWrite(rightMotor3pin1, HIGH);
  digitalWrite(rightMotor3pin2, LOW);
  
  digitalWrite(rightMotor4pin3, HIGH);
  digitalWrite(rightMotor4pin4, LOW);
  
  // Start motor
  analogWrite(leftEnablePinA, motorSpeed);
  analogWrite(leftEnablePinB, motorSpeed);
  analogWrite(rightEnablePinA, motorSpeed);
  analogWrite(rightEnablePinB, motorSpeed);
}

void loop() {
  // Read sensor values
  int sensor1Value = digitalRead(sensor1);
  int sensor2Value = digitalRead(sensor2);
  int sensor3Value = digitalRead(sensor3);
  
  // Line following logic
  if (sensor1Value == LOW && sensor2Value == LOW && sensor3Value == LOW) {
    // Move forward if the middle sensor detects the line
    Serial.println("Forward");
    digitalWrite(leftMotor1pin1, HIGH);
    digitalWrite(leftMotor1pin2, LOW);
    
    digitalWrite(leftMotor2pin3, LOW);
    digitalWrite(leftMotor2pin4, HIGH);
    
    digitalWrite(rightMotor3pin1, LOW);
    digitalWrite(rightMotor3pin2, HIGH);
    
    digitalWrite(rightMotor4pin3, HIGH);
    digitalWrite(rightMotor4pin4, LOW);
    
  }
  else if (sensor1Value == LOW && sensor2Value == HIGH && sensor3Value == LOW) {
    // Move forward if the middle sensor detects the line
    Serial.println("Forward");
    digitalWrite(leftMotor1pin1, HIGH);
    digitalWrite(leftMotor1pin2, LOW);
    
    digitalWrite(leftMotor2pin3, LOW);
    digitalWrite(leftMotor2pin4, HIGH);
    
    digitalWrite(rightMotor3pin1, LOW);
    digitalWrite(rightMotor3pin2, HIGH);
    
    digitalWrite(rightMotor4pin3, HIGH);
    digitalWrite(rightMotor4pin4, LOW);
    
  } else if (sensor1Value == HIGH && sensor2Value == LOW && sensor3Value == LOW) {
    // Turn left if the left sensor detects the line
    Serial.println("left");
    digitalWrite(leftMotor1pin1, LOW);
    digitalWrite(leftMotor1pin2, HIGH);
    
    digitalWrite(leftMotor2pin3, HIGH);
    digitalWrite(leftMotor2pin4, LOW);

    digitalWrite(rightMotor3pin1, HIGH);
    digitalWrite(rightMotor3pin2, LOW);
    
    digitalWrite(rightMotor4pin3, HIGH);
    digitalWrite(rightMotor4pin4, LOW);
      
  } else if (sensor1Value == LOW && sensor2Value == LOW && sensor3Value == HIGH) {
    // Turn right if the right sensor detects the line
    Serial.println("Right");
    digitalWrite(leftMotor1pin1, HIGH);
    digitalWrite(leftMotor1pin2, LOW);
    
    digitalWrite(leftMotor2pin3, LOW);
    digitalWrite(leftMotor2pin4, HIGH);
    
    digitalWrite(rightMotor3pin1, LOW);
    digitalWrite(rightMotor3pin2, HIGH);
    
    digitalWrite(rightMotor4pin3, LOW);
    digitalWrite(rightMotor4pin4, HIGH);
  }
}
