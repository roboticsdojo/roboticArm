int trig=28;
int echo=29;
int timeInMicro;
int distanceInCm;
void setup() {
  Serial.begin(9600);
  pinMode(7,OUTPUT);
  pinMode (6, INPUT);
  // put your setup code here, to run once:

}

void loop() {
  digitalWrite(trig,LOW);
  delayMicroseconds(2);
  digitalWrite(trig,HIGH);
  delayMicroseconds(10);


  timeInMicro=pulseIn(echo, HIGH);
  distanceInCm=timeInMicro/29 /2;
  Serial.println(distanceInCm);
  delay(100);
  // put your main code here, to run repeatedly:

}
