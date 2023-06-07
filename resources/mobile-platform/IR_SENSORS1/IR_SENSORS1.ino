int sensorright=53;
int sensorcenter=51;
int sensorleft=49;
int x;
int y;
int z;

void setup() 
{
  Serial.begin(9600);
  Serial;
  pinMode(53,INPUT);
  pinMode(51,INPUT);
  pinMode(49,INPUT);

}

void loop()
 {
   x=digitalRead(sensorright);
     y=digitalRead(sensorcenter);
   z=digitalRead(sensorleft);
  // put your main code heyre, to run repeatedly:


Serial.println(x);
Serial.println(y);
Serial.println(z);

}