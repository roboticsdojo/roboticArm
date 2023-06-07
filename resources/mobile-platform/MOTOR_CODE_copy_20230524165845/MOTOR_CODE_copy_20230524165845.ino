int leftin1=8;
int leftin2=9;
int leftin3=10;
int leftin4=13;
int LENPinA = 4;
int LENPinB=5;
int rightin1=12;
int rightin2=11;
int rightin3=6;
int rightin4=7;
int RENPinA=2;
int RENPinB=3;
int SPEED=155;


void setup()
{
  pinMode(4,SPEED);
  pinMode(5,SPEED);
  pinMode(8,OUTPUT);
  pinMode(9,OUTPUT);
  pinMode(10,OUTPUT);
  pinMode(13,OUTPUT);
  pinMode(2,SPEED);
  pinMode(3,SPEED);
  pinMode(12,OUTPUT);
  pinMode(11,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(7,OUTPUT);

}
void foward(){
  analogWrite(LENPinA,SPEED);
  analogWrite(LENPinB,SPEED);
  digitalWrite(leftin1,HIGH);
  digitalWrite(leftin2,LOW);
  digitalWrite(leftin3,LOW);
  digitalWrite(leftin4,HIGH);

  analogWrite(RENPinA,SPEED);
  analogWrite(RENPinB,SPEED);
  digitalWrite(rightin1,LOW);
  digitalWrite(rightin2,HIGH);
  digitalWrite(rightin3,HIGH);
  digitalWrite(rightin4,LOW);

  
}
void turnright(){
  
  analogWrite(LENPinA,SPEED);
  analogWrite(LENPinB,SPEED);
  digitalWrite(leftin1,HIGH);
  digitalWrite(leftin2,LOW);
  digitalWrite(leftin3,HIGH);
  digitalWrite(leftin4,LOW);

  analogWrite(RENPinA,0);
  analogWrite(RENPinB,0);
  digitalWrite(rightin1,LOW);
  digitalWrite(rightin2,LOW);
  digitalWrite(rightin3,LOW);
  digitalWrite(rightin4,LOW);


}
voidturnleft(){
  analogWrite(LENPinA,0);
  analogWrite(LENPinB,0);
  digitalWrite(leftin1,LOW);
  digitalWrite(leftin2,LOW);
  digitalWrite(leftin3,LOW);
  digitalWrite(leftin4,LOW);

  analogWrite(RENPinA,SPEED);
  analogWrite(RENPinB,SPEED);
  digitalWrite(rightin1,LOW);
  digitalWrite(rightin2,HIGH);
  digitalWrite(rightin3,LOW);
  digitalWrite(rightin4,HIGH);
}
voidstop(){
  analogWrite(LENPinA,0);
  analogWrite(LENPinB,0);
  digitalWrite(leftin1,LOW);
  digitalWrite(leftin2,LOW);
  digitalWrite(leftin3,LOW);
  digitalWrite(leftin4,LOW);

  analogWrite(RENPinA,0);
  analogWrite(RENPinB,0);
  digitalWrite(rightin1,LOW);
  digitalWrite(rightin2,LOW);
  digitalWrite(rightin3,LOW);
  digitalWrite(rightin4,LOW);
}
