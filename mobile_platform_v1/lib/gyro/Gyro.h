#ifndef GYRO_H
#define GYRO_H

#include <Wire.h>

class Gyro {
private:
  const int MPU = 0x68; // MPU6050 I2C address
  float AccX, AccY, AccZ; //linear acceleration
  float GyroX, GyroY, GyroZ; //angular velocity
  float accAngleX, accAngleY, gyroAngleX, gyroAngleY, gyroAngleZ; //used in void loop()
  float roll, pitch, yaw;
  float AccErrorX, AccErrorY, GyroErrorX, GyroErrorY, GyroErrorZ;
  float elapsedTime, currentTime, previousTime;
  int c = 0;

public:
  Gyro();
  void setup();
  void calculateError();
  void readAcceleration();
  void readGyro();
  void setGyroReadings();
};

#endif
