#ifndef MobilePlatform_h
#define MobilePlatform_h

#include <Arduino.h>
#include "Preferences.h"

class MobilePlatform {
  private:
    bool turning = false;
    void rotateMotor(int rightMotorSpeed, int leftMotorSpeed);
    void setMotorState(int motorPin1, int motorPin2, int motorSpeed);
    void handleFarTurnValues();
    void handleSensorValues();
    void configurePins();
    void configurePWMFrequency();

  public:
    MobilePlatform();
    void setup();
    void loop();
};

#endif
