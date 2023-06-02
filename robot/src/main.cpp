#include <Arduino.h>
#include "MobilePlatform.h"
#include "Preferences.h"

MobilePlatform platform;

void setup() 
{
  platform.setup();
  Serial.begin(9600);
  
  delay(20000);
}

void loop() 
{
  platform.loop();
}
