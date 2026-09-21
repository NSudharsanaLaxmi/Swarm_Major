#ifndef DISPLAY_MANAGER_H
#define DISPLAY_MANAGER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include "Config.h"

class DisplayManager {
public:
  DisplayManager();
  bool init();
  void render(const char* stateStr, float x, float y, float ang, const char* rfidTag, bool wifiOk);

private:
  Adafruit_SSD1306 display;
  bool isOnline;
};

#endif // DISPLAY_MANAGER_H
