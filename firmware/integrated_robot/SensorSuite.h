#ifndef SENSOR_SUITE_H
#define SENSOR_SUITE_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <VL53L0X.h>
#include <MFRC522.h>
#include "Config.h"

class SensorSuite {
public:
  SensorSuite();
  void init();
  void update();

  uint16_t getTofDistanceMM();
  float getUltrasonicDistanceCM();
  String getLastScannedRFID();
  bool isForwardPathBlocked();

private:
  VL53L0X tof;
  MFRC522 rfid;

  bool tofReady;
  bool rfidReady;

  uint16_t currentTofDistMM;
  float currentUltrasonicCM;
  String currentRFIDTag;
  unsigned long lastSensorPoll;

  float readUltrasonicRaw();
  String readRFIDRaw();
};

#endif // SENSOR_SUITE_H
