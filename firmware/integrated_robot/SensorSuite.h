#ifndef SENSOR_SUITE_H
#define SENSOR_SUITE_H

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <VL53L0X.h>
#include <MFRC522.h>
#include "Config.h"

struct SensorTelemetry {
  uint16_t tofDistanceMm;
  float ultrasonicDistanceCm;
  String lastRfidTag;
  bool obstacleDetected;
  bool tofOnline;
  bool rfidOnline;
};

class SensorSuite {
public:
  SensorSuite();
  void init();
  void update();

  uint16_t getTofDistanceMM() const { return currentTofDistMM; }
  float getUltrasonicDistanceCM() const { return currentUltrasonicCM; }
  String getLastScannedRFID() const { return currentRFIDTag; }
  void clearRFID() { currentRFIDTag = ""; }

  bool isForwardPathBlocked() const;
  bool isDockingClearanceReached() const;

  SensorTelemetry getTelemetry() const;

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
