#ifndef TELEMETRY_H
#define TELEMETRY_H

#include <Arduino.h>
#include <ArduinoJson.h>
#include "Config.h"
#include "SwarmComms.h"
#include "MotorDriver.h"
#include "MecanumController.h"
#include "ArmController.h"
#include "SensorSuite.h"
#include "SafetyController.h"
#include "MissionController.h"

class Telemetry {
public:
  Telemetry();
  void init(SwarmComms* comms,
            MotorDriver* motors,
            MecanumController* mecanum,
            ArmController* arm,
            SensorSuite* sensors,
            SafetyController* safety,
            MissionController* mission);

  // Periodic telemetry broadcast tick (typically 10 Hz)
  void update();

  // Generate serialized JSON string on demand
  String buildJsonPayload();

private:
  SwarmComms* commsModule;
  MotorDriver* motorDriver;
  MecanumController* mecanumController;
  ArmController* armController;
  SensorSuite* sensorSuite;
  SafetyController* safetyController;
  MissionController* missionController;

  unsigned long lastTelemetryBroadcast;
};

#endif // TELEMETRY_H
