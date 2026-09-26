#ifndef MECANUM_CONTROLLER_H
#define MECANUM_CONTROLLER_H

#include <Arduino.h>
#include "Config.h"
#include "MotorDriver.h"

enum DriveMode {
  DRIVE_MODE_TESTED_2CHANNEL_SKID_STEER,  // ACTIVE: Spliced Left/Right PWM on GPIO 4/5
  DRIVE_MODE_INDEPENDENT_4W_MECANUM       // FUTURE: 4 independent PWM channels for omnidirectional strafing
};

struct WheelVelocities {
  float frontLeft;
  float frontRight;
  float rearLeft;
  float rearRight;
};

class MecanumController {
public:
  MecanumController();
  void init(MotorDriver* driver);

  // Command robot velocity in body frame (vx: forward/back, vy: lateral strafe, omega: yaw rotation)
  void driveVelocity(float vx, float vy, float omega);

  // Stop locomotion immediately
  void stop();

  // Mode configuration
  void setDriveMode(DriveMode mode) { activeMode = mode; }
  DriveMode getDriveMode() const { return activeMode; }

  // Telemetry accessors
  WheelVelocities getWheelCommands() const { return currentWheelVelocities; }
  float getLastVx() const { return lastVx; }
  float getLastVy() const { return lastVy; }
  float getLastOmega() const { return lastOmega; }

private:
  MotorDriver* motorDriver;
  DriveMode activeMode;

  WheelVelocities currentWheelVelocities;
  float lastVx;
  float lastVy;
  float lastOmega;

  void executeTestedSkidSteer(float vx, float omega);
  void computeMecanumIK(float vx, float vy, float omega, WheelVelocities& out);
};

#endif // MECANUM_CONTROLLER_H
