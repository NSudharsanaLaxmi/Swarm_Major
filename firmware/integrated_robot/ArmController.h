#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Config.h"

struct ArmTelemetry {
  int baseAngle;
  int shoulderAngle;
  int elbowAngle;
  int joint4Angle;
  int joint5Angle;
  bool isMoving;
  ArmPoseType activePoseType;
  bool isOnline;
};

class ArmController {
public:
  ArmController();
  bool begin();

  // Non-blocking tick for smooth trajectory interpolation
  void update();

  // Individual joint control (clamped to configured limits)
  void setJointAngle(uint8_t jointIndex, int targetAngle);

  // Full 5-DOF pose transition with duration-based interpolation
  void movePose(int base, int shoulder, int elbow, int joint4, int joint5, int durationMs = 800);
  void movePose(const ArmPose& pose, int durationMs = 800);

  // Named mission pose execution
  void setNamedPose(ArmPoseType poseType, int durationMs = 800);

  // Home and emergency stop
  void home(int durationMs = 1000);
  void stop();

  // Gripper convenience helpers
  void openGripper();
  void closeGripper();

  // Joint calibration & limits
  void setJointConfig(uint8_t jointIndex, const JointConfig& config);
  JointConfig getJointConfig(uint8_t jointIndex) const;

  // Status & Telemetry
  bool isMoving() const { return moving; }
  bool isOnline() const { return driverOnline; }
  ArmTelemetry getTelemetry() const;
  int getCurrentAngle(uint8_t jointIndex) const;

private:
  Adafruit_PWMServoDriver pca;
  bool driverOnline;
  JointConfig jointConfigs[NUM_ARM_JOINTS];

  float currentAngles[NUM_ARM_JOINTS];
  float targetAngles[NUM_ARM_JOINTS];
  float stepIncrements[NUM_ARM_JOINTS];

  bool moving;
  unsigned long lastStepTime;
  unsigned long moveStartTime;
  int totalMoveDurationMs;
  ArmPoseType currentNamedPose;

  int angleToPulse(uint8_t jointIndex, float angle);
  void applyHardwarePwm(uint8_t jointIndex, float angle);
};

#endif // ARM_CONTROLLER_H
