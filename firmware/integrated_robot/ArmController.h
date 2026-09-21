#ifndef ARM_CONTROLLER_H
#define ARM_CONTROLLER_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include "Config.h"

enum ArmPose {
  POSE_REST,
  POSE_HOVER,
  POSE_PICK,
  POSE_LIFT,
  POSE_DROP
};

class ArmController {
public:
  ArmController();
  bool init();
  void setServoAngle(uint8_t channel, int angle);
  void setPose(ArmPose pose, int durationMs = 600);
  void openGripper();
  void closeGripper();

private:
  Adafruit_PWMServoDriver pca;
  bool isOnline;
  int currentAngles[5];

  int angleToPulse(int angle);
  void interpolatePose(int targetBase, int targetShoulder, int targetElbow, int targetWrist, int durationMs);
};

#endif // ARM_CONTROLLER_H
