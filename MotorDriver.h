#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include "Config.h"

class MotorDriver {
public:
  MotorDriver();
  void init();
  void drive(float linear, float angular);
  void setRawMotors(int leftPwm, int rightPwm);
  void stop();

private:
  int targetLeftPwm;
  int targetRightPwm;
  int currentLeftPwm;
  int currentRightPwm;

  void applyMotorPolarity(int left, int right);
};

#endif // MOTOR_DRIVER_H
