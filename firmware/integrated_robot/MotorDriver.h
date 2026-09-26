#ifndef MOTOR_DRIVER_H
#define MOTOR_DRIVER_H

#include <Arduino.h>
#include "Config.h"

class MotorDriver {
public:
  MotorDriver();
  void init();
  
  // Set Left and Right raw PWM values (-255 to +255)
  void setRawMotors(int leftPwm, int rightPwm);
  
  // Emergency stop and cut all PWM/direction lines
  void stop();
  
  // Status accessors
  int getLeftPwm() const { return currentLeftPwm; }
  int getRightPwm() const { return currentRightPwm; }
  bool isStopped() const { return (currentLeftPwm == 0 && currentRightPwm == 0); }

private:
  int targetLeftPwm;
  int targetRightPwm;
  int currentLeftPwm;
  int currentRightPwm;

  void applyMotorPolarity(int left, int right);
};

#endif // MOTOR_DRIVER_H
