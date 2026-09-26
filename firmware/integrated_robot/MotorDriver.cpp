#include "MotorDriver.h"

MotorDriver::MotorDriver() : targetLeftPwm(0), targetRightPwm(0), currentLeftPwm(0), currentRightPwm(0) {}

void MotorDriver::init() {
  // Configure native ESP32 PWM pins
  pinMode(PIN_PWM_LEFT, OUTPUT);
  pinMode(PIN_PWM_RIGHT, OUTPUT);

  // Configure Front Driver direction pins (TB6612 #1)
  pinMode(PIN_F_AIN1, OUTPUT);
  pinMode(PIN_F_AIN2, OUTPUT);
  pinMode(PIN_F_BIN1, OUTPUT);
  pinMode(PIN_F_BIN2, OUTPUT);

  // Configure Rear Driver direction pins (TB6612 #2)
  pinMode(PIN_R_AIN1, OUTPUT);
  pinMode(PIN_R_AIN2, OUTPUT);
  pinMode(PIN_R_BIN1, OUTPUT);
  pinMode(PIN_R_BIN2, OUTPUT);

  stop();
  Serial.println(F("[MOTOR_DRIVER] 4WD TB6612 Subsystem Initialized (Pins D4/D5 PWM, D25/26/27/14 & D12/13/32/33 Dirs)."));
}

void MotorDriver::applyMotorPolarity(int left, int right) {
  // --- LEFT SIDE MOTORS (Front Left & Rear Left) ---
  if (left > 0) {
    digitalWrite(PIN_F_AIN1, HIGH); digitalWrite(PIN_F_AIN2, LOW);
    digitalWrite(PIN_R_AIN1, HIGH); digitalWrite(PIN_R_AIN2, LOW);
  } else if (left < 0) {
    digitalWrite(PIN_F_AIN1, LOW); digitalWrite(PIN_F_AIN2, HIGH);
    digitalWrite(PIN_R_AIN1, LOW); digitalWrite(PIN_R_AIN2, HIGH);
  } else {
    digitalWrite(PIN_F_AIN1, LOW); digitalWrite(PIN_F_AIN2, LOW);
    digitalWrite(PIN_R_AIN1, LOW); digitalWrite(PIN_R_AIN2, LOW);
  }

  // --- RIGHT SIDE MOTORS (Front Right & Rear Right) ---
  if (right > 0) {
    digitalWrite(PIN_F_BIN1, HIGH); digitalWrite(PIN_F_BIN2, LOW);
    digitalWrite(PIN_R_BIN1, HIGH); digitalWrite(PIN_R_BIN2, LOW);
  } else if (right < 0) {
    digitalWrite(PIN_F_BIN1, LOW); digitalWrite(PIN_F_BIN2, HIGH);
    digitalWrite(PIN_R_BIN1, LOW); digitalWrite(PIN_R_BIN2, HIGH);
  } else {
    digitalWrite(PIN_F_BIN1, LOW); digitalWrite(PIN_F_BIN2, LOW);
    digitalWrite(PIN_R_BIN1, LOW); digitalWrite(PIN_R_BIN2, LOW);
  }

  // Output 8-bit native PWM directly from ESP32 pins 4 and 5
  int pwmLeftVal  = constrain(abs(left), 0, 255);
  int pwmRightVal = constrain(abs(right), 0, 255);

  analogWrite(PIN_PWM_LEFT, pwmLeftVal);
  analogWrite(PIN_PWM_RIGHT, pwmRightVal);

  currentLeftPwm  = left;
  currentRightPwm = right;
}

void MotorDriver::setRawMotors(int leftPwm, int rightPwm) {
  targetLeftPwm  = constrain(leftPwm, -255, 255);
  targetRightPwm = constrain(rightPwm, -255, 255);
  applyMotorPolarity(targetLeftPwm, targetRightPwm);
}

void MotorDriver::stop() {
  digitalWrite(PIN_F_AIN1, LOW); digitalWrite(PIN_F_AIN2, LOW);
  digitalWrite(PIN_R_AIN1, LOW); digitalWrite(PIN_R_AIN2, LOW);
  digitalWrite(PIN_F_BIN1, LOW); digitalWrite(PIN_F_BIN2, LOW);
  digitalWrite(PIN_R_BIN1, LOW); digitalWrite(PIN_R_BIN2, LOW);

  analogWrite(PIN_PWM_LEFT, 0);
  analogWrite(PIN_PWM_RIGHT, 0);

  targetLeftPwm   = 0;
  targetRightPwm  = 0;
  currentLeftPwm  = 0;
  currentRightPwm = 0;
}
