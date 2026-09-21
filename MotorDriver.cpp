#include "MotorDriver.h"

MotorDriver::MotorDriver() : targetLeftPwm(0), targetRightPwm(0), currentLeftPwm(0), currentRightPwm(0) {}

void MotorDriver::init() {
  pinMode(PIN_PWM_LEFT, OUTPUT);
  pinMode(PIN_PWM_RIGHT, OUTPUT);

  pinMode(PIN_F_AIN1, OUTPUT); pinMode(PIN_F_AIN2, OUTPUT);
  pinMode(PIN_F_BIN1, OUTPUT); pinMode(PIN_F_BIN2, OUTPUT);
  pinMode(PIN_R_AIN1, OUTPUT); pinMode(PIN_R_AIN2, OUTPUT);
  pinMode(PIN_R_BIN1, OUTPUT); pinMode(PIN_R_BIN2, OUTPUT);

  stop();
  Serial.println(F("[MOTOR] 4WD Locomotion Subsystem Initialized."));
}

void MotorDriver::applyMotorPolarity(int left, int right) {
  // Left Side Direction Control (Front & Rear)
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

  // Right Side Direction Control (Front & Rear)
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

  // Output 8-bit PWM
  analogWrite(PIN_PWM_LEFT, constrain(abs(left), 0, 255));
  analogWrite(PIN_PWM_RIGHT, constrain(abs(right), 0, 255));
}

void MotorDriver::drive(float linear, float angular) {
  float left_speed  = linear - angular;
  float right_speed = linear + angular;

  left_speed  = constrain(left_speed, -1.0f, 1.0f);
  right_speed = constrain(right_speed, -1.0f, 1.0f);

  int left_pwm  = (abs(left_speed) > 0.05f) ? (int)(left_speed * 255.0f) : 0;
  int right_pwm = (abs(right_speed) > 0.05f) ? (int)(right_speed * 255.0f) : 0;

  setRawMotors(left_pwm, right_pwm);
}

void MotorDriver::setRawMotors(int leftPwm, int rightPwm) {
  targetLeftPwm = leftPwm;
  targetRightPwm = rightPwm;
  applyMotorPolarity(targetLeftPwm, targetRightPwm);
}

void MotorDriver::stop() {
  setRawMotors(0, 0);
}
