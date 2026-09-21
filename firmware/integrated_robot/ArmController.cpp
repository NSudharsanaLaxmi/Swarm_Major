#include "ArmController.h"

ArmController::ArmController() : pca(ADDR_PCA9685), isOnline(false) {
  for (int i = 0; i < 5; i++) currentAngles[i] = 90;
}

int ArmController::angleToPulse(int angle) {
  angle = constrain(angle, 0, 180);
  return map(angle, 0, 180, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
}

bool ArmController::init() {
  Wire.beginTransmission(ADDR_PCA9685);
  if (Wire.endTransmission() != 0) {
    Serial.println(F("[ERROR][ARM] PCA9685 not detected at 0x40!"));
    isOnline = false;
    return false;
  }

  pca.begin();
  pca.setPWMFreq(50); // 50 Hz for analog/digital hobby servos (MG90S)
  isOnline = true;

  setPose(POSE_REST, 400);
  openGripper();
  Serial.println(F("[ARM] Manipulator initialized to REST pose."));
  return true;
}

void ArmController::setServoAngle(uint8_t channel, int angle) {
  if (!isOnline) return;
  pca.setPWM(channel, 0, angleToPulse(angle));
  if (channel < 5) currentAngles[channel] = angle;
}

void ArmController::openGripper() {
  setServoAngle(SERVO_CH_GRIPPER, GRIPPER_OPEN_DEG);
}

void ArmController::closeGripper() {
  setServoAngle(SERVO_CH_GRIPPER, GRIPPER_CLOSED_DEG);
}

void ArmController::interpolatePose(int targetBase, int targetShoulder, int targetElbow, int targetWrist, int durationMs) {
  if (!isOnline) return;

  int steps = max(10, durationMs / 20);
  float startBase     = currentAngles[SERVO_CH_BASE];
  float startShoulder = currentAngles[SERVO_CH_SHOULDER];
  float startElbow    = currentAngles[SERVO_CH_ELBOW];
  float startWrist    = currentAngles[SERVO_CH_WRIST];

  for (int s = 1; s <= steps; s++) {
    float t = (float)s / (float)steps;
    setServoAngle(SERVO_CH_BASE,     (int)(startBase     + t * (targetBase - startBase)));
    setServoAngle(SERVO_CH_SHOULDER, (int)(startShoulder + t * (targetShoulder - startShoulder)));
    setServoAngle(SERVO_CH_ELBOW,    (int)(startElbow    + t * (targetElbow - startElbow)));
    setServoAngle(SERVO_CH_WRIST,    (int)(startWrist    + t * (targetWrist - startWrist)));
    delay(20);
  }
}

void ArmController::setPose(ArmPose pose, int durationMs) {
  switch (pose) {
    case POSE_REST:
      // Folded back to isolate center of mass during high-speed transit
      interpolatePose(90, 30, 150, 80, durationMs);
      break;

    case POSE_HOVER:
      // Raised and looking forward towards target payload
      interpolatePose(90, 75, 90, 90, durationMs);
      break;

    case POSE_PICK:
      // Lowered to floor level / conveyor level
      interpolatePose(90, 110, 65, 45, durationMs);
      break;

    case POSE_LIFT:
      // Lifted with payload secured in claw
      interpolatePose(90, 60, 110, 80, durationMs);
      break;

    case POSE_DROP:
      // Extended over target delivery drop bucket
      interpolatePose(90, 95, 80, 60, durationMs);
      break;
  }
}
