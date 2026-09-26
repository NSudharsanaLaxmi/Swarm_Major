#include "ArmController.h"

ArmController::ArmController()
  : pca(Adafruit_PWMServoDriver(ADDR_PCA9685)),
    driverOnline(false),
    moving(false),
    lastStepTime(0),
    moveStartTime(0),
    totalMoveDurationMs(800),
    currentNamedPose(POSE_TYPE_HOME) {
  for (int i = 0; i < NUM_ARM_JOINTS; i++) {
    jointConfigs[i] = DEFAULT_JOINT_CONFIGS[i];
    currentAngles[i] = (float)DEFAULT_JOINT_CONFIGS[i].homeAngle;
    targetAngles[i]  = currentAngles[i];
    stepIncrements[i] = 0.0f;
  }
}

bool ArmController::begin() {
  Wire.beginTransmission(ADDR_PCA9685);
  byte err = Wire.endTransmission();

  if (err != 0) {
    Serial.printf("[ARM_CONTROLLER] ERROR: PCA9685 not detected at I2C address 0x%02X!\n", ADDR_PCA9685);
    driverOnline = false;
    return false;
  }

  pca.begin();
  pca.setPWMFreq(PCA9685_PWM_FREQ); // 50 Hz for MG996R servos
  driverOnline = true;

  // Initialize all joints to home position
  for (int i = 0; i < NUM_ARM_JOINTS; i++) {
    currentAngles[i] = (float)jointConfigs[i].homeAngle;
    targetAngles[i]  = currentAngles[i];
    applyHardwarePwm(i, currentAngles[i]);
  }

  Serial.println(F("[ARM_CONTROLLER] PCA9685 5-DOF MG996R Arm Controller Online (CH0-CH4). Initialized to HOME."));
  return true;
}

void ArmController::setJointConfig(uint8_t jointIndex, const JointConfig& config) {
  if (jointIndex < NUM_ARM_JOINTS) {
    jointConfigs[jointIndex] = config;
  }
}

JointConfig ArmController::getJointConfig(uint8_t jointIndex) const {
  if (jointIndex < NUM_ARM_JOINTS) {
    return jointConfigs[jointIndex];
  }
  return DEFAULT_JOINT_CONFIGS[0];
}

int ArmController::angleToPulse(uint8_t jointIndex, float angle) {
  // Apply calibration offset & direction
  float calibratedAngle = (angle * jointConfigs[jointIndex].direction) + jointConfigs[jointIndex].offset;
  calibratedAngle = constrain(calibratedAngle, 0.0f, 180.0f);

  // Map 0-180 degrees to servo pulse width (typically 150-600 on PCA9685 12-bit scale)
  int pulse = map((long)calibratedAngle, 0, 180, SERVO_PULSE_MIN, SERVO_PULSE_MAX);
  return pulse;
}

void ArmController::applyHardwarePwm(uint8_t jointIndex, float angle) {
  if (!driverOnline || jointIndex >= NUM_ARM_JOINTS) return;
  int pulse = angleToPulse(jointIndex, angle);
  pca.setPWM(jointConfigs[jointIndex].channel, 0, pulse);
}

void ArmController::setJointAngle(uint8_t jointIndex, int targetAngle) {
  if (jointIndex >= NUM_ARM_JOINTS) return;

  // Clamp to software limit
  int clamped = constrain(targetAngle, jointConfigs[jointIndex].minAngle, jointConfigs[jointIndex].maxAngle);
  targetAngles[jointIndex] = (float)clamped;
  currentNamedPose = POSE_TYPE_CUSTOM;

  // Calculate increment for non-blocking move
  float delta = targetAngles[jointIndex] - currentAngles[jointIndex];
  stepIncrements[jointIndex] = delta / 20.0f; // 20 steps
  moving = true;
  lastStepTime = millis();
}

void ArmController::movePose(int base, int shoulder, int elbow, int joint4, int joint5, int durationMs) {
  int targets[NUM_ARM_JOINTS] = { base, shoulder, elbow, joint4, joint5 };

  for (int i = 0; i < NUM_ARM_JOINTS; i++) {
    int clamped = constrain(targets[i], jointConfigs[i].minAngle, jointConfigs[i].maxAngle);
    targetAngles[i] = (float)clamped;
  }

  totalMoveDurationMs = max(100, durationMs);
  moveStartTime = millis();
  lastStepTime = millis();

  int numSteps = max(5, totalMoveDurationMs / 20); // 20ms update period (~50 Hz)
  for (int i = 0; i < NUM_ARM_JOINTS; i++) {
    stepIncrements[i] = (targetAngles[i] - currentAngles[i]) / (float)numSteps;
  }

  moving = true;
  currentNamedPose = POSE_TYPE_CUSTOM;
}

void ArmController::movePose(const ArmPose& pose, int durationMs) {
  movePose(pose.base, pose.shoulder, pose.elbow, pose.joint4, pose.joint5, durationMs);
}

void ArmController::setNamedPose(ArmPoseType poseType, int durationMs) {
  if (poseType >= 0 && poseType < 7) {
    currentNamedPose = poseType;
    movePose(DEFAULT_ARM_POSES[poseType], durationMs);
  }
}

void ArmController::home(int durationMs) {
  setNamedPose(POSE_TYPE_HOME, durationMs);
}

void ArmController::stop() {
  for (int i = 0; i < NUM_ARM_JOINTS; i++) {
    targetAngles[i] = currentAngles[i];
    stepIncrements[i] = 0.0f;
  }
  moving = false;
}

void ArmController::openGripper() {
  setJointAngle(SERVO_JOINT5, jointConfigs[SERVO_JOINT5].maxAngle); // 180°
}

void ArmController::closeGripper() {
  setJointAngle(SERVO_JOINT5, jointConfigs[SERVO_JOINT5].minAngle); // 45°
}

void ArmController::update() {
  if (!driverOnline || !moving) return;

  unsigned long now = millis();
  if (now - lastStepTime < 20) return; // 50 Hz interpolation loop
  lastStepTime = now;

  bool allReached = true;

  for (int i = 0; i < NUM_ARM_JOINTS; i++) {
    float diff = targetAngles[i] - currentAngles[i];
    if (fabs(diff) > fabs(stepIncrements[i])) {
      currentAngles[i] += stepIncrements[i];
      allReached = false;
    } else {
      currentAngles[i] = targetAngles[i];
    }
    applyHardwarePwm(i, currentAngles[i]);
  }

  if (allReached) {
    moving = false;
  }
}

int ArmController::getCurrentAngle(uint8_t jointIndex) const {
  if (jointIndex < NUM_ARM_JOINTS) {
    return (int)round(currentAngles[jointIndex]);
  }
  return 90;
}

ArmTelemetry ArmController::getTelemetry() const {
  ArmTelemetry telem;
  telem.baseAngle      = (int)round(currentAngles[0]);
  telem.shoulderAngle  = (int)round(currentAngles[1]);
  telem.elbowAngle     = (int)round(currentAngles[2]);
  telem.joint4Angle    = (int)round(currentAngles[3]);
  telem.joint5Angle    = (int)round(currentAngles[4]);
  telem.isMoving       = moving;
  telem.activePoseType = currentNamedPose;
  telem.isOnline       = driverOnline;
  return telem;
}
