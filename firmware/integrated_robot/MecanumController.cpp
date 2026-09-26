#include "MecanumController.h"

MecanumController::MecanumController()
  : motorDriver(nullptr),
    activeMode(DRIVE_MODE_TESTED_2CHANNEL_SKID_STEER),
    lastVx(0.0f),
    lastVy(0.0f),
    lastOmega(0.0f) {
  currentWheelVelocities = {0.0f, 0.0f, 0.0f, 0.0f};
}

void MecanumController::init(MotorDriver* driver) {
  motorDriver = driver;
  stop();
  Serial.println(F("[MECANUM_CONTROLLER] Locomotion Kinematics Layer Initialized in ACTIVE 2-Channel Spliced Mode."));
}

void MecanumController::driveVelocity(float vx, float vy, float omega) {
  lastVx = constrain(vx, -1.0f, 1.0f);
  lastVy = constrain(vy, -1.0f, 1.0f);
  lastOmega = constrain(omega, -1.0f, 1.0f);

  if (activeMode == DRIVE_MODE_TESTED_2CHANNEL_SKID_STEER) {
    executeTestedSkidSteer(lastVx, lastOmega);
  } else {
    // Compute full 4-wheel independent inverse kinematics
    computeMecanumIK(lastVx, lastVy, lastOmega, currentWheelVelocities);
    // Note: In 2-channel hardware wiring, FL/RL and FR/RR are paired
    executeTestedSkidSteer(lastVx, lastOmega);
  }
}

void MecanumController::executeTestedSkidSteer(float vx, float omega) {
  if (!motorDriver) return;

  // Differential / Skid-steer kinematics mapping
  float v_left  = vx - (omega * ROBOT_WHEEL_BASE_M / 2.0f);
  float v_right = vx + (omega * ROBOT_WHEEL_BASE_M / 2.0f);

  v_left  = constrain(v_left, -1.0f, 1.0f);
  v_right = constrain(v_right, -1.0f, 1.0f);

  // Apply deadband
  int leftPwm  = (fabs(v_left) > 0.05f) ? (int)(v_left * 255.0f) : 0;
  int rightPwm = (fabs(v_right) > 0.05f) ? (int)(v_right * 255.0f) : 0;

  currentWheelVelocities.frontLeft  = v_left;
  currentWheelVelocities.rearLeft   = v_left;
  currentWheelVelocities.frontRight = v_right;
  currentWheelVelocities.rearRight  = v_right;

  motorDriver->setRawMotors(leftPwm, rightPwm);
}

void MecanumController::computeMecanumIK(float vx, float vy, float omega, WheelVelocities& out) {
  // Kinematic parameters: L = half length + half width
  float k = ROBOT_WHEEL_BASE_M;

  out.frontLeft  = vx - vy - (omega * k);
  out.frontRight = vx + vy + (omega * k);
  out.rearLeft   = vx + vy - (omega * k);
  out.rearRight  = vx - vy + (omega * k);

  // Normalize if any velocity exceeds 1.0
  float maxVal = max(max(fabs(out.frontLeft), fabs(out.frontRight)),
                     max(fabs(out.rearLeft), fabs(out.rearRight)));

  if (maxVal > 1.0f) {
    out.frontLeft  /= maxVal;
    out.frontRight /= maxVal;
    out.rearLeft   /= maxVal;
    out.rearRight  /= maxVal;
  }
}

void MecanumController::stop() {
  lastVx = 0.0f;
  lastVy = 0.0f;
  lastOmega = 0.0f;
  currentWheelVelocities = {0.0f, 0.0f, 0.0f, 0.0f};

  if (motorDriver) {
    motorDriver->stop();
  }
}
