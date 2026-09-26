#include "NavigationController.h"

NavigationController::NavigationController()
  : mecanumController(nullptr),
    goalReached(true),
    currentDistanceToGoal(0.0f),
    currentHeadingError(0.0f) {
  activeGoal = { 0.0f, 0.0f, 0.0f, false };
}

void NavigationController::init(MecanumController* mecanum) {
  mecanumController = mecanum;
  clearGoal();
  Serial.println(F("[NAVIGATION_CONTROLLER] Closed-Loop Navigation Subsystem Initialized."));
}

void NavigationController::setGoal(float targetX, float targetY, float targetTheta) {
  activeGoal.x      = targetX;
  activeGoal.y      = targetY;
  activeGoal.theta  = targetTheta;
  activeGoal.active = true;
  goalReached       = false;
  Serial.printf("[NAVIGATION] New Autonomous Goal Set: Target (%.1f, %.1f) cm | Theta: %.1f deg\n",
                targetX, targetY, targetTheta);
}

void NavigationController::clearGoal() {
  activeGoal.active = false;
  goalReached       = true;
  if (mecanumController) {
    mecanumController->stop();
  }
}

float NavigationController::normalizeAngle(float angleDeg) {
  while (angleDeg > 180.0f)  angleDeg -= 360.0f;
  while (angleDeg < -180.0f) angleDeg += 360.0f;
  return angleDeg;
}

bool NavigationController::update(const GlobalPose& currentPose) {
  if (!activeGoal.active || !mecanumController) {
    return false;
  }

  if (!currentPose.valid) {
    // Cannot navigate without valid localization
    mecanumController->stop();
    return false;
  }

  float dx = activeGoal.x - currentPose.x;
  float dy = activeGoal.y - currentPose.y;
  currentDistanceToGoal = sqrt(dx * dx + dy * dy);

  // Check Arrival condition
  if (currentDistanceToGoal <= ARRIVAL_THRESH_CM) {
    mecanumController->stop();
    goalReached = true;
    activeGoal.active = false;
    Serial.printf("[NAVIGATION] Goal Arrived at (%.1f, %.1f) cm! Remaining error: %.1f cm\n",
                  currentPose.x, currentPose.y, currentDistanceToGoal);
    return true;
  }

  // Calculate desired heading angle to target waypoint
  float desiredAngle = atan2(dy, dx) * 180.0f / M_PI;
  currentHeadingError = normalizeAngle(desiredAngle - currentPose.theta);

  // Guidance control: Pivot in place if heading error is large; drive forward with trim if aligned
  if (fabs(currentHeadingError) > HEADING_DEADBAND_DEG) {
    // In-place pivot rotation
    float rotSpeed = (currentHeadingError > 0) ? 0.55f : -0.55f;
    mecanumController->driveVelocity(0.0f, 0.0f, rotSpeed);
  } else {
    // Proportional forward speed tracking with heading steering trim
    float linearSpeed = ROBOT_MAX_SPEED_MPS;
    // Scale speed down near goal
    if (currentDistanceToGoal < 25.0f) {
      linearSpeed = map((long)currentDistanceToGoal, (long)ARRIVAL_THRESH_CM, 25, 18, 45) / 100.0f;
    }
    float steerTrim = (currentHeadingError / HEADING_DEADBAND_DEG) * 0.35f;
    mecanumController->driveVelocity(linearSpeed, 0.0f, steerTrim);
  }

  return false;
}

void NavigationController::stop() {
  if (mecanumController) {
    mecanumController->stop();
  }
}
