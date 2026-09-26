#include "SafetyController.h"

SafetyController::SafetyController()
  : motorDriver(nullptr),
    armController(nullptr),
    emergencyStopLatched(false),
    obstacleBrakeActive(false),
    swarmYieldActive(false),
    commTimeoutActive(false),
    currentStatus(SAFETY_STATUS_NOMINAL) {}

void SafetyController::init(MotorDriver* motors, ArmController* arm) {
  motorDriver   = motors;
  armController = arm;
  emergencyStopLatched = false;
  currentStatus = SAFETY_STATUS_NOMINAL;
  Serial.println(F("[SAFETY_CONTROLLER] Failsafe & Interlock System Initialized (500ms Timeout, <120mm ToF Brake, E-Stop)."));
}

void SafetyController::triggerEmergencyStop() {
  emergencyStopLatched = true;
  currentStatus = SAFETY_STATUS_EMERGENCY_STOP;

  if (motorDriver) {
    motorDriver->stop();
  }
  if (armController) {
    armController->stop();
  }
  Serial.println(F("[SAFETY_CONTROLLER] !!! CRITICAL EMERGENCY STOP TRIGGERED !!! All Actuators Halted."));
}

void SafetyController::resumeFromEmergencyStop() {
  emergencyStopLatched = false;
  currentStatus = SAFETY_STATUS_NOMINAL;
  Serial.println(F("[SAFETY_CONTROLLER] Emergency Stop Cleared. System Resumed."));
}

SafetyStatus SafetyController::evaluate(const SensorSuite& sensors, const SwarmComms& comms, bool allowCloseDocking) {
  // --- 1. HIGHEST PRIORITY: EMERGENCY STOP LATCH ---
  if (emergencyStopLatched) {
    if (motorDriver) motorDriver->stop();
    if (armController) armController->stop();
    currentStatus = SAFETY_STATUS_EMERGENCY_STOP;
    return currentStatus;
  }

  // --- 2. FORWARD OBSTACLE BRAKING ---
  if (!allowCloseDocking && sensors.isForwardPathBlocked()) {
    if (motorDriver) motorDriver->stop();
    obstacleBrakeActive = true;
    currentStatus = SAFETY_STATUS_OBSTACLE_BRAKE;
    return currentStatus;
  }
  obstacleBrakeActive = false;

  // --- 3. SWARM COLLISION RIGHT-OF-WAY YIELDING ---
  if (comms.isPeerNear(SWARM_SAFE_DIST_CM)) {
    // Priority Rule: Lower ROBOT_ID has right-of-way; Higher ROBOT_ID yields
    if (ROBOT_ID > PEER_ROBOT_ID && !allowCloseDocking) {
      if (motorDriver) motorDriver->stop();
      swarmYieldActive = true;
      currentStatus = SAFETY_STATUS_SWARM_YIELD;
      return currentStatus;
    }
  }
  swarmYieldActive = false;

  // --- 4. HARDWARE/COMMUNICATION TIMEOUT ---
  if (comms.isCommandTimedOut()) {
    commTimeoutActive = true;
    currentStatus = SAFETY_STATUS_COMM_TIMEOUT;
    return currentStatus;
  }
  commTimeoutActive = false;

  currentStatus = SAFETY_STATUS_NOMINAL;
  return currentStatus;
}
