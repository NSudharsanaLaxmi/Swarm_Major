#ifndef SAFETY_CONTROLLER_H
#define SAFETY_CONTROLLER_H

#include <Arduino.h>
#include "Config.h"
#include "MotorDriver.h"
#include "ArmController.h"
#include "SensorSuite.h"
#include "SwarmComms.h"

enum SafetyStatus {
  SAFETY_STATUS_NOMINAL,
  SAFETY_STATUS_OBSTACLE_BRAKE,
  SAFETY_STATUS_SWARM_YIELD,
  SAFETY_STATUS_COMM_TIMEOUT,
  SAFETY_STATUS_EMERGENCY_STOP
};

class SafetyController {
public:
  SafetyController();
  void init(MotorDriver* motors, ArmController* arm);

  void triggerEmergencyStop();
  void resumeFromEmergencyStop();

  // Evaluates prioritized safety state and applies immediate failsafes
  SafetyStatus evaluate(const SensorSuite& sensors, const SwarmComms& comms, bool allowCloseDocking = false);

  bool isEmergencyStopped() const { return emergencyStopLatched; }
  bool isObstacleBraked() const { return obstacleBrakeActive; }
  bool isSwarmYielding() const { return swarmYieldActive; }
  SafetyStatus getStatus() const { return currentStatus; }

private:
  MotorDriver* motorDriver;
  ArmController* armController;

  bool emergencyStopLatched;
  bool obstacleBrakeActive;
  bool swarmYieldActive;
  bool commTimeoutActive;

  SafetyStatus currentStatus;
};

#endif // SAFETY_CONTROLLER_H
