#ifndef MISSION_CONTROLLER_H
#define MISSION_CONTROLLER_H

#include <Arduino.h>
#include "Config.h"
#include "NavigationController.h"
#include "ArmController.h"
#include "SensorSuite.h"
#include "SafetyController.h"
#include "MecanumController.h"

class MissionController {
public:
  MissionController();
  void init(NavigationController* nav,
            ArmController* arm,
            SensorSuite* sensors,
            SafetyController* safety,
            MecanumController* mecanum);

  // Non-blocking state machine tick executed in loop()
  void update(const GlobalPose& currentPose);

  // External commands
  void startMission(float pickX, float pickY, float dropX, float dropY);
  void abortMission();
  void setManualMissionState(RobotMissionState state);

  RobotMissionState getCurrentState() const { return currentState; }
  const char* getStateName() const;

private:
  NavigationController* navigation;
  ArmController* arm;
  SensorSuite* sensors;
  SafetyController* safety;
  MecanumController* mecanum;

  RobotMissionState currentState;
  RobotMissionState savedStateBeforeYield;

  WaypointGoal pickLocation;
  WaypointGoal dropLocation;

  unsigned long stateStartTime;
  int subStep;

  void transitionTo(RobotMissionState newState);
  void executeState(const GlobalPose& currentPose);
};

#endif // MISSION_CONTROLLER_H
