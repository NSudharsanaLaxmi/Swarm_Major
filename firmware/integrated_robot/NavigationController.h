#ifndef NAVIGATION_CONTROLLER_H
#define NAVIGATION_CONTROLLER_H

#include <Arduino.h>
#include "Config.h"
#include "SwarmComms.h"
#include "MecanumController.h"

struct WaypointGoal {
  float x;
  float y;
  float theta;
  bool active;
};

class NavigationController {
public:
  NavigationController();
  void init(MecanumController* mecanum);

  void setGoal(float targetX, float targetY, float targetTheta = 0.0f);
  void clearGoal();

  // Execute closed-loop navigation tick
  bool update(const GlobalPose& currentPose);

  bool isGoalReached() const { return goalReached; }
  float getDistanceToGoal() const { return currentDistanceToGoal; }
  float getHeadingError() const { return currentHeadingError; }
  WaypointGoal getActiveGoal() const { return activeGoal; }

  void stop();

private:
  MecanumController* mecanumController;
  WaypointGoal activeGoal;

  bool goalReached;
  float currentDistanceToGoal;
  float currentHeadingError;

  float normalizeAngle(float angleDeg);
};

#endif // NAVIGATION_CONTROLLER_H
