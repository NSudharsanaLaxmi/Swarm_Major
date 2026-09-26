#include "MissionController.h"

MissionController::MissionController()
  : navigation(nullptr),
    arm(nullptr),
    sensors(nullptr),
    safety(nullptr),
    mecanum(nullptr),
    currentState(STATE_IDLE),
    savedStateBeforeYield(STATE_IDLE),
    stateStartTime(0),
    subStep(0) {
  // Default pick and drop locations customized by ROBOT_ID
  pickLocation = (ROBOT_ID == 0) ? WaypointGoal{35.0f, 32.0f, 90.0f, true} : WaypointGoal{85.0f, 32.0f, 90.0f, true};
  dropLocation = WaypointGoal{60.0f, 98.0f, -90.0f, true};
}

void MissionController::init(NavigationController* nav,
                             ArmController* armCtrl,
                             SensorSuite* sens,
                             SafetyController* safe,
                             MecanumController* mec) {
  navigation = nav;
  arm        = armCtrl;
  sensors    = sens;
  safety     = safe;
  mecanum    = mec;

  transitionTo(STATE_IDLE);
  Serial.println(F("[MISSION_CONTROLLER] Autonomous Mission State Machine Initialized."));
}

const char* MissionController::getStateName() const {
  switch (currentState) {
    case STATE_IDLE:               return "IDLE";
    case STATE_NAV_TO_PICK:        return "NAV_TO_PICK";
    case STATE_RACK_VERIFY:        return "RACK_VERIFY";
    case STATE_PRECISION_DOCK:     return "PRECISION_DOCK";
    case STATE_PICK_PAYLOAD:       return "PICK_PAYLOAD";
    case STATE_LIFT:               return "LIFT";
    case STATE_NAV_TO_DROP:        return "NAV_TO_DROP";
    case STATE_POSITION_FOR_DROP:  return "POSITION_FOR_DROP";
    case STATE_RELEASE_PAYLOAD:    return "RELEASE_PAYLOAD";
    case STATE_RETRACT:            return "RETRACT";
    case STATE_YIELDING:           return "YIELDING";
    case STATE_SAFETY_STOP:        return "SAFETY_STOP";
    case STATE_EMERGENCY_STOP:     return "EMERGENCY_STOP";
    case STATE_COMMUNICATION_LOSS: return "COMMUNICATION_LOSS";
    case STATE_FAULT:              return "FAULT";
    default:                       return "UNKNOWN";
  }
}

void MissionController::transitionTo(RobotMissionState newState) {
  currentState   = newState;
  stateStartTime = millis();
  subStep        = 0;
  Serial.printf("[MISSION] State Transition -> %s (T=%lu ms)\n", getStateName(), stateStartTime);
}

void MissionController::startMission(float pickX, float pickY, float dropX, float dropY) {
  pickLocation = { pickX, pickY, 90.0f, true };
  dropLocation = { dropX, dropY, -90.0f, true };

  if (arm) arm->setNamedPose(POSE_TYPE_HOME, 600);
  if (navigation) navigation->setGoal(pickLocation.x, pickLocation.y, pickLocation.theta);

  transitionTo(STATE_NAV_TO_PICK);
}

void MissionController::abortMission() {
  if (navigation) navigation->stop();
  if (mecanum) mecanum->stop();
  if (arm) arm->stop();
  transitionTo(STATE_IDLE);
}

void MissionController::setManualMissionState(RobotMissionState state) {
  transitionTo(state);
}

void MissionController::update(const GlobalPose& currentPose) {
  // Check Safety Interlocks
  if (safety) {
    if (safety->isEmergencyStopped()) {
      if (currentState != STATE_EMERGENCY_STOP) {
        transitionTo(STATE_EMERGENCY_STOP);
      }
      return;
    }

    if (safety->isSwarmYielding()) {
      if (currentState != STATE_YIELDING && currentState != STATE_PICK_PAYLOAD && currentState != STATE_RELEASE_PAYLOAD) {
        savedStateBeforeYield = currentState;
        transitionTo(STATE_YIELDING);
      }
      return;
    } else if (currentState == STATE_YIELDING) {
      // Resume prior trajectory once corridor is cleared
      transitionTo(savedStateBeforeYield);
    }
  }

  executeState(currentPose);
}

void MissionController::executeState(const GlobalPose& currentPose) {
  unsigned long elapsed = millis() - stateStartTime;

  switch (currentState) {

    case STATE_IDLE:
      if (mecanum) mecanum->stop();
      break;

    case STATE_NAV_TO_PICK:
      if (navigation) {
        bool arrived = navigation->update(currentPose);
        if (arrived) {
          transitionTo(STATE_RACK_VERIFY);
        }
      }
      break;

    case STATE_RACK_VERIFY:
      if (mecanum) mecanum->stop();
      // Verify RFID tag or proceed after verification timeout
      if ((sensors && sensors->getLastScannedRFID().length() > 0) || elapsed > 1500) {
        if (arm) arm->setNamedPose(POSE_TYPE_APPROACH, 800);
        transitionTo(STATE_PRECISION_DOCK);
      }
      break;

    case STATE_PRECISION_DOCK:
      // Creep forward slowly until ToF docking clearance is reached (60 mm)
      if (sensors && !sensors->isDockingClearanceReached()) {
        if (mecanum) mecanum->driveVelocity(0.12f, 0.0f, 0.0f);
      } else {
        if (mecanum) mecanum->stop();
        transitionTo(STATE_PICK_PAYLOAD);
      }
      break;

    case STATE_PICK_PAYLOAD:
      if (mecanum) mecanum->stop();
      if (subStep == 0) {
        if (arm) arm->setNamedPose(POSE_TYPE_PICK, 700);
        subStep = 1;
        stateStartTime = millis();
      } else if (subStep == 1 && elapsed > 800) {
        if (arm) arm->closeGripper();
        subStep = 2;
        stateStartTime = millis();
      } else if (subStep == 2 && elapsed > 600) {
        transitionTo(STATE_LIFT);
      }
      break;

    case STATE_LIFT:
      if (subStep == 0) {
        if (arm) arm->setNamedPose(POSE_TYPE_LIFT, 700);
        subStep = 1;
        stateStartTime = millis();
      } else if (subStep == 1 && elapsed > 800) {
        if (arm) arm->setNamedPose(POSE_TYPE_TRANSPORT, 600);
        if (navigation) navigation->setGoal(dropLocation.x, dropLocation.y, dropLocation.theta);
        transitionTo(STATE_NAV_TO_DROP);
      }
      break;

    case STATE_NAV_TO_DROP:
      if (navigation) {
        bool arrived = navigation->update(currentPose);
        if (arrived) {
          transitionTo(STATE_POSITION_FOR_DROP);
        }
      }
      break;

    case STATE_POSITION_FOR_DROP:
      if (mecanum) mecanum->stop();
      if (arm) arm->setNamedPose(POSE_TYPE_APPROACH, 700);
      if (elapsed > 800) {
        transitionTo(STATE_RELEASE_PAYLOAD);
      }
      break;

    case STATE_RELEASE_PAYLOAD:
      if (mecanum) mecanum->stop();
      if (subStep == 0) {
        if (arm) arm->setNamedPose(POSE_TYPE_DROP, 700);
        subStep = 1;
        stateStartTime = millis();
      } else if (subStep == 1 && elapsed > 800) {
        if (arm) arm->openGripper();
        subStep = 2;
        stateStartTime = millis();
      } else if (subStep == 2 && elapsed > 600) {
        transitionTo(STATE_RETRACT);
      }
      break;

    case STATE_RETRACT:
      if (subStep == 0) {
        if (arm) arm->setNamedPose(POSE_TYPE_RETRACT, 700);
        subStep = 1;
        stateStartTime = millis();
      } else if (subStep == 1 && elapsed > 800) {
        if (arm) arm->setNamedPose(POSE_TYPE_HOME, 600);
        transitionTo(STATE_IDLE);
      }
      break;

    case STATE_YIELDING:
    case STATE_SAFETY_STOP:
    case STATE_EMERGENCY_STOP:
    case STATE_COMMUNICATION_LOSS:
    case STATE_FAULT:
    default:
      if (mecanum) mecanum->stop();
      break;
  }
}
