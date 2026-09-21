/*
 * ======================================================================================
 * MASTER INTEGRATED AUTONOMOUS SWARM MOBILE MANIPULATOR FIRMWARE
 * ======================================================================================
 * Project: AI-Driven Cooperative Autonomous Mobile Manipulator Swarm for Smart Warehouse
 *
 * Integrated Subsystems:
 *   1. 4WD Locomotion: Two TB6612 drivers (PWM on D4/D5, Dir on 25,26,27,14,12,13,32,33)
 *   2. 4-DOF Arm & Gripper: PCA9685 I2C driver (CH0-CH4) with smooth pose interpolation
 *   3. Sensor Suite: VL53L0X (ToF 0x29), HC-SR04 (Ultrasonic), MFRC522 (RFID)
 *   4. Telemetry Display: SSD1306 0.96" OLED (0x3C)
 *   5. Swarm Comms: Dual UDP listener (Port 8888 velocity, Port 5005 global pose)
 *   6. Decentralized Swarm Arbitration: Automatic right-of-way yielding based on Robot ID
 * ======================================================================================
 */

#include "Config.h"
#include "MotorDriver.h"
#include "ArmController.h"
#include "SensorSuite.h"
#include "DisplayManager.h"
#include "SwarmComms.h"

// Global Subsystem Instances
MotorDriver    motors;
ArmController  arm;
SensorSuite    sensors;
DisplayManager display;
SwarmComms     comms;

// Warehouse Mission State Machine
enum RobotMissionState {
  STATE_IDLE,
  STATE_NAV_TO_PICK,
  STATE_RACK_VERIFY,
  STATE_PRECISION_DOCK,
  STATE_PICK_PAYLOAD,
  STATE_NAV_TO_DROP,
  STATE_RELEASE_PAYLOAD,
  STATE_YIELDING
};

RobotMissionState currentState  = STATE_NAV_TO_PICK;
RobotMissionState previousState = STATE_NAV_TO_PICK;

// Waypoints (Customized for Robot 0 and Robot 1)
struct Waypoint {
  float x;
  float y;
};

Waypoint pickLocation = (MY_ROBOT_ID == 0) ? Waypoint{25.0f, 30.0f} : Waypoint{25.0f, 90.0f};
Waypoint dropLocation = (MY_ROBOT_ID == 0) ? Waypoint{95.0f, 30.0f} : Waypoint{95.0f, 90.0f};
Waypoint activeGoal   = pickLocation;

unsigned long lastTelemetryUpdate = 0;

// Autonomous Waypoint Guidance Math
void navigateTowards(float targetX, float targetY, SwarmPose pose) {
  float dx = targetX - pose.x;
  float dy = targetY - pose.y;
  float distance = sqrt(dx * dx + dy * dy);

  float desiredAngle = atan2(dy, dx) * 180.0f / M_PI;
  float angleError   = desiredAngle - pose.ang;

  // Bounding angle error to [-180, 180]
  while (angleError > 180.0f)  angleError -= 360.0f;
  while (angleError < -180.0f) angleError += 360.0f;

  if (abs(angleError) > HEADING_DEADBAND_DEG) {
    // In-place pivot rotation
    if (angleError > 0) {
      motors.setRawMotors(-130, 130); // Pivot CCW
    } else {
      motors.setRawMotors(130, -130); // Pivot CW
    }
  } else {
    // Proportional forward tracking
    int basePwm = 135;
    int trim = (int)(angleError * 1.4f);
    motors.setRawMotors(basePwm - trim, basePwm + trim);
  }
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("\n========================================================"));
  Serial.printf(  "   SWARM ROBOT R0%d: SYSTEM BOOT & PERIPHERAL INITIALIZATION\n", MY_ROBOT_ID + 1);
  Serial.println(F("========================================================"));

  // 1. Initialize I2C Bus (SDA=21, SCL=22)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);

  // 2. Initialize Subsystems
  display.init();
  motors.init();
  arm.init();
  sensors.init();
  comms.init();

  Serial.println(F("[BOOT] All hardware layers online. Starting Swarm Mission."));
}

void loop() {
  // Update Network & Sensor Inputs
  comms.update();
  sensors.update();

  SwarmPose myPose = comms.getMyPose();

  // --- 1. LOCAL OBSTACLE SAFETY OVERRIDE ---
  if (sensors.isForwardPathBlocked() && currentState != STATE_PRECISION_DOCK && currentState != STATE_PICK_PAYLOAD) {
    motors.stop();
    Serial.println(F("[SAFETY] Immediate Obstacle Obstruction! Motors halted."));
    display.render("SAFETY STOP", myPose.x, myPose.y, myPose.ang, sensors.getLastScannedRFID().c_str(), comms.isConnected());
    return;
  }

  // --- 2. DECENTRALIZED SWARM COLLISION ARBITRATION ---
  if (comms.isPeerNear(SWARM_SAFE_DIST_CM)) {
    // Priority Rule: Lower ID has priority; Higher ID yields
    // Exception: If currently picking or docking, maintain right-of-way
    if (MY_ROBOT_ID > PEER_ROBOT_ID && currentState != STATE_PICK_PAYLOAD && currentState != STATE_RELEASE_PAYLOAD) {
      if (currentState != STATE_YIELDING) {
        previousState = currentState;
        currentState = STATE_YIELDING;
        Serial.printf("[SWARM] Yielding corridor to Peer Robot R0%d.\n", PEER_ROBOT_ID + 1);
      }
    }
  } else if (currentState == STATE_YIELDING) {
    currentState = previousState;
    Serial.println(F("[SWARM] Corridor cleared. Resuming trajectory."));
  }

  // --- 3. MANUAL VELOCITY OVERRIDE (TELEOP / TEST OVERRIDE) ---
  float overrideLin, overrideAng;
  if (comms.hasVelocityOverride(overrideLin, overrideAng)) {
    motors.drive(overrideLin, overrideAng);
    display.render("MANUAL OVERRIDE", myPose.x, myPose.y, myPose.ang, sensors.getLastScannedRFID().c_str(), comms.isConnected());
    return;
  }

  // --- 4. AUTONOMOUS MISSION STATE MACHINE ---
  switch (currentState) {

    case STATE_IDLE:
      motors.stop();
      break;

    case STATE_YIELDING:
      motors.stop();
      break;

    case STATE_NAV_TO_PICK: {
      activeGoal = pickLocation;
      float distToPick = sqrt(sq(activeGoal.x - myPose.x) + sq(activeGoal.y - myPose.y));

      if (distToPick <= ARRIVAL_THRESH_CM && myPose.valid) {
        motors.stop();
        currentState = STATE_RACK_VERIFY;
        Serial.println(F("[TASK] Arrived at Rack Station. Verifying RFID..."));
      } else if (myPose.valid) {
        navigateTowards(activeGoal.x, activeGoal.y, myPose);
      }
      break;
    }

    case STATE_RACK_VERIFY: {
      motors.stop();
      String tag = sensors.getLastScannedRFID();
      if (tag != "NONE" && tag.length() > 0) {
        Serial.printf("[TASK] Rack Verified! Tag ID: %s\n", tag.c_str());
        currentState = STATE_PRECISION_DOCK;
      } else {
        // Slow alignment creep to scan RFID tag
        motors.setRawMotors(85, 85);
        delay(100);
        motors.stop();
        delay(150);
      }
      break;
    }

    case STATE_PRECISION_DOCK: {
      uint16_t tofDist = sensors.getTofDistanceMM();
      Serial.printf("[DOCK] Distance to Payload: %d mm\n", tofDist);

      if (tofDist > TOF_DOCK_DIST_MM && tofDist < 250) {
        // Slow precision approach
        motors.setRawMotors(80, 80);
        delay(80);
        motors.stop();
      } else {
        motors.stop();
        currentState = STATE_PICK_PAYLOAD;
        Serial.println(F("[DOCK] Payload within gripping tolerance. Actuating Manipulator."));
      }
      break;
    }

    case STATE_PICK_PAYLOAD: {
      motors.stop();
      // Arm Maneuver: Hover -> Open -> Lower -> Grip -> Lift -> Rest
      arm.openGripper();
      arm.setPose(POSE_HOVER, 400);
      delay(200);
      arm.setPose(POSE_PICK, 500);
      delay(300);
      arm.closeGripper();
      delay(500);
      arm.setPose(POSE_LIFT, 400);
      delay(300);
      arm.setPose(POSE_REST, 400);

      // Back off slightly from the rack
      motors.setRawMotors(-115, -115);
      delay(400);
      motors.stop();

      currentState = STATE_NAV_TO_DROP;
      Serial.println(F("[TASK] Payload secured. En route to Delivery Station."));
      break;
    }

    case STATE_NAV_TO_DROP: {
      activeGoal = dropLocation;
      float distToDrop = sqrt(sq(activeGoal.x - myPose.x) + sq(activeGoal.y - myPose.y));

      if (distToDrop <= ARRIVAL_THRESH_CM && myPose.valid) {
        motors.stop();
        currentState = STATE_RELEASE_PAYLOAD;
        Serial.println(F("[TASK] Arrived at Delivery Station. Releasing payload..."));
      } else if (myPose.valid) {
        navigateTowards(activeGoal.x, activeGoal.y, myPose);
      }
      break;
    }

    case STATE_RELEASE_PAYLOAD: {
      motors.stop();
      arm.setPose(POSE_DROP, 500);
      delay(300);
      arm.openGripper();
      delay(400);
      arm.setPose(POSE_REST, 400);

      // Back off from drop bucket
      motors.setRawMotors(-115, -115);
      delay(450);
      motors.stop();

      currentState = STATE_IDLE;
      Serial.println(F("[TASK] Mission Complete! Robot in IDLE."));
      break;
    }
  }

  // --- 5. RENDER OLED DASHBOARD TELEMETRY (10 Hz) ---
  if (millis() - lastTelemetryUpdate > 100) {
    lastTelemetryUpdate = millis();

    const char* stateNames[] = {
      "IDLE", "NAV -> PICK", "VERIFY RACK", "PRECISION DOCK",
      "PICKING", "NAV -> DROP", "DROPPING", "YIELDING"
    };

    display.render(
      stateNames[currentState],
      myPose.x, myPose.y, myPose.ang,
      sensors.getLastScannedRFID().c_str(),
      comms.isConnected()
    );
  }
}
