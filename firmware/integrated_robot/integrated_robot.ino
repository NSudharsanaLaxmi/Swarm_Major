/*
 * ======================================================================================
 * MASTER INTEGRATED AUTONOMOUS SWARM MOBILE MANIPULATOR FIRMWARE
 * ======================================================================================
 * Project: AI-Driven Cooperative Autonomous Mobile Manipulator Swarm for Smart Warehouse
 *
 * Integrated Subsystems:
 *   1. 4WD Locomotion: Two TB6612 drivers (PWM on D4/D5, Dir on 25,26,27,14,12,13,32,33)
 *   2. Kinematic Abstraction: MecanumController (Active 2-Channel Spliced & 4-Wheel Mecanum IK)
 *   3. 5-DOF Manipulator: MG996R Servos via PCA9685 I2C (CH0-CH4) with smooth interpolation
 *   4. Sensor Suite: VL53L0X (ToF 0x29), HC-SR04 (Ultrasonic), MFRC522 (RFID)
 *   5. Swarm Comms: Dual UDP listener (Port 8888 velocity/JSON, Port 5005 global pose)
 *   6. Closed-Loop Navigation: ArUco-based waypoint tracking with heading error regulation
 *   7. Prioritized Safety: Emergency Stop > Obstacle Brake (<120mm) > Swarm Yielding (28cm) > 500ms Timeout
 *   8. Autonomous Mission State Machine: 15-state non-blocking warehouse fulfillment FSM
 *   9. Real-Time Telemetry: 10 Hz JSON telemetry broadcast to Arduino UNO Q and Digital Twin
 *  10. Local OLED Dashboard: SSD1306 128x64 display (0x3C)
 * ======================================================================================
 */

#include "Config.h"
#include "MotorDriver.h"
#include "MecanumController.h"
#include "ArmController.h"
#include "SensorSuite.h"
#include "SwarmComms.h"
#include "NavigationController.h"
#include "SafetyController.h"
#include "MissionController.h"
#include "Telemetry.h"
#include "DisplayManager.h"

// =========================================================================
// GLOBAL SUBSYSTEM INSTANCES
// =========================================================================
MotorDriver           motors;
MecanumController     mecanum;
ArmController         arm;
SensorSuite           sensors;
SwarmComms            comms;
NavigationController  navigation;
SafetyController      safety;
MissionController     mission;
Telemetry             telemetry;
DisplayManager        display;

unsigned long lastDisplayUpdate = 0;
bool manualOverrideActive = false;

// =========================================================================
// INITIALIZATION (SETUP)
// =========================================================================
void setup() {
  Serial.begin(SERIAL_BAUD_RATE);
  delay(1000);

  Serial.println(F("\n========================================================"));
  Serial.printf(  "   SWARM ROBOT R0%d FIRMWARE BOOT: 5-DOF & MECANUM STACK\n", ROBOT_ID + 1);
  Serial.printf(  "   Firmware Version: %s\n", FIRMWARE_VERSION);
  Serial.println(F("========================================================"));

  // 1. Initialize Master I2C Bus (SDA=21, SCL=22)
  Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
  Wire.setClock(400000); // 400 kHz Fast Mode

  // 2. Initialize OLED Dashboard
  display.init();

  // 3. Initialize Physical TB6612 Motor Hardware (Native ESP32 PWM on Pins 4 & 5)
  motors.init();
  mecanum.init(&motors);

  // 4. Initialize PCA9685 5-DOF MG996R Robotic Arm (CH0-CH4)
  arm.begin();

  // 5. Initialize Onboard Sensors (Laser ToF, Ultrasonic, RFID)
  sensors.init();

  // 6. Initialize Swarm Networking (Wi-Fi STA, UDP 8888 Command & 5005 Vision)
  comms.init();

  // 7. Initialize Closed-Loop Navigation Subsystem
  navigation.init(&mecanum);

  // 8. Initialize Safety Failsafes (E-Stop, 500ms Timeout, Obstacle Brake, Swarm Yield)
  safety.init(&motors, &arm);

  // 9. Initialize Autonomous Mission State Machine
  mission.init(&navigation, &arm, &sensors, &safety, &mecanum);

  // 10. Initialize Real-Time Telemetry Generator (10 Hz)
  telemetry.init(&comms, &motors, &mecanum, &arm, &sensors, &safety, &mission);

  Serial.println(F("[BOOT] All 10 modular subsystems online. Ready for Swarm Operations."));
}

// =========================================================================
// REAL-TIME EXECUTIVE LOOP
// =========================================================================
void loop() {
  // 1. Poll Network Sockets & Global ArUco Perception
  comms.update();

  // 2. Poll Onboard Sensors (Laser ToF, Ultrasonic, RFID)
  sensors.update();

  // 3. Update Non-Blocking Arm Trajectory Interpolation
  arm.update();

  GlobalPose currentPose = comms.getMyGlobalPose();

  // 4. Handle Incoming Commands (UDP Port 8888 or Hardware UART)
  IncomingCommand cmd;
  if (comms.hasNewCommand(cmd)) {
    switch (cmd.type) {
      case CMD_EMERGENCY_STOP:
        safety.triggerEmergencyStop();
        mission.setManualMissionState(STATE_EMERGENCY_STOP);
        manualOverrideActive = false;
        break;

      case CMD_RESUME:
        safety.resumeFromEmergencyStop();
        mission.setManualMissionState(STATE_IDLE);
        manualOverrideActive = false;
        break;

      case CMD_STOP:
        mecanum.stop();
        navigation.clearGoal();
        mission.abortMission();
        manualOverrideActive = false;
        break;

      case CMD_VELOCITY:
        if (!safety.isEmergencyStopped()) {
          manualOverrideActive = true;
          mecanum.driveVelocity(cmd.linear, 0.0f, cmd.angular);
        }
        break;

      case CMD_NAV_GOAL:
        if (!safety.isEmergencyStopped()) {
          manualOverrideActive = false;
          navigation.setGoal(cmd.goalX, cmd.goalY, cmd.goalTheta);
          mission.setManualMissionState(STATE_NAV_TO_PICK);
        }
        break;

      case CMD_ARM_POSE:
        if (!safety.isEmergencyStopped()) {
          arm.movePose(cmd.armBase, cmd.armShoulder, cmd.armElbow, cmd.armJoint4, cmd.armJoint5, 800);
        }
        break;

      case CMD_ARM_NAMED_POSE:
        if (!safety.isEmergencyStopped()) {
          arm.setNamedPose(cmd.namedPose, 800);
        }
        break;

      case CMD_PICK:
        if (!safety.isEmergencyStopped()) {
          manualOverrideActive = false;
          mission.setManualMissionState(STATE_PICK_PAYLOAD);
        }
        break;

      case CMD_DROP:
        if (!safety.isEmergencyStopped()) {
          manualOverrideActive = false;
          mission.setManualMissionState(STATE_RELEASE_PAYLOAD);
        }
        break;

      case CMD_SET_POSE:
        // Calibration override
        break;

      default:
        break;
    }
  }

  // 5. Evaluate Prioritized Safety Failsafes
  bool isDocking = (mission.getCurrentState() == STATE_PRECISION_DOCK || mission.getCurrentState() == STATE_PICK_PAYLOAD);
  SafetyStatus safetyState = safety.evaluate(sensors, comms, isDocking);

  // 6. Execute Autonomous Navigation / Mission Logic if not manually overriding
  if (safetyState == SAFETY_STATUS_EMERGENCY_STOP) {
    mecanum.stop();
    arm.stop();
  } else if (safetyState == SAFETY_STATUS_OBSTACLE_BRAKE) {
    mecanum.stop();
  } else if (safetyState == SAFETY_STATUS_SWARM_YIELD) {
    mecanum.stop();
  } else if (safetyState == SAFETY_STATUS_COMM_TIMEOUT && manualOverrideActive) {
    // 500ms failsafe timeout on manual drive commands
    mecanum.stop();
  } else if (!manualOverrideActive) {
    // Autonomous Mission State Machine Tick
    mission.update(currentPose);
  }

  // 7. Broadcast Telemetry to UNO Q and Web Digital Twin (10 Hz)
  telemetry.update();

  // 8. Refresh OLED Dashboard (10 Hz)
  unsigned long now = millis();
  if (now - lastDisplayUpdate >= 100) {
    lastDisplayUpdate = now;
    ArmTelemetry aTelem = arm.getTelemetry();
    display.render(mission.getStateName(),
                   currentPose.x,
                   currentPose.y,
                   currentPose.theta,
                   sensors.getLastScannedRFID().c_str(),
                   comms.isConnected(),
                   aTelem.baseAngle,
                   aTelem.shoulderAngle);
  }
}
