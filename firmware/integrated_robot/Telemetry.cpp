#include "Telemetry.h"

Telemetry::Telemetry()
  : commsModule(nullptr),
    motorDriver(nullptr),
    mecanumController(nullptr),
    armController(nullptr),
    sensorSuite(nullptr),
    safetyController(nullptr),
    missionController(nullptr),
    lastTelemetryBroadcast(0) {}

void Telemetry::init(SwarmComms* comms,
                      MotorDriver* motors,
                      MecanumController* mecanum,
                      ArmController* arm,
                      SensorSuite* sensors,
                      SafetyController* safety,
                      MissionController* mission) {
  commsModule        = comms;
  motorDriver        = motors;
  mecanumController  = mecanum;
  armController      = arm;
  sensorSuite        = sensors;
  safetyController   = safety;
  missionController  = mission;

  Serial.println(F("[TELEMETRY] Real-Time Telemetry Generator Online."));
}

String Telemetry::buildJsonPayload() {
  StaticJsonDocument<768> doc;

  doc["robot_id"]         = ROBOT_ID;
  doc["version"]          = FIRMWARE_VERSION;
  doc["timestamp"]        = millis();
  doc["connected"]        = commsModule ? commsModule->isConnected() : false;
  doc["mission_state"]    = missionController ? missionController->getStateName() : "UNKNOWN";

  // Pose Telemetry
  if (commsModule) {
    GlobalPose pose = commsModule->getMyGlobalPose();
    JsonObject pObj = doc.createNestedObject("pose");
    pObj["x"]       = pose.x;
    pObj["y"]       = pose.y;
    pObj["theta"]   = pose.theta;
    pObj["valid"]   = pose.valid;
  }

  // Motor & Drive Telemetry
  JsonObject mObj = doc.createNestedObject("motors");
  mObj["pwm_left"]  = motorDriver ? motorDriver->getLeftPwm() : 0;
  mObj["pwm_right"] = motorDriver ? motorDriver->getRightPwm() : 0;
  if (mecanumController) {
    mObj["vx"]    = mecanumController->getLastVx();
    mObj["vy"]    = mecanumController->getLastVy();
    mObj["omega"] = mecanumController->getLastOmega();
  }

  // Sensors Telemetry
  if (sensorSuite) {
    SensorTelemetry sTelem = sensorSuite->getTelemetry();
    JsonObject sObj = doc.createNestedObject("sensors");
    sObj["tof_mm"]         = sTelem.tofDistanceMm;
    sObj["ultrasonic_cm"]  = sTelem.ultrasonicDistanceCm;
    sObj["rfid_tag"]       = sTelem.lastRfidTag;
    sObj["blocked"]        = sTelem.obstacleDetected;
  }

  // Arm Telemetry
  if (armController) {
    ArmTelemetry aTelem = armController->getTelemetry();
    JsonObject aObj = doc.createNestedObject("arm");
    aObj["base"]      = aTelem.baseAngle;
    aObj["shoulder"]  = aTelem.shoulderAngle;
    aObj["elbow"]     = aTelem.elbowAngle;
    aObj["joint4"]    = aTelem.joint4Angle;
    aObj["joint5"]    = aTelem.joint5Angle;
    aObj["moving"]    = aTelem.isMoving;
    aObj["pose_type"] = (int)aTelem.activePoseType;
  }

  // Safety Status Telemetry
  if (safetyController) {
    JsonObject safObj = doc.createNestedObject("safety");
    safObj["e_stop"]   = safetyController->isEmergencyStopped();
    safObj["tof_stop"] = safetyController->isObstacleBraked();
    safObj["yielding"] = safetyController->isSwarmYielding();
    safObj["status"]   = (int)safetyController->getStatus();
  }

  String output;
  serializeJson(doc, output);
  return output;
}

void Telemetry::update() {
  unsigned long now = millis();
  if (now - lastTelemetryBroadcast < 100) return; // 10 Hz Telemetry Broadcast
  lastTelemetryBroadcast = now;

  if (commsModule) {
    String payload = buildJsonPayload();
    commsModule->sendTelemetry(payload);
  }
}
