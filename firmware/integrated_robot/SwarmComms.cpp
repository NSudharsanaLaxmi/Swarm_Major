#include "SwarmComms.h"

SwarmComms::SwarmComms()
  : newCommandPending(false),
    lastCommandPacketTime(0),
    lastSwarmPacketTime(0) {
  myGlobalPose   = { 0.0f, 0.0f, 0.0f, false, 0 };
  peerGlobalPose = { 0.0f, 0.0f, 0.0f, false, 0 };
  activeCommand  = { CMD_NONE, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 90, 90, 90, 90, 180, POSE_TYPE_HOME, 0 };
}

void SwarmComms::init() {
  Serial.print(F("[SWARM_COMMS] Connecting to Wi-Fi SSID: "));
  Serial.println(WIFI_SSID);

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  int attempts = 0;
  while (WiFi.status() != WL_CONNECTED && attempts < 20) {
    delay(300);
    Serial.print(".");
    attempts++;
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("\n[SWARM_COMMS] Wi-Fi Link Established."));
    Serial.print(F("[SWARM_COMMS] ESP32 Assigned IP: "));
    Serial.println(WiFi.localIP());

    cmdUdp.begin(UDP_CMD_PORT);
    swarmUdp.begin(UDP_SWARM_PORT);

    Serial.printf("[SWARM_COMMS] UDP Command Sockets bound to Port %d (Control) and Port %d (Swarm State).\n",
                  UDP_CMD_PORT, UDP_SWARM_PORT);
  } else {
    Serial.println(F("\n[SWARM_COMMS] WARNING: Wi-Fi connection timed out. Running in offline autonomous failsafe."));
  }
}

bool SwarmComms::isConnected() const {
  return (WiFi.status() == WL_CONNECTED);
}

bool SwarmComms::isCommandTimedOut() const {
  if (lastCommandPacketTime == 0) return true;
  return (millis() - lastCommandPacketTime > COMMAND_TIMEOUT_MS);
}

unsigned long SwarmComms::getLastPacketAgeMs() const {
  if (lastCommandPacketTime == 0) return 9999;
  return (millis() - lastCommandPacketTime);
}

bool SwarmComms::isPeerNear(float thresholdCm) const {
  if (!myGlobalPose.valid || !peerGlobalPose.valid) return false;
  float dx = myGlobalPose.x - peerGlobalPose.x;
  float dy = myGlobalPose.y - peerGlobalPose.y;
  float dist = sqrt(dx * dx + dy * dy);
  return (dist < thresholdCm);
}

bool SwarmComms::hasNewCommand(IncomingCommand& outCmd) {
  if (newCommandPending) {
    outCmd = activeCommand;
    newCommandPending = false;
    return true;
  }
  return false;
}

void SwarmComms::processCommandPacket() {
  int packetSize = cmdUdp.parsePacket();
  if (!packetSize) return;

  int len = cmdUdp.read(rxBuffer, sizeof(rxBuffer) - 1);
  if (len <= 0) return;
  rxBuffer[len] = '\0';

  lastCommandPacketTime = millis();

  // 1. Try structured JSON command
  if (rxBuffer[0] == '{') {
    if (parseJsonCommand(rxBuffer)) {
      newCommandPending = true;
    }
  } else {
    // 2. Legacy comma-separated velocity string "linear,angular"
    if (parseLegacyVelocity(rxBuffer)) {
      newCommandPending = true;
    }
  }
}

bool SwarmComms::parseJsonCommand(const char* jsonStr) {
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, jsonStr);
  if (err) return false;

  const char* type = doc["type"] | "";

  if (strcmp(type, "VELOCITY") == 0) {
    activeCommand.type      = CMD_VELOCITY;
    activeCommand.linear    = doc["linear"] | 0.0f;
    activeCommand.angular   = doc["angular"] | 0.0f;
    activeCommand.timestamp = millis();
    return true;
  }
  else if (strcmp(type, "NAV_GOAL") == 0) {
    activeCommand.type      = CMD_NAV_GOAL;
    activeCommand.goalX     = doc["x"] | 0.0f;
    activeCommand.goalY     = doc["y"] | 0.0f;
    activeCommand.goalTheta = doc["theta"] | 0.0f;
    activeCommand.timestamp = millis();
    return true;
  }
  else if (strcmp(type, "ARM_POSE") == 0) {
    activeCommand.type        = CMD_ARM_POSE;
    activeCommand.armBase     = doc["base"] | 90;
    activeCommand.armShoulder = doc["shoulder"] | 90;
    activeCommand.armElbow    = doc["elbow"] | 90;
    activeCommand.armJoint4   = doc["joint4"] | 90;
    activeCommand.armJoint5   = doc["joint5"] | 180;
    activeCommand.timestamp   = millis();
    return true;
  }
  else if (strcmp(type, "ARM_NAMED_POSE") == 0) {
    activeCommand.type      = CMD_ARM_NAMED_POSE;
    activeCommand.namedPose = (ArmPoseType)(doc["pose"] | 0);
    activeCommand.timestamp = millis();
    return true;
  }
  else if (strcmp(type, "STOP") == 0) {
    activeCommand.type      = CMD_STOP;
    activeCommand.timestamp = millis();
    return true;
  }
  else if (strcmp(type, "EMERGENCY_STOP") == 0) {
    activeCommand.type      = CMD_EMERGENCY_STOP;
    activeCommand.timestamp = millis();
    return true;
  }
  else if (strcmp(type, "RESUME") == 0) {
    activeCommand.type      = CMD_RESUME;
    activeCommand.timestamp = millis();
    return true;
  }
  else if (strcmp(type, "PICK") == 0) {
    activeCommand.type      = CMD_PICK;
    activeCommand.timestamp = millis();
    return true;
  }
  else if (strcmp(type, "DROP") == 0) {
    activeCommand.type      = CMD_DROP;
    activeCommand.timestamp = millis();
    return true;
  }
  else if (strcmp(type, "SET_POSE") == 0) {
    activeCommand.type      = CMD_SET_POSE;
    activeCommand.goalX     = doc["x"] | 0.0f;
    activeCommand.goalY     = doc["y"] | 0.0f;
    activeCommand.goalTheta = doc["ang"] | 0.0f;
    activeCommand.timestamp = millis();
    return true;
  }

  return false;
}

bool SwarmComms::parseLegacyVelocity(const char* rawStr) {
  float lin = 0.0f;
  float ang = 0.0f;
  if (sscanf(rawStr, "%f,%f", &lin, &ang) == 2) {
    activeCommand.type      = CMD_VELOCITY;
    activeCommand.linear    = lin;
    activeCommand.angular   = ang;
    activeCommand.timestamp = millis();
    return true;
  }
  return false;
}

void SwarmComms::processSwarmBroadcast() {
  int packetSize = swarmUdp.parsePacket();
  if (!packetSize) return;

  int len = swarmUdp.read(rxBuffer, sizeof(rxBuffer) - 1);
  if (len <= 0) return;
  rxBuffer[len] = '\0';

  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, rxBuffer);
  if (err) return;

  lastSwarmPacketTime = millis();

  JsonObject bots = doc["bots"];
  if (bots.isNull()) return;

  // Extract My Global Pose from overhead ArUco tracker
  String myKey = "id" + String(ROBOT_ID);
  if (bots.containsKey(myKey)) {
    myGlobalPose.x            = bots[myKey]["x"] | 0.0f;
    myGlobalPose.y            = bots[myKey]["y"] | 0.0f;
    myGlobalPose.theta        = bots[myKey]["ang"] | 0.0f;
    myGlobalPose.valid        = true;
    myGlobalPose.lastUpdateMs = millis();
  }

  // Extract Peer Global Pose for collision avoidance
  String peerKey = "id" + String(PEER_ROBOT_ID);
  if (bots.containsKey(peerKey)) {
    peerGlobalPose.x            = bots[peerKey]["x"] | 0.0f;
    peerGlobalPose.y            = bots[peerKey]["y"] | 0.0f;
    peerGlobalPose.theta        = bots[peerKey]["ang"] | 0.0f;
    peerGlobalPose.valid        = true;
    peerGlobalPose.lastUpdateMs = millis();
  }
}

void SwarmComms::sendTelemetry(const String& jsonPayload) {
  if (!isConnected()) return;

  // Broadcast to global swarm telemetry port 5005
  IPAddress broadcastIp(255, 255, 255, 255);
  cmdUdp.beginPacket(broadcastIp, UDP_SWARM_PORT);
  cmdUdp.write((const uint8_t*)jsonPayload.c_str(), jsonPayload.length());
  cmdUdp.endPacket();

  // Also mirror to Serial for hardware UART link to UNO Q
  Serial.println(jsonPayload);
}

void SwarmComms::update() {
  processCommandPacket();
  processSwarmBroadcast();
}
