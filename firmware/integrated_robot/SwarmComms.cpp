#include "SwarmComms.h"

SwarmComms::SwarmComms() 
  : cmdLinear(0.0f), cmdAngular(0.0f), newCmdAvailable(false),
    lastCmdTime(0), lastSwarmPacketTime(0) {
  myPose   = {0.0f, 0.0f, 0.0f, false};
  peerPose = {0.0f, 0.0f, 0.0f, false};
}

void SwarmComms::init() {
  Serial.printf("[COMM] Connecting to SSID: %s\n", WIFI_SSID);
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(400);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println(F("\n[COMM] Wi-Fi Link Established."));
    Serial.printf("[COMM] Robot R0%d IP Address: %s\n", MY_ROBOT_ID + 1, WiFi.localIP().toString().c_str());

    cmdUdp.begin(UDP_CMD_PORT);
    swarmUdp.begin(UDP_SWARM_PORT);
    Serial.printf("[COMM] Ports Open -> Command: %d, Swarm State: %d\n", UDP_CMD_PORT, UDP_SWARM_PORT);
  } else {
    Serial.println(F("\n[WARNING][COMM] Wi-Fi connection timed out. Will retry in loop."));
  }
}

bool SwarmComms::isConnected() {
  return (WiFi.status() == WL_CONNECTED);
}

void SwarmComms::parseVelocityPacket() {
  int packetSize = cmdUdp.parsePacket();
  if (packetSize) {
    char buf[128];
    int len = cmdUdp.read(buf, 127);
    if (len > 0) buf[len] = '\0';

    String data = String(buf);
    int comma = data.indexOf(',');
    if (comma > 0) {
      cmdLinear  = data.substring(0, comma).toFloat();
      cmdAngular = data.substring(comma + 1).toFloat();
      newCmdAvailable = true;
      lastCmdTime = millis();
    }
  }
}

void SwarmComms::parseSwarmBroadcast() {
  int packetSize = swarmUdp.parsePacket();
  if (packetSize) {
    char buf[512];
    int len = swarmUdp.read(buf, 511);
    if (len > 0) buf[len] = '\0';

    JsonDocument doc;
    if (!deserializeJson(doc, buf)) {
      String myKey   = "id" + String(MY_ROBOT_ID);
      String peerKey = "id" + String(PEER_ROBOT_ID);

      if (doc["bots"].containsKey(myKey)) {
        myPose.x   = doc["bots"][myKey]["x"];
        myPose.y   = doc["bots"][myKey]["y"];
        myPose.ang = doc["bots"][myKey]["ang"];
        myPose.valid = true;
        lastSwarmPacketTime = millis();
      }

      if (doc["bots"].containsKey(peerKey)) {
        peerPose.x   = doc["bots"][peerKey]["x"];
        peerPose.y   = doc["bots"][peerKey]["y"];
        peerPose.ang = doc["bots"][peerKey]["ang"];
        peerPose.valid = true;
      } else {
        peerPose.valid = false;
      }
    }
  }
}

void SwarmComms::update() {
  if (WiFi.status() != WL_CONNECTED) {
    // Attempt background reconnect
    static unsigned long lastReconnect = 0;
    if (millis() - lastReconnect > 5000) {
      lastReconnect = millis();
      WiFi.reconnect();
    }
    return;
  }

  parseVelocityPacket();
  parseSwarmBroadcast();
}

bool SwarmComms::hasVelocityOverride(float &outLinear, float &outAngular) {
  if (millis() - lastCmdTime < WATCHDOG_TIMEOUT_MS && newCmdAvailable) {
    outLinear = cmdLinear;
    outAngular = cmdAngular;
    return true;
  }
  return false;
}

SwarmPose SwarmComms::getMyPose() {
  return myPose;
}

SwarmPose SwarmComms::getPeerPose() {
  return peerPose;
}

bool SwarmComms::isPeerNear(float thresholdCm) {
  if (!myPose.valid || !peerPose.valid) return false;
  float dx = myPose.x - peerPose.x;
  float dy = myPose.y - peerPose.y;
  return (sqrt(dx * dx + dy * dy) < thresholdCm);
}

unsigned long SwarmComms::getLastHeardTime() {
  return max(lastCmdTime, lastSwarmPacketTime);
}
