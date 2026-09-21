#ifndef SWARM_COMMS_H
#define SWARM_COMMS_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "Config.h"

struct SwarmPose {
  float x;
  float y;
  float ang;
  bool valid;
};

class SwarmComms {
public:
  SwarmComms();
  void init();
  void update();

  bool isConnected();
  SwarmPose getMyPose();
  SwarmPose getPeerPose();
  bool isPeerNear(float thresholdCm = SWARM_SAFE_DIST_CM);
  bool hasVelocityOverride(float &outLinear, float &outAngular);
  unsigned long getLastHeardTime();

private:
  WiFiUDP cmdUdp;
  WiFiUDP swarmUdp;

  SwarmPose myPose;
  SwarmPose peerPose;

  float cmdLinear;
  float cmdAngular;
  bool newCmdAvailable;
  unsigned long lastCmdTime;
  unsigned long lastSwarmPacketTime;

  void parseVelocityPacket();
  void parseSwarmBroadcast();
};

#endif // SWARM_COMMS_H
