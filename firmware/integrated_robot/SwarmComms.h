#ifndef SWARM_COMMS_H
#define SWARM_COMMS_H

#include <Arduino.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <ArduinoJson.h>
#include "Config.h"

struct GlobalPose {
  float x;
  float y;
  float theta;
  bool valid;
  unsigned long lastUpdateMs;
};

enum CommandType {
  CMD_NONE,
  CMD_VELOCITY,
  CMD_NAV_GOAL,
  CMD_ARM_POSE,
  CMD_ARM_NAMED_POSE,
  CMD_STOP,
  CMD_EMERGENCY_STOP,
  CMD_RESUME,
  CMD_PICK,
  CMD_DROP,
  CMD_SET_POSE
};

struct IncomingCommand {
  CommandType type;
  float linear;
  float angular;
  float goalX;
  float goalY;
  float goalTheta;
  int armBase;
  int armShoulder;
  int armElbow;
  int armJoint4;
  int armJoint5;
  ArmPoseType namedPose;
  unsigned long timestamp;
};

class SwarmComms {
public:
  SwarmComms();
  void init();
  void update();

  bool isConnected() const;
  bool isCommandTimedOut() const;
  unsigned long getLastPacketAgeMs() const;

  GlobalPose getMyGlobalPose() const { return myGlobalPose; }
  GlobalPose getPeerGlobalPose() const { return peerGlobalPose; }
  bool isPeerNear(float thresholdCm = SWARM_SAFE_DIST_CM) const;

  bool hasNewCommand(IncomingCommand& outCmd);
  void sendTelemetry(const String& jsonPayload);

private:
  WiFiUDP cmdUdp;
  WiFiUDP swarmUdp;

  GlobalPose myGlobalPose;
  GlobalPose peerGlobalPose;

  IncomingCommand activeCommand;
  bool newCommandPending;

  unsigned long lastCommandPacketTime;
  unsigned long lastSwarmPacketTime;

  char rxBuffer[512];

  void processCommandPacket();
  void processSwarmBroadcast();
  bool parseJsonCommand(const char* jsonStr);
  bool parseLegacyVelocity(const char* rawStr);
};

#endif // SWARM_COMMS_H
