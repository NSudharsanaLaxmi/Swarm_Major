#include "SensorSuite.h"

SensorSuite::SensorSuite()
  : rfid(RFID_SS_PIN, RFID_RST_PIN),
    tofReady(false),
    rfidReady(false),
    currentTofDistMM(800),
    currentUltrasonicCM(100.0f),
    currentRFIDTag(""),
    lastSensorPoll(0) {}

void SensorSuite::init() {
  // 1. Initialize Ultrasonic Pins
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);
  digitalWrite(PIN_US_TRIG, LOW);

  // 2. Initialize VL53L0X Laser ToF Sensor on I2C
  tof.setTimeout(500);
  if (tof.init()) {
    tof.startContinuous();
    tofReady = true;
    Serial.println(F("[SENSOR] VL53L0X Time-of-Flight Sensor Online (0x29)."));
  } else {
    Serial.println(F("[SENSOR] WARNING: VL53L0X ToF sensor not detected on I2C bus!"));
  }

  // 3. Initialize RC522 RFID Sensor on SPI
  SPI.begin(RFID_SCK_PIN, RFID_MISO_PIN, RFID_MOSI_PIN, RFID_SS_PIN);
  rfid.PCD_Init();
  byte v = rfid.PCD_ReadRegister(rfid.VersionReg);
  if (v == 0x91 || v == 0x92 || v == 0x12) {
    rfidReady = true;
    Serial.printf("[SENSOR] MFRC522 RFID Reader Online (Ver 0x%02X).\n", v);
  } else {
    Serial.println(F("[SENSOR] WARNING: MFRC522 RFID reader not detected on SPI bus."));
  }
}

float SensorSuite::readUltrasonicRaw() {
  digitalWrite(PIN_US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);

  long duration = pulseIn(PIN_US_ECHO, HIGH, 25000); // 25ms timeout (~4m max)
  if (duration == 0) return 400.0f; // No echo
  return (float)(duration * 0.0343 / 2.0);
}

String SensorSuite::readRFIDRaw() {
  if (!rfidReady) return "";
  if (!rfid.PICC_IsNewCardPresent()) return "";
  if (!rfid.PICC_ReadCardSerial())   return "";

  String tagID = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    tagID += String(rfid.uid.uidByte[i] < 0x10 ? "0" : "");
    tagID += String(rfid.uid.uidByte[i], HEX);
  }
  tagID.toUpperCase();
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  return tagID;
}

void SensorSuite::update() {
  unsigned long now = millis();
  if (now - lastSensorPoll < 50) return; // 20 Hz non-blocking sensor poll
  lastSensorPoll = now;

  // Read ToF
  if (tofReady) {
    uint16_t dist = tof.readRangeContinuousMillimeters();
    if (!tof.timeoutOccurred() && dist > 0 && dist < 2000) {
      currentTofDistMM = dist;
    }
  }

  // Read Ultrasonic
  currentUltrasonicCM = readUltrasonicRaw();

  // Read RFID
  String newTag = readRFIDRaw();
  if (newTag.length() > 0) {
    currentRFIDTag = newTag;
    Serial.printf("[SENSOR] Ground-Truth RFID Tag Scanned: %s\n", currentRFIDTag.c_str());
  }
}

bool SensorSuite::isForwardPathBlocked() const {
  return (currentTofDistMM < TOF_BRAKE_DIST_MM || currentUltrasonicCM < 12.0f);
}

bool SensorSuite::isDockingClearanceReached() const {
  return (currentTofDistMM <= TOF_DOCK_DIST_MM);
}

SensorTelemetry SensorSuite::getTelemetry() const {
  SensorTelemetry telem;
  telem.tofDistanceMm        = currentTofDistMM;
  telem.ultrasonicDistanceCm = currentUltrasonicCM;
  telem.lastRfidTag          = currentRFIDTag;
  telem.obstacleDetected     = isForwardPathBlocked();
  telem.tofOnline            = tofReady;
  telem.rfidOnline           = rfidReady;
  return telem;
}
