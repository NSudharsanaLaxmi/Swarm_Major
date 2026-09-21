#include "SensorSuite.h"

SensorSuite::SensorSuite() 
  : rfid(RFID_SS_PIN, RFID_RST_PIN),
    tofReady(false),
    rfidReady(false),
    currentTofDistMM(9999),
    currentUltrasonicCM(999.0f),
    currentRFIDTag("NONE"),
    lastSensorPoll(0) {}

void SensorSuite::init() {
  // Ultrasonic Pins
  pinMode(PIN_US_TRIG, OUTPUT);
  pinMode(PIN_US_ECHO, INPUT);

  // VL53L0X Initialization
  tof.setTimeout(150);
  if (tof.init()) {
    tof.startContinuous();
    tofReady = true;
    Serial.println(F("[SENSOR] VL53L0X Laser ToF Sensor Online."));
  } else {
    Serial.println(F("[WARNING][SENSOR] VL53L0X not found at 0x29!"));
  }

  // RC522 RFID Initialization
  SPI.begin(RFID_SCK_PIN, RFID_MISO_PIN, RFID_MOSI_PIN, RFID_SS_PIN);
  rfid.PCD_Init();
  byte v = rfid.PCD_ReadRegister(MFRC522::VersionReg);
  if (v == 0x91 || v == 0x92) {
    rfidReady = true;
    Serial.printf("[SENSOR] MFRC522 RFID Reader Online (Version: 0x%02X)\n", v);
  } else {
    Serial.println(F("[WARNING][SENSOR] MFRC522 RFID Reader offline or disconnected."));
  }
}

float SensorSuite::readUltrasonicRaw() {
  digitalWrite(PIN_US_TRIG, LOW);
  delayMicroseconds(2);
  digitalWrite(PIN_US_TRIG, HIGH);
  delayMicroseconds(10);
  digitalWrite(PIN_US_TRIG, LOW);

  long duration = pulseIn(PIN_US_ECHO, HIGH, 20000); // 20ms timeout (~3.4m)
  if (duration == 0) return 999.0f;
  return (duration * 0.0343f) / 2.0f;
}

String SensorSuite::readRFIDRaw() {
  if (!rfidReady) return "";
  if (!rfid.PICC_IsNewCardPresent() || !rfid.PICC_ReadCardSerial()) return "";

  String uidStr = "";
  for (byte i = 0; i < rfid.uid.size; i++) {
    uidStr += String(rfid.uid.uidByte[i] < 0x10 ? "0" : "");
    uidStr += String(rfid.uid.uidByte[i], HEX);
  }
  rfid.PICC_HaltA();
  rfid.PCD_StopCrypto1();
  uidStr.toUpperCase();
  return uidStr;
}

void SensorSuite::update() {
  if (millis() - lastSensorPoll < 50) return; // 20 Hz update rate
  lastSensorPoll = millis();

  // Poll ToF
  if (tofReady) {
    uint16_t d = tof.readRangeContinuousMillimeters();
    if (!tof.timeoutOccurred()) currentTofDistMM = d;
  }

  // Poll Ultrasonic
  currentUltrasonicCM = readUltrasonicRaw();

  // Poll RFID
  String newTag = readRFIDRaw();
  if (newTag.length() > 0) {
    currentRFIDTag = newTag;
    Serial.printf("[RFID] Scanned Tag: %s\n", currentRFIDTag.c_str());
  }
}

uint16_t SensorSuite::getTofDistanceMM() {
  return currentTofDistMM;
}

float SensorSuite::getUltrasonicDistanceCM() {
  return currentUltrasonicCM;
}

String SensorSuite::getLastScannedRFID() {
  return currentRFIDTag;
}

bool SensorSuite::isForwardPathBlocked() {
  // If ToF sees an obstacle closer than 12 cm or Ultrasonic closer than 15 cm
  return (currentTofDistMM < 120 || currentUltrasonicCM < 15.0f);
}
