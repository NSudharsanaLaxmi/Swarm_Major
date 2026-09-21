/*
 * ======================================================================================
 * STEP 2: VL53L0X Time-of-Flight (ToF) & I2C SCANNER DIAGNOSTIC
 * ======================================================================================
 * Purpose: Scan I2C bus on GPIO 21 (SDA) and GPIO 22 (SCL). Detect the VL53L0X
 *          at address 0x29 and stream live millimeter distance measurements.
 *
 * Wiring:
 *   - VL53L0X VIN -> ESP32 3V3 (or 5V if module has onboard 3.3V LDO)
 *   - VL53L0X GND -> ESP32 GND
 *   - VL53L0X SDA -> ESP32 GPIO 21
 *   - VL53L0X SCL -> ESP32 GPIO 22
 * ======================================================================================
 */

#include <Wire.h>
#include <VL53L0X.h>

VL53L0X sensor;

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("\n========================================"));
  Serial.println(F("[BOOT] STEP 2: ToF & I2C Bus Diagnostic"));
  Serial.println(F("========================================"));

  Wire.begin(21, 22);

  // --- 1. Scan I2C bus ---
  Serial.println(F("[INFO][I2C] Scanning I2C bus (SDA=21, SCL=22)..."));
  byte count = 0;
  for (byte i = 8; i < 120; i++) {
    Wire.beginTransmission(i);
    if (Wire.endTransmission() == 0) {
      Serial.printf("[INFO][I2C] Detected active device at: 0x%02X\n", i);
      count++;
    }
  }

  if (count == 0) {
    Serial.println(F("[ERROR][I2C] No I2C devices found! Check 3.3V, GND, SDA(21), SCL(22)."));
  } else {
    Serial.printf("[INFO][I2C] Total devices detected: %d\n", count);
  }

  // --- 2. Initialize VL53L0X ---
  sensor.setTimeout(500);
  if (!sensor.init()) {
    Serial.println(F("[ERROR][TOF] Failed to initialize VL53L0X!"));
    Serial.println(F("[ERROR][TOF] Check that XSHUT pin is not pulled LOW."));
    while (1) {
      delay(500);
    }
  }

  // Set long range / continuous mode
  sensor.startContinuous();
  Serial.println(F("[STATE] VL53L0X Initialized successfully. Streaming measurements..."));
}

void loop() {
  uint16_t distMM = sensor.readRangeContinuousMillimeters();

  if (sensor.timeoutOccurred()) {
    Serial.println(F("[WARNING][TOF] Measurement TIMEOUT"));
  } else {
    Serial.printf("[INFO][TOF] Distance: %4d mm | %.1f cm\n", distMM, distMM / 10.0);
  }
  delay(100);
}
