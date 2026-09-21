/*
 * ======================================================================================
 * STEP 3: INTEGRATED SWARM AGENT FIRMWARE (ESP32)
 * ======================================================================================
 * Purpose: Full production firmware for the mobile robot:
 *          - Connects to Wi-Fi hotspot.
 *          - Listens on UDP Port 8888 for velocity packets ("linear_x,angular_z").
 *          - Executes 4WD skid-steer drive on native ESP32 PWM (D4, D5).
 *          - Autonomous Local Safety: Cuts motor power if VL53L0X detects obstacle < 120 mm.
 *          - Failsafe: Stops robot if UDP stream drops for more than 500 ms.
 * ======================================================================================
 */

#include <WiFi.h>
#include <WiFiUdp.h>
#include <Wire.h>
#include <VL53L0X.h>

// ==========================================
// 1. NETWORK & SWARM IDENTITY
// ==========================================
// Set YOUR hotspot credentials here
const char* ssid     = "YOUR_HOTSPOT_NAME";
const char* password = "YOUR_HOTSPOT_PASSWORD";
const int udpPort    = 8888;

WiFiUDP udp;
char packetBuffer[255];

// ==========================================
// 2. VERIFIED MOTOR PIN MAPPING (FROZEN)
// ==========================================
const int PIN_PWM_LEFT  = 4;  // Spliced Left PWMA
const int PIN_PWM_RIGHT = 5;  // Spliced Right PWMB

const int PIN_F_AIN1 = 25;    // Front Left Dir 1
const int PIN_F_AIN2 = 26;    // Front Left Dir 2
const int PIN_F_BIN1 = 27;    // Front Right Dir 1
const int PIN_F_BIN2 = 14;    // Front Right Dir 2

const int PIN_R_AIN1 = 12;    // Rear Left Dir 1
const int PIN_R_AIN2 = 13;    // Rear Left Dir 2
const int PIN_R_BIN1 = 32;    // Rear Right Dir 1
const int PIN_R_BIN2 = 33;    // Rear Right Dir 2

// ==========================================
// 3. SENSORS & FAILSAFE PARAMETERS
// ==========================================
VL53L0X tof;
bool tofOnline = false;
const uint16_t COLLISION_THRESHOLD_MM = 120; // 12 cm emergency stopping distance

unsigned long lastCommandTime = 0;
const int timeoutMs = 500; // Stop robot if UDP commands cease for 500ms

// ==========================================
// 4. LOW-LEVEL DRIVE IMPLEMENTATION
// ==========================================
void stopMotors() {
  digitalWrite(PIN_F_AIN1, LOW); digitalWrite(PIN_F_AIN2, LOW);
  digitalWrite(PIN_R_AIN1, LOW); digitalWrite(PIN_R_AIN2, LOW);
  digitalWrite(PIN_F_BIN1, LOW); digitalWrite(PIN_F_BIN2, LOW);
  digitalWrite(PIN_R_BIN1, LOW); digitalWrite(PIN_R_BIN2, LOW);
  
  analogWrite(PIN_PWM_LEFT, 0);
  analogWrite(PIN_PWM_RIGHT, 0);
}

void driveRobot(float linear, float angular) {
  // Autonomous Safety Override: Check forward obstacle
  if (tofOnline && linear > 0) {
    uint16_t dist = tof.readRangeContinuousMillimeters();
    if (!tof.timeoutOccurred() && dist < COLLISION_THRESHOLD_MM) {
      Serial.printf("[SAFETY] Obstacle detected at %d mm! Emergency Brake Active.\n", dist);
      stopMotors();
      return;
    }
  }

  float left_speed  = linear - angular;
  float right_speed = linear + angular;

  left_speed  = constrain(left_speed, -1.0f, 1.0f);
  right_speed = constrain(right_speed, -1.0f, 1.0f);

  int pwm_left  = map((int)(abs(left_speed) * 100), 0, 100, 0, 255);
  int pwm_right = map((int)(abs(right_speed) * 100), 0, 100, 0, 255);

  // --- Left Side Direction ---
  if (left_speed > 0.05f) {
    digitalWrite(PIN_F_AIN1, HIGH); digitalWrite(PIN_F_AIN2, LOW);
    digitalWrite(PIN_R_AIN1, HIGH); digitalWrite(PIN_R_AIN2, LOW);
  } else if (left_speed < -0.05f) {
    digitalWrite(PIN_F_AIN1, LOW); digitalWrite(PIN_F_AIN2, HIGH);
    digitalWrite(PIN_R_AIN1, LOW); digitalWrite(PIN_R_AIN2, HIGH);
  } else {
    digitalWrite(PIN_F_AIN1, LOW); digitalWrite(PIN_F_AIN2, LOW);
    digitalWrite(PIN_R_AIN1, LOW); digitalWrite(PIN_R_AIN2, LOW);
    pwm_left = 0;
  }

  // --- Right Side Direction ---
  if (right_speed > 0.05f) {
    digitalWrite(PIN_F_BIN1, HIGH); digitalWrite(PIN_F_BIN2, LOW);
    digitalWrite(PIN_R_BIN1, HIGH); digitalWrite(PIN_R_BIN2, LOW);
  } else if (right_speed < -0.05f) {
    digitalWrite(PIN_F_BIN1, LOW); digitalWrite(PIN_F_BIN2, HIGH);
    digitalWrite(PIN_R_BIN1, LOW); digitalWrite(PIN_R_BIN2, HIGH);
  } else {
    digitalWrite(PIN_F_BIN1, LOW); digitalWrite(PIN_F_BIN2, LOW);
    digitalWrite(PIN_R_BIN1, LOW); digitalWrite(PIN_R_BIN2, LOW);
    pwm_right = 0;
  }

  analogWrite(PIN_PWM_LEFT, pwm_left);
  analogWrite(PIN_PWM_RIGHT, pwm_right);
}

// ==========================================
// 5. SETUP
// ==========================================
void setup() {
  Serial.begin(115200);
  delay(500);

  Serial.println(F("\n========================================"));
  Serial.println(F("[BOOT] Swarm Robot Edge Agent Starting"));
  Serial.println(F("========================================"));

  // Initialize motor pins
  pinMode(PIN_PWM_LEFT, OUTPUT);
  pinMode(PIN_PWM_RIGHT, OUTPUT);
  pinMode(PIN_F_AIN1, OUTPUT); pinMode(PIN_F_AIN2, OUTPUT);
  pinMode(PIN_F_BIN1, OUTPUT); pinMode(PIN_F_BIN2, OUTPUT);
  pinMode(PIN_R_AIN1, OUTPUT); pinMode(PIN_R_AIN2, OUTPUT);
  pinMode(PIN_R_BIN1, OUTPUT); pinMode(PIN_R_BIN2, OUTPUT);
  stopMotors();

  // Initialize I2C & ToF Sensor on GPIO 21, 22
  Wire.begin(21, 22);
  tof.setTimeout(200);
  if (tof.init()) {
    tof.startContinuous();
    tofOnline = true;
    Serial.println(F("[INFO][TOF] VL53L0X Active (SDA=21, SCL=22)"));
  } else {
    Serial.println(F("[WARNING][TOF] VL53L0X not found! Running in blind mode."));
  }

  // Connect to Wi-Fi
  Serial.printf("[COMM] Connecting to SSID: %s\n", ssid);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.status() != WL_CONNECTED) {
    delay(400);
    Serial.print(".");
  }

  Serial.println(F("\n[COMM] WiFi Connected."));
  Serial.print(F("[COMM] Robot IP: "));
  Serial.println(WiFi.localIP());

  udp.begin(udpPort);
  Serial.printf("[COMM] UDP Listener active on port %d\n", udpPort);
  Serial.println(F("[STATE] Agent Ready for Swarm Coordination."));
}

// ==========================================
// 6. MAIN LOOP
// ==========================================
void loop() {
  int packetSize = udp.parsePacket();
  if (packetSize) {
    int len = udp.read(packetBuffer, 254);
    if (len > 0) packetBuffer[len] = '\0';

    String data = String(packetBuffer);
    int commaIdx = data.indexOf(',');
    if (commaIdx > 0) {
      float linear_x  = data.substring(0, commaIdx).toFloat();
      float angular_z = data.substring(commaIdx + 1).toFloat();

      driveRobot(linear_x, angular_z);
      lastCommandTime = millis();
    }
  }

  // Safety Failsafe: stop if connection silent > timeoutMs
  if (millis() - lastCommandTime > timeoutMs) {
    stopMotors();
  }
}
