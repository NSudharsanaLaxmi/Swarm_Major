/*
 * ======================================================================================
 * STEP 1: ESP32 4WD LOW-LEVEL MOTOR HARDWARE DIAGNOSTIC
 * ======================================================================================
 * Purpose: Verify that all 4 motors and both TB6612FNG drivers function correctly
 *          without any Wi-Fi, ROS 2, or sensor dependencies.
 *
 * Hardware Pinout (FROZEN):
 *   - Left PWM (Front & Rear PWMA spliced):  GPIO 4
 *   - Right PWM (Front & Rear PWMB spliced): GPIO 5
 *   - Front-Left AIN1: GPIO 25, AIN2: GPIO 26
 *   - Front-Right BIN1: GPIO 27, BIN2: GPIO 14
 *   - Rear-Left AIN1: GPIO 12, AIN2: GPIO 13
 *   - Rear-Right BIN1: GPIO 32, BIN2: GPIO 33
 *   - STBY pins on BOTH drivers: Hardwired to 3.3V
 * ======================================================================================
 */

// Spliced PWM Pins
const int PIN_PWM_LEFT  = 4;
const int PIN_PWM_RIGHT = 5;

// 8 Direction Pins
const int PIN_F_AIN1 = 25;
const int PIN_F_AIN2 = 26;
const int PIN_F_BIN1 = 27;
const int PIN_F_BIN2 = 14;

const int PIN_R_AIN1 = 12;
const int PIN_R_AIN2 = 13;
const int PIN_R_BIN1 = 32;
const int PIN_R_BIN2 = 33;

void setLeftMotors(int speed) {
  if (speed > 0) {
    digitalWrite(PIN_F_AIN1, HIGH); digitalWrite(PIN_F_AIN2, LOW);
    digitalWrite(PIN_R_AIN1, HIGH); digitalWrite(PIN_R_AIN2, LOW);
  } else if (speed < 0) {
    digitalWrite(PIN_F_AIN1, LOW); digitalWrite(PIN_F_AIN2, HIGH);
    digitalWrite(PIN_R_AIN1, LOW); digitalWrite(PIN_R_AIN2, HIGH);
  } else {
    digitalWrite(PIN_F_AIN1, LOW); digitalWrite(PIN_F_AIN2, LOW);
    digitalWrite(PIN_R_AIN1, LOW); digitalWrite(PIN_R_AIN2, LOW);
  }
  analogWrite(PIN_PWM_LEFT, constrain(abs(speed), 0, 255));
}

void setRightMotors(int speed) {
  if (speed > 0) {
    digitalWrite(PIN_F_BIN1, HIGH); digitalWrite(PIN_F_BIN2, LOW);
    digitalWrite(PIN_R_BIN1, HIGH); digitalWrite(PIN_R_BIN2, LOW);
  } else if (speed < 0) {
    digitalWrite(PIN_F_BIN1, LOW); digitalWrite(PIN_F_BIN2, HIGH);
    digitalWrite(PIN_R_BIN1, LOW); digitalWrite(PIN_R_BIN2, HIGH);
  } else {
    digitalWrite(PIN_F_BIN1, LOW); digitalWrite(PIN_F_BIN2, LOW);
    digitalWrite(PIN_R_BIN1, LOW); digitalWrite(PIN_R_BIN2, LOW);
  }
  analogWrite(PIN_PWM_RIGHT, constrain(abs(speed), 0, 255));
}

void stopAllMotors() {
  setLeftMotors(0);
  setRightMotors(0);
}

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println(F("\n========================================"));
  Serial.println(F("[BOOT] STEP 1: 4WD Hardware Diagnostic"));
  Serial.println(F("[BOOT] Native ESP32 PWM on Pins D4 & D5"));
  Serial.println(F("========================================"));

  pinMode(PIN_PWM_LEFT, OUTPUT);
  pinMode(PIN_PWM_RIGHT, OUTPUT);

  pinMode(PIN_F_AIN1, OUTPUT); pinMode(PIN_F_AIN2, OUTPUT);
  pinMode(PIN_F_BIN1, OUTPUT); pinMode(PIN_F_BIN2, OUTPUT);
  pinMode(PIN_R_AIN1, OUTPUT); pinMode(PIN_R_AIN2, OUTPUT);
  pinMode(PIN_R_BIN1, OUTPUT); pinMode(PIN_R_BIN2, OUTPUT);

  stopAllMotors();
  Serial.println(F("[MOTOR] Pins configured. Starting test cycle in 2 seconds..."));
  delay(2000);
}

void loop() {
  Serial.println(F("[MOTOR] Driving FORWARD (PWM: 140)..."));
  setLeftMotors(140);
  setRightMotors(140);
  delay(1500);

  Serial.println(F("[MOTOR] STOP"));
  stopAllMotors();
  delay(1000);

  Serial.println(F("[MOTOR] Driving REVERSE (PWM: 140)..."));
  setLeftMotors(-140);
  setRightMotors(-140);
  delay(1500);

  Serial.println(F("[MOTOR] STOP"));
  stopAllMotors();
  delay(1000);

  Serial.println(F("[MOTOR] Pivoting LEFT (CCW)..."));
  setLeftMotors(-130);
  setRightMotors(130);
  delay(1000);

  Serial.println(F("[MOTOR] STOP"));
  stopAllMotors();
  delay(3000);
}
