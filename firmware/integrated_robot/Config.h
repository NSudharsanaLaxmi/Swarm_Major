#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =========================================================================
// 1. SWARM IDENTITY & ARUCO DICTIONARY (DICT_4X4_50)
// =========================================================================
// Set MY_ROBOT_ID to 0 for Robot 1 (Marker ID 0); set to 1 for Robot 2 (Marker ID 1)
#define MY_ROBOT_ID         0
#define PEER_ROBOT_ID       ((MY_ROBOT_ID == 0) ? 1 : 0)

#define ARUCO_DICT_NAME     "DICT_4X4_50"

// Marker ID Mapping (DICT_4X4_50):
// ID 0: ROBOT_1 | ID 1: ROBOT_2 
// ID 2: RACK_1  | ID 3: RACK_2 | ID 4: RACK_3
// ID 6: ROBOT_1_START | ID 7: ROBOT_2_START | ID 8: DELIVERY_ZONE
// ID 9: BOUNDARY_TL   | ID 10: BOUNDARY_TR  | ID 11: BOUNDARY_BR | ID 12: BOUNDARY_BL

#define WIFI_SSID           "YOUR_HOTSPOT_NAME"
#define WIFI_PASS           "YOUR_HOTSPOT_PASSWORD"

#define UDP_CMD_PORT        8888  // Direct velocity streaming port
#define UDP_SWARM_PORT      5005  // Global ArUco swarm state broadcast port

// =========================================================================
// 2. TB6612FNG MOTOR CONTROLLER PINS (FROZEN PHYSICAL BASELINE)
// =========================================================================
// Native ESP32 hardware PWM (8-bit, 0-255)
#define PIN_PWM_LEFT        4     // Spliced Front-Left and Rear-Left PWMA
#define PIN_PWM_RIGHT       5     // Spliced Front-Right and Rear-Right PWMB

// Front Motor Driver (TB6612 #1)
#define PIN_F_AIN1          25    // Front-Left Direction 1
#define PIN_F_AIN2          26    // Front-Left Direction 2
#define PIN_F_BIN1          27    // Front-Right Direction 1
#define PIN_F_BIN2          14    // Front-Right Direction 2

// Rear Motor Driver (TB6612 #2)
#define PIN_R_AIN1          12    // Rear-Left Direction 1
#define PIN_R_AIN2          13    // Rear-Left Direction 2
#define PIN_R_BIN1          32    // Rear-Right Direction 1
#define PIN_R_BIN2          33    // Rear-Right Direction 2

// Note: Both TB6612 STBY pins are hardwired directly to 3.3V

// =========================================================================
// 3. I2C BUS ALLOCATION (GPIO 21 = SDA, GPIO 22 = SCL)
// =========================================================================
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22

#define ADDR_PCA9685        0x40  // 16-Channel PWM Servo Driver
#define ADDR_OLED           0x3C  // 0.96" 128x64 SSD1306 Display
#define ADDR_VL53L0X        0x29  // Time-of-Flight Laser Distance Sensor

// =========================================================================
// 4. PCA9685 SERVO CHANNEL ALLOCATION (4-DOF ARM + GRIPPER)
// =========================================================================
#define SERVO_CH_BASE       0
#define SERVO_CH_SHOULDER   1
#define SERVO_CH_ELBOW      2
#define SERVO_CH_WRIST      3
#define SERVO_CH_GRIPPER    4

#define SERVO_PULSE_MIN     150   // ~0 degrees (MG90S)
#define SERVO_PULSE_MAX     600   // ~180 degrees (MG90S)

#define GRIPPER_OPEN_DEG    180
#define GRIPPER_CLOSED_DEG  85

// =========================================================================
// 5. SPI BUS & RC522 RFID READER PINS
// =========================================================================
#define RFID_SS_PIN         5
#define RFID_RST_PIN        2
#define RFID_SCK_PIN        18
#define RFID_MISO_PIN       19
#define RFID_MOSI_PIN       23

// =========================================================================
// 6. ULTRASONIC HC-SR04 PROXIMITY SENSOR
// =========================================================================
#define PIN_US_TRIG         15
#define PIN_US_ECHO         34    // Input-only pin (via 5V->3.3V divider)

// =========================================================================
// 7. SWARM KINEMATICS & SAFETY THRESHOLDS
// =========================================================================
#define SWARM_SAFE_DIST_CM    28.0f // Inter-robot yield threshold
#define ARRIVAL_THRESH_CM     8.0f  // Waypoint arrival tolerance
#define HEADING_DEADBAND_DEG  20.0f // Pivoting threshold
#define TOF_DOCK_DIST_MM      60    // Precision pallet pick distance
#define WATCHDOG_TIMEOUT_MS   600   // Failsafe brake timeout

#endif // CONFIG_H
