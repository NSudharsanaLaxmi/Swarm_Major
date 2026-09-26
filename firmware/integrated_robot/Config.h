#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>

// =========================================================================
// 1. SWARM IDENTITY & ROBOT CONFIGURATION
// =========================================================================
// Set ROBOT_ID to 0 for Robot 1 (Marker ID 0); set to 1 for Robot 2 (Marker ID 1)
#ifndef ROBOT_ID
#define ROBOT_ID            0
#endif

#define FIRMWARE_VERSION    "v2.4.0-5DOF-MECANUM"
#define PEER_ROBOT_ID       ((ROBOT_ID == 0) ? 1 : 0)
#define ARUCO_DICT_NAME     "DICT_4X4_50"

// ArUco Fiducial Marker IDs (DICT_4X4_50):
// ID 0: ROBOT_1 | ID 1: ROBOT_2 
// ID 2: RACK_1  | ID 3: RACK_2 | ID 4: RACK_3
// ID 6: ROBOT_1_START | ID 7: ROBOT_2_START | ID 8: DELIVERY_ZONE
// ID 9: BOUNDARY_TL   | ID 10: BOUNDARY_TR  | ID 11: BOUNDARY_BR | ID 12: BOUNDARY_BL

// =========================================================================
// 2. NETWORK & UDP COMMUNICATION CONSTANTS
// =========================================================================
#define WIFI_SSID           "iPhone 17"
#define WIFI_PASS           "s1s2c7.v"

#define UDP_CMD_PORT        8888  // Direct velocity & JSON commands port
#define UDP_SWARM_PORT      5005  // Global ArUco swarm state broadcast port
#define SERIAL_BAUD_RATE    115200

// =========================================================================
// 3. TB6612FNG MOTOR CONTROLLER PINS (PRESERVED TESTED HARDWARE BASELINE)
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

// Note: Both TB6612 STBY pins are permanently tied to 3.3V

// =========================================================================
// 4. I2C BUS ALLOCATION (GPIO 21 = SDA, GPIO 22 = SCL)
// =========================================================================
#define I2C_SDA_PIN         21
#define I2C_SCL_PIN         22

#define ADDR_PCA9685        0x40  // 16-Channel PWM Servo Driver (5-DOF Arm Only)
#define ADDR_OLED           0x3C  // 0.96" 128x64 SSD1306 Display
#define ADDR_VL53L0X        0x29  // Time-of-Flight Laser Distance Sensor

// =========================================================================
// 5. PCA9685 5-DOF MG996R ROBOTIC ARM CHANNEL MAPPING (RESERVED FOR ARM ONLY)
// =========================================================================
#define SERVO_BASE          0     // Base Rotational Yaw (CH0)
#define SERVO_SHOULDER      1     // Shoulder Pitch (CH1)
#define SERVO_ELBOW         2     // Elbow Pitch (CH2)
#define SERVO_JOINT4        3     // Joint 4 / Wrist Pitch (CH3)
#define SERVO_JOINT5        4     // Joint 5 / End-Effector Gripper (CH4)
#define NUM_ARM_JOINTS      5

#define PCA9685_PWM_FREQ    50    // 50 Hz standard for MG996R analog/digital servos
#define SERVO_PULSE_MIN     150   // ~0 degrees (MG996R)
#define SERVO_PULSE_MAX     600   // ~180 degrees (MG996R)

// Joint limits and calibration structures
struct JointConfig {
  uint8_t channel;
  int minAngle;
  int maxAngle;
  int homeAngle;
  int direction; // +1 or -1
  int offset;    // Calibration trim
};

struct ArmPose {
  int base;
  int shoulder;
  int elbow;
  int joint4;
  int joint5;
};

// =========================================================================
// 6. SPI BUS & RC522 RFID READER PINS
// =========================================================================
#define RFID_SS_PIN         5     // Shared SPI CS / Hardware SPI
#define RFID_RST_PIN        2
#define RFID_SCK_PIN        18
#define RFID_MISO_PIN       19
#define RFID_MOSI_PIN       23

// =========================================================================
// 7. ULTRASONIC HC-SR04 PROXIMITY SENSOR
// =========================================================================
#define PIN_US_TRIG         15
#define PIN_US_ECHO         34    // Input-only pin (via 5V->3.3V divider)

// =========================================================================
// 8. SAFETY & KINEMATICS THRESHOLDS
// =========================================================================
#define COMMAND_TIMEOUT_MS    500   // Failsafe motor timeout
#define TOF_BRAKE_DIST_MM     120   // Safety obstacle brake distance
#define TOF_DOCK_DIST_MM      60    // Precision pallet pick distance
#define SWARM_SAFE_DIST_CM    28.0f // Inter-robot yield threshold
#define ARRIVAL_THRESH_CM     8.0f  // Waypoint arrival tolerance
#define HEADING_DEADBAND_DEG  15.0f // In-place rotation threshold

// Robot Physical Dimensions
#define ROBOT_WHEEL_BASE_M    0.15f // Distance between left and right wheels
#define ROBOT_MAX_SPEED_MPS   0.45f // Scaled max linear velocity

// =========================================================================
// 9. MISSION STATE DEFINITIONS
// =========================================================================
enum RobotMissionState {
  STATE_IDLE,
  STATE_NAV_TO_PICK,
  STATE_RACK_VERIFY,
  STATE_PRECISION_DOCK,
  STATE_PICK_PAYLOAD,
  STATE_LIFT,
  STATE_NAV_TO_DROP,
  STATE_POSITION_FOR_DROP,
  STATE_RELEASE_PAYLOAD,
  STATE_RETRACT,
  STATE_YIELDING,
  STATE_SAFETY_STOP,
  STATE_EMERGENCY_STOP,
  STATE_COMMUNICATION_LOSS,
  STATE_FAULT
};

enum ArmPoseType {
  POSE_TYPE_HOME,
  POSE_TYPE_APPROACH,
  POSE_TYPE_PICK,
  POSE_TYPE_LIFT,
  POSE_TYPE_TRANSPORT,
  POSE_TYPE_DROP,
  POSE_TYPE_RETRACT,
  POSE_TYPE_CUSTOM
};

// Global default joint configurations
extern const JointConfig DEFAULT_JOINT_CONFIGS[NUM_ARM_JOINTS];
extern const ArmPose DEFAULT_ARM_POSES[7];

#endif // CONFIG_H
