#include "Config.h"

// Default configurable Joint Configurations (Base, Shoulder, Elbow, Joint4, Joint5)
// Values reflect mechanical safety envelopes without hardcoded unverified assumptions
const JointConfig DEFAULT_JOINT_CONFIGS[NUM_ARM_JOINTS] = {
  { SERVO_BASE,     0,   180, 90,  1, 0 },  // CH0: Base Yaw (0-180°, Home=90°)
  { SERVO_SHOULDER, 15,  165, 90,  1, 0 },  // CH1: Shoulder Pitch (15-165°, Home=90°)
  { SERVO_ELBOW,    10,  170, 90,  1, 0 },  // CH2: Elbow Pitch (10-170°, Home=90°)
  { SERVO_JOINT4,   10,  170, 90,  1, 0 },  // CH3: Joint 4 / Wrist Pitch (10-170°, Home=90°)
  { SERVO_JOINT5,   45,  180, 180, 1, 0 }   // CH4: Joint 5 / Gripper (45° Closed, 180° Open)
};

// Default configurable Mission Poses for 5-DOF Arm:
// Order: { Base, Shoulder, Elbow, Joint4, Joint5 }
const ArmPose DEFAULT_ARM_POSES[7] = {
  { 90, 90,  90,  90, 180 }, // POSE_TYPE_HOME
  { 90, 65, 115,  90, 180 }, // POSE_TYPE_APPROACH (Hover over rack)
  { 90, 45, 135,  90,  45 }, // POSE_TYPE_PICK (Grip payload)
  { 90, 80, 100,  90,  45 }, // POSE_TYPE_LIFT (Elevate secured payload)
  { 90, 110, 70,  90,  45 }, // POSE_TYPE_TRANSPORT (Compact center of mass)
  { 90, 45, 135,  90, 180 }, // POSE_TYPE_DROP (Release payload at drop station)
  { 90, 100, 80,  90, 180 }  // POSE_TYPE_RETRACT (Safe tuck before driving)
};
