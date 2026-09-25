# Autonomous Warehouse Swarm Platform: ArUco Boundary Architecture

This project implements an AI-driven, multi-agent warehouse swarm robotics platform. The vision perception engine uses **`DICT_4X4_50`** ArUco fiducials to dynamically calibrate the workspace frame and enforce hard workcell boundary conditions.

---

## 1. ArUco Marker Set (`DICT_4X4_50`)

| ID | Name | Role & Function |
|:---:|:---|:---|
| **0** | `ROBOT_1` | Moving marker on Robot 1 |
| **1** | `ROBOT_2` | Moving marker on Robot 2 |
| **2** | `RACK_1` | Fixed marker on Storage Rack 1 |
| **3** | `RACK_2` | Fixed marker on Storage Rack 2 |
| **4** | `RACK_3` | Fixed marker on Storage Rack 3 |
| **5** | `RACK_4` | Fixed marker on Storage Rack 4 |
| **6** | `ROBOT_1_START` | Staging location for Robot 1 |
| **7** | `ROBOT_2_START` | Staging location for Robot 2 |
| **8** | `DELIVERY_ZONE` | Product drop-off destination |
| **9** | `BOUNDARY_TL` | Workcell Calibration: Top-Left Corner |
| **10** | `BOUNDARY_TR` | Workcell Calibration: Top-Right Corner |
| **11** | `BOUNDARY_BR` | Workcell Calibration: Bottom-Right Corner |
| **12** | `BOUNDARY_BL` | Workcell Calibration: Bottom-Left Corner |

---

## 2. Perception-to-Control Flow

```
ArUco Detection (DICT_4X4_50)
        │
        ▼
Boundary Calibration (IDs 9,10,11,12)
        │
        ▼
Workspace Coordinate Frame (Homography Matrix H)
        │
        ▼
Robot & Fixed Landmark Localization (IDs 0-8)
        │
        ▼
Occupancy & Boundary Map (8 cm Safety Buffer)
        │
        ▼
Task Allocation & Path Planning
        │
        ▼
Collision Avoidance & Boundary Override
        │
        ▼
Local ESP32 Motor & Manipulator Control
```

---

## 3. Directory Structure & Files

```
swarm-warehouse/
├── README.md
├── firmware/
│   ├── step1_motor_diagnostic/step1_motor_diagnostic.ino
│   ├── step2_tof_diagnostic/step2_tof_diagnostic.ino
│   ├── step3_esp32_swarm_agent/step3_esp32_swarm_agent.ino
│   └── integrated_robot/
│       ├── Config.h
│       ├── MotorDriver.h / MotorDriver.cpp
│       ├── ArmController.h / ArmController.cpp
│       ├── SensorSuite.h / SensorSuite.cpp
│       ├── DisplayManager.h / DisplayManager.cpp
│       ├── SwarmComms.h / SwarmComms.cpp
│       └── integrated_robot.ino
└── server/
    ├── warehouse_central_server.py      # Main ArUco boundary perception & dispatch server
    ├── standalone_swarm_coordinator.py  # Standalone boundary-aware coordinator
    ├── step4_udp_bridge.py              # ROS 2 /cmd_vel UDP bridge
    ├── step5_overhead_vision_tracker.py # DICT_4X4_50 Vision tracker
    ├── step6_closed_loop_coordinator.py # ROS 2 closed-loop coordinator
    ├── teleop_keyboard_udp.py           # Standalone keyboard teleop
    └── uno_q_swarm_agent.py             # Arduino UNO Q Linux MPU UART agent
```

---

## 4. Execution Commands

### 1. Launch Warehouse Central Server & Vision Dispatcher
```bash
python server/warehouse_central_server.py --source 0
```

### 2. Launch Standalone Swarm Autonomous Coordinator
```bash
python server/standalone_swarm_coordinator.py --robot_ip <ESP32_IP> --bot_id 0 --tx 30.0 --ty 50.0
```
