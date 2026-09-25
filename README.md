# Autonomous Warehouse Swarm Platform: ArUco Boundary Architecture

This project implements an AI-driven, multi-agent warehouse swarm robotics platform. The vision perception engine uses **`DICT_4X4_50`** ArUco fiducials to dynamically calibrate the workspace frame and enforce hard workcell boundary conditions.

---

## 1. ArUco Marker Set (`DICT_4X4_50`)

| ID | Name | Role & Function | Physical Coordinate |
|:---:|:---|:---|:---|
| **0** | `ROBOT_1` | Moving marker on Robot 1 | Dynamic Pose $(X, Y, \theta)$ |
| **1** | `ROBOT_2` | Moving marker on Robot 2 | Dynamic Pose $(X, Y, \theta)$ |
| **2** | `RACK_1` | Fixed Storage Rack 1 (Pickup face: South) | $(35.0, 25.0)\text{ cm}$ |
| **3** | `RACK_2` | Fixed Storage Rack 2 (Pickup face: South) | $(85.0, 25.0)\text{ cm}$ |
| **4** | `RACK_3` | Fixed Storage Rack 3 (Pickup face: North) | $(60.0, 75.0)\text{ cm}$ |
| **5** | `NAV_CENTER` | Optional central navigation reference | $(60.0, 50.0)\text{ cm}$ |
| **6** | `ROBOT_1_START` | Staging location for Robot 1 | $(20.0, 102.0)\text{ cm}$ |
| **7** | `ROBOT_2_START` | Staging location for Robot 2 | $(100.0, 102.0)\text{ cm}$ |
| **8** | `DELIVERY_ZONE` | Product drop-off destination (Pickup face: North) | $(60.0, 102.0)\text{ cm}$ |
| **9** | `BOUNDARY_TL` | Workcell Calibration: Top-Left Corner | $(0.0, 0.0)\text{ cm}$ |
| **10** | `BOUNDARY_TR` | Workcell Calibration: Top-Right Corner | $(120.0, 0.0)\text{ cm}$ |
| **11** | `BOUNDARY_BR` | Workcell Calibration: Bottom-Right Corner | $(120.0, 120.0)\text{ cm}$ |
| **12** | `BOUNDARY_BL` | Workcell Calibration: Bottom-Left Corner | $(0.0, 120.0)\text{ cm}$ |

---

## 2. Warehouse Layout & Navigation Corridors

The arena is calibrated to a metric **120 cm × 120 cm** coordinate space with an **8.0 cm inner safety buffer margin**.

```text
┌────────────────────────────────────────────────────────┐
│ [9] BOUNDARY_TL                    [10] BOUNDARY_TR    │
│ ┌────────────────────────────────────────────────────┐ │
│ │ 8 cm SAFETY BUFFER MARGIN                          │ │
│ │       ┌─────────┐                ┌─────────┐       │ │
│ │       │ RACK 1  │ (35, 25)       │ RACK 2  │(85,25)│ │
│ │       │  (ID 2) │                │  (ID 3) │       │ │
│ │       └────▼────┘                └────▼────┘       │ │
│ │            │ (South Approach)         │            │ │
│ │            ▼                          ▼            │ │
│ │       ──────────────────────────────────────       │ │
│ │            OPEN CENTRAL MANEUVERING ZONE           │ │
│ │            (X: 25-95 cm, Y: 38-65 cm)              │ │
│ │       ──────────────────────────────────────       │ │
│ │                          ▲                         │ │
│ │                          │ (North Approach)        │ │
│ │                     ┌────▲────┐                    │ │
│ │                     │ RACK 3  │ (60, 75)           │ │
│ │                     │  (ID 4) │                    │ │
│ │                     └─────────┘                    │ │
│ │                                                    │ │
│ │   [6] START 1         [8] DROP          [7] START 2│ │
│ │    (20, 102)          (60, 102)          (100, 102)│ │
│ └────────────────────────────────────────────────────┘ │
│ [12] BOUNDARY_BL                   [11] BOUNDARY_BR    │
└────────────────────────────────────────────────────────┘
```

### Approach / Pickup / Exit Navigation Poses:
* **Rack 1**: Approach $(35.0, 45.0, 90^\circ) \to$ Pickup $(35.0, 32.0, 90^\circ) \to$ Exit $(35.0, 55.0, 90^\circ)$
* **Rack 2**: Approach $(85.0, 45.0, 90^\circ) \to$ Pickup $(85.0, 32.0, 90^\circ) \to$ Exit $(85.0, 55.0, 90^\circ)$
* **Rack 3**: Approach $(60.0, 55.0, -90^\circ) \to$ Pickup $(60.0, 68.0, -90^\circ) \to$ Exit $(60.0, 45.0, -90^\circ)$
* **Delivery Zone**: Approach $(60.0, 88.0, -90^\circ) \to$ Drop $(60.0, 98.0, -90^\circ) \to$ Exit $(60.0, 85.0, -90^\circ)$

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
