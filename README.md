# Swarm Warehouse Robotics: Master Run & Verification Guide

This directory contains the complete step-by-step codebase for the autonomous warehouse swarm platform.
All code has been revised to use your **experimentally validated hardware baseline**:
- **Native ESP32 PWM:** Left Side on `GPIO 4`, Right Side on `GPIO 5`.
- **Direction GPIOs (8 pins):** `25, 26, 27, 14, 12, 13, 32, 33`.
- **STBY Pins:** Hardwired to `3.3V` (HIGH).
- **I2C Bus:** `GPIO 21` (SDA), `GPIO 22` (SCL) for VL53L0X ToF & OLED.

---

## Directory Structure

```
swarm-warehouse/
├── firmware/
│   ├── step1_motor_diagnostic/
│   │   └── step1_motor_diagnostic.ino       # Isolated 4WD hardware test
│   ├── step2_tof_diagnostic/
│   │   └── step2_tof_diagnostic.ino         # I2C bus scan & VL53L0X distance reading
│   └── step3_esp32_swarm_agent/
│       └── step3_esp32_swarm_agent.ino     # Full production robot firmware with active ToF braking
└── server/
    ├── step4_udp_bridge.py                 # ROS 2 /cmd_vel -> UDP port 8888 bridge
    ├── step5_overhead_vision_tracker.py    # Overhead camera ArUco marker tracker (Port 5005)
    └── step6_closed_loop_coordinator.py    # Waypoint navigation & collision avoidance coordinator
```

---

## Step-by-Step Execution Sequence

### Step 1: 4WD Motor Diagnostic (Isolated Hardware Check)
1. Open `firmware/step1_motor_diagnostic/step1_motor_diagnostic.ino` in Arduino IDE.
2. Select Board: **ESP32 Dev Module**, select your **COM port**.
3. Prop the robot chassis on a stand so all 4 wheels can spin freely.
4. Upload and open Serial Monitor at **115200 baud**.
5. **Expected Observation:**
   - The robot cycles: Forward (1.5s) -> Stop -> Reverse (1.5s) -> Stop -> Pivot Left (1s) -> Stop.
   - If any wheel spins the wrong way, swap its two direction pins in code.

---

### Step 2: Time-of-Flight (VL53L0X) Diagnostic
1. Connect sensor: `VIN` -> 3.3V, `GND` -> GND, `SDA` -> `GPIO 21`, `SCL` -> `GPIO 22`.
2. Open `firmware/step2_tof_diagnostic/step2_tof_diagnostic.ino` in Arduino IDE.
3. Install the **VL53L0X** library by Pololu from Library Manager.
4. Upload and open Serial Monitor at **115200 baud**.
5. **Expected Observation:**
   - I2C scanner reports `Found active device at: 0x29`.
   - Continuous distance readings in mm stream to the monitor.

---

### Step 3: Production ESP32 Swarm Agent Firmware
1. Open `firmware/step3_esp32_swarm_agent/step3_esp32_swarm_agent.ino`.
2. Update `ssid` and `password` with your hotspot credentials.
3. Upload to the ESP32 and open Serial Monitor.
4. **Expected Observation:**
   - `[COMM] WiFi Connected.`
   - `[COMM] Robot IP: 172.20.10.x` (Take note of this assigned IP address).
   - `[INFO][TOF] VL53L0X Active`.
   - `[COMM] UDP Listener active on port 8888`.

---

### Step 4: ROS 2 to UDP Velocity Bridge
Run this on your Linux machine / Docker container running ROS 2 Humble:
```bash
python3 server/step4_udp_bridge.py --ip <ESP32_IP_FROM_STEP_3> --port 8888
```
* Every velocity message published to `/cmd_vel` is serialized to `"linear_x,angular_z"` and sent directly to your robot.

---

### Step 5: Overhead Computer Vision Tracker
Run this in a separate terminal:
```bash
# For default webcam:
python3 server/step5_overhead_vision_tracker.py --source 0

# Or for phone IP webcam stream:
python3 server/step5_overhead_vision_tracker.py --source "http://172.20.10.2:8080/video"
```
* Tracks ArUco token `ID 0` (Robot 0) and `ID 1` (Robot 1) and broadcasts global metric positions across the local network on UDP port `5005`.

---

### Step 6: Closed-Loop Swarm Coordinator & Collision Avoidance
Run this in another terminal:
```bash
python3 server/step6_closed_loop_coordinator.py --bot_id 0 --tx 35.0 --ty 50.0
```
* Drives Robot 0 to $(35\text{ cm}, 50\text{ cm})$ autonomously.
* If Robot 1 approaches within $28\text{ cm}$, the priority logic yields right-of-way and halts to prevent collisions.
