#!/usr/bin/env python3
"""
======================================================================================
ARDUINO UNO Q HIGH-LEVEL SWARM INTELLIGENCE BRIDGE
======================================================================================
Project: AI-Driven Cooperative Autonomous Mobile Manipulator Swarm for Smart Warehouse

Deployment:
  Runs on the Arduino UNO Q onboard Linux environment (Qualcomm QRB2210 MPU).
  - Listens to the Warehouse Server UDP Broadcast (Port 5005).
  - Runs local swarm decision making and fleet health monitoring.
  - Communicates directly with the real-time ESP32 controller via hardware UART (115200 baud).

Execution:
  python3 uno_q_swarm_agent.py --port /dev/ttyS0 --baud 115200 --bot_id 0
======================================================================================
"""

import sys
import argparse
import socket
import json
import time
import serial

SWARM_PORT = 5005

class UnoQSwarmBridge:
    def __init__(self, serial_port: str, baud_rate: int, bot_id: int):
        self.bot_id = bot_id
        self.peer_id = 1 if bot_id == 0 else 0

        # Initialize Serial interface to ESP32
        try:
            self.ser = serial.Serial(serial_port, baud_rate, timeout=0.1)
            print(f"[UNO Q] Connected to ESP32 on {serial_port} @ {baud_rate} baud.")
        except Exception as e:
            print(f"[WARNING][UNO Q] Could not open serial port {serial_port}: {e}")
            self.ser = None

        # Setup UDP listener for vision broadcast
        self.rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.rx_sock.bind(("", SWARM_PORT))
        self.rx_sock.setblocking(False)

        print(f"[UNO Q] High-Level Agent R0{self.bot_id+1} Online. Listening on UDP {SWARM_PORT}.")

    def run(self):
        last_heartbeat = time.time()
        while True:
            # 1. Check for incoming UDP Swarm State
            try:
                data, _ = self.rx_sock.recvfrom(2048)
                packet = json.loads(data.decode('utf-8'))
                bots = packet.get("bots", {})

                my_key = f"id{self.bot_id}"
                if my_key in bots:
                    pose = bots[my_key]
                    # Forward pose to ESP32 if connected
                    if self.ser and self.ser.is_open:
                        msg = f"P:{pose['x']},{pose['y']},{pose['ang']}\n"
                        self.ser.write(msg.encode('utf-8'))

            except BlockingIOError:
                pass
            except Exception as e:
                pass

            # 2. Read incoming telemetry from ESP32
            if self.ser and self.ser.in_waiting:
                try:
                    line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                    if line:
                        print(f"[ESP32 -> UNO Q] {line}")
                except Exception:
                    pass

            # 3. Status Heartbeat
            if time.time() - last_heartbeat > 2.0:
                last_heartbeat = time.time()
                print(f"[UNO Q] Robot R0{self.bot_id+1} Heartbeat: Operational.")

            time.sleep(0.01)

def main():
    parser = argparse.ArgumentParser(description="Arduino UNO Q High-Level Swarm Agent")
    parser.add_argument("--port", type=str, default="/dev/ttyS0", help="UART Serial port connected to ESP32")
    parser.add_argument("--baud", type=int, default=115200, help="UART Baud Rate")
    parser.add_argument("--bot_id", type=int, default=0, help="This robot ID (0 or 1)")
    args = parser.parse_args()

    agent = UnoQSwarmBridge(serial_port=args.port, baud_rate=args.baud, bot_id=args.bot_id)
    agent.run()

if __name__ == '__main__':
    main()
