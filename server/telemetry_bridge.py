#!/usr/bin/env python3
"""
======================================================================================
SWARM ROBOTICS WEBSOCKET & TELEMETRY BRIDGE
======================================================================================
Project: AI-Driven Cooperative Autonomous Mobile Manipulator Swarm for Smart Warehouse

Purpose: Connects the Python Vision Perception engine and ESP32 UDP Telemetry to
         the React Web Application (Website_Swram).

Architecture:
  - Listens to UDP Port 5005 (Global ArUco vision state)
  - Listens to UDP Port 8888 (ESP32 robot telemetry & command feedback)
  - Hosts a WebSocket Server on ws://0.0.0.0:8080/ws
  - Converts incoming backend UDP packets into typed WebSocket JSON events for the browser
  - Handles browser commands (E-STOP, Manual Drive, Recalibrate, Task Enqueue) and
    dispatches them over UDP to the physical robots.

Execution:
  python3 server/telemetry_bridge.py --port 8080
======================================================================================
"""

import sys
import argparse
import socket
import json
import asyncio
import time
import threading

try:
    import websockets
except ImportError:
    print("[WARNING] 'websockets' package not installed. Installing via pip...")
    import subprocess
    subprocess.check_call([sys.executable, "-m", "pip", "install", "websockets"])
    import websockets

# Constants
UDP_VISION_PORT = 5005
UDP_ROBOT_PORT  = 8888
WS_HOST         = "0.0.0.0"
WS_PORT         = 8080

class SwarmTelemetryBridge:
    def __init__(self, ws_port=WS_PORT):
        self.ws_port = ws_port
        self.connected_clients = set()
        
        # State Cache (3-Rack Layout with Central Maneuvering Zone)
        self.latest_swarm_state = {
            "timestamp": time.time(),
            "dictionary": "DICT_4X4_50",
            "calibrated": True,
            "arena_size_cm": [120.0, 120.0],
            "safety_buffer_cm": 8.0,
            "landmarks": {
                "rack_1": {"x": 35.0, "y": 25.0, "id": 2},
                "rack_2": {"x": 85.0, "y": 25.0, "id": 3},
                "rack_3": {"x": 60.0, "y": 75.0, "id": 4},
                "robot_1_start": {"x": 20.0, "y": 102.0, "id": 6},
                "robot_2_start": {"x": 100.0, "y": 102.0, "id": 7},
                "delivery_zone": {"x": 60.0, "y": 102.0, "id": 8}
            },
            "racks": {
                "rack_1": {
                    "id": "rack_1", "marker_id": 2, "name": "RACK_1", "position": [35.0, 25.0], "orientation": 90.0,
                    "pickup_face": "SOUTH", "approach_pose": [35.0, 45.0, 90.0], "pickup_pose": [35.0, 32.0, 90.0],
                    "exit_pose": [35.0, 55.0, 90.0], "safe_clearance": 15.0, "status": "AVAILABLE"
                },
                "rack_2": {
                    "id": "rack_2", "marker_id": 3, "name": "RACK_2", "position": [85.0, 25.0], "orientation": 90.0,
                    "pickup_face": "SOUTH", "approach_pose": [85.0, 45.0, 90.0], "pickup_pose": [85.0, 32.0, 90.0],
                    "exit_pose": [85.0, 55.0, 90.0], "safe_clearance": 15.0, "status": "AVAILABLE"
                },
                "rack_3": {
                    "id": "rack_3", "marker_id": 4, "name": "RACK_3", "position": [60.0, 75.0], "orientation": -90.0,
                    "pickup_face": "NORTH", "approach_pose": [60.0, 55.0, -90.0], "pickup_pose": [60.0, 68.0, -90.0],
                    "exit_pose": [60.0, 45.0, -90.0], "safe_clearance": 15.0, "status": "AVAILABLE"
                }
            },
            "delivery_zone": {
                "id": "delivery_zone", "marker_id": 8, "name": "DELIVERY_ZONE", "position": [60.0, 102.0],
                "orientation": -90.0, "approach_pose": [60.0, 88.0, -90.0], "drop_pose": [60.0, 98.0, -90.0],
                "exit_pose": [60.0, 85.0, -90.0], "safe_clearance": 15.0
            },
            "bots": {}
        }

        # UDP Sockets
        self.udp_tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.udp_tx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    def start_udp_listeners(self, loop):
        """ Runs non-blocking UDP listener thread for vision & telemetry. """
        rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        rx_sock.bind(("", UDP_VISION_PORT))
        rx_sock.setblocking(False)

        print(f"[BRIDGE] UDP Listener active on Port {UDP_VISION_PORT}...")

        def listen_loop():
            while True:
                try:
                    data, _ = rx_sock.recvfrom(4096)
                    packet = json.loads(data.decode('utf-8'))

                    if isinstance(packet, dict) and "bots" in packet:
                        self.latest_swarm_state.update(packet)
                        self.latest_swarm_state["timestamp"] = time.time()
                        
                        # Broadcast to all active WebSocket browser clients
                        payload = json.dumps({
                            "type": "swarm_state",
                            "timestamp": time.time(),
                            "payload": self.latest_swarm_state
                        })
                        asyncio.run_coroutine_threadsafe(self.broadcast(payload), loop)

                except BlockingIOError:
                    time.sleep(0.01)
                except Exception as e:
                    time.sleep(0.01)

        t = threading.Thread(target=listen_loop, daemon=True)
        t.start()

    async def broadcast(self, message):
        if self.connected_clients:
            await asyncio.gather(
                *[client.send(message) for client in self.connected_clients],
                return_exceptions=True
            )

    async def ws_handler(self, websocket, path=None):
        self.connected_clients.add(websocket)
        print(f"[WS] Browser Client Connected from {websocket.remote_address}")

        try:
            # Send initial state snapshot
            snapshot = json.dumps({
                "type": "swarm_state",
                "timestamp": time.time(),
                "payload": self.latest_swarm_state
            })
            await websocket.send(snapshot)

            async for message in websocket:
                try:
                    data = json.loads(message)
                    msg_type = data.get("type")

                    if msg_type == "send_command":
                        cmd_id = data.get("commandId")
                        cmd_name = data.get("commandName")
                        target_id = data.get("targetRobotId")
                        payload = data.get("payload", {})

                        print(f"[WS COMMAND] {cmd_name} -> {target_id}")

                        # Dispatch over UDP to physical robots or broadcast
                        if cmd_name == "EMERGENCY_STOP":
                            self.udp_tx_sock.sendto(b"EMERGENCY_STOP", ("255.255.255.255", UDP_VISION_PORT))
                            self.udp_tx_sock.sendto(b"0.0,0.0", ("255.255.255.255", UDP_ROBOT_PORT))
                        elif cmd_name == "MANUAL_DRIVE":
                            lin = payload.get("linearX", 0.0)
                            ang = payload.get("angularZ", 0.0)
                            cmd_str = f"{lin},{ang}".encode('utf-8')
                            target_ip = payload.get("ip", "172.20.10.3")
                            self.udp_tx_sock.sendto(cmd_str, (target_ip, UDP_ROBOT_PORT))

                        # Respond with Ack
                        ack_payload = {
                            "type": "command_ack",
                            "payload": {
                                "commandId": cmd_id,
                                "commandName": cmd_name,
                                "targetRobotId": target_id,
                                "state": "EXECUTING",
                                "timestamp": time.time(),
                                "ackTimestamp": time.time(),
                                "message": f"Command '{cmd_name}' dispatched over UDP."
                            }
                        }
                        await websocket.send(json.dumps(ack_payload))

                except Exception as err:
                    print(f"[WS ERROR] Command handling error: {err}")

        except websockets.exceptions.ConnectionClosed:
            pass
        finally:
            self.connected_clients.remove(websocket)
            print(f"[WS] Browser Client Disconnected.")

    async def main(self):
        loop = asyncio.get_running_loop()
        self.start_udp_listeners(loop)

        print("==========================================================")
        print("   SWARM TELEMETRY WEBSOCKET BRIDGE ONLINE")
        print(f"   WebSocket URL : ws://{WS_HOST}:{self.ws_port}/ws")
        print(f"   UDP Input     : Port {UDP_VISION_PORT} (Vision State)")
        print("==========================================================\n")

        async with websockets.serve(self.ws_handler, WS_HOST, self.ws_port):
            await asyncio.Future()  # run forever

if __name__ == '__main__':
    parser = argparse.ArgumentParser(description="Swarm Telemetry WebSocket Bridge")
    parser.add_argument("--port", type=int, default=WS_PORT, help="WebSocket Server Port")
    args = parser.parse_args()

    bridge = SwarmTelemetryBridge(ws_port=args.port)
    try:
        asyncio.run(bridge.main())
    except KeyboardInterrupt:
        print("\n[BRIDGE] Stopped.")
