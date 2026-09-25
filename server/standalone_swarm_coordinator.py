#!/usr/bin/env python3
"""
======================================================================================
STANDALONE CLOSED-LOOP AUTONOMOUS COORDINATOR WITH WORKCELL BOUNDARY SAFETY
======================================================================================
Purpose: Consumes ArUco coordinates from warehouse_central_server.py (DICT_4X4_50)
         and sends velocity commands directly to the ESP32 via UDP Port 8888.
         Enforces dynamic inter-robot collision avoidance AND hard workcell boundary safety!

Features:
  - Real-time waypoint tracking to (Target X, Target Y).
  - Inter-Robot Collision Avoidance (Bot 1 yields to Bot 0 if < 28 cm).
  - Hard Workcell Boundary Safety Override: If out_of_bounds flag is set by the vision
    engine, automatically steers robot back towards workcell center (60, 60) cm.

Usage:
  python server/standalone_swarm_coordinator.py --robot_ip <ESP32_IP> --bot_id 0 --tx 30.0 --ty 50.0
======================================================================================
"""

import sys
import argparse
import socket
import json
import math
import time

SAFE_DISTANCE_CM  = 28.0
ARRIVAL_RADIUS_CM = 8.0
CENTER_X_CM       = 60.0
CENTER_Y_CM       = 60.0

# 3-Rack Structured Approach & Pickup Poses
RACK_POSES = {
    1: {"name": "RACK_1", "approach": (35.0, 45.0), "pickup": (35.0, 32.0), "exit": (35.0, 55.0)},
    2: {"name": "RACK_2", "approach": (85.0, 45.0), "pickup": (85.0, 32.0), "exit": (85.0, 55.0)},
    3: {"name": "RACK_3", "approach": (60.0, 55.0), "pickup": (60.0, 68.0), "exit": (60.0, 45.0)},
}
DELIVERY_POSE = {"name": "DELIVERY_ZONE", "approach": (60.0, 88.0), "drop": (60.0, 98.0), "exit": (60.0, 85.0)}

def main():
    parser = argparse.ArgumentParser(description="Standalone Swarm Autonomous Boundary Coordinator")
    parser.add_argument("--robot_ip", type=str, default="172.20.10.3", help="Target ESP32 IP address")
    parser.add_argument("--robot_port", type=int, default=8888, help="ESP32 UDP velocity port")
    parser.add_argument("--vision_port", type=int, default=5005, help="Vision UDP listener port")
    parser.add_argument("--bot_id", type=int, default=0, help="This robot ID (0 or 1)")
    parser.add_argument("--rack", type=int, default=0, help="Target Rack (1, 2, or 3) for approach corridor")
    parser.add_argument("--tx", type=float, default=35.0, help="Goal X in cm")
    parser.add_argument("--ty", type=float, default=45.0, help="Goal Y in cm")
    args = parser.parse_args()

    my_id = args.bot_id
    peer_id = 1 if my_id == 0 else 0

    if args.rack in RACK_POSES:
        target_x, target_y = RACK_POSES[args.rack]["approach"]
        rack_label = f"[{RACK_POSES[args.rack]['name']} Approach]"
    else:
        target_x, target_y = args.tx, args.ty
        rack_label = "[Custom Waypoint]"

    rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx_sock.bind(("", args.vision_port))
    rx_sock.setblocking(False)

    tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("==========================================================")
    print(f"   STANDALONE BOUNDARY-AWARE COORDINATOR: BOT {my_id}")
    print(f"   ArUco Dictionary   : DICT_4X4_50")
    print(f"   Destination Target : ({target_x}, {target_y}) cm {rack_label}")
    print(f"   Robot Velocity UDP : {args.robot_ip}:{args.robot_port}")
    print(f"   Vision Listener    : Port {args.vision_port}")
    print("==========================================================\n")

    def send_vel(lin: float, ang: float):
        payload = f"{round(lin, 2)},{round(ang, 2)}".encode('utf-8')
        tx_sock.sendto(payload, (args.robot_ip, args.robot_port))

    try:
        while True:
            latest_packet = None
            while True:
                try:
                    data, _ = rx_sock.recvfrom(2048)
                    latest_packet = data
                except BlockingIOError:
                    break

            if latest_packet is not None:
                try:
                    state = json.loads(latest_packet.decode('utf-8'))
                    bots = state.get("bots", {})
                except Exception:
                    bots = {}

                my_key = f"id{my_id}"
                peer_key = f"id{peer_id}"

                if my_key in bots:
                    my_data = bots[my_key]
                    x, y, theta = my_data["x"], my_data["y"], my_data["ang"]
                    out_of_bounds = my_data.get("out_of_bounds", False)

                    # --- 1. HARD WORKCELL BOUNDARY OVERRIDE ---
                    if out_of_bounds:
                        print(f"\r[SAFETY] Boundary Breach! Steering away from edge towards center...  ", end="")
                        # Vector towards workcell center
                        dx_c = CENTER_X_CM - x
                        dy_c = CENTER_Y_CM - y
                        target_heading_c = math.degrees(math.atan2(dy_c, dx_c))
                        heading_err_c = target_heading_c - theta
                        while heading_err_c > 180.0:  heading_err_c -= 360.0
                        while heading_err_c < -180.0: heading_err_c += 360.0

                        if abs(heading_err_c) > 30.0:
                            send_vel(0.0, 0.7 if heading_err_c > 0 else -0.7)
                        else:
                            send_vel(0.35, 0.0) # Slow recovery drive
                        time.sleep(0.05)
                        continue

                    # --- 2. DECENTRALIZED COLLISION AVOIDANCE CHECK ---
                    collision_risk = False
                    if peer_key in bots:
                        peer_data = bots[peer_key]
                        d_peer = math.sqrt((x - peer_data["x"])**2 + (y - peer_data["y"])**2)
                        if d_peer < SAFE_DISTANCE_CM:
                            if my_id > peer_id:
                                collision_risk = True
                                print(f"\r[SAFETY] Peer Bot {peer_id} within {d_peer:.1f} cm! Yielding.        ", end="")
                                send_vel(0.0, 0.0)

                    if not collision_risk:
                        # --- 3. TARGET NAVIGATION VECTOR ---
                        dx = target_x - x
                        dy = target_y - y
                        dist_to_goal = math.sqrt(dx**2 + dy**2)

                        if dist_to_goal <= ARRIVAL_RADIUS_CM:
                            print(f"\r[SUCCESS] Arrived at destination ({target_x}, {target_y})! Stopping.     ", end="")
                            send_vel(0.0, 0.0)
                        else:
                            target_heading = math.degrees(math.atan2(dy, dx))
                            heading_error = target_heading - theta

                            while heading_error > 180.0:  heading_error -= 360.0
                            while heading_error < -180.0: heading_error += 360.0

                            if abs(heading_error) > 25.0:
                                turn_speed = 0.75 if heading_error > 0 else -0.75
                                send_vel(0.0, turn_speed)
                                print(f"\r[NAV] Pivoting to Target | Angle Error: {heading_error:+.1f} deg        ", end="")
                            else:
                                linear_speed = 0.45
                                angular_trim = heading_error * 0.015
                                send_vel(linear_speed, angular_trim)
                                print(f"\r[NAV] Tracking -> Goal: {dist_to_goal:.1f} cm | Err: {heading_error:+.1f} deg", end="")

            time.sleep(0.05)

    except KeyboardInterrupt:
        send_vel(0.0, 0.0)
        print("\n[COORDINATOR] Stopped.")

if __name__ == '__main__':
    main()
