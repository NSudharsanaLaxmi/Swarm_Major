#!/usr/bin/env python3
"""
======================================================================================
STANDALONE CLOSED-LOOP AUTONOMOUS COORDINATOR (NO ROS 2 REQUIRED)
======================================================================================
Purpose: Consumes ArUco coordinates from step5_overhead_vision_tracker.py via UDP
         and sends velocity commands directly to the ESP32 via UDP Port 8888.
         Works natively on Windows, macOS, and Linux out of the box!

Features:
  - Real-time waypoint tracking to (Target X, Target Y).
  - Decentralized priority collision avoidance (Bot 1 yields to Bot 0 if < 28 cm).
  - Proportional heading steering & center-axis in-place pivoting.

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

def main():
    parser = argparse.ArgumentParser(description="Standalone Swarm Autonomous Coordinator")
    parser.add_argument("--robot_ip", type=str, default="172.20.10.3", help="Target ESP32 IP address")
    parser.add_argument("--robot_port", type=int, default=8888, help="ESP32 UDP velocity port")
    parser.add_argument("--vision_port", type=int, default=5005, help="Vision UDP listener port")
    parser.add_argument("--bot_id", type=int, default=0, help="This robot ID (0 or 1)")
    parser.add_argument("--tx", type=float, default=30.0, help="Goal X in cm")
    parser.add_argument("--ty", type=float, default=50.0, help="Goal Y in cm")
    args = parser.parse_args()

    my_id = args.bot_id
    peer_id = 1 if my_id == 0 else 0

    # Socket to receive vision broadcasts
    rx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    rx_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    rx_sock.bind(("", args.vision_port))
    rx_sock.setblocking(False)

    # Socket to send velocity to ESP32
    tx_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    print("==========================================================")
    print(f"   STANDALONE AUTONOMOUS COORDINATOR: BOT {my_id}")
    print(f"   Destination Target : ({args.tx}, {args.ty}) cm")
    print(f"   Robot Velocity UDP : {args.robot_ip}:{args.robot_port}")
    print(f"   Vision Listener    : Port {args.vision_port}")
    print("==========================================================\n")

    def send_vel(lin: float, ang: float):
        payload = f"{round(lin, 2)},{round(ang, 2)}".encode('utf-8')
        tx_sock.sendto(payload, (args.robot_ip, args.robot_port))

    try:
        while True:
            latest_packet = None
            # Drain socket to get newest visual frame
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

                    # 1. Swarm Collision Avoidance Check
                    collision_risk = False
                    if peer_key in bots:
                        peer_data = bots[peer_key]
                        d_peer = math.sqrt((x - peer_data["x"])**2 + (y - peer_data["y"])**2)
                        if d_peer < SAFE_DISTANCE_CM:
                            # Higher ID yields to lower ID
                            if my_id > peer_id:
                                collision_risk = True
                                print(f"\r[SAFETY] Peer Bot {peer_id} within {d_peer:.1f} cm! Yielding right-of-way.  ", end="")
                                send_vel(0.0, 0.0)

                    if not collision_risk:
                        # 2. Navigation Math to Goal
                        dx = args.tx - x
                        dy = args.ty - y
                        dist_to_goal = math.sqrt(dx**2 + dy**2)

                        if dist_to_goal <= ARRIVAL_RADIUS_CM:
                            print(f"\r[SUCCESS] Arrived at destination ({args.tx}, {args.ty})! Stopping.     ", end="")
                            send_vel(0.0, 0.0)
                        else:
                            target_heading = math.degrees(math.atan2(dy, dx))
                            heading_error = target_heading - theta

                            while heading_error > 180.0:  heading_error -= 360.0
                            while heading_error < -180.0: heading_error += 360.0

                            if abs(heading_error) > 25.0:
                                # Pivot in-place
                                turn_speed = 0.75 if heading_error > 0 else -0.75
                                send_vel(0.0, turn_speed)
                                print(f"\r[NAV] Pivoting to Target | Angle Error: {heading_error:+.1f} deg        ", end="")
                            else:
                                # Forward track
                                linear_speed = 0.45
                                angular_trim = heading_error * 0.015
                                send_vel(linear_speed, angular_trim)
                                print(f"\r[NAV] Tracking -> Goal: {dist_to_goal:.1f} cm | Err: {heading_error:+.1f} deg", end="")

            time.sleep(0.05) # 20 Hz execution loop

    except KeyboardInterrupt:
        send_vel(0.0, 0.0)
        print("\n[COORDINATOR] Stopped.")

if __name__ == '__main__':
    main()
