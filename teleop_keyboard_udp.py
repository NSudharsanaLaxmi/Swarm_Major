#!/usr/bin/env python3
"""
======================================================================================
STANDALONE KEYBOARD TELEOP CONTROLLER (NO ROS 2 REQUIRED)
======================================================================================
Purpose: Control your ESP32 robot directly from your PC keyboard over Wi-Fi UDP.
         Works natively on Windows, macOS, and Linux without Docker or ROS 2.

Controls:
  [W] : Forward
  [S] : Reverse
  [A] : Pivot Left
  [D] : Pivot Right
  [Space] / [X] : Emergency Stop
  [+] / [-]     : Increase / Decrease speed
  [Q]           : Quit

Usage:
  python server/teleop_keyboard_udp.py --ip <ESP32_IP> --port 8888
======================================================================================
"""

import sys
import socket
import time
import argparse

# Cross-platform single key press detection
if sys.platform == "win32":
    import msvcrt
    def get_key():
        if msvcrt.kbhit():
            ch = msvcrt.getch()
            # Handle special keys/arrows
            if ch in (b'\x00', b'\xe0'):
                ch = msvcrt.getch()
                if ch == b'H': return 'w' # Up arrow
                if ch == b'P': return 's' # Down arrow
                if ch == b'K': return 'a' # Left arrow
                if ch == b'M': return 'd' # Right arrow
            return ch.decode('latin1', errors='ignore').lower()
        return None
else:
    import select
    import tty
    import termios
    def get_key():
        dr, _, _ = select.select([sys.stdin], [], [], 0.05)
        if dr:
            return sys.stdin.read(1).lower()
        return None

def main():
    parser = argparse.ArgumentParser(description="Standalone UDP Keyboard Teleop")
    parser.add_argument("--ip", type=str, default="172.20.10.3", help="Target ESP32 IP address")
    parser.add_argument("--port", type=int, default=8888, help="Target ESP32 UDP port")
    args = parser.parse_args()

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    speed_step = 0.1
    current_linear = 0.0
    current_angular = 0.0
    base_linear_max = 0.55
    base_angular_max = 0.85

    print("==========================================================")
    print("   STANDALONE ROBOT KEYBOARD TELEOPERATION")
    print(f"   Target: UDP {args.ip}:{args.port}")
    print("==========================================================")
    print(" [W / Up Arrow]    : Drive Forward")
    print(" [S / Down Arrow]  : Drive Reverse")
    print(" [A / Left Arrow]  : Pivot Left")
    print(" [D / Right Arrow] : Pivot Right")
    print(" [Space / X]       : Emergency Stop")
    print(" [Q]               : Quit")
    print("==========================================================\n")

    # For Linux terminal raw mode
    old_settings = None
    if sys.platform != "win32":
        old_settings = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())

    try:
        last_transmit = time.time()
        while True:
            key = get_key()

            if key:
                if key == 'w':
                    current_linear = base_linear_max
                    current_angular = 0.0
                    print(f"\r[TELEOP] FORWARD  (Lin: {current_linear:.2f}, Ang: {current_angular:.2f})", end="")
                elif key == 's':
                    current_linear = -base_linear_max
                    current_angular = 0.0
                    print(f"\r[TELEOP] REVERSE  (Lin: {current_linear:.2f}, Ang: {current_angular:.2f})", end="")
                elif key == 'a':
                    current_linear = 0.0
                    current_angular = base_angular_max
                    print(f"\r[TELEOP] PIVOT CCW (Lin: {current_linear:.2f}, Ang: {current_angular:.2f})", end="")
                elif key == 'd':
                    current_linear = 0.0
                    current_angular = -base_angular_max
                    print(f"\r[TELEOP] PIVOT CW  (Lin: {current_linear:.2f}, Ang: {current_angular:.2f})", end="")
                elif key in (' ', 'x'):
                    current_linear = 0.0
                    current_angular = 0.0
                    print(f"\r[TELEOP] STOPPED   (Lin: 0.00, Ang: 0.00)                 ", end="")
                elif key == 'q':
                    # Send final stop
                    sock.sendto(b"0.0,0.0", (args.ip, args.port))
                    print("\n[TELEOP] Exiting teleop.")
                    break

            # Send heartbeat packet every 100ms
            if time.time() - last_transmit >= 0.1:
                payload = f"{round(current_linear, 2)},{round(current_angular, 2)}".encode('utf-8')
                sock.sendto(payload, (args.ip, args.port))
                last_transmit = time.time()

            time.sleep(0.02)

    except KeyboardInterrupt:
        sock.sendto(b"0.0,0.0", (args.ip, args.port))
        print("\n[TELEOP] Interrupted.")
    finally:
        if sys.platform != "win32" and old_settings:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

if __name__ == '__main__':
    main()
