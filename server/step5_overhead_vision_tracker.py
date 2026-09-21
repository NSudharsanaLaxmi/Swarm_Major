#!/usr/bin/env python3
"""
======================================================================================
STEP 5: OVERHEAD ARUCO COMPUTER VISION TRACKER
======================================================================================
Purpose: Tracks multiple robots via overhead camera using ArUco fiducial markers.
         - Scales raw pixel positions to metric coordinates (120 cm x 120 cm arena).
         - Broadcasts the swarm state JSON dictionary over UDP Port 5005.

JSON Packet Format:
  {
    "timestamp": 1726928100.123,
    "bots": {
      "id0": {"x": 25.4, "y": 30.1, "ang": 89.2},
      "id1": {"x": 88.0, "y": 91.5, "ang": -12.4}
    }
  }

Execution:
  python3 step5_overhead_vision_tracker.py --source 0
  # Or with IP webcam:
  python3 step5_overhead_vision_tracker.py --source "http://172.20.10.2:8080/video"
======================================================================================
"""

import sys
import argparse
import socket
import json
import math
import time
import cv2
import cv2.aruco as aruco
import numpy as np

# Physical Arena Dimensions in Centimeters
ARENA_WIDTH_CM  = 120.0
ARENA_HEIGHT_CM = 120.0
CANVAS_SIZE     = 800

SCALE_X = CANVAS_SIZE / ARENA_WIDTH_CM
SCALE_Y = CANVAS_SIZE / ARENA_HEIGHT_CM

BROADCAST_IP = "255.255.255.255"
UDP_PORT     = 5005

def main():
    parser = argparse.ArgumentParser(description="Overhead ArUco Swarm Tracker")
    parser.add_argument("--source", type=str, default="0", help="Camera index (0) or HTTP video stream URL")
    parser.add_argument("--port", type=int, default=UDP_PORT, help="UDP Broadcast Port")
    args = parser.parse_args()

    # Parse camera input
    camera_source = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(camera_source)

    if not cap.isOpened():
        print(f"[ERROR] Could not open video source: {camera_source}")
        sys.exit(1)

    # Setup UDP Broadcast Socket
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    # ArUco Dictionary Setup (DICT_4X4_100)
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_100)
    aruco_params = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, aruco_params)

    print(f"[VISION] Tracker Active. Broadcasting Swarm coordinates on {BROADCAST_IP}:{args.port}")
    print("[VISION] Press 'q' to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.02)
            continue

        frame = cv2.resize(frame, (CANVAS_SIZE, CANVAS_SIZE))
        corners, ids, _ = detector.detectMarkers(frame)

        swarm_state = {
            "timestamp": round(time.time(), 3),
            "bots": {}
        }

        if ids is not None:
            for i in range(len(ids)):
                bot_id = int(ids[i][0])
                c = corners[i][0]

                # Centroid Calculation (pixels)
                cx = float(np.mean(c[:, 0]))
                cy = float(np.mean(c[:, 1]))

                # Real-world metric position in cm
                x_cm = round(cx / SCALE_X, 1)
                y_cm = round(cy / SCALE_Y, 1)

                # Heading Angle (theta) in degrees
                # Vector from bottom-left corner to top-left corner
                dx_p = c[0][0] - c[3][0]
                dy_p = c[0][1] - c[3][1]
                angle_rad = math.atan2(dy_p, dx_p)
                angle_deg = round(math.degrees(angle_rad), 1)

                swarm_state["bots"][f"id{bot_id}"] = {
                    "x": x_cm,
                    "y": y_cm,
                    "ang": angle_deg
                }

                # Visual overlay
                cv2.circle(frame, (int(cx), int(cy)), 22, (0, 255, 0), 2)
                arrow_x = int(cx + 35 * math.cos(angle_rad))
                arrow_y = int(cy + 35 * math.sin(angle_rad))
                cv2.arrowedLine(frame, (int(cx), int(cy)), (arrow_x, arrow_y), (0, 0, 255), 2)
                cv2.putText(frame, f"ID {bot_id}: ({x_cm}, {y_cm})", (int(cx) - 40, int(cy) - 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)

        # Broadcast state
        if swarm_state["bots"]:
            payload = json.dumps(swarm_state).encode('utf-8')
            sock.sendto(payload, (BROADCAST_IP, args.port))

        cv2.imshow("Overhead Swarm Tracking Matrix", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
