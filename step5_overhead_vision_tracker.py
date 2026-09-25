#!/usr/bin/env python3
"""
======================================================================================
STEP 5: ARUCO DYNAMIC BOUNDARY & SWARM TRACKER (DICT_4X4_50)
======================================================================================
Purpose: Tracks mobile robots and workcell boundaries using DICT_4X4_50 ArUco markers.
         - Calibrates workspace scale and perspective using boundary markers 9, 10, 11, 12.
         - Broadcasts the swarm state JSON telemetry over UDP Port 5005.

JSON Packet Format:
  {
    "timestamp": 1726928100.123,
    "calibrated": true,
    "bots": {
      "id0": {"x": 25.4, "y": 30.1, "ang": 89.2, "out_of_bounds": false},
      "id1": {"x": 88.0, "y": 91.5, "ang": -12.4, "out_of_bounds": false}
    }
  }

Execution:
  python3 server/step5_overhead_vision_tracker.py --source 0
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

ARENA_WIDTH_CM  = 120.0
ARENA_HEIGHT_CM = 120.0
CANVAS_SIZE     = 800

SCALE_X = CANVAS_SIZE / ARENA_WIDTH_CM
SCALE_Y = CANVAS_SIZE / ARENA_HEIGHT_CM

BROADCAST_IP = "255.255.255.255"
UDP_PORT     = 5005
BOUNDARY_MARGIN_CM = 8.0

# ArUco Registry (DICT_4X4_50)
ID_TL = 9; ID_TR = 10; ID_BR = 11; ID_BL = 12

def main():
    parser = argparse.ArgumentParser(description="ArUco DICT_4X4_50 Overhead Vision Tracker")
    parser.add_argument("--source", type=str, default="0", help="Camera index (0) or stream URL")
    parser.add_argument("--port", type=int, default=UDP_PORT, help="UDP Broadcast Port")
    args = parser.parse_args()

    camera_source = int(args.source) if args.source.isdigit() else args.source
    cap = cv2.VideoCapture(camera_source)

    if not cap.isOpened():
        print(f"[ERROR] Could not open camera source: {camera_source}")
        sys.exit(1)

    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
    aruco_params = aruco.DetectorParameters()
    detector = aruco.ArucoDetector(aruco_dict, aruco_params)

    homography_matrix = None
    is_calibrated = False

    print(f"[VISION] Tracker Active (DICT_4X4_50). Broadcasting to {BROADCAST_IP}:{args.port}")

    while True:
        ret, frame = cap.read()
        if not ret:
            time.sleep(0.02)
            continue

        frame = cv2.resize(frame, (CANVAS_SIZE, CANVAS_SIZE))
        corners, ids, _ = detector.detectMarkers(frame)

        detected = {}
        if ids is not None:
            for i in range(len(ids)):
                m_id = int(ids[i][0])
                c = corners[i][0]
                cx = float(np.mean(c[:, 0]))
                cy = float(np.mean(c[:, 1]))
                dx = c[0][0] - c[3][0]
                dy = c[0][1] - c[3][1]
                angle_deg = round(math.degrees(math.atan2(dy, dx)), 1)
                detected[m_id] = {'centroid': (cx, cy), 'ang': angle_deg}

        # Check Homography Calibration
        if all(b_id in detected for b_id in [ID_TL, ID_TR, ID_BR, ID_BL]):
            src_pts = np.float32([
                detected[ID_TL]['centroid'],
                detected[ID_TR]['centroid'],
                detected[ID_BR]['centroid'],
                detected[ID_BL]['centroid']
            ])
            dst_pts = np.float32([
                [0.0, 0.0],
                [ARENA_WIDTH_CM, 0.0],
                [ARENA_WIDTH_CM, ARENA_HEIGHT_CM],
                [0.0, ARENA_HEIGHT_CM]
            ])
            homography_matrix = cv2.getPerspectiveTransform(src_pts, dst_pts)
            is_calibrated = True

        swarm_state = {
            "timestamp": round(time.time(), 3),
            "calibrated": is_calibrated,
            "bots": {}
        }

        # Process Robots 0 & 1
        for bot_id in [0, 1]:
            if bot_id in detected:
                px, py = detected[bot_id]['centroid']
                ang = detected[bot_id]['ang']

                if homography_matrix is not None:
                    pt = np.array([[[px, py]]], dtype=np.float32)
                    tf = cv2.perspectiveTransform(pt, homography_matrix)
                    x_cm = round(float(tf[0][0][0]), 1)
                    y_cm = round(float(tf[0][0][1]), 1)
                else:
                    x_cm = round(px / SCALE_X, 1)
                    y_cm = round(py / SCALE_Y, 1)

                oob = (
                    x_cm < BOUNDARY_MARGIN_CM or x_cm > (ARENA_WIDTH_CM - BOUNDARY_MARGIN_CM) or
                    y_cm < BOUNDARY_MARGIN_CM or y_cm > (ARENA_HEIGHT_CM - BOUNDARY_MARGIN_CM)
                )

                swarm_state["bots"][f"id{bot_id}"] = {
                    "x": x_cm,
                    "y": y_cm,
                    "ang": ang,
                    "out_of_bounds": oob
                }

                color = (0, 0, 255) if oob else ((0, 255, 255) if bot_id == 0 else (255, 0, 255))
                cv2.circle(frame, (int(px), int(py)), 22, color, 2)
                cv2.putText(frame, f"ID {bot_id}: ({x_cm},{y_cm})", (int(px) - 40, int(py) - 28),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.45, color, 2)

        # Broadcast state
        if swarm_state["bots"]:
            sock.sendto(json.dumps(swarm_state).encode('utf-8'), (BROADCAST_IP, args.port))

        cv2.imshow("Overhead ArUco Tracker (DICT_4X4_50)", frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
