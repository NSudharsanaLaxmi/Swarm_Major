#!/usr/bin/env python3
"""
======================================================================================
WAREHOUSE CENTRAL SERVER & ARUCO DYNAMIC BOUNDARY CALIBRATOR
======================================================================================
Project: AI-Driven Cooperative Autonomous Mobile Manipulator Swarm for Smart Warehouse

Core Architecture:
  1. ArUco Dictionary: DICT_4X4_50
  2. Boundary Calibration: Markers ID 9 (TL), 10 (TR), 11 (BR), 12 (BL) define the
     physical workcell perimeter and compute the dynamic Homography Matrix (H).
  3. Metric Transformation: Converts pixel locations into metric centimeters (120x120 cm).
  4. Dynamic Landmark Detection:
     - IDs 0, 1: Mobile Manipulator Robots (Robot 1, Robot 2)
     - IDs 2, 3, 4, 5: Fixed Storage Racks (Rack 1, 2, 3, 4)
     - IDs 6, 7: Robot Starting Positions (Start 1, Start 2)
     - ID 8: Product Delivery Zone
     - IDs 9, 10, 11, 12: Workcell Perimeter Boundary Calibration Corners
  5. Hard Boundary Safety: Enforces an 8 cm inner safety buffer; sets out_of_bounds flag.
  6. UDP Telemetry Broadcast: Transmits JSON state over UDP Port 5005.

Execution:
  python3 server/warehouse_central_server.py --source 0
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

# Physical Arena Metric Dimensions (in cm)
ARENA_WIDTH_CM  = 120.0
ARENA_HEIGHT_CM = 120.0
CANVAS_SIZE     = 800

SCALE_X = CANVAS_SIZE / ARENA_WIDTH_CM
SCALE_Y = CANVAS_SIZE / ARENA_HEIGHT_CM

BROADCAST_IP = "255.255.255.255"
SWARM_PORT   = 5005
BOUNDARY_BUFFER_CM = 8.0  # 8 cm inner safety buffer

# ARUCO MARKER REGISTRY (DICT_4X4_50)
ID_ROBOT_1       = 0
ID_ROBOT_2       = 1
ID_RACK_1        = 2
ID_RACK_2        = 3
ID_RACK_3        = 4
ID_RACK_4        = 5
ID_ROBOT_1_START = 6
ID_ROBOT_2_START = 7
ID_DELIVERY_ZONE = 8
ID_BOUNDARY_TL   = 9
ID_BOUNDARY_TR   = 10
ID_BOUNDARY_BR   = 11
ID_BOUNDARY_BL   = 12

MARKER_NAMES = {
    0: "ROBOT_1", 1: "ROBOT_2",
    2: "RACK_1", 3: "RACK_2", 4: "RACK_3", 5: "RACK_4",
    6: "ROBOT_1_START", 7: "ROBOT_2_START", 8: "DELIVERY_ZONE",
    9: "BOUNDARY_TL", 10: "BOUNDARY_TR", 11: "BOUNDARY_BR", 12: "BOUNDARY_BL"
}

class WorkcellVisionEngine:
    def __init__(self, camera_source, udp_port):
        self.camera_source = int(camera_source) if str(camera_source).isdigit() else camera_source
        self.udp_port = udp_port

        self.cap = cv2.VideoCapture(self.camera_source)
        if not self.cap.isOpened():
            print(f"[ERROR] Could not open camera source: {self.camera_source}")
            sys.exit(1)

        # Setup UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_BROADCAST, 1)

        # ArUco Detector for DICT_4X4_50
        self.aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_50)
        self.aruco_params = aruco.DetectorParameters()
        self.detector = aruco.ArucoDetector(self.aruco_dict, self.aruco_params)

        # Homography Matrix State
        self.homography_matrix = None
        self.inv_homography_matrix = None
        self.is_calibrated = False
        self.last_calibration_time = 0

        # Default fallback landmarks if uncalibrated (in cm)
        self.landmarks = {
            "rack_1": {"x": 25.0, "y": 30.0, "id": 2},
            "rack_2": {"x": 25.0, "y": 90.0, "id": 3},
            "rack_3": {"x": 55.0, "y": 30.0, "id": 4},
            "rack_4": {"x": 55.0, "y": 90.0, "id": 5},
            "robot_1_start": {"x": 15.0, "y": 60.0, "id": 6},
            "robot_2_start": {"x": 105.0, "y": 60.0, "id": 7},
            "delivery_zone": {"x": 95.0, "y": 60.0, "id": 8}
        }

    def compute_boundary_homography(self, detected_markers):
        """
        Extracts centroids of boundary markers ID 9 (TL), 10 (TR), 11 (BR), 12 (BL)
        and computes the perspective transformation matrix H mapping camera pixels to cm.
        """
        required = [ID_BOUNDARY_TL, ID_BOUNDARY_TR, ID_BOUNDARY_BR, ID_BOUNDARY_BL]
        if not all(rid in detected_markers for rid in required):
            return False

        src_pts = np.float32([
            detected_markers[ID_BOUNDARY_TL]['pixel_centroid'],
            detected_markers[ID_BOUNDARY_TR]['pixel_centroid'],
            detected_markers[ID_BOUNDARY_BR]['pixel_centroid'],
            detected_markers[ID_BOUNDARY_BL]['pixel_centroid']
        ])

        dst_pts = np.float32([
            [0.0, 0.0],
            [ARENA_WIDTH_CM, 0.0],
            [ARENA_WIDTH_CM, ARENA_HEIGHT_CM],
            [0.0, ARENA_HEIGHT_CM]
        ])

        H = cv2.getPerspectiveTransform(src_pts, dst_pts)
        if H is not None:
            self.homography_matrix = H
            self.inv_homography_matrix = np.linalg.inv(H)
            self.is_calibrated = True
            self.last_calibration_time = time.time()
            return True
        return False

    def transform_pixel_to_cm(self, px, py):
        """ Transforms a single pixel coordinate (px, py) to real-world cm using H. """
        if self.homography_matrix is None:
            # Fallback simple linear scaling if H not yet computed
            return px / SCALE_X, py / SCALE_Y

        pt = np.array([[[px, py]]], dtype=np.float32)
        transformed = cv2.perspectiveTransform(pt, self.homography_matrix)
        return float(transformed[0][0][0]), float(transformed[0][0][1])

    def run(self):
        print("==========================================================")
        print("   ARUCO WORKCELL BOUNDARY & SWARM PERCEPTRON ONLINE")
        print("   Dictionary: DICT_4X4_50")
        print(f"   Broadcasting Telemetry -> {BROADCAST_IP}:{self.udp_port}")
        print("   Boundary Calibration Points: IDs 9(TL), 10(TR), 11(BR), 12(BL)")
        print("   Keys: [S] Emergency Halt | [C] Force Recalibrate | [Q] Quit")
        print("==========================================================\n")

        while True:
            ret, raw_frame = self.cap.read()
            if not ret:
                time.sleep(0.02)
                continue

            h_f, w_f = raw_frame.shape[:2]
            display_frame = cv2.resize(raw_frame, (CANVAS_SIZE, CANVAS_SIZE))

            corners, ids, _ = self.detector.detectMarkers(raw_frame)
            detected_markers = {}

            if ids is not None:
                for i in range(len(ids)):
                    m_id = int(ids[i][0])
                    c = corners[i][0]

                    # Scale raw image corners to CANVAS_SIZE for visualization
                    c_vis = c.copy()
                    c_vis[:, 0] *= (CANVAS_SIZE / w_f)
                    c_vis[:, 1] *= (CANVAS_SIZE / h_f)

                    cx_raw = float(np.mean(c[:, 0]))
                    cy_raw = float(np.mean(c[:, 1]))

                    # Orientation calculation in raw pixels
                    dx_p = c[0][0] - c[3][0]
                    dy_p = c[0][1] - c[3][1]
                    angle_deg = round(math.degrees(math.atan2(dy_p, dx_p)), 1)

                    detected_markers[m_id] = {
                        'pixel_centroid': (cx_raw, cy_raw),
                        'vis_centroid': (float(np.mean(c_vis[:, 0])), float(np.mean(c_vis[:, 1]))),
                        'angle_deg': angle_deg,
                        'corners_vis': c_vis
                    }

            # 1. Update Dynamic Homography Matrix from Corner Boundary Markers
            calib_success = self.compute_boundary_homography(detected_markers)

            # 2. Build Warp Metric View Canvas (800x800 px)
            metric_canvas = np.zeros((CANVAS_SIZE, CANVAS_SIZE, 3), dtype=np.uint8)

            # Draw Metric Workspace Grid
            for grid_cm in range(20, int(ARENA_WIDTH_CM), 20):
                gx = int(grid_cm * SCALE_X)
                gy = int(grid_cm * SCALE_Y)
                cv2.line(metric_canvas, (gx, 0), (gx, CANVAS_SIZE), (35, 35, 35), 1)
                cv2.line(metric_canvas, (0, gy), (CANVAS_SIZE, gy), (35, 35, 35), 1)

            # Draw Safety Boundary Buffer Polygon
            buf_px = int(BOUNDARY_BUFFER_CM * SCALE_X)
            cv2.rectangle(metric_canvas, (0, 0), (CANVAS_SIZE, CANVAS_SIZE), (0, 255, 255), 2)
            cv2.rectangle(metric_canvas, (buf_px, buf_px), (CANVAS_SIZE - buf_px, CANVAS_SIZE - buf_px), (0, 165, 255), 1)
            cv2.putText(metric_canvas, "WORKCELL BOUNDARY (DICT_4X4_50)", (15, 25),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 2)

            # Build Swarm State Telemetry
            swarm_state = {
                "timestamp": round(time.time(), 3),
                "dictionary": "DICT_4X4_50",
                "calibrated": self.is_calibrated,
                "arena_size_cm": [ARENA_WIDTH_CM, ARENA_HEIGHT_CM],
                "landmarks": {},
                "bots": {}
            }

            # 3. Process Detected Markers into Transformed Workspace Frame
            for m_id, m_info in detected_markers.items():
                px, py = m_info['pixel_centroid']
                ang = m_info['angle_deg']

                # Transform via Homography
                x_cm, y_cm = self.transform_pixel_to_cm(px, py)
                x_cm = round(max(0.0, min(ARENA_WIDTH_CM, x_cm)), 1)
                y_cm = round(max(0.0, min(ARENA_HEIGHT_CM, y_cm)), 1)

                m_name = MARKER_NAMES.get(m_id, f"UNKNOWN_{m_id}")

                # Draw on Camera Feed
                v_cx, v_cy = m_info['vis_centroid']
                cv2.circle(display_frame, (int(v_cx), int(v_cy)), 18, (0, 255, 0), 2)
                cv2.putText(display_frame, f"{m_name} ({m_id})", (int(v_cx) - 35, int(v_cy) - 22),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

                # Draw on Metric Workcell View
                m_px = int(x_cm * SCALE_X)
                m_py = int(y_cm * SCALE_Y)

                # Category: Boundary Corners (IDs 9-12)
                if 9 <= m_id <= 12:
                    cv2.circle(metric_canvas, (m_px, m_py), 10, (0, 255, 255), -1)
                    cv2.putText(metric_canvas, m_name, (m_px - 25, m_py - 12),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 255, 255), 1)

                # Category: Racks (IDs 2-5)
                elif 2 <= m_id <= 5:
                    cv2.rectangle(metric_canvas, (m_px - 25, m_py - 25), (m_px + 25, m_py + 25), (0, 140, 255), 2)
                    cv2.putText(metric_canvas, f"RACK {m_id-1}", (m_px - 22, m_py - 30),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 140, 255), 1)
                    swarm_state["landmarks"][f"rack_{m_id-1}"] = {"x": x_cm, "y": y_cm, "id": m_id}

                # Category: Delivery Zone (ID 8)
                elif m_id == ID_DELIVERY_ZONE:
                    cv2.rectangle(metric_canvas, (m_px - 30, m_py - 30), (m_px + 30, m_py + 30), (0, 255, 100), 2)
                    cv2.putText(metric_canvas, "DROP ZONE", (m_px - 28, m_py - 35),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 100), 1)
                    swarm_state["landmarks"]["delivery_zone"] = {"x": x_cm, "y": y_cm, "id": m_id}

                # Category: Start Positions (IDs 6, 7)
                elif m_id in (ID_ROBOT_1_START, ID_ROBOT_2_START):
                    cv2.circle(metric_canvas, (m_px, m_py), 15, (200, 200, 200), 1)
                    cv2.putText(metric_canvas, f"START {m_id-5}", (m_px - 25, m_py - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)
                    swarm_state["landmarks"][f"start_{m_id-5}"] = {"x": x_cm, "y": y_cm, "id": m_id}

                # Category: Robots (IDs 0, 1)
                elif m_id in (ID_ROBOT_1, ID_ROBOT_2):
                    # Check Safety Boundary Breach
                    out_of_bounds = (
                        x_cm < BOUNDARY_BUFFER_CM or x_cm > (ARENA_WIDTH_CM - BOUNDARY_BUFFER_CM) or
                        y_cm < BOUNDARY_BUFFER_CM or y_cm > (ARENA_HEIGHT_CM - BOUNDARY_BUFFER_CM)
                    )

                    bot_color = (0, 0, 255) if out_of_bounds else ((0, 255, 255) if m_id == 0 else (255, 0, 255))
                    cv2.circle(metric_canvas, (m_px, m_py), 22, bot_color, 2)

                    # Heading arrow
                    rad = math.radians(ang)
                    ax = int(m_px + 30 * math.cos(rad))
                    ay = int(m_py + 30 * math.sin(rad))
                    cv2.arrowedLine(metric_canvas, (m_px, m_py), (ax, ay), (0, 255, 0), 2)

                    status_str = "WARNING: OOB" if out_of_bounds else "NORMAL"
                    cv2.putText(metric_canvas, f"R0{m_id+1}: ({x_cm},{y_cm}) {status_str}",
                                (m_px - 45, m_py - 28), cv2.FONT_HERSHEY_SIMPLEX, 0.4, bot_color, 1)

                    swarm_state["bots"][f"id{m_id}"] = {
                        "x": x_cm,
                        "y": y_cm,
                        "ang": ang,
                        "out_of_bounds": out_of_bounds
                    }

            # Draw Boundary Perimeter Quad on Raw Frame if boundary markers present
            req_b = [ID_BOUNDARY_TL, ID_BOUNDARY_TR, ID_BOUNDARY_BR, ID_BOUNDARY_BL]
            if all(b in detected_markers for b in req_b):
                pts = np.array([
                    detected_markers[ID_BOUNDARY_TL]['vis_centroid'],
                    detected_markers[ID_BOUNDARY_TR]['vis_centroid'],
                    detected_markers[ID_BOUNDARY_BR]['vis_centroid'],
                    detected_markers[ID_BOUNDARY_BL]['vis_centroid']
                ], np.int32).reshape((-1, 1, 2))
                cv2.polylines(display_frame, [pts], True, (0, 255, 255), 2)

            # Calibration Status Display
            calib_str = "CALIBRATED (DYNAMIC H-MATRIX)" if self.is_calibrated else "CALIBRATING (WAITING FOR BOUNDARY MARKERS 9-12)..."
            calib_color = (0, 255, 0) if self.is_calibrated else (0, 0, 255)
            cv2.putText(metric_canvas, calib_str, (15, CANVAS_SIZE - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, calib_color, 1)

            # Broadcast UDP Telemetry
            if swarm_state["bots"]:
                sock_payload = json.dumps(swarm_state).encode('utf-8')
                self.sock.sendto(sock_payload, (BROADCAST_IP, self.udp_port))

            # Combine Side-by-Side View
            combined_view = np.hstack((display_frame, metric_canvas))
            cv2.imshow("Warehouse Swarm Perception & Boundary System (DICT_4X4_50)", combined_view)

            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('s'):
                print("\n[ALERT] Emergency Halt Broadcasted.")
                self.sock.sendto(b"EMERGENCY_STOP", (BROADCAST_IP, self.udp_port))

        self.cap.release()
        cv2.destroyAllWindows()

def main():
    parser = argparse.ArgumentParser(description="ArUco Workcell Boundary Perception Engine")
    parser.add_argument("--source", type=str, default="0", help="Camera index or RTSP stream URL")
    parser.add_argument("--port", type=int, default=SWARM_PORT, help="UDP Broadcast port")
    args = parser.parse_args()

    engine = WorkcellVisionEngine(camera_source=args.source, udp_port=args.port)
    engine.run()

if __name__ == '__main__':
    main()
