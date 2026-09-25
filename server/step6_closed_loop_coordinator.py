#!/usr/bin/env python3
"""
======================================================================================
STEP 6: ROS 2 CLOSED-LOOP SWARM COORDINATOR WITH BOUNDARY OVERRIDE (DICT_4X4_50)
======================================================================================
Purpose: Consumes DICT_4X4_50 ArUco tracking packets from Step 5, enforces inter-robot
         collision avoidance and workcell boundary safety, and publishes /cmd_vel.

Execution:
  python3 step6_closed_loop_coordinator.py --bot_id 0 --tx 30.0 --ty 45.0
======================================================================================
"""

import sys
import argparse
import socket
import json
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

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

class ClosedLoopSwarmCoordinator(Node):
    def __init__(self, my_id: int, target_x: float, target_y: float, listen_port: int, rack_id: int = 0):
        super().__init__(f'swarm_coordinator_bot_{my_id}')
        self.my_id = my_id
        self.peer_id = 1 if my_id == 0 else 0
        
        if rack_id in RACK_POSES:
            # Default to approach pose for designated rack
            self.target_x, self.target_y = RACK_POSES[rack_id]["approach"]
            self.get_logger().info(f"[SWARM] Initialized rack mission: {RACK_POSES[rack_id]['name']} Approach Pose ({self.target_x}, {self.target_y})")
        else:
            self.target_x = target_x
            self.target_y = target_y

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", listen_port))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"[SWARM] Bot {self.my_id} Coordinator (DICT_4X4_50) Online. Target: ({self.target_x}, {self.target_y})")

    def control_loop(self):
        latest_packet = None
        while True:
            try:
                data, _ = self.sock.recvfrom(2048)
                latest_packet = data
            except BlockingIOError:
                break

        if latest_packet is None:
            return

        try:
            state = json.loads(latest_packet.decode('utf-8'))
            bots = state.get("bots", {})
        except Exception as e:
            self.get_logger().error(f"[SWARM] Failed to parse JSON: {e}")
            return

        my_key = f"id{self.my_id}"
        peer_key = f"id{self.peer_id}"

        if my_key not in bots:
            return

        my_data = bots[my_key]
        x, y, theta = my_data["x"], my_data["y"], my_data["ang"]
        out_of_bounds = my_data.get("out_of_bounds", False)

        # 1. Hard Workcell Boundary Override
        if out_of_bounds:
            self.get_logger().warn("[SAFETY] Workcell Boundary Breach! Adjusting course towards center.")
            dx_c = CENTER_X_CM - x
            dy_c = CENTER_Y_CM - y
            target_heading_c = math.degrees(math.atan2(dy_c, dx_c))
            heading_err_c = target_heading_c - theta
            while heading_err_c > 180: heading_err_c -= 360
            while heading_err_c < -180: heading_err_c += 360

            if abs(heading_err_c) > 30:
                self.publish_vel(0.0, 0.7 if heading_err_c > 0 else -0.7)
            else:
                self.publish_vel(0.35, 0.0)
            return

        # 2. Inter-Robot Collision Avoidance
        if peer_key in bots:
            peer_data = bots[peer_key]
            dist_to_peer = math.sqrt((x - peer_data["x"])**2 + (y - peer_data["y"])**2)
            if dist_to_peer < SAFE_DISTANCE_CM:
                if self.my_id > self.peer_id:
                    self.get_logger().warn(f"[SAFETY] Peer Bot {self.peer_id} within {dist_to_peer:.1f} cm! Yielding.")
                    self.publish_vel(0.0, 0.0)
                    return

        # 3. Target Navigation Vector
        dx = self.target_x - x
        dy = self.target_y - y
        dist_to_goal = math.sqrt(dx**2 + dy**2)

        if dist_to_goal <= ARRIVAL_RADIUS_CM:
            self.get_logger().info("[TASK] Target Arrived!")
            self.publish_vel(0.0, 0.0)
            return

        target_heading = math.degrees(math.atan2(dy, dx))
        heading_error = target_heading - theta

        while heading_error > 180: heading_error -= 360
        while heading_error < -180: heading_error += 360

        if abs(heading_error) > 25.0:
            turn_speed = 0.7 if heading_error > 0 else -0.7
            self.publish_vel(0.0, turn_speed)
        else:
            self.publish_vel(0.4, heading_error * 0.02)

    def publish_vel(self, lin, ang):
        t = Twist()
        t.linear.x = float(lin)
        t.angular.z = float(ang)
        self.cmd_pub.publish(t)

def main(args=None):
    parser = argparse.ArgumentParser(description="Swarm Autonomous Closed-Loop Coordinator")
    parser.add_argument("--bot_id", type=int, default=0, help="This robot ID (0 or 1)")
    parser.add_argument("--rack", type=int, default=0, help="Target Rack (1, 2, or 3) for approach corridor")
    parser.add_argument("--tx", type=float, default=35.0, help="Target X coordinate (cm)")
    parser.add_argument("--ty", type=float, default=45.0, help="Target Y coordinate (cm)")
    parser.add_argument("--port", type=int, default=5005, help="Vision UDP listener port")
    cli_args, remaining_args = parser.parse_known_args(args=sys.argv[1:])

    rclpy.init(args=remaining_args)
    node = ClosedLoopSwarmCoordinator(
        my_id=cli_args.bot_id,
        target_x=cli_args.tx,
        target_y=cli_args.ty,
        listen_port=cli_args.port,
        rack_id=cli_args.rack
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
