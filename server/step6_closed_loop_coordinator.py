#!/usr/bin/env python3
"""
======================================================================================
STEP 6: CLOSED-LOOP SWARM COORDINATOR & AUTONOMOUS COLLISION AVOIDANCE
======================================================================================
Purpose: Consumes global localization telemetry from Step 5, calculates goal heading
         and distance vectors, enforces dynamic decentralized collision avoidance
         between Robot 0 and Robot 1, and publishes velocity commands to '/cmd_vel'
         (which Step 4 forwards over UDP to the physical ESP32).

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

SAFE_DISTANCE_CM   = 28.0  # Collision avoidance clearance threshold
ARRIVAL_RADIUS_CM  = 8.0   # Goal threshold

class ClosedLoopSwarmCoordinator(Node):
    def __init__(self, my_id: int, target_x: float, target_y: float, listen_port: int):
        super().__init__(f'swarm_coordinator_bot_{my_id}')
        self.my_id = my_id
        self.peer_id = 1 if my_id == 0 else 0
        self.target_x = target_x
        self.target_y = target_y

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Setup non-blocking UDP socket to listen for vision tracker broadcast
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self.sock.bind(("", listen_port))
        self.sock.setblocking(False)

        self.timer = self.create_timer(0.05, self.control_loop) # 20 Hz control loop
        self.get_logger().info(f"[SWARM] Bot {self.my_id} Coordinator Online. Target: ({self.target_x}, {self.target_y})")

    def control_loop(self):
        # 1. Drain latest tracking packet
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
            return # My marker is currently occluded or not detected

        my_data = bots[my_key]
        x, y, theta = my_data["x"], my_data["y"], my_data["ang"]

        # 2. Decentralized Collision Avoidance Check
        if peer_key in bots:
            peer_data = bots[peer_key]
            d_peer = math.sqrt((x - peer_data["x"])**2 + (y - peer_data["y"])**2)

            if d_peer < SAFE_DISTANCE_CM:
                # Priority Protocol: Bot 0 has right of way; Bot 1 yields
                if self.my_id > self.peer_id:
                    self.get_logger().warn(f"[SAFETY] Peer Bot {self.peer_id} within {d_peer:.1f} cm! Yielding right-of-way.")
                    self.publish_twist(0.0, 0.0)
                    return

        # 3. Target Navigation Vector
        dx = self.target_x - x
        dy = self.target_y - y
        distance_to_goal = math.sqrt(dx**2 + dy**2)

        if distance_to_goal <= ARRIVAL_RADIUS_CM:
            self.get_logger().info(f"[TASK] Arrived at Target ({self.target_x}, {self.target_y})!")
            self.publish_twist(0.0, 0.0)
            return

        # 4. Heading error calculation
        target_heading = math.degrees(math.atan2(dy, dx))
        heading_error = target_heading - theta

        # Bounding heading error between -180 and 180 degrees
        while heading_error > 180.0:  heading_error -= 360.0
        while heading_error < -180.0: heading_error += 360.0

        # 5. Differential Kinematics Dispatch
        if abs(heading_error) > 25.0:
            # Pivot in-place towards target
            turn_rate = 0.8 if heading_error > 0 else -0.8
            self.publish_twist(0.0, turn_rate)
        else:
            # Drive forward with proportional heading trimming
            forward_speed = 0.45
            steering_trim = heading_error * 0.015
            self.publish_twist(forward_speed, steering_trim)

    def publish_twist(self, linear_x: float, angular_z: float):
        t = Twist()
        t.linear.x = float(linear_x)
        t.angular.z = float(angular_z)
        self.cmd_pub.publish(t)

def main(args=None):
    parser = argparse.ArgumentParser(description="Swarm Autonomous Closed-Loop Coordinator")
    parser.add_argument("--bot_id", type=int, default=0, help="This robot ID (0 or 1)")
    parser.add_argument("--tx", type=float, default=25.0, help="Target X coordinate (cm)")
    parser.add_argument("--ty", type=float, default=45.0, help="Target Y coordinate (cm)")
    parser.add_argument("--port", type=int, default=5005, help="Vision UDP listener port")
    cli_args, remaining_args = parser.parse_known_args(args=sys.argv[1:])

    rclpy.init(args=remaining_args)
    node = ClosedLoopSwarmCoordinator(
        my_id=cli_args.bot_id,
        target_x=cli_args.tx,
        target_y=cli_args.ty,
        listen_port=cli_args.port
    )

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
