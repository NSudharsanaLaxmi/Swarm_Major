#!/usr/bin/env python3
"""
======================================================================================
STEP 4: ROS 2 / PYTHON UDP VELOCITY BRIDGE
======================================================================================
Purpose: Subscribes to geometry_msgs/msg/Twist on '/cmd_vel' and transmits
         serialized velocity strings ("<linear_x>,<angular_z>") via UDP to the ESP32.

Execution:
  python3 step4_udp_bridge.py --ip <ESP32_IP> --port 8888
======================================================================================
"""

import sys
import argparse
import socket
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class UDPBridgeNode(Node):
    def __init__(self, target_ip: str, target_port: int):
        super().__init__('udp_velocity_bridge')
        self.target_ip = target_ip
        self.target_port = target_port

        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        self.get_logger().info(f"[BRIDGE] Initialized.")
        self.get_logger().info(f"[BRIDGE] Forwarding '/cmd_vel' -> UDP {self.target_ip}:{self.target_port}")

    def cmd_vel_callback(self, msg: Twist):
        lin_x = round(msg.linear.x, 2)
        ang_z = round(msg.angular.z, 2)
        payload = f"{lin_x},{ang_z}".encode('utf-8')
        try:
            self.sock.sendto(payload, (self.target_ip, self.target_port))
        except Exception as e:
            self.get_logger().error(f"[BRIDGE] Send error: {e}")

def main(args=None):
    parser = argparse.ArgumentParser(description="ROS 2 to UDP Robot Velocity Bridge")
    parser.add_argument("--ip", type=str, default="172.20.10.3", help="Target ESP32 IP address")
    parser.add_argument("--port", type=int, default=8888, help="Target ESP32 UDP port")
    cli_args, remaining_args = parser.parse_known_args(args=sys.argv[1:])

    rclpy.init(args=remaining_args)
    node = UDPBridgeNode(target_ip=cli_args.ip, target_port=cli_args.port)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("[BRIDGE] Shutting down.")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
