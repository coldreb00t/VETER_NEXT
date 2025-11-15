#!/usr/bin/env python3
"""
UDP Control Server for VETER Robot
Listens for UDP commands and publishes to ROS2 /cmd_vel topic

Multi-robot support:
- Each robot has unique ID (1-10)
- UDP port: 9000 + robot_id (9001, 9002, etc.)
- ROS_DOMAIN_ID: 10 + robot_id (11, 12, etc.)

Usage:
    python3 udp_control_server.py [robot_id]

Example:
    python3 udp_control_server.py 1  # Robot 1, port 9001, ROS_DOMAIN_ID=11
    python3 udp_control_server.py 2  # Robot 2, port 9002, ROS_DOMAIN_ID=12
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import socket
import json
import sys
import os


class UDPControlServer(Node):
    def __init__(self, robot_id=1):
        super().__init__(f'udp_control_server_robot{robot_id}')

        self.robot_id = robot_id
        self.port = 9000 + robot_id  # Port: 9001, 9002, 9003, etc.

        # Create publisher for velocity commands
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Create UDP socket
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.sock.bind(('0.0.0.0', self.port))
        self.sock.settimeout(0.01)  # Non-blocking with timeout

        self.get_logger().info(f'Robot #{robot_id}: UDP Control Server started on port {self.port}')
        self.get_logger().info(f'Robot #{robot_id}: ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")}')

        # Timer to check for incoming messages
        self.create_timer(0.01, self.check_udp)  # 100 Hz check rate

    def check_udp(self):
        """Check for incoming UDP messages"""
        try:
            data, addr = self.sock.recvfrom(1024)

            # Parse JSON
            msg_dict = json.loads(data.decode('utf-8'))

            # Create Twist message
            twist = Twist()
            twist.linear.x = float(msg_dict.get('linear', 0.0))
            twist.angular.z = float(msg_dict.get('angular', 0.0))

            # Publish
            self.cmd_vel_pub.publish(twist)

        except socket.timeout:
            # No data available, that's OK
            pass
        except json.JSONDecodeError as e:
            self.get_logger().error(f'Invalid JSON: {e}')
        except Exception as e:
            self.get_logger().error(f'Error: {e}')


def main():
    # Get robot ID from command line argument
    robot_id = 1
    if len(sys.argv) > 1:
        try:
            robot_id = int(sys.argv[1])
            if robot_id < 1 or robot_id > 10:
                print(f"Error: Robot ID must be between 1 and 10")
                sys.exit(1)
        except ValueError:
            print(f"Error: Invalid robot ID '{sys.argv[1]}'")
            sys.exit(1)

    # Use default ROS_DOMAIN_ID (0) to match MAVROS and other nodes
    # Multi-robot isolation will be handled by UDP port separation only
    ros_domain_id = int(os.environ.get('ROS_DOMAIN_ID', '0'))

    print(f"=== VETER Robot #{robot_id} UDP Control Server ===")
    print(f"UDP Port: {9000 + robot_id}")
    print(f"ROS_DOMAIN_ID: {ros_domain_id}")
    print(f"Publishing to: /cmd_vel")
    print("=" * 50)

    rclpy.init()
    server = UDPControlServer(robot_id=robot_id)

    try:
        rclpy.spin(server)
    except KeyboardInterrupt:
        print(f"\nRobot #{robot_id}: Shutting down...")

    server.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
