#!/usr/bin/env python3
"""
UDP Telemetry Publisher for VETER Robot
Collects telemetry from ROS2 topics and sends to client via UDP

Multi-robot support:
- Each robot has unique ID (1-10)
- UDP port: 9100 + robot_id (9101, 9102, etc.)
- ROS_DOMAIN_ID: 10 + robot_id (11, 12, etc.)

Usage:
    python3 udp_telemetry_publisher.py [robot_id] [client_ip]

Example:
    python3 udp_telemetry_publisher.py 1 100.112.41.1  # Robot 1, send to client at 100.112.41.1
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import Twist
import socket
import json
import sys
import os


class TelemetryPublisher(Node):
    def __init__(self, robot_id=1, client_ip=None):
        super().__init__(f'telemetry_publisher_robot{robot_id}')

        self.robot_id = robot_id
        self.client_ip = client_ip
        self.port = 9100 + robot_id  # Telemetry port: 9101, 9102, etc.

        if not self.client_ip:
            self.get_logger().error('Client IP is required!')
            return

        # Create UDP socket for sending telemetry
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

        self.get_logger().info(f'Robot #{robot_id}: Telemetry Publisher started')
        self.get_logger().info(f'Robot #{robot_id}: Sending to {client_ip}:{self.port}')
        self.get_logger().info(f'Robot #{robot_id}: ROS_DOMAIN_ID={os.environ.get("ROS_DOMAIN_ID", "0")}')

        # Telemetry data
        self.battery_voltage = 57.6  # TODO: Subscribe to battery topic
        self.battery_percent = 100   # TODO: Calculate from voltage
        self.latitude = 0.0
        self.longitude = 0.0
        self.speed = 0.0
        self.current_left = 0.0   # TODO: Subscribe to VESC status
        self.current_right = 0.0  # TODO: Subscribe to VESC status

        # Subscribe to GPS data
        self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            10
        )

        # Subscribe to cmd_vel to estimate speed
        self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        # Timer to send telemetry at 5 Hz
        self.create_timer(0.2, self.send_telemetry)

    def gps_callback(self, msg):
        """Update GPS position"""
        self.latitude = msg.latitude
        self.longitude = msg.longitude

    def cmd_vel_callback(self, msg):
        """Update speed from cmd_vel (approximation)"""
        # Linear speed in m/s
        self.speed = abs(msg.linear.x)

    def send_telemetry(self):
        """Send telemetry data to client via UDP"""
        try:
            # Create telemetry packet
            telemetry = {
                'battery_voltage': self.battery_voltage,
                'battery_percent': self.battery_percent,
                'latitude': self.latitude,
                'longitude': self.longitude,
                'speed': self.speed,
                'current_left': self.current_left,
                'current_right': self.current_right
            }

            # Send UDP packet
            data = json.dumps(telemetry).encode('utf-8')
            self.sock.sendto(data, (self.client_ip, self.port))

        except Exception as e:
            self.get_logger().error(f'Error sending telemetry: {e}')


def main():
    # Get robot ID from command line argument
    robot_id = 1
    client_ip = None

    if len(sys.argv) > 1:
        try:
            robot_id = int(sys.argv[1])
            if robot_id < 1 or robot_id > 10:
                print(f"Error: Robot ID must be between 1 and 10")
                sys.exit(1)
        except ValueError:
            print(f"Error: Invalid robot ID '{sys.argv[1]}'")
            sys.exit(1)

    if len(sys.argv) > 2:
        client_ip = sys.argv[2]
    else:
        print(f"Error: Client IP address required")
        print(f"Usage: python3 udp_telemetry_publisher.py [robot_id] [client_ip]")
        sys.exit(1)

    # Set ROS_DOMAIN_ID based on robot ID (to match control server)
    ros_domain_id = 10 + robot_id  # Domain 11, 12, 13, etc.
    os.environ['ROS_DOMAIN_ID'] = str(ros_domain_id)

    print(f"=== VETER Robot #{robot_id} Telemetry Publisher ===")
    print(f"Sending to: {client_ip}:{9100 + robot_id}")
    print(f"ROS_DOMAIN_ID: {ros_domain_id}")
    print(f"Update rate: 5 Hz")
    print("=" * 50)

    rclpy.init()
    publisher = TelemetryPublisher(robot_id=robot_id, client_ip=client_ip)

    try:
        rclpy.spin(publisher)
    except KeyboardInterrupt:
        print(f"\nRobot #{robot_id}: Shutting down...")

    publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
