#!/usr/bin/env python3
"""
Channel Manager Node

Multi-channel communication manager with dynamic failover for VETER_NEXT.
Manages 6 communication channels with operator-configurable priority chain.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import String
import yaml
import os

from .channel_health import ChannelHealthManager, ChannelState
from .failover_logic import FailoverManager


class ChannelManagerNode(Node):
    """ROS2 node for channel management"""

    def __init__(self):
        super().__init__('channel_manager')

        # Declare parameters
        self.declare_parameter('config_file', 'channels_default.yaml')
        self.declare_parameter('update_rate', 10.0)  # Hz

        # Get parameters
        config_file = self.get_parameter('config_file').value
        update_rate = self.get_parameter('update_rate').value

        # Initialize managers
        self.health_manager = ChannelHealthManager()
        self.failover_manager = FailoverManager(self.health_manager)

        # Load configuration
        self.load_config(config_file)

        # Create publishers
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/channel_manager/status', 10)
        self.active_channel_pub = self.create_publisher(String, '/channel_manager/active_channel', 10)

        # Create subscribers for each channel
        self.cmd_vel_subs = {}
        for channel in self.config.get('enabled_channels', []):
            topic = f'/cmd_vel_{channel}'
            self.cmd_vel_subs[channel] = self.create_subscription(
                Twist,
                topic,
                lambda msg, ch=channel: self.cmd_vel_callback(msg, ch),
                10
            )

        # Create timer for periodic updates
        timer_period = 1.0 / update_rate
        self.create_timer(timer_period, self.update_callback)

        # Create timer for status publishing
        self.create_timer(1.0, self.publish_status)

        # Last received command
        self.last_cmd_vel = None

        self.get_logger().info('Channel Manager started')
        self.get_logger().info(f'Enabled channels: {self.config.get("enabled_channels", [])}')
        self.get_logger().info(f'Priority chain: {self.failover_manager.get_priority_chain()}')

    def load_config(self, config_file: str) -> None:
        """
        Load configuration from YAML file

        Args:
            config_file: Configuration file name
        """
        # Find config file in package share directory
        try:
            from ament_index_python.packages import get_package_share_directory
            pkg_dir = get_package_share_directory('veter_channel_manager')
            config_path = os.path.join(pkg_dir, 'config', config_file)
        except Exception:
            # Fallback to relative path for development
            config_path = os.path.join(
                os.path.dirname(__file__),
                '..',
                'config',
                config_file
            )

        # Load YAML
        try:
            with open(config_path, 'r') as f:
                full_config = yaml.safe_load(f)
                self.config = full_config.get('channel_manager', {})
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')
            # Use default config
            self.config = {
                'enabled_channels': ['expresslrs'],
                'priority_chain': ['expresslrs', 'safe_stop'],
                'timeouts': {'expresslrs': 1.0},
                'hysteresis_time': 5.0
            }

        # Initialize health monitoring for enabled channels
        for channel in self.config.get('enabled_channels', []):
            timeout = self.config.get('timeouts', {}).get(channel, 2.0)
            self.health_manager.add_channel(channel, timeout)

        # Set priority chain
        priority_chain = []
        for priority, channel in sorted(self.config.get('priority_chain', {}).items()):
            priority_chain.append(channel)
        self.failover_manager.set_priority_chain(priority_chain)

        # Set hysteresis time
        hysteresis = self.config.get('hysteresis_time', 5.0)
        self.failover_manager.set_hysteresis_time(hysteresis)

    def cmd_vel_callback(self, msg: Twist, channel: str) -> None:
        """
        Handle cmd_vel from specific channel

        Args:
            msg: Twist message
            channel: Source channel name
        """
        # Update health: received data from this channel
        self.health_manager.update_data(channel)

        # Check if this is the active channel
        current_channel = self.failover_manager.get_current_channel()
        if current_channel == channel:
            # Forward command to main cmd_vel topic
            self.cmd_vel_pub.publish(msg)
            self.last_cmd_vel = msg

    def update_callback(self) -> None:
        """Periodic update callback"""
        # Check all channel health
        self.health_manager.check_all()

        # Select best channel (may trigger failover)
        self.failover_manager.select_best_channel()

        # Publish safe stop if no active channel
        current_channel = self.failover_manager.get_current_channel()
        if current_channel == 'safe_stop' or current_channel is None:
            # Publish zero velocity
            stop_cmd = Twist()
            self.cmd_vel_pub.publish(stop_cmd)

    def publish_status(self) -> None:
        """Publish status information"""
        # Get current channel
        current_channel = self.failover_manager.get_current_channel()

        # Publish active channel
        channel_msg = String()
        channel_msg.data = current_channel if current_channel else 'none'
        self.active_channel_pub.publish(channel_msg)

        # Get failover status
        failover_status = self.failover_manager.get_status()

        # Get health info
        health_info = self.health_manager.get_all_info()

        # Build status message
        status_lines = []
        status_lines.append(f"Active Channel: {current_channel}")
        status_lines.append(f"Priority Chain: {', '.join(failover_status['priority_chain'])}")
        status_lines.append(f"Time Since Switch: {failover_status['time_since_switch']:.1f}s")
        status_lines.append("")
        status_lines.append("Channel Health:")

        for channel, info in health_info.items():
            state = info['state']
            time_since = info['time_since_contact']
            error_rate = info['error_rate']

            status_lines.append(
                f"  {channel:12s} | {state:8s} | "
                f"Last: {time_since:5.1f}s | Errors: {error_rate*100:5.1f}%"
            )

        # Publish status
        status_msg = String()
        status_msg.data = '\n'.join(status_lines)
        self.status_pub.publish(status_msg)

        # Log active channel changes
        if hasattr(self, '_last_logged_channel'):
            if self._last_logged_channel != current_channel:
                self.get_logger().info(f'Channel switched: {self._last_logged_channel} -> {current_channel}')
                self._last_logged_channel = current_channel
        else:
            self._last_logged_channel = current_channel


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    try:
        node = ChannelManagerNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in channel manager node: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
