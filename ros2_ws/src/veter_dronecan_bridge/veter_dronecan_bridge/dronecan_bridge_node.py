#!/usr/bin/env python3
"""
DroneCAN Bridge Node

ROS2 node that bridges DroneCAN (CAN bus) and ROS2 topics.
Subscribes to ROS2 topics and publishes to CAN bus, and vice versa.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String, Float32, Int32
from sensor_msgs.msg import Range, Temperature, RelativeHumidity, FluidPressure
from geometry_msgs.msg import Twist

from .can_interface import CANInterface, DroneCAN


class DroneCANBridgeNode(Node):
    """ROS2 node for DroneCAN bridge"""

    def __init__(self):
        super().__init__('dronecan_bridge')

        # Declare parameters
        self.declare_parameter('can_interface', 'can0')
        self.declare_parameter('can_bitrate', 1000000)
        self.declare_parameter('jetson_node_id', 20)
        self.declare_parameter('motor_controller_node_id', 10)
        self.declare_parameter('sensor_hub_node_id', 11)
        self.declare_parameter('publish_rate', 10.0)

        # Get parameters
        can_interface = self.get_parameter('can_interface').value
        can_bitrate = self.get_parameter('can_bitrate').value
        self.jetson_node_id = self.get_parameter('jetson_node_id').value
        self.motor_node_id = self.get_parameter('motor_controller_node_id').value
        self.sensor_node_id = self.get_parameter('sensor_hub_node_id').value
        publish_rate = self.get_parameter('publish_rate').value

        self.get_logger().info(f'Starting DroneCAN Bridge on {can_interface} @ {can_bitrate} bps')

        # Initialize CAN interface
        self.can = CANInterface(interface=can_interface, bitrate=can_bitrate)
        if not self.can.start():
            self.get_logger().error('Failed to start CAN interface!')
            raise RuntimeError('CAN initialization failed')

        # Register CAN receive callback
        self.can.register_callback(self.can_receive_callback)
        self.can.start_receive_thread()

        # QoS profile for sensor data
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Publishers (CAN -> ROS2)
        self.pub_range_front = self.create_publisher(Range, '/sensors/range/front', sensor_qos)
        self.pub_range_rear = self.create_publisher(Range, '/sensors/range/rear', sensor_qos)
        self.pub_range_left = self.create_publisher(Range, '/sensors/range/left', sensor_qos)
        self.pub_range_right = self.create_publisher(Range, '/sensors/range/right', sensor_qos)

        self.pub_temperature = self.create_publisher(Temperature, '/sensors/temperature', sensor_qos)
        self.pub_humidity = self.create_publisher(RelativeHumidity, '/sensors/humidity', sensor_qos)
        self.pub_pressure = self.create_publisher(FluidPressure, '/sensors/pressure', sensor_qos)

        self.pub_collision_warning = self.create_publisher(String, '/collision/warning', 10)

        self.pub_motor_status = self.create_publisher(String, '/motor_controller/status', 10)
        self.pub_sensor_status = self.create_publisher(String, '/sensor_hub/status', 10)

        # Subscribers (ROS2 -> CAN)
        self.sub_cmd_vel = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10
        )

        self.sub_servo_pan = self.create_subscription(
            Int32,
            '/camera/servo/pan',
            self.servo_pan_callback,
            10
        )

        self.sub_servo_tilt = self.create_subscription(
            Int32,
            '/camera/servo/tilt',
            self.servo_tilt_callback,
            10
        )

        self.sub_led_mode = self.create_subscription(
            Int32,
            '/lighting/mode',
            self.led_mode_callback,
            10
        )

        self.sub_led_brightness = self.create_subscription(
            Int32,
            '/lighting/brightness',
            self.led_brightness_callback,
            10
        )

        # State variables
        self.servo_pan = 90
        self.servo_tilt = 90
        self.led_mode = 1  # AUTO
        self.led_brightness = 128
        self.last_left_cmd = 0   # Last motor command (for periodic sending)
        self.last_right_cmd = 0

        # Timers
        self.create_timer(1.0 / publish_rate, self.timer_callback)
        # Motor command timer (100 Hz for VESC watchdog)
        self.create_timer(0.01, self.motor_command_timer)

        self.get_logger().info('DroneCAN Bridge initialized successfully')

    def can_receive_callback(self, msg):
        """
        Callback for received CAN messages

        Args:
            msg: can.Message object
        """
        # Extract message type and source node
        msg_type, node_id = DroneCAN.extract_message_info(msg.arbitration_id)

        # Parse and publish based on message type
        if msg_type == DroneCAN.MSG_TYPE_HEARTBEAT:
            self.handle_heartbeat(node_id, msg.data)

        elif msg_type == DroneCAN.MSG_TYPE_RANGE_SENSOR:
            self.handle_range_sensor(node_id, msg.data)

        elif msg_type == DroneCAN.MSG_TYPE_AIR_DATA:
            self.handle_air_data(node_id, msg.data)

        elif msg_type == DroneCAN.MSG_TYPE_COLLISION_WARNING:
            self.handle_collision_warning(node_id, msg.data)

    def handle_heartbeat(self, node_id: int, data: bytes):
        """Handle heartbeat message"""
        heartbeat = DroneCAN.parse_heartbeat(data)
        if not heartbeat:
            return

        health_names = ['OK', 'WARNING', 'ERROR', 'CRITICAL']
        health_str = health_names[heartbeat['health']] if heartbeat['health'] < 4 else 'UNKNOWN'

        status_msg = String()
        status_msg.data = f"uptime={heartbeat['uptime_sec']}s health={health_str} mode={heartbeat['mode']}"

        if node_id == self.motor_node_id:
            self.pub_motor_status.publish(status_msg)
        elif node_id == self.sensor_node_id:
            self.pub_sensor_status.publish(status_msg)

    def handle_range_sensor(self, node_id: int, data: bytes):
        """Handle range sensor (ultrasonic) message"""
        ranges = DroneCAN.parse_range_sensor(data)
        if not ranges:
            return

        timestamp = self.get_clock().now().to_msg()

        # Publish individual range sensors
        for direction, distance in ranges.items():
            if distance is None:
                continue

            range_msg = Range()
            range_msg.header.stamp = timestamp
            range_msg.header.frame_id = f'sonar_{direction}'
            range_msg.radiation_type = Range.ULTRASOUND
            range_msg.field_of_view = 0.26  # ~15 degrees
            range_msg.min_range = 0.02  # 2 cm
            range_msg.max_range = 4.0  # 400 cm
            range_msg.range = distance / 100.0  # Convert cm to meters

            # Publish to appropriate topic
            if direction == 'front':
                self.pub_range_front.publish(range_msg)
            elif direction == 'rear':
                self.pub_range_rear.publish(range_msg)
            elif direction == 'left':
                self.pub_range_left.publish(range_msg)
            elif direction == 'right':
                self.pub_range_right.publish(range_msg)

    def handle_air_data(self, node_id: int, data: bytes):
        """Handle air data (BME280) message"""
        air_data = DroneCAN.parse_air_data(data)
        if not air_data:
            return

        timestamp = self.get_clock().now().to_msg()

        # Temperature
        temp_msg = Temperature()
        temp_msg.header.stamp = timestamp
        temp_msg.header.frame_id = 'bme280'
        temp_msg.temperature = air_data['temperature']
        temp_msg.variance = 0.5  # ±0.5°C typical accuracy
        self.pub_temperature.publish(temp_msg)

        # Humidity
        hum_msg = RelativeHumidity()
        hum_msg.header.stamp = timestamp
        hum_msg.header.frame_id = 'bme280'
        hum_msg.relative_humidity = air_data['humidity'] / 100.0  # Convert to 0-1 range
        hum_msg.variance = 0.03  # ±3% typical accuracy
        self.pub_humidity.publish(hum_msg)

        # Pressure
        pres_msg = FluidPressure()
        pres_msg.header.stamp = timestamp
        pres_msg.header.frame_id = 'bme280'
        pres_msg.fluid_pressure = air_data['pressure'] * 100.0  # Convert hPa to Pa
        pres_msg.variance = 100.0  # ±1 hPa = ±100 Pa
        self.pub_pressure.publish(pres_msg)

    def handle_collision_warning(self, node_id: int, data: bytes):
        """Handle collision warning message"""
        warning = DroneCAN.parse_collision_warning(data)
        if not warning:
            return

        severity_names = ['INFO', 'WARNING', 'CRITICAL']
        severity_str = severity_names[warning['severity']] if warning['severity'] < 3 else 'UNKNOWN'

        msg = String()
        msg.data = f"[{severity_str}] {warning['direction']}: {warning['distance_cm']:.1f} cm"

        self.pub_collision_warning.publish(msg)

        # Log critical warnings
        if warning['severity'] == 2:
            self.get_logger().warn(f"COLLISION CRITICAL: {msg.data}")

    def cmd_vel_callback(self, msg: Twist):
        """
        Handle /cmd_vel messages and send motor commands

        Args:
            msg: Twist message with linear.x and angular.z
        """
        # Convert Twist to differential motor commands
        # linear.x: forward/backward velocity (m/s)
        # angular.z: rotation velocity (rad/s)

        # Scale factors (tune these based on robot characteristics)
        max_linear_velocity = 1.0  # m/s
        max_angular_velocity = 2.0  # rad/s
        max_motor_command = 8191

        # Normalize inputs
        linear = max(min(msg.linear.x / max_linear_velocity, 1.0), -1.0)
        angular = max(min(msg.angular.z / max_angular_velocity, 1.0), -1.0)

        # Differential drive mixing
        left = linear + angular * 0.5
        right = linear - angular * 0.5

        # Constrain to [-1, 1]
        left = max(min(left, 1.0), -1.0)
        right = max(min(right, 1.0), -1.0)

        # Convert to motor command range
        left_cmd = int(left * max_motor_command)
        right_cmd = int(right * max_motor_command)

        # Build and send ESC command
        can_id, data = DroneCAN.build_esc_command(
            self.jetson_node_id,
            left_cmd,
            right_cmd
        )

        self.can.send_frame(can_id, data)

        # Store last command for periodic sending
        self.last_left_cmd = left_cmd
        self.last_right_cmd = right_cmd

    def motor_command_timer(self):
        """Send motor commands at 100 Hz (VESC watchdog requirement)"""
        can_id, data = DroneCAN.build_esc_command(
            self.jetson_node_id,
            self.last_left_cmd,
            self.last_right_cmd
        )
        self.can.send_frame(can_id, data)

    def servo_pan_callback(self, msg: Int32):
        """Handle servo pan command"""
        self.servo_pan = max(min(msg.data, 180), 0)
        self.send_servo_command()

    def servo_tilt_callback(self, msg: Int32):
        """Handle servo tilt command"""
        self.servo_tilt = max(min(msg.data, 180), 30)
        self.send_servo_command()

    def send_servo_command(self):
        """Send servo command to sensor hub"""
        can_id, data = DroneCAN.build_servo_command(
            self.jetson_node_id,
            self.servo_pan,
            self.servo_tilt
        )
        self.can.send_frame(can_id, data)

    def led_mode_callback(self, msg: Int32):
        """Handle LED mode command"""
        self.led_mode = max(min(msg.data, 4), 0)
        self.send_led_command()

    def led_brightness_callback(self, msg: Int32):
        """Handle LED brightness command"""
        self.led_brightness = max(min(msg.data, 255), 0)
        self.send_led_command()

    def send_led_command(self):
        """Send LED command to sensor hub"""
        can_id, data = DroneCAN.build_led_command(
            self.jetson_node_id,
            self.led_mode,
            self.led_brightness,
            0, 0, 0, 0  # Individual channels (0 = use global brightness)
        )
        self.can.send_frame(can_id, data)

    def timer_callback(self):
        """Periodic timer callback"""
        # Could send periodic status or heartbeat here if needed
        pass

    def destroy_node(self):
        """Clean shutdown"""
        self.get_logger().info('Shutting down DroneCAN Bridge')
        self.can.stop()
        super().destroy_node()


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    try:
        node = DroneCANBridgeNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error in DroneCAN Bridge: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
