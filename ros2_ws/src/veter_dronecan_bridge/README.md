# VETER DroneCAN Bridge

ROS2 package providing bidirectional bridge between DroneCAN (CAN bus) and ROS2 topics for VETER_NEXT robot.

## Overview

This package enables communication between:
- **ESP32 Motor Controller** (Node ID 10) - Motor control via DroneCAN
- **ESP32 Sensor Hub** (Node ID 11) - Sensor data publishing via DroneCAN
- **Jetson Orin Nano** (Node ID 20) - ROS2 navigation and control

## Features

- **CAN to ROS2**: Receives DroneCAN messages and publishes to ROS2 topics
- **ROS2 to CAN**: Subscribes to ROS2 topics and sends DroneCAN commands
- **Multi-threaded**: Asynchronous CAN reception with callbacks
- **Configurable**: YAML-based parameter configuration
- **Sensor Support**: Ultrasonic, BME280 environmental sensor
- **Motor Control**: Twist-based differential drive control
- **Camera Control**: Servo pan/tilt commands
- **Lighting Control**: LED mode and brightness control

## Dependencies

- ROS2 Humble
- python3-can
- python3-numpy
- CAN interface (can0 configured and active)

## Installation

```bash
cd ~/jetson-robot-project/ros2_ws
colcon build --packages-select veter_dronecan_bridge
source install/setup.bash
```

## Usage

### Launch the bridge

```bash
ros2 launch veter_dronecan_bridge dronecan_bridge.launch.py
```

### Launch with custom config

```bash
ros2 launch veter_dronecan_bridge dronecan_bridge.launch.py \
    config_file:=/path/to/custom_params.yaml
```

### Run node directly

```bash
ros2 run veter_dronecan_bridge dronecan_bridge
```

## Published Topics (CAN → ROS2)

### Range Sensors (Ultrasonic)
- `/sensors/range/front` (sensor_msgs/Range)
- `/sensors/range/rear` (sensor_msgs/Range)
- `/sensors/range/left` (sensor_msgs/Range)
- `/sensors/range/right` (sensor_msgs/Range)

### Environmental Sensors (BME280)
- `/sensors/temperature` (sensor_msgs/Temperature)
- `/sensors/humidity` (sensor_msgs/RelativeHumidity)
- `/sensors/pressure` (sensor_msgs/FluidPressure)

### Status and Warnings
- `/collision/warning` (std_msgs/String) - Collision warnings
- `/motor_controller/status` (std_msgs/String) - Motor controller heartbeat
- `/sensor_hub/status` (std_msgs/String) - Sensor hub heartbeat

## Subscribed Topics (ROS2 → CAN)

### Motor Control
- `/cmd_vel` (geometry_msgs/Twist) - Velocity commands
  - `linear.x`: Forward/backward velocity (m/s)
  - `angular.z`: Rotation velocity (rad/s)

### Camera Control
- `/camera/servo/pan` (std_msgs/Int32) - Pan angle (0-180°)
- `/camera/servo/tilt` (std_msgs/Int32) - Tilt angle (30-150°)

### Lighting Control
- `/lighting/mode` (std_msgs/Int32) - LED mode (0=OFF, 1=AUTO, 2=ON, 3=BLINK)
- `/lighting/brightness` (std_msgs/Int32) - Brightness (0-255)

## DroneCAN Messages

### Received from CAN bus

| Message Type | ID (hex) | Source | Description |
|--------------|----------|--------|-------------|
| NodeStatus | 0x155 | All nodes | Heartbeat with uptime, health, mode |
| RangeSensor | 0x41A | Node 11 | 4x ultrasonic distances |
| AirData | 0x424 | Node 11 | Temperature, humidity, pressure |
| CollisionWarning | 0x42E | Node 11 | Direction, distance, severity |

### Sent to CAN bus

| Message Type | ID (hex) | Target | Description |
|--------------|----------|--------|-------------|
| ESC RawCommand | 0x406 | Node 10 | Left/right motor commands |
| ServoCommand | 0x480 | Node 11 | Pan/tilt servo angles |
| LEDCommand | 0x490 | Node 11 | LED mode and brightness |

## Parameters

Edit `config/dronecan_params.yaml`:

```yaml
dronecan_bridge:
  ros__parameters:
    can_interface: 'can0'           # CAN interface name
    can_bitrate: 1000000            # CAN bitrate (1 Mbps)
    jetson_node_id: 20              # This node's DroneCAN ID
    motor_controller_node_id: 10    # Motor controller node ID
    sensor_hub_node_id: 11          # Sensor hub node ID
    publish_rate: 10.0              # Sensor publish rate (Hz)
```

## Testing

### Check CAN interface

```bash
ip link show can0
candump can0
```

### Monitor ROS2 topics

```bash
# List all topics
ros2 topic list

# Monitor range sensors
ros2 topic echo /sensors/range/front

# Monitor motor status
ros2 topic echo /motor_controller/status

# Monitor collision warnings
ros2 topic echo /collision/warning
```

### Send test commands

```bash
# Send velocity command (forward 0.5 m/s)
ros2 topic pub /cmd_vel geometry_msgs/Twist "{linear: {x: 0.5}, angular: {z: 0.0}}"

# Send servo pan command (90 degrees)
ros2 topic pub /camera/servo/pan std_msgs/Int32 "{data: 90}"

# Send LED brightness (medium)
ros2 topic pub /lighting/brightness std_msgs/Int32 "{data: 128}"
```

## Troubleshooting

### CAN interface not found

Check if can0 is up and configured:

```bash
ip link show can0
sudo ip link set can0 up type can bitrate 1000000
```

### No messages received

Check CAN bus traffic:

```bash
candump can0
```

Verify ESP32 nodes are powered and transmitting heartbeats.

### Permission denied on CAN interface

Add user to required group:

```bash
sudo usermod -a -G dialout $USER
# Logout and login again
```

### ROS2 topics not publishing

Check node status:

```bash
ros2 node list
ros2 node info /dronecan_bridge
```

Enable debug logging:

```bash
ros2 run veter_dronecan_bridge dronecan_bridge --ros-args --log-level debug
```

## Architecture

```
┌─────────────────┐
│  ESP32 Motor    │ Node ID 10
│  Controller     │───┐
└─────────────────┘   │
                      │
┌─────────────────┐   │    ┌──────────────────┐
│  ESP32 Sensor   │ Node ID 11    │  Jetson Orin     │ Node ID 20
│  Hub            │───┼────┤  DroneCAN Bridge │
└─────────────────┘   │    └──────────────────┘
                      │             │
                   CAN Bus      ROS2 Topics
                   1 Mbps
```

## Future Enhancements

- [ ] DroneCAN parameter service
- [ ] Firmware update via CAN
- [ ] Battery monitoring
- [ ] IMU data integration
- [ ] GPS integration
- [ ] Advanced diagnostics
- [ ] Dynamic node discovery
- [ ] Message filtering and rate limiting

## License

MIT License - Part of VETER_NEXT project

## Authors

- Eugene Melnik
- Claude Code (AI Assistant)

---

*Last Updated: November 2025*
