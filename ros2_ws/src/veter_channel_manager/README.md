# VETER Channel Manager

Multi-channel communication manager with dynamic failover for VETER_NEXT robot platform.

## Overview

The Channel Manager provides intelligent failover between 6 different communication channels:
- **Fiber Optic** (up to 3km, lowest latency)
- **Starlink Mini** (global satellite coverage)
- **4G/5G** (cellular connectivity)
- **WiFi** (local high-bandwidth)
- **DMR Radio** (voice control, emergency)
- **ExpressLRS** (868MHz RC, always available)

## Features

- **Dynamic Priority Chain**: Operator configures failover order per mission
- **Health Monitoring**: Real-time monitoring of all channels with configurable timeouts
- **Automatic Failover**: Seamless switching between channels based on availability
- **Hysteresis**: Prevents rapid channel switching (flapping)
- **Preset Configurations**: 4 ready-to-use configurations for common scenarios
- **Hot-Reload**: Configuration can be changed without restarting
- **Safe Stop**: Automatic motor stop if all channels fail

## Architecture

```
┌──────────────────────────────────────────────────────────────┐
│                    Channel Manager Node                       │
│                                                                │
│  ┌────────────────┐         ┌──────────────────┐             │
│  │ Health Monitor │◄────────┤ Failover Manager │             │
│  │  - Timeouts    │         │ - Priority chain │             │
│  │  - Error rate  │         │ - Hysteresis     │             │
│  └────────────────┘         └──────────────────┘             │
│         │                            │                        │
│         ▼                            ▼                        │
│  ┌──────────────────────────────────────────┐                │
│  │     Active Channel Selection             │                │
│  └──────────────────────────────────────────┘                │
└──────────────────────────────────────────────────────────────┘
         │                                        │
         │ Subscribes:                            │ Publishes:
         │ /cmd_vel_fiber                         │ /cmd_vel
         │ /cmd_vel_starlink                      │ /channel_manager/status
         │ /cmd_vel_4g                            │ /channel_manager/active_channel
         │ /cmd_vel_wifi                          │
         │ /cmd_vel_dmr                           │
         │ /cmd_vel_expresslrs                    │
         └────────────────────────────────────────┘
```

## Installation

Package is already part of the VETER_NEXT workspace. Build with:

```bash
cd ~/jetson-robot-project/ros2_ws
colcon build --packages-select veter_channel_manager
source install/setup.bash
```

## Usage

### Basic Usage

Launch with default configuration (all channels):
```bash
ros2 launch veter_channel_manager channel_manager.launch.py
```

### Preset Configurations

**RC Only Mode** (manual control only):
```bash
ros2 launch veter_channel_manager channel_manager.launch.py config:=channels_rc_only.yaml
```

**Long Range Mode** (satellite and cellular):
```bash
ros2 launch veter_channel_manager channel_manager.launch.py config:=channels_long_range.yaml
```

**Voice Control Mode** (DMR radio):
```bash
ros2 launch veter_channel_manager channel_manager.launch.py config:=channels_voice.yaml
```

### Custom Configuration

Create your own YAML configuration file:

```yaml
channel_manager:
  enabled_channels:
    - starlink
    - expresslrs

  priority_chain:
    1: starlink
    2: expresslrs
    3: safe_stop

  timeouts:
    starlink: 3.0
    expresslrs: 1.0

  hysteresis_time: 5.0
  update_rate: 10.0
```

Then launch with:
```bash
ros2 launch veter_channel_manager channel_manager.launch.py config:=my_config.yaml
```

## Topics

### Subscribed Topics

- `/cmd_vel_fiber` (geometry_msgs/Twist) - Commands from fiber optic channel
- `/cmd_vel_starlink` (geometry_msgs/Twist) - Commands from Starlink
- `/cmd_vel_4g` (geometry_msgs/Twist) - Commands from 4G/5G
- `/cmd_vel_wifi` (geometry_msgs/Twist) - Commands from WiFi
- `/cmd_vel_dmr` (geometry_msgs/Twist) - Commands from DMR radio
- `/cmd_vel_expresslrs` (geometry_msgs/Twist) - Commands from RC receiver

### Published Topics

- `/cmd_vel` (geometry_msgs/Twist) - Unified output to motor controller
- `/channel_manager/status` (std_msgs/String) - Detailed status of all channels
- `/channel_manager/active_channel` (std_msgs/String) - Currently active channel name

## Monitoring

### View Current Status

```bash
# Watch active channel
ros2 topic echo /channel_manager/active_channel

# Watch detailed status
ros2 topic echo /channel_manager/status
```

### Test Channel Inputs

Simulate commands from different channels:

```bash
# Simulate fiber optic command
ros2 topic pub /cmd_vel_fiber geometry_msgs/Twist "{linear: {x: 0.5}}"

# Simulate RC command
ros2 topic pub /cmd_vel_expresslrs geometry_msgs/Twist "{linear: {x: 0.3}}"
```

## Configuration Parameters

### Channel Timeouts

Timeout before channel considered failed (seconds):
- `fiber`: 2.0 (low latency expected)
- `starlink`: 3.0-5.0 (satellite latency)
- `4g`: 3.0-4.0 (variable cellular latency)
- `wifi`: 2.0 (fast local network)
- `dmr`: 5.0-8.0 (voice processing time)
- `expresslrs`: 1.0 (critical for safety)

### Hysteresis Time

Minimum time before switching back to higher priority channel (seconds):
- Short (2-3s): Fast response to channel recovery
- Medium (5-7s): Balanced stability
- Long (10s+): Prevents frequent switching on unstable links

## Failover Behavior

### Priority-Based Selection

1. Channel Manager continuously monitors all enabled channels
2. Selects highest priority **healthy** channel from priority chain
3. Only switches to lower priority if current channel fails
4. Switches to higher priority after hysteresis time expires

### Health States

- **UNKNOWN**: No data received yet
- **HEALTHY**: Receiving data within timeout, low error rate
- **DEGRADED**: Receiving data but high error rate (>30%)
- **FAILED**: No data received within timeout

### Example Failover Sequence

```
Initial State:
  Priority: [fiber, starlink, 4g, expresslrs]
  Active: fiber

Fiber fails:
  → Switch to starlink (next available)
  Active: starlink

Fiber recovers:
  → Wait 5s (hysteresis)
  → Switch back to fiber (higher priority)
  Active: fiber

All channels fail:
  → Switch to safe_stop
  → Publish zero velocity
  Active: safe_stop
```

## Integration with VETER_NEXT

### With DroneCAN Bridge

The ExpressLRS channel is already implemented in the DroneCAN bridge:

```bash
# Terminal 1: DroneCAN Bridge (provides /cmd_vel_expresslrs)
ros2 launch veter_dronecan_bridge dronecan_bridge.launch.py

# Terminal 2: Channel Manager
ros2 launch veter_channel_manager channel_manager.launch.py
```

### Full System Launch

```bash
# Launch complete system with RC-only mode
ros2 launch veter_bringup veter_minimal.launch.py channel_config:=channels_rc_only.yaml
```

## Troubleshooting

### No Commands Being Published

Check if any channel is healthy:
```bash
ros2 topic echo /channel_manager/status
```

Look for channels in HEALTHY or DEGRADED state.

### Rapid Channel Switching

Increase hysteresis time in configuration:
```yaml
hysteresis_time: 10.0  # Increase from 5.0
```

### Channel Not Detected

1. Verify channel is enabled in config
2. Check timeout is appropriate for channel latency
3. Ensure commands are being published to correct topic:
   ```bash
   ros2 topic list | grep cmd_vel
   ```

## Development

### Running Tests

```bash
cd ~/jetson-robot-project/ros2_ws
colcon test --packages-select veter_channel_manager
```

### Code Structure

```
veter_channel_manager/
├── veter_channel_manager/
│   ├── __init__.py                  # Package init
│   ├── channel_health.py            # Health monitoring logic
│   ├── failover_logic.py            # Failover algorithm
│   └── channel_manager_node.py      # Main ROS2 node
├── config/
│   ├── channels_default.yaml        # All channels config
│   ├── channels_rc_only.yaml        # RC only config
│   ├── channels_long_range.yaml     # Satellite/cellular config
│   └── channels_voice.yaml          # Voice control config
├── launch/
│   └── channel_manager.launch.py    # Launch file
└── README.md                         # This file
```

## License

MIT License - Part of VETER_NEXT project

## Author

Eugene Melnik (eugene.a.melnik@gmail.com)

## See Also

- [VETER_NEXT Main Documentation](../../docs/README.md)
- [DroneCAN Bridge Package](../veter_dronecan_bridge/README.md)
- [CLAUDE.md](../../CLAUDE.md) - Project overview for Claude Code
