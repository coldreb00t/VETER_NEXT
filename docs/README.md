# VETER_NEXT Documentation

## Project Overview

VETER_NEXT (Versatile Electronic Terrain Explorer Robot) is a comprehensive autonomous tracked robot platform designed for multiple applications including security patrol, snow removal, lawn mowing, and fire suppression.

## Documentation Index

### Setup Guides
- [DroneCAN Setup](DRONECAN_SETUP.md) - Complete guide for configuring DroneCAN communication
- [VPS Tunnel Setup](vps-tunnel-setup.md) - Remote access configuration
- [Remote Access Guide](remote-access-guide.md) - How to connect remotely

### System Documentation
- [Failsafe Logic](FAILSAFE_LOGIC.md) - Safety systems and emergency stop procedures
- [Security Mode](SECURITY_MODE.md) - Autonomous patrol and threat detection
- [Voice Control](VOICE_CONTROL.md) - Voice command system setup

### Hardware Documentation
- ESP32 Motor Controller - DroneCAN motor control firmware
- ESP32 Sensor Hub - Sensor integration and peripheral control
- VESC Configuration - Electronic speed controller setup
- ArduRover Integration - GPS navigation and IMU data

### Software Documentation
- ROS2 Architecture - Node structure and communication
- DroneCAN Bridge - Protocol translation layer
- Perception System - YOLO-based object detection
- Telegram Bot - Remote monitoring and control

## Quick Links

- **GitHub Repository**: https://github.com/coldreb00t/VETER_NEXT
- **Main README**: ../README.md
- **CLAUDE.md**: ../CLAUDE.md (development guidance)

## System Architecture

### Hardware Layer
```
┌─────────────────────────────────────────────────────────┐
│  Jetson Orin Nano (Main Computer)                      │
│  - ROS2 Humble                                          │
│  - Computer Vision (YOLO)                               │
│  - High-level control                                   │
└──────────────────┬──────────────────────────────────────┘
                   │ CAN Bus (1 Mbps DroneCAN)
          ┌────────┴────────┬────────────────┬────────────┐
          │                 │                │            │
    ┌─────▼─────┐    ┌─────▼─────┐   ┌────▼─────┐  ┌───▼────┐
    │  ESP32 #1 │    │  ESP32 #2 │   │ VESC #1  │  │VESC #2 │
    │   Motor   │    │  Sensor   │   │  (Left)  │  │(Right) │
    │Controller │    │    Hub    │   │   ESC    │  │  ESC   │
    └─────┬─────┘    └─────┬─────┘   └────┬─────┘  └───┬────┘
          │                │              │            │
    ┌─────▼─────┐    ┌─────▼─────┐   ┌────▼─────┐  ┌───▼────┐
    │ExpressLRS │    │Ultrasonic │   │BM1418ZXF │  │BM1418ZXF│
    │  Receiver │    │ BME280    │   │1000W     │  │1000W   │
    │Emergency  │    │Servo/LED  │   │BLDC      │  │BLDC    │
    └───────────┘    └───────────┘   └──────────┘  └────────┘
```

### Control Priority Hierarchy
1. **Hardware Emergency Stop** (Physical button) - HIGHEST PRIORITY
2. **CRSF Manual Control** (ExpressLRS RC transmitter)
3. **DMR Voice Commands** (Radio voice control)
4. **Jetson Autonomous** (AI-driven autonomous operation)
5. **ArduRover GPS Navigation** (Waypoint navigation)
6. **Safe Stop** (Default safe state) - LOWEST PRIORITY

### Communication Channels
- **Fiber Optic**: Up to 3km range
- **Starlink Mini**: Global satellite connectivity
- **4G/5G**: Cellular network (Quectel EC25)
- **ExpressLRS**: 868MHz long-range RC (primary manual control)
- **WiFi**: Built-in Jetson wireless
- **DMR Radio**: Voice command channel (optional)

## Development Phases

### Phase 1: Basic Platform (Current)
- [x] Remote access via VPS tunnel
- [x] Project structure creation
- [ ] ESP32 firmware development
- [ ] DroneCAN communication setup
- [ ] Basic motor control
- [ ] ROS2 integration

### Phase 2: Sensors and Safety
- [ ] Ultrasonic sensor integration
- [ ] Camera setup and servo control
- [ ] ArduRover GPS integration
- [ ] Failsafe system implementation
- [ ] Battery monitoring

### Phase 3: Security Mode
- [ ] YOLO object detection
- [ ] Threat detection algorithms
- [ ] Alarm system (siren, strobe)
- [ ] Video recording
- [ ] Telegram notifications

### Phase 4: Voice Control
- [ ] DMR audio bridge
- [ ] Whisper STT (speech-to-text)
- [ ] Qwen command processing
- [ ] Voice command execution

### Phase 5: Deployment
- [ ] Docker containerization
- [ ] System service setup
- [ ] Testing and optimization
- [ ] Field trials

## Safety Features

### Emergency Stop System
- Hardware button with immediate motor cutoff
- Software watchdog timer
- Automatic stop on signal loss
- Low battery return-to-base

### Failsafe Conditions
- **Signal Loss** (>1 sec): Immediate stop
- **Low Battery** (<50.4V): Return to base
- **Overheating** (>70°C): Power reduction
- **GPS Loss** (>30 sec): Stop and wait
- **CAN Bus Error**: Switch to safe mode

### Battery Protection (18S LiFePO4)
- **Critical** (46.8V / 2.6V/cell): Emergency shutdown
- **Low** (50.4V / 2.8V/cell): Return to base
- **Normal** (57.6V / 3.2V/cell): Normal operation
- **Full** (65.7V / 3.65V/cell): Fully charged

## Getting Started

### Prerequisites
```bash
# Install system dependencies
sudo apt update
sudo apt install -y ros-humble-desktop can-utils python3-pip platformio

# Install Python packages
pip3 install pydronecan ultralytics transformers python-telegram-bot
```

### Clone and Setup
```bash
# Already in /home/jetson/jetson-robot-project
cd /home/jetson/jetson-robot-project

# Build ROS2 workspace
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# Configure CAN bus
sudo bash ../scripts/setup/configure_can.sh
```

### Flash ESP32 Firmware
```bash
# Motor controller
cd firmware/esp32_motor_controller
pio run -t upload

# Sensor hub
cd ../esp32_sensor_hub
pio run -t upload
```

### Launch System
```bash
# Minimal configuration (for testing)
ros2 launch veter_bringup veter_minimal.launch.py

# Full configuration (all features)
ros2 launch veter_bringup veter_full.launch.py

# Security mode only
ros2 launch veter_bringup veter_security.launch.py
```

## Troubleshooting

### CAN Bus Not Working
```bash
# Check interface
ip link show can0

# Reconfigure CAN
sudo ip link set can0 down
sudo ip link set can0 type can bitrate 1000000
sudo ip link set can0 up

# Monitor CAN traffic
candump can0
```

### Motors Not Responding
1. Check VESC power connection (57V battery)
2. Verify VESC CAN configuration (DroneCAN mode)
3. Check CAN termination resistors (120Ω each end)
4. Monitor DroneCAN heartbeat: `candump can0 | grep 341`

### ROS2 Nodes Not Starting
```bash
# Check ROS2 installation
ros2 doctor

# List available nodes
ros2 node list

# Check specific node
ros2 node info /dronecan_bridge
```

## Performance Targets

- **Control Latency**: < 10ms end-to-end
- **YOLO Detection**: >= 24 FPS @ 1080p
- **Battery Life**: 4-6 hours continuous operation
- **Max Speed**: ~5 km/h on flat terrain
- **Max Climb**: 30° angle
- **Operating Range**: Up to 3km with fiber, unlimited with Starlink

## Contributing

This is a personal project, but feedback and suggestions are welcome.

## License

Proprietary - All rights reserved

## Contact

- **Email**: eugene.a.melnik@gmail.com
- **GitHub**: https://github.com/coldreb00t/VETER_NEXT
- **Remote Access**: `ssh -p 2223 jetson@81.200.157.230`

---

*Last Updated: November 2025*
*Development Phase: Phase 1 - Basic Platform*
