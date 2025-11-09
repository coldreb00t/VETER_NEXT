# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project Overview

**VETER_NEXT** (Versatile Electronic Terrain Explorer Robot) is a multi-functional autonomous tracked robot platform based on NVIDIA Jetson Orin Nano.

**GitHub Repository**: https://github.com/coldreb00t/VETER_NEXT

### Key Capabilities
- Perimeter security patrol
- Snow removal
- Lawn mowing
- Fire suppression
- Swarm operations support

### Critical Features
- **Full redundancy**: Multiple control channels with priority hierarchy
- **Unified protocol**: DroneCAN for all devices
- **Modular architecture**: Hot-swappable functional modules
- **Fail-safe design**: Emergency stop, automatic failsafe on signal loss

## Hardware Platform

### Computing
- **Main**: NVIDIA Jetson Orin Nano Super Developer Kit (8GB, 40 TOPS)
- **MCU #1**: ESP32-S3 (16MB Flash, 8MB PSRAM) - Motor Controller (DroneCAN Node ID 10)
- **MCU #2**: ESP32-S3 (16MB Flash, 8MB PSRAM) - Sensor Hub (DroneCAN Node ID 11)
- **Flight Controller**: Mini Pixhawk (ArduRover firmware) - GPS/IMU/Compass

### Propulsion
- **Motors**: 2x BM1418ZXF BLDC 1000W 48V
- **ESCs**: 2x VESC 75200 (DroneCAN mode)
- **Tracks**: 2.82m circumference, 220mm width
- **Battery**: 18S LiFePO4 (57.6V nominal, 105Ah)

### Sensors
- **Camera**: Sony IMX477 12MP (MIPI CSI to Jetson CAM0)
- **GPS**: U-blox M9N with compass (on ArduRover)
- **UWB**: DW3000 for swarm positioning
- **Ultrasonic**: 4x HC-SR04 (on ESP32 #2)
- **Environment**: BME280 temperature/humidity (I2C)

### Communications (Equal Priority Channels)
1. Fiber optic (up to 3km)
2. Starlink Mini
3. 4G/5G (Quectel EC25)
4. ExpressLRS 868MHz (to ESP32 #1)
5. WiFi (built-in Jetson)
6. DMR radio (optional voice control)

## Software Architecture

### Core Technologies
- **Protocol**: DroneCAN (1 Mbps CAN bus)
- **Framework**: ROS2 Humble
- **ESP32**: Arduino framework via PlatformIO
- **Vision**: YOLOv8n optimized for TensorRT
- **Voice**: Whisper STT + Qwen3-1.5B

### Control Priority Hierarchy
1. Hardware Emergency Stop (HIGHEST)
2. CRSF Manual Control (ExpressLRS)
3. DMR Voice Commands
4. Jetson Autonomous Mode
5. ArduRover GPS Navigation
6. Safe Stop (LOWEST)

### Project Structure
```
veter/
├── firmware/              # ESP32 firmware (PlatformIO)
│   ├── esp32_motor_controller/
│   ├── esp32_sensor_hub/
│   └── esp32_voice_bridge/
├── ros2_ws/              # ROS2 workspace
│   └── src/
│       ├── veter_bringup/
│       ├── veter_dronecan_bridge/
│       ├── veter_perception/
│       ├── veter_security/
│       ├── veter_voice/
│       └── veter_teleop/
├── config/               # Configuration files
│   ├── dronecan/
│   ├── vesc/
│   ├── ardurover/
│   └── security/
├── scripts/              # Setup and diagnostic scripts
│   ├── setup/
│   ├── diagnostics/
│   └── startup/
├── docker/               # Docker configs for deployment
├── docs/                 # Documentation
└── tests/               # Unit and integration tests
```

## Common Commands

### Remote Access
```bash
# Connect from anywhere
ssh -p 2223 jetson@81.200.157.230
```

### CAN Bus Setup
```bash
# Configure CAN interface
sudo bash scripts/setup/configure_can.sh

# Monitor CAN traffic
candump can0

# Test DroneCAN nodes
python3 scripts/diagnostics/can_monitor.py
```

### ESP32 Development
```bash
# Flash motor controller
cd firmware/esp32_motor_controller
pio run -t upload

# Monitor serial output
pio device monitor
```

### ROS2 Development
```bash
# Build workspace
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# Launch minimal configuration
ros2 launch veter_bringup veter_minimal.launch.py

# Launch security mode
ros2 launch veter_bringup veter_security.launch.py

# Teleop control
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### VESC Configuration
```bash
# Connect to VESC Tool via CAN
# Set App to: No App
# Set CAN Mode: UAVCAN
# Set VESC ID: 1 (left), 2 (right)
# Set UAVCAN ESC Index: 0 (left), 1 (right)
# Set CAN Baud Rate: CAN_BAUD_1M
```

### System Diagnostics
```bash
# Motor test
python3 scripts/diagnostics/motor_test.py

# System health
python3 scripts/diagnostics/system_health.py

# DroneCAN GUI
bash scripts/diagnostics/dronecan_gui.sh
```

## Development Guidelines

### DroneCAN Message Types
- `uavcan.equipment.esc.RawCommand` (ID: 1030) - Motor commands
- `uavcan.equipment.esc.Status` (ID: 1034) - ESC status
- `uavcan.protocol.NodeStatus` (ID: 341) - Heartbeat
- `uavcan.equipment.range_sensor.Measurement` (ID: 1050) - Ultrasonic
- `uavcan.equipment.air_data.StaticTemperature` (ID: 1028) - Temperature

### Battery Thresholds (18S LiFePO4)
- **CRITICAL**: 46.8V (2.6V/cell) - Emergency stop
- **LOW**: 50.4V (2.8V/cell) - Return to base
- **NOMINAL**: 57.6V (3.2V/cell) - Normal operation
- **FULL**: 65.7V (3.65V/cell) - Fully charged

### Failsafe Conditions
- Signal loss > 1 sec → Stop
- Low battery → Return to base
- Emergency Stop → Full stop until manual reset
- Overheating (>70°C) → Power reduction
- GPS lost > 30 sec → Stop

### Code Style
- **ESP32**: Arduino framework, C++, English comments
- **ROS2**: Primarily Python for rapid development, C++ for critical paths
- **Comments**: English only
- **Tests**: Unit tests for all critical functions
- **Documentation**: Markdown format in docs/

### Critical Requirements
1. **SAFETY FIRST**: Emergency stop must always work
2. **Fail-safe**: Automatic safety measures on any fault
3. **Performance**: Control latency < 10ms, YOLO >= 24 FPS
4. **Modularity**: Hot-swappable modules, independent ROS nodes
5. **Standard protocols**: DroneCAN, ROS2, no proprietary solutions

## Dependencies Installation

### System Packages
```bash
sudo apt update
sudo apt install -y \
    ros-humble-desktop \
    ros-humble-navigation2 \
    ros-humble-teleop-twist-keyboard \
    can-utils \
    python3-pip \
    platformio
```

### Python Packages
```bash
pip3 install \
    pydronecan \
    ultralytics \
    transformers \
    python-telegram-bot \
    pyserial
```

## Testing

### Hardware-in-Loop Tests
```bash
# Run HIL test suite
cd tests/hardware_in_loop
python3 test_motor_control.py
python3 test_sensor_hub.py
```

### Integration Tests
```bash
cd tests/integration
pytest test_dronecan_bridge.py
pytest test_perception.py
```

## Deployment

### Docker (Recommended)
```bash
cd docker
docker-compose up -d
```

### Systemd Service
```bash
sudo cp scripts/startup/veter.service /etc/systemd/system/
sudo systemctl enable veter
sudo systemctl start veter
```

## Troubleshooting

### CAN Bus Issues
```bash
# Check interface
ip link show can0

# Reset CAN
sudo ip link set can0 down
sudo ip link set can0 up type can bitrate 1000000
```

### VESC Communication
```bash
# Check DroneCAN heartbeat
candump can0 | grep "341"

# Check ESC status
candump can0 | grep "1034"
```

### ROS2 Issues
```bash
# Check nodes
ros2 node list

# Check topics
ros2 topic list

# Monitor specific topic
ros2 topic echo /cmd_vel
```

## Documentation

### Core Documentation
- `docs/README.md` - Main project documentation
- `docs/CAN_SETUP.md` - CAN bus configuration guide (✅ Complete)
- `docs/INSTALLATION.md` - Software installation guide (✅ Complete)
- `docs/DEVELOPMENT_STATUS.md` - **Current project status** (✅ Updated Nov 9, 2025)
- `docs/PIXHAWK_INTEGRATION_OPTIONS.md` - **Mini Pixhawk integration options** (⏸️ Decision pending)

### Component Documentation
- `firmware/esp32_motor_controller/README.md` - Motor controller firmware (✅ Complete)
- `firmware/esp32_sensor_hub/README.md` - Sensor hub firmware (✅ Complete)
- `ros2_ws/src/veter_dronecan_bridge/README.md` - DroneCAN bridge package (✅ Complete)

### Planned Documentation
- `docs/FAILSAFE_LOGIC.md` - Failsafe system documentation
- `docs/SECURITY_MODE.md` - Security patrol mode guide
- `docs/VOICE_CONTROL.md` - Voice control setup

## Git Workflow

```bash
# Make changes
git add .
git commit -m "Description of changes"
git push

# Check status
git status

# View history
git log --oneline
```

## Contact

- GitHub: https://github.com/coldreb00t/VETER_NEXT
- Email: eugene.a.melnik@gmail.com

---

## 📊 Current Development Status (November 9, 2025)

### PHASE 1 Progress: 75% Complete

#### ✅ Completed Components
1. **Infrastructure Setup**
   - Remote SSH access via VPS tunnel (port 2223)
   - Git repository connected to GitHub
   - Complete directory structure
   - ROS2 Humble + Navigation2 installed
   - CAN interface configured (can0 @ 1 Mbps)

2. **ESP32 Motor Controller Firmware** (1,077 lines)
   - DroneCAN Node ID: 10
   - CRSF/ExpressLRS input (420k baud)
   - Differential steering mixing
   - Hardware emergency stop
   - Failsafe modes
   - Status: ✅ **Ready for hardware testing**

3. **ESP32 Sensor Hub Firmware** (1,618 lines)
   - DroneCAN Node ID: 11
   - 4x HC-SR04 ultrasonic sensors
   - BME280 environmental sensor
   - Camera servo control (pan/tilt)
   - 4-channel LED lighting
   - Collision detection
   - Status: ✅ **Ready for hardware testing**

4. **ROS2 DroneCAN Bridge** (1,131 lines)
   - DroneCAN Node ID: 20
   - Bidirectional CAN ↔ ROS2 bridge
   - 10 ROS2 topics (sensors, control, status)
   - Successfully built with colcon
   - Status: ✅ **Ready for testing**

#### ⏸️ Pending Decision
**Mini Pixhawk Integration Architecture**
- See `docs/PIXHAWK_INTEGRATION_OPTIONS.md` for detailed analysis
- 3 options documented with pros/cons
- Decision needed before proceeding

#### 🔜 Next Tasks (After Pixhawk Decision)
1. ROS2 Bringup package
2. Master launch files
3. Systemd auto-start service
4. Hardware integration testing

#### 📈 Statistics
- **Total Code:** 3,826 lines
- **Files Created:** 28
- **Git Commits:** 6
- **Documentation:** 5 major documents

### Important Notes for Claude Code
- **Current blocker:** Architectural decision on Mini Pixhawk integration role
- **Read first:** `docs/DEVELOPMENT_STATUS.md` for complete status
- **Review options:** `docs/PIXHAWK_INTEGRATION_OPTIONS.md` before proceeding
- **All firmware ready:** ESP32 code complete, awaiting hardware test
- **ROS2 bridge functional:** Builds successfully, ready for CAN testing

---

**Development Phase**: PHASE 1 - Basic Platform (75% complete)
**Priority**: Safety and fail-safe mechanisms first, features second
**Next Session:** Review Pixhawk integration options and make architectural decision
