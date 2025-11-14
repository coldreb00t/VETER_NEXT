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
- **Camera**: Sony IMX477 12MP (MIPI CSI to Jetson CAM0/CAM1, dual overlay)
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
pio run -t upload --upload-port /dev/ttyACM1

# Monitor serial output (recommended - works with USB CDC)
python3 scripts/esp32_monitor.py

# Alternative: PlatformIO monitor (may not work over SSH)
pio device monitor --port /dev/ttyACM1
```

**Note:** ESP32-S3 uses USB CDC (USB Serial) which requires pyserial to open properly. Use `scripts/esp32_monitor.py` for reliable serial monitoring.

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

### MAVROS & Sensor Fusion (GPS/IMU Integration)
```bash
# Launch MAVROS for GPS/IMU from Crossflight
cd ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch veter_bringup mavros.launch.py

# Launch EKF Sensor Fusion (MAVROS + robot_localization)
ros2 launch veter_bringup sensor_fusion.launch.py

# View IMU data (10 Hz)
ros2 topic echo /mavros/data_raw

# View fused odometry (10 Hz)
ros2 topic echo /odometry/local

# Check publication rates
ros2 topic hz /mavros/data_raw
ros2 topic hz /odometry/local

# Visualize TF tree
ros2 run tf2_tools view_frames

# See full documentation
cat docs/MAVROS_GPS_IMU_INTEGRATION.md
cat docs/EKF_SENSOR_FUSION.md
```

### Camera (IMX477 Vision)
```bash
# Launch camera for ROS2 (Sony IMX477 12MP @ 1920x1080)
cd ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch veter_bringup camera.launch.py

# View camera topics
ros2 topic list | grep camera
# /camera/image_raw
# /camera/camera_info

# Check frame rate
ros2 topic hz /camera/image_raw
# Output: ~15 Hz

# View in RViz2
rviz2
# Add: Displays → By topic → /camera/image_raw → Image

# === GLOBAL CAMERA STREAMING (from anywhere in the world) ===

# MediaMTX streaming (RECOMMENDED - production-ready)
# Jetson → UDP → VPS → MediaMTX → Clients
./scripts/start_camera_mediamtx.sh

# View from ANYWHERE:
# rtsp://81.200.157.230:8555/camera

# On remote device (MacBook/PC) - VLC (easiest):
vlc rtsp://81.200.157.230:8555/camera

# Or GStreamer (auto TCP interleaved mode):
gst-launch-1.0 -v rtspsrc location=rtsp://81.200.157.230:8555/camera latency=100 ! \
  rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink

# Alternative: Direct RTSP via SSH tunnel
./scripts/start_camera_global.sh
vlc rtsp://81.200.157.230:8554/camera

# === LOCAL NETWORK STREAMING (ultra-low latency) ===

# UDP streaming (80-150ms latency, local network only)
./scripts/start_camera_udp.sh <YOUR_IP>

# View UDP stream on MacBook:
gst-launch-1.0 udpsrc port=5600 \
  caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! \
  rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! osxvideosink

# === SRT STREAMING (ultra-reliable for mobile robots) ===

# Start SRT streaming (best for 4G/5G/Starlink, handles packet loss)
./scripts/start_camera_srt.sh

# View from ANYWHERE via SRT:
# srt://81.200.157.230:9001

# On remote device (MacBook/PC):
./scripts/view_srt_stream.sh

# OR manual GStreamer:
gst-launch-1.0 srtsrc uri="srt://81.200.157.230:9001" latency=200 mode=caller ! \
  tsdemux ! h264parse ! avdec_h264 ! videoconvert ! autovideosink

# See full documentation
cat docs/CAMERA_SETUP.md
cat docs/CAMERA_MEDIAMTX_STREAMING.md  # ✅ RECOMMENDED for global access
cat docs/CAMERA_UDP_STREAMING.md
cat docs/CAMERA_GLOBAL_STREAMING.md
cat docs/CAMERA_SRT_STREAMING.md
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
- `docs/DEVELOPMENT_STATUS.md` - **Current project status** (✅ Updated Nov 14, 2025)
- `docs/PIXHAWK_INTEGRATION_OPTIONS.md` - **Mini Pixhawk integration options** (⏸️ Decision pending)
- `docs/MAVROS_GPS_IMU_INTEGRATION.md` - **GPS/IMU integration via MAVROS** (✅ Complete Nov 11, 2025)
- `docs/EKF_SENSOR_FUSION.md` - **EKF sensor fusion for pose estimation** (✅ Complete Nov 11, 2025)
- `docs/CAMERA_SETUP.md` - **IMX477 camera setup and usage** (✅ Complete Nov 11, 2025)
- `docs/CAMERA_MEDIAMTX_STREAMING.md` - **✅ PRODUCTION: Global UDP streaming via MediaMTX** (✅ Complete Nov 11, 2025)
- `docs/CAMERA_UDP_STREAMING.md` - **Ultra-low latency local streaming** (✅ Complete Nov 11, 2025)
- `docs/CAMERA_GLOBAL_STREAMING.md` - **Global access via VPS (RTSP)** (✅ Complete Nov 11, 2025)
- `docs/CAMERA_SRT_STREAMING.md` - **Experimental SRT streaming** (⏸️ On hold)
- `docs/DUAL_CAMERA_SYSTEM.md` - **✅ Dual camera system architecture** (✅ Complete Nov 13, 2025)
- `docs/FIRE_SUPPRESSION_SYSTEM.md` - **✅ Fire suppression with stepper motors** (✅ Complete Nov 13, 2025)
- `docs/JETSON_CAN_INTEGRATION.md` - **✅ Jetson → VESC CAN integration + bug fixes** (✅ Complete Nov 14, 2025)

### Component Documentation
- `firmware/esp32_motor_controller/TWAI_SUCCESS.md` - ESP32 → VESC CAN integration report (✅ Complete Nov 10, 2025)
- `firmware/esp32_motor_controller/README.md` - Motor controller firmware (✅ Complete)
- `firmware/esp32_sensor_hub/README.md` - Sensor hub firmware (✅ Complete)
- `ros2_ws/src/veter_dronecan_bridge/README.md` - DroneCAN bridge package (✅ Complete)

### Planned Documentation
- `docs/FAILSAFE_LOGIC.md` - Failsafe system documentation
- `docs/SECURITY_MODE.md` - Security patrol mode guide
- `docs/VOICE_CONTROL.md` - Voice control setup
- `ros2_ws/src/veter_camera_manager/` - Camera Mode Manager package (pending)
- `ros2_ws/src/veter_fire_suppression/` - Fire Suppression package (pending)
- `firmware/esp32_fire_controller/` - Fire controller firmware (pending)

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

## 📊 Current Development Status (November 13, 2025)

### PHASE 1: 100% Complete | PHASE 3: 40-50% Complete

#### ✅ Software Development: COMPLETE
1. **Infrastructure Setup**
   - Remote SSH access via VPS tunnel (port 2223)
   - Git repository connected to GitHub
   - Complete directory structure
   - ROS2 Humble + Navigation2 installed
   - CAN interface configured (can0 @ 1 Mbps)

2. **ESP32 Motor Controller Firmware** (13,901 bytes)
   - DroneCAN Node ID: 10
   - **ESP32 TWAI native CAN controller** (GPIO4/5 @ 1 Mbps)
   - CRSF/ExpressLRS input (420k baud)
   - Differential steering mixing
   - Hardware emergency stop
   - Failsafe modes
   - Status: ✅ **HARDWARE TESTED - WORKING WITH VESC** (Nov 10, 2025)

3. **ESP32 Sensor Hub Firmware** (1,618 lines)
   - DroneCAN Node ID: 11
   - 4x HC-SR04 ultrasonic sensors
   - BME280 environmental sensor
   - Camera servo control (pan/tilt)
   - 4-channel LED lighting
   - Collision detection
   - Status: ✅ **Complete, ready for deployment**

4. **ROS2 DroneCAN Bridge** (1,131 lines)
   - DroneCAN Node ID: 20
   - Bidirectional CAN ↔ ROS2 bridge
   - 21 ROS2 topics (sensors, control, status)
   - Successfully built with colcon
   - Status: ✅ **Tested, launches successfully**

5. **ROS2 Channel Manager** (900 lines)
   - 6-channel communication failover
   - Priority-based channel selection
   - Health monitoring with hysteresis
   - 4 preset configurations
   - Status: ✅ **Tested, working correctly**

6. **ROS2 Bringup Package** (650 lines)
   - 5 launch files (minimal, full, teleop, mavros, sensor_fusion)
   - Robot configuration parameters
   - MAVROS configuration
   - EKF sensor fusion configuration
   - Systemd auto-start service
   - Status: ✅ **Tested, system launches without errors**

7. **MAVROS GPS/IMU Integration** (November 11, 2025)
   - Connected to Radiolink Crossflight (ArduRover)
   - MAVLink 2.0 protocol via USB (/dev/ttyACM0:115200)
   - IMU data publishing at 10 Hz (/mavros/data_raw)
   - GPS data publishing at 10 Hz (/mavros/global_position/global)
   - U-blox M9N GPS module support
   - ROS2 Humble compatible configuration
   - Status: ✅ **Tested and operational** (see docs/MAVROS_GPS_IMU_INTEGRATION.md)

8. **EKF Sensor Fusion (robot_localization)** (November 11, 2025)
   - Extended Kalman Filter for pose estimation
   - Fuses IMU data from MAVROS @ 10 Hz
   - GPS fusion prepared (disabled indoors - no GPS fix)
   - Publishes fused odometry on /odometry/local @ 10 Hz
   - TF tree: map → odom → base_link → imu_link/gps_link
   - Static transform publishers for sensor frames
   - navsat_transform for GPS→Odometry conversion (outdoor use)
   - Status: ✅ **IMU-only fusion working, GPS ready for outdoor testing**

9. **Camera Integration (Sony IMX477)** (November 11, 2025)
   - Sony IMX477 12MP camera on MIPI CSI CAM0 port
   - ROS2 gscam integration for computer vision pipeline
   - Ultra-low latency UDP/RTP streaming (80-150ms glass-to-glass)
   - H.264 encoding with x264enc (software encoder)
   - Publishing to /camera/image_raw @ ~15 Hz
   - RTSP streaming option (200-300ms latency, more reliable)
   - Scripts: start_camera_udp.sh, enable_max_performance.sh
   - Status: ✅ **Tested and operational** (see docs/CAMERA_SETUP.md, docs/CAMERA_UDP_STREAMING.md)

#### ✅ Software Testing Complete
**Test Date:** November 11, 2025 (updated)
- ✅ ROS2 package compilation (all 3 packages)
- ✅ System launch verification (`veter_minimal.launch.py`)
- ✅ Topic creation validated (21 topics)
- ✅ Node communication framework (2 nodes: dronecan_bridge, channel_manager)
- ✅ ESP32 firmware execution test (via USB serial)
- ✅ CAN interface operational (can0 @ 1 Mbps UP)
- ✅ Configuration file loading
- ✅ MAVROS GPS/IMU integration (`mavros.launch.py`)
- ✅ IMU data streaming at 10 Hz (/mavros/data_raw)
- ✅ GPS topic available at 10 Hz (/mavros/global_position/global)
- ✅ EKF Sensor Fusion (`sensor_fusion.launch.py`)
- ✅ Fused odometry publishing at 10 Hz (/odometry/local)
- ✅ TF tree operational (map → odom → base_link)

**System successfully launches and operates with sensor fusion!**

#### 🟡 Phase 3 Hardware Integration: 40-50% COMPLETE

**✅ COMPLETED (November 10, 2025):**
- ✅ ESP32 Motor Controller flashed to production hardware
- ✅ **Physical CAN bus communication WORKING** (ESP32 TWAI ↔ VESC)
- ✅ **67,000+ CAN messages exchanged** in 5-minute test
- ✅ ONE VESC 75200 connected and tested in UAVCAN mode
- ✅ **RC control chain verified**: ExpressLRS → ESP32 → VESC
- ✅ **VESC LED responds to commands** - physical confirmation
- ✅ **DroneCAN protocol compliant**: ESC RawCommand, NodeStatus, ESC Status
- ✅ Failsafe logic active and working
- 📄 **Full report**: `firmware/esp32_motor_controller/TWAI_SUCCESS.md`

**⏸️ NOT YET DONE:**
- Second VESC integration (only 1 of 2 tested)
- Actual motor connection (VESC works, motors not connected)
- 120Ω CAN termination resistor (missing at ESP32 end)
- ESP32 Sensor Hub physical wiring
- Jetson CAN interface physical connection
- Wire sensors (4× HC-SR04, BME280)
- Wire servos, LEDs, E-Stop button

#### ✅ Mini Pixhawk (Crossflight) Integration: COMPLETE
**Role:** GPS/IMU provider via MAVROS + Mission planning interface (NOT motor controller)
- ✅ **GPS/IMU data integrated** via MAVROS (November 11, 2025)
- Motors always controlled via: Jetson → ESP32 → VESC (DroneCAN)
- Manual mode: ExpressLRS → ESP32 (direct hardware path)
- Auto mode: Jetson Nav2 uses GPS/IMU from MAVROS, sends cmd_vel
- Mission planning: Via ArduRover Ground Control Station (QGroundControl) - optional

#### 📈 Statistics
- **Total Code:** 5,376 lines (firmware + ROS2 packages) + bug fixes in dronecan_bridge
- **Configuration:** ~400 lines (mavros_config.yaml, ekf.yaml, launch files)
- **Files Created:** 58 (added JETSON_CAN_INTEGRATION.md)
- **Git Commits:** 6 (will increase with next commit including bug fixes)
- **Documentation:** 27 total documents (11 major design docs)
- **Hardware Milestones:** 2 (ESP32→VESC Nov 10, Jetson→VESC Nov 14)

### Important Notes for Claude Code
- **PHASE 1 SOFTWARE:** ✅ 100% COMPLETE - All code written, built, and tested
- **PHASE 3 HARDWARE:** 🟡 **60-70% COMPLETE** - ESP32+VESC+Jetson integration WORKING!
- **ESP32 TWAI CAN:** ✅ TESTED - 67,000+ messages with VESC (Nov 10, 2025)
- **JETSON CAN:** ✅ **NEW** - 126,000+ messages with VESC, 3 bugs fixed (Nov 14, 2025)
- **ROS2 → VESC:** ✅ WORKING - Motor control via /cmd_vel @ 100 Hz
- **GPS/IMU INTEGRATED:** Crossflight connected via MAVROS (November 11, 2025)
- **EKF SENSOR FUSION:** ✅ OPERATIONAL IMU-only @ 10 Hz (November 11, 2025)
- **Camera:** ✅ Sony IMX477 streaming @ 15 Hz (November 11-12, 2025)
- **Web Interface:** ✅ WORKING - FPV stream + telemetry (November 12, 2025)
- **Read first:** `docs/JETSON_CAN_INTEGRATION.md` for latest hardware milestone (Nov 14, 2025)
- **Also read:** `firmware/esp32_motor_controller/TWAI_SUCCESS.md` for ESP32 testing report
- **Also read:** `docs/DEVELOPMENT_STATUS.md` for complete status (updated Nov 14, 2025)
- **Also read:** `docs/EKF_SENSOR_FUSION.md` for sensor fusion details
- **Also read:** `docs/MAVROS_GPS_IMU_INTEGRATION.md` for GPS/IMU setup

---

**Development Phase**: PHASE 3 - Hardware Integration (60-70% complete)
**Software Status**: ✅ 100% COMPLETE and TESTED
**Hardware Status**: 🟡 **60-70% COMPLETE** - ESP32+VESC+Jetson CAN working! (+20% Nov 14)
**Jetson CAN**: ✅ **NEW** - J17 soldered, ROS2→VESC working, 3 bugs fixed
**GPS/IMU Status**: ✅ INTEGRATED via MAVROS
**EKF Sensor Fusion**: ✅ OPERATIONAL (IMU-only, GPS ready)
**Camera**: ✅ OPERATIONAL (Sony IMX477 @ 15 Hz)
**Web UI**: ✅ WORKING (FPV + telemetry)
**Priority**: Connect ESP32 to CAN bus → test tri-node (Jetson ↔ ESP32 ↔ VESC)
**Next Session:** Connect ESP32 to CAN, test 3-way communication, OR implement VESC telemetry decoder
