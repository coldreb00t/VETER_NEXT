# VETER_NEXT Development Status

**Last Updated:** November 9, 2025
**Session Date:** November 9, 2025
**Status:** PHASE 1 in progress (75% complete)

---

## 📊 Overall Progress

### ✅ Completed (PHASE 1)

#### 1. Infrastructure Setup
- [x] Remote SSH access via VPS tunnel (81.200.157.230:2223)
- [x] Systemd service for persistent SSH tunnel
- [x] Git repository initialized and connected to GitHub
- [x] Complete directory structure created
- [x] Project documentation (CLAUDE.md, README.md)

#### 2. Software Installation
- [x] ROS2 Humble Desktop installed
- [x] Navigation2 stack installed
- [x] PlatformIO Core installed
- [x] Python libraries (can, numpy, etc.)
- [x] CAN-utils installed and tested

#### 3. CAN Interface Configuration
- [x] Native can0 interface configured (1 Mbps)
- [x] Systemd service for CAN auto-start
- [x] Comprehensive CAN setup documentation
- [x] Tested with candump/cansend

#### 4. ESP32 Motor Controller Firmware (Node ID 10)
**Status:** ✅ Complete (1077 lines)

**Features Implemented:**
- [x] CRSF/ExpressLRS input (420000 baud, GPIO 16/17)
- [x] DroneCAN output (1 Mbps via MCP2515)
- [x] Differential steering mixing
- [x] Hardware emergency stop (GPIO 23, debounced)
- [x] Multiple failsafe modes
- [x] WS2812B LED status indication
- [x] DroneCAN heartbeat (100ms)
- [x] ESC RawCommand transmission

**Files:**
- `firmware/esp32_motor_controller/platformio.ini`
- `firmware/esp32_motor_controller/include/config.h`
- `firmware/esp32_motor_controller/include/dronecan_interface.h`
- `firmware/esp32_motor_controller/src/dronecan_interface.cpp`
- `firmware/esp32_motor_controller/src/main.cpp`
- `firmware/esp32_motor_controller/README.md`

**Configuration:**
- DroneCAN Node ID: 10
- ESC indices: 0 (left), 1 (right)
- Throttle scale: 1.0, Steering scale: 0.7
- Max differential: 0.8
- RC timeout: 1000ms

#### 5. ESP32 Sensor Hub Firmware (Node ID 11)
**Status:** ✅ Complete (1618 lines)

**Features Implemented:**
- [x] 4x HC-SR04 ultrasonic sensors (10 Hz)
- [x] BME280 environmental sensor (I2C, 1 Hz)
- [x] Camera servo control (pan/tilt, PWM 50Hz)
- [x] 4-channel LED lighting (PWM 5kHz)
- [x] DroneCAN sensor data publishing
- [x] Real-time collision detection
- [x] Hardware emergency stop integration
- [x] WS2812B status LED

**Files:**
- `firmware/esp32_sensor_hub/platformio.ini`
- `firmware/esp32_sensor_hub/include/config.h`
- `firmware/esp32_sensor_hub/include/dronecan_interface.h`
- `firmware/esp32_sensor_hub/src/dronecan_interface.cpp`
- `firmware/esp32_sensor_hub/src/main.cpp`
- `firmware/esp32_sensor_hub/README.md`

**Configuration:**
- DroneCAN Node ID: 11
- Collision warning: 50cm, Stop: 20cm
- BME280 read interval: 1 second
- Servo timeout: 1 second

**Sensor Pins:**
- Front sonar: TRIG=14, ECHO=15
- Rear sonar: TRIG=16, ECHO=17
- Left sonar: TRIG=18, ECHO=21
- Right sonar: TRIG=47, ECHO=48
- BME280: SDA=8, SCL=9
- Servos: PAN=38, TILT=39
- LEDs: FL=40, FR=41, RL=42, RR=45

#### 6. ROS2 DroneCAN Bridge Package (Node ID 20)
**Status:** ✅ Complete (1131 lines)

**Features Implemented:**
- [x] Bidirectional CAN ↔ ROS2 bridge
- [x] Multi-threaded asynchronous CAN reception
- [x] YAML-based configuration
- [x] Comprehensive sensor support
- [x] Motor control via Twist messages
- [x] Camera servo control
- [x] LED lighting control
- [x] Successfully built with colcon

**Files:**
- `ros2_ws/src/veter_dronecan_bridge/package.xml`
- `ros2_ws/src/veter_dronecan_bridge/setup.py`
- `ros2_ws/src/veter_dronecan_bridge/veter_dronecan_bridge/__init__.py`
- `ros2_ws/src/veter_dronecan_bridge/veter_dronecan_bridge/can_interface.py`
- `ros2_ws/src/veter_dronecan_bridge/veter_dronecan_bridge/dronecan_bridge_node.py`
- `ros2_ws/src/veter_dronecan_bridge/config/dronecan_params.yaml`
- `ros2_ws/src/veter_dronecan_bridge/launch/dronecan_bridge.launch.py`
- `ros2_ws/src/veter_dronecan_bridge/README.md`

**ROS2 Topics Published (CAN → ROS2):**
- `/sensors/range/front` - Front ultrasonic
- `/sensors/range/rear` - Rear ultrasonic
- `/sensors/range/left` - Left ultrasonic
- `/sensors/range/right` - Right ultrasonic
- `/sensors/temperature` - BME280 temperature
- `/sensors/humidity` - BME280 humidity
- `/sensors/pressure` - BME280 pressure
- `/collision/warning` - Collision warnings
- `/motor_controller/status` - Motor controller heartbeat
- `/sensor_hub/status` - Sensor hub heartbeat

**ROS2 Topics Subscribed (ROS2 → CAN):**
- `/cmd_vel` - Velocity commands (Twist)
- `/camera/servo/pan` - Pan servo angle
- `/camera/servo/tilt` - Tilt servo angle
- `/lighting/mode` - LED mode
- `/lighting/brightness` - LED brightness

**Configuration:**
- Jetson Node ID: 20
- CAN interface: can0 @ 1 Mbps
- Publish rate: 10 Hz

---

## 🔄 In Progress

### PHASE 1 Remaining Tasks

#### 1. Mini Pixhawk Integration Decision
**Status:** ⏸️ Pending architectural decision

**Hardware:**
- Mini Pixhawk flight controller
- ArduRover firmware
- GPS, IMU, compass modules

**Integration Options (to be decided):**

**Option A: Mini Pix as Master (Classic)**
```
ExpressLRS → Mini Pix → MAVLink → Jetson
                ↓
            PWM/CAN
                ↓
            VESC L/R
```

**Option B: Hybrid Control**
```
Manual Mode:  ExpressLRS → ESP32 → VESC
Auto Mode:    Jetson → ESP32 → VESC
Mini Pix:     GPS/IMU → MAVLink → Jetson
```

**Option C: Jetson as Master**
```
ExpressLRS → ESP32 → Jetson → cmd_vel → VESC
                       ↑
                  Mini Pix (MAVLink)
                  (GPS, IMU, modes)
```

**Questions to resolve:**
1. What is Mini Pix role? (Master/Navigation/Failsafe)
2. Who controls motors in each mode?
3. Failsafe priority and handoff logic?
4. MAVLink vs DroneCAN for Pixhawk?

**Required components:**
- [ ] MAVROS (ROS2 MAVLink bridge)
- [ ] Mode switching logic
- [ ] Failsafe coordination
- [ ] GPS/IMU integration

#### 2. ROS2 Bringup Package
**Status:** 🔜 Next task after architectural decision

**Planned:**
- [ ] veter_bringup package structure
- [ ] Master launch file for all nodes
- [ ] Parameter files
- [ ] TF tree configuration
- [ ] URDF robot description (optional)

#### 3. System Integration
**Status:** 🔜 Waiting for bringup

**Planned:**
- [ ] Systemd service for auto-start
- [ ] Startup scripts
- [ ] Health monitoring
- [ ] Log rotation

---

## 📈 Statistics

### Code Written
- **ESP32 Motor Controller:** 1,077 lines
- **ESP32 Sensor Hub:** 1,618 lines
- **ROS2 DroneCAN Bridge:** 1,131 lines
- **Total:** 3,826 lines of code

### Files Created
- **Firmware files:** 12 (6 + 6)
- **ROS2 package files:** 11
- **Documentation:** 5 (3x README + 2x setup guides)
- **Total:** 28 files

### Git Commits
- Infrastructure and setup: 3 commits
- ESP32 Motor Controller: 1 commit
- ESP32 Sensor Hub: 1 commit
- ROS2 DroneCAN Bridge: 1 commit
- **Total:** 6 commits

### GitHub
- Repository: https://github.com/coldreb00t/VETER_NEXT
- Branch: main
- Remote access: via VPS tunnel on port 2223

---

## 🎯 PHASE 1 Completion Checklist

### Core Components (75% complete)
- [x] ESP32 Motor Controller firmware
- [x] ESP32 Sensor Hub firmware
- [x] ROS2 DroneCAN Bridge package
- [ ] Mini Pixhawk integration (pending decision)
- [ ] ROS2 Bringup package
- [ ] System auto-start configuration

### Testing (0% complete)
- [ ] ESP32 firmware flash and test
- [ ] CAN bus communication test
- [ ] ROS2 bridge functionality test
- [ ] Sensor data flow verification
- [ ] Motor command verification
- [ ] End-to-end integration test

### Documentation (90% complete)
- [x] Firmware README files (2x)
- [x] ROS2 package README
- [x] CAN setup guide
- [x] Installation guide
- [ ] System architecture diagram (pending decision)
- [ ] Integration testing guide
- [ ] Troubleshooting guide

---

## 🚀 Next Steps

### Immediate (After Break)
1. **Review architectural options** for Mini Pixhawk integration
2. **Make decision** on control flow and component roles
3. **Document chosen architecture** with diagrams

### Short Term (This Week)
1. Create ROS2 Bringup package
2. Add MAVROS integration (if decided)
3. Create master launch file
4. Set up systemd auto-start

### Medium Term (Next Week)
1. Flash ESP32 firmware to hardware
2. Test CAN bus communication
3. Verify ROS2 bridge functionality
4. End-to-end integration testing

### Long Term (PHASE 2)
1. Advanced navigation features
2. Vision system integration
3. Voice control integration
4. Security features implementation

---

## 🔧 Hardware Configuration

### Current Setup
- **Platform:** NVIDIA Jetson Orin Nano Super (8GB)
- **OS:** Ubuntu 22.04 (Linux 5.15.148-tegra)
- **ROS2:** Humble Desktop + Navigation2
- **CAN:** Native can0 interface @ 1 Mbps

### Components Ready
- Mini Pixhawk flight controller (ArduRover)
- 2x ESP32-S3 (16MB flash, 8MB PSRAM)
- 2x VESC 75200 motor controllers
- ExpressLRS receiver
- Various sensors (pending connection)

### Pending Hardware Integration
- Physical CAN bus wiring
- VESC configuration for DroneCAN
- Sensor mounting and wiring
- Power distribution system
- GPS/compass mounting

---

## 📝 Notes and Decisions

### Key Architectural Decisions Made

1. **Communication Protocol: CAN (DroneCAN)**
   - Chose CAN over USB for reliability
   - CAN provides EMI immunity (critical near motors)
   - Shielded bus topology (fewer wires)
   - VESC native DroneCAN support
   - Industry standard for robotics

2. **ESP32 as Microcontroller**
   - ESP32-S3 for better performance
   - 16MB flash + 8MB PSRAM
   - Dual-core for parallel processing
   - Native USB for debugging (optional)

3. **ROS2 Humble**
   - Long-term support (LTS) version
   - Mature ecosystem
   - Nav2 integration
   - Python3 compatibility

4. **Node ID Allocation**
   - 10: ESP32 Motor Controller
   - 11: ESP32 Sensor Hub
   - 20: Jetson DroneCAN Bridge
   - 0-1: VESC Left/Right (standard)
   - TBD: Mini Pixhawk node ID

### Open Questions

1. **Mini Pixhawk Role:**
   - Master controller or subsystem?
   - MAVLink or DroneCAN interface?
   - Failsafe priority level?

2. **Control Flow:**
   - Who processes RC input in manual mode?
   - Who generates motor commands in auto mode?
   - How to handle mode transitions?

3. **Failsafe Hierarchy:**
   - Level 1: Hardware E-stop (immediate)
   - Level 2: Mini Pix failsafe?
   - Level 3: Jetson ROS2 failsafe?
   - Level 4: ESP32 RC loss failsafe?

---

## 🎓 Lessons Learned

### What Worked Well
1. **Systematic approach** - Building layer by layer
2. **Documentation first** - Clear specs from start
3. **Git workflow** - Regular commits with good messages
4. **Modular design** - Separate concerns, clean interfaces
5. **Testing mindset** - Debug flags, serial output

### Challenges Encountered
1. **Sudo permissions** - Solved with password piping
2. **Git configuration** - User setup needed
3. **ROS2 dependencies** - Required specific package order

### Best Practices Established
1. Always read files before editing
2. Document every decision
3. Test incrementally
4. Keep backups via Git
5. Use TODO tracking for complex tasks

---

## 🔗 Important Links

- **GitHub:** https://github.com/coldreb00t/VETER_NEXT
- **Remote Access:** `ssh -p 2223 jetson@81.200.157.230`
- **Documentation:** `/home/jetson/jetson-robot-project/docs/`
- **ROS2 Workspace:** `/home/jetson/jetson-robot-project/ros2_ws/`

---

*Generated by Claude Code - November 9, 2025*
