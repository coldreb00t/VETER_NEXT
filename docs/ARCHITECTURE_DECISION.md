# Architecture Decision: Enhanced Jetson Master with GCS Integration

**Date:** November 9, 2025
**Status:** ✅ **DECIDED**
**Selected Option:** Variant C+ (Enhanced Jetson Master with Mission Planner)

---

## 🎯 Decision Summary

We will implement a **Jetson-centric architecture** where:
- **Jetson Orin Nano** is the master controller running ROS2
- **Mini Pixhawk** provides GPS/IMU/Compass + mission planning via GCS
- **Mission Planner/QGroundControl** used for waypoint planning
- **MAVROS** bridges MAVLink ↔ ROS2
- **Nav2** handles autonomous navigation
- **DroneCAN** controls motors via VESC

This combines the **flexibility of ROS2** with the **convenience of GCS tools**.

---

## 📐 System Architecture

### High-Level Data Flow

```
┌──────────────────────────────────────────────────────────────────┐
│                    CONTROL & NAVIGATION FLOW                     │
├──────────────────────────────────────────────────────────────────┤
│                                                                  │
│  Ground Control Station                                         │
│  (Mission Planner / QGroundControl)                             │
│         │                                                        │
│         │ MAVLink over WiFi/4G/Fiber                            │
│         │ - Upload waypoints                                    │
│         │ - Monitor telemetry                                   │
│         │ - Adjust parameters                                   │
│         ↓                                                        │
│  ┌──────────────────────────────────┐                           │
│  │     Mini Pixhawk (ArduRover)     │                           │
│  │  ┌────────────────────────────┐  │                           │
│  │  │  GPS (U-blox M9N)          │  │                           │
│  │  │  IMU (6-axis)              │  │                           │
│  │  │  Compass                   │  │                           │
│  │  │  Mission storage           │  │                           │
│  │  └────────────────────────────┘  │                           │
│  └──────────────┬───────────────────┘                           │
│                 │                                                │
│                 │ MAVLink (USB serial: /dev/ttyUSB0)            │
│                 │ Baudrate: 921600                              │
│                 ↓                                                │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │          Jetson Orin Nano (Master Controller)            │   │
│  │                                                           │   │
│  │  ┌─────────────────────────────────────────────────────┐ │   │
│  │  │              MAVROS (ROS2 Package)                  │ │   │
│  │  │  - Subscribe to MAVLink topics                      │ │   │
│  │  │  - Publish GPS → /mavros/global_position/global    │ │   │
│  │  │  - Publish IMU → /mavros/imu/data                  │ │   │
│  │  │  - Publish Compass → /mavros/global_position/compass│ │   │
│  │  │  - Receive waypoints → /mavros/mission/waypoints   │ │   │
│  │  │  - Send telemetry back to GCS                      │ │   │
│  │  └────────────┬────────────────────────────────────────┘ │   │
│  │               │                                           │   │
│  │               ↓                                           │   │
│  │  ┌─────────────────────────────────────────────────────┐ │   │
│  │  │         ROS2 Navigation Stack (Nav2)                │ │   │
│  │  │  ┌──────────────────────────────────────────────┐   │ │   │
│  │  │  │  Global Costmap                              │   │ │   │
│  │  │  │  - GPS waypoints from MAVROS                 │   │ │   │
│  │  │  │  - Convert lat/lon → map coordinates         │   │ │   │
│  │  │  └──────────────────────────────────────────────┘   │ │   │
│  │  │  ┌──────────────────────────────────────────────┐   │ │   │
│  │  │  │  Global Planner                              │   │ │   │
│  │  │  │  - Plan path through waypoints               │   │ │   │
│  │  │  │  - Avoid obstacles from costmap              │   │ │   │
│  │  │  └──────────────────────────────────────────────┘   │ │   │
│  │  │  ┌──────────────────────────────────────────────┐   │ │   │
│  │  │  │  Local Costmap                               │   │ │   │
│  │  │  │  - Ultrasonic sensors (4x HC-SR04)           │   │ │   │
│  │  │  │  - Camera obstacles (YOLO)                   │   │ │   │
│  │  │  └──────────────────────────────────────────────┘   │ │   │
│  │  │  ┌──────────────────────────────────────────────┐   │ │   │
│  │  │  │  Local Planner (DWB)                         │   │ │   │
│  │  │  │  - Dynamic window approach                   │   │ │   │
│  │  │  │  - Real-time obstacle avoidance              │   │ │   │
│  │  │  │  - Generate cmd_vel commands                 │   │ │   │
│  │  │  └──────────────────────────────────────────────┘   │ │   │
│  │  └────────────┬────────────────────────────────────────┘ │   │
│  │               │                                           │   │
│  │               ↓ /cmd_vel                                 │   │
│  │  ┌─────────────────────────────────────────────────────┐ │   │
│  │  │         DroneCAN Bridge (Node ID 20)                │ │   │
│  │  │  - Subscribe to /cmd_vel                            │ │   │
│  │  │  - Convert Twist → ESC commands                     │ │   │
│  │  │  - Send via DroneCAN to VESC                        │ │   │
│  │  └────────────┬────────────────────────────────────────┘ │   │
│  └───────────────┼──────────────────────────────────────────┘   │
│                  │                                              │
│                  │ DroneCAN Bus (1 Mbps)                        │
│                  ↓                                              │
│         ┌────────┴──────────┐                                   │
│         ↓                   ↓                                   │
│    ┌─────────┐         ┌─────────┐                             │
│    │ VESC    │         │ VESC    │                             │
│    │ Left    │         │ Right   │                             │
│    │ (ID 0)  │         │ (ID 1)  │                             │
│    └────┬────┘         └────┬────┘                             │
│         │                   │                                   │
│         ↓                   ↓                                   │
│    Motor Left          Motor Right                             │
│                                                                  │
└──────────────────────────────────────────────────────────────────┘
```

---

## 🎛️ Control Modes

### Mode 1: Manual RC Control
**Priority:** Highest (after E-stop)

```
ExpressLRS RX → ESP32 Motor Controller → DroneCAN → Jetson
                                                       ↓
                                                 /cmd_vel → VESC
```

**Implementation:**
- ESP32 decodes CRSF
- Publishes to DroneCAN as motor commands
- Jetson DroneCAN bridge receives and republishes to /cmd_vel
- OR: Bypass Jetson entirely (ESP32 → VESC direct)

### Mode 2: Waypoint Navigation (AUTO)
**Priority:** Medium

```
GCS → Mission Planner → Upload waypoints → Mini Pix
                                              ↓ MAVLink
                                           MAVROS
                                              ↓
                                        Nav2 Global Planner
                                              ↓
                                        Nav2 Local Planner
                                              ↓
                                          /cmd_vel → VESC
```

**Implementation:**
1. User plans mission in Mission Planner
2. Uploads waypoints to Mini Pix
3. Sets ArduRover to AUTO mode
4. Mini Pix streams waypoints via MAVLink
5. MAVROS converts to ROS2 nav goals
6. Nav2 navigates to each waypoint
7. Collision avoidance from ultrasonic sensors

### Mode 3: Teleoperation
**Priority:** Medium

```
User keyboard/joystick → ROS2 teleop → /cmd_vel → VESC
                              ↑
                        GPS/heading from Mini Pix
```

**Implementation:**
- `teleop_twist_keyboard` or `joy` node
- Direct /cmd_vel publishing
- GPS for orientation reference

### Mode 4: Autonomous Patrol
**Priority:** Low

```
Jetson AI → Path planning → Nav2 → /cmd_vel → VESC
    ↑
GPS fence, YOLO detections, security logic
```

**Implementation:**
- Custom security patrol node
- Uses Nav2 for navigation
- Vision processing with YOLO
- GPS geo-fencing

---

## 🔧 Required Components

### Software Packages

#### Already Installed ✅
- [x] ROS2 Humble Desktop
- [x] Navigation2 stack
- [x] DroneCAN Bridge (custom package)
- [x] python3-can

#### To Install 📦
```bash
# MAVROS for ROS2 Humble
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# Download GeographicLib datasets (required for GPS)
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Robot localization (sensor fusion)
sudo apt install ros-humble-robot-localization

# Additional Nav2 plugins
sudo apt install ros-humble-nav2-dwb-controller
```

### Hardware Configuration

#### Mini Pixhawk Setup
1. **Firmware:** ArduRover 4.5.x (latest stable)
2. **Connection:** USB to Jetson (`/dev/ttyUSB0` or `/dev/ttyACM0`)
3. **Baudrate:** 921600 baud
4. **Parameters:**
   ```
   SERIAL1_PROTOCOL = 2 (MAVLink2)
   SERIAL1_BAUD = 921600
   SYSID_THISMAV = 1
   ```

#### GPS Module
- **Model:** U-blox M9N
- **Connection:** To Mini Pixhawk GPS port
- **Update rate:** 10 Hz

#### Compass
- **Built-in** to GPS module or Mini Pixhawk
- **Calibration:** Required before use

---

## 📦 ROS2 Package Structure

### New Packages to Create

#### 1. `veter_bringup`
**Purpose:** Launch all nodes

```
veter_bringup/
├── launch/
│   ├── veter_full.launch.py          # Everything
│   ├── veter_minimal.launch.py       # Basic (no Nav2)
│   ├── veter_navigation.launch.py    # Nav2 only
│   └── veter_mavros.launch.py        # MAVROS only
├── config/
│   ├── mavros_config.yaml
│   ├── nav2_params.yaml
│   └── robot_localization.yaml
└── package.xml
```

#### 2. `veter_navigation`
**Purpose:** Nav2 configuration for tracked robot

```
veter_navigation/
├── config/
│   ├── costmap_common.yaml
│   ├── global_costmap.yaml
│   ├── local_costmap.yaml
│   ├── planner_server.yaml
│   └── controller_server.yaml
├── maps/
│   └── (optional pre-built maps)
└── package.xml
```

#### 3. `veter_localization`
**Purpose:** Sensor fusion (GPS + IMU + odometry)

```
veter_localization/
├── config/
│   ├── ekf_localization.yaml         # Extended Kalman Filter
│   └── navsat_transform.yaml         # GPS → map transform
└── package.xml
```

---

## 🚀 Implementation Plan

### Phase 1: MAVROS Setup (Week 1)
**Goal:** Get GPS/IMU data into ROS2

**Tasks:**
1. Install MAVROS packages
2. Configure Mini Pixhawk serial connection
3. Create MAVROS launch file
4. Test GPS data: `ros2 topic echo /mavros/global_position/global`
5. Test IMU data: `ros2 topic echo /mavros/imu/data`
6. Verify compass: `ros2 topic echo /mavros/global_position/compass_hdg`

**Success Criteria:**
- GPS coordinates visible in ROS2
- IMU data streaming at 50+ Hz
- Compass heading accurate

### Phase 2: Robot Localization (Week 1-2)
**Goal:** Fuse GPS + IMU for accurate pose

**Tasks:**
1. Install robot_localization package
2. Configure EKF (Extended Kalman Filter)
3. Set up navsat_transform (GPS → map coordinates)
4. Publish /odometry/filtered topic
5. Test position accuracy

**Success Criteria:**
- Smooth odometry output
- GPS drift filtered by IMU
- TF tree complete: map → odom → base_link

### Phase 3: Nav2 Configuration (Week 2)
**Goal:** Navigate to waypoints

**Tasks:**
1. Configure global/local costmaps
2. Set up DWB controller for tracked robot
3. Integrate ultrasonic sensors into local costmap
4. Test navigation to single GPS waypoint
5. Tune parameters (max speed, acceleration, etc.)

**Success Criteria:**
- Robot navigates to GPS coordinate
- Avoids obstacles from ultrasonics
- Smooth motion without oscillation

### Phase 4: Mission Planning Integration (Week 2-3)
**Goal:** Upload missions from Mission Planner

**Tasks:**
1. Create waypoint converter node (MAVLink → Nav2)
2. Test waypoint upload from Mission Planner
3. Implement mission execution logic
4. Add progress monitoring
5. Handle mission abort/resume

**Success Criteria:**
- Mission uploaded successfully
- Robot follows all waypoints in order
- Reaches each waypoint within tolerance
- Mission completes or can be aborted

### Phase 5: GCS Telemetry (Week 3)
**Goal:** Real-time monitoring in Mission Planner

**Tasks:**
1. Configure MAVROS to send telemetry
2. Map ROS2 topics → MAVLink messages
3. Test real-time position display in GCS
4. Add battery status telemetry
5. Add custom status messages

**Success Criteria:**
- Robot position visible on GCS map
- Battery level shown
- Mode changes reflected
- Errors/warnings displayed

---

## 📡 ROS2 Topic Map

### MAVROS Topics (Published)
```
/mavros/global_position/global        # GPS coordinates (sensor_msgs/NavSatFix)
/mavros/imu/data                      # IMU data (sensor_msgs/Imu)
/mavros/global_position/compass_hdg   # Compass heading (std_msgs/Float64)
/mavros/mission/waypoints             # Uploaded waypoints (mavros_msgs/WaypointList)
/mavros/state                         # ArduRover mode/armed status
/mavros/battery                       # Battery voltage/current
```

### MAVROS Topics (Subscribed)
```
/mavros/setpoint_position/global      # Send GPS goals to ArduRover (optional)
/mavros/mission/push                  # Upload waypoints programmatically
```

### Nav2 Topics
```
/goal_pose                            # Single navigation goal
/navigate_to_pose/_action/goal        # Action-based navigation
/cmd_vel                              # Velocity commands to motors
/local_costmap/costmap                # Obstacle map (local)
/global_costmap/costmap               # Obstacle map (global)
```

### DroneCAN Bridge Topics
```
/sensors/range/front                  # Ultrasonic sensors
/sensors/range/rear
/sensors/range/left
/sensors/range/right
/cmd_vel                              # Motor commands (subscribed)
```

---

## 🔒 Failsafe Logic

### Failsafe Priority Chain

1. **Hardware E-Stop** (HIGHEST)
   - GPIO pin on ESP32
   - Immediate motor stop
   - Cannot be overridden

2. **RC Signal Loss**
   - ESP32 detects CRSF timeout
   - Stops motors via DroneCAN
   - After 1 second of no signal

3. **Jetson Crash/Freeze**
   - Watchdog timer on ESP32
   - No /cmd_vel for 2 seconds → stop
   - Heartbeat monitoring

4. **GPS Loss** (in AUTO mode)
   - Nav2 detects GPS timeout
   - Switches to HOLD mode
   - Stops navigation, maintains position

5. **Collision Imminent**
   - Ultrasonic < 20cm
   - Nav2 emergency stop
   - Waits for clear path

6. **Low Battery**
   - Below 50.4V (18S LiFePO4)
   - Triggers Return to Home (if GPS available)
   - Or stop in place

### Mode Transitions

```
         ┌──────────┐
         │  MANUAL  │ ← RC input active
         └────┬─────┘
              │
    RC lost   │  RC available
              ↓
         ┌──────────┐
         │   AUTO   │ ← Waypoint navigation
         └────┬─────┘
              │
   Mission    │  Waypoint reached
   complete   ↓
         ┌──────────┐
         │   HOLD   │ ← Maintain position
         └────┬─────┘
              │
    E-stop    │  Emergency
              ↓
         ┌──────────┐
         │  E-STOP  │ ← Full stop
         └──────────┘
```

---

## 📊 Expected Performance

### Navigation Accuracy
- **GPS:** ±2.5m (open sky, M9N)
- **GPS + IMU fusion:** ±1m
- **Waypoint tolerance:** 2m (configurable)

### Update Rates
- **GPS:** 10 Hz
- **IMU:** 50 Hz
- **Fused odometry:** 30 Hz
- **cmd_vel:** 10 Hz
- **Sensor updates:** 10 Hz

### Latency
- **GPS to ROS2:** <50ms
- **Navigation command:** <100ms
- **Total control latency:** <150ms

---

## 🧪 Testing Checklist

### Unit Tests
- [ ] MAVROS connection established
- [ ] GPS data valid and accurate
- [ ] IMU data streaming correctly
- [ ] Compass calibrated properly
- [ ] Waypoint upload successful
- [ ] Nav2 path planning functional
- [ ] Obstacle avoidance working
- [ ] Failsafe triggers correctly

### Integration Tests
- [ ] Manual RC control works
- [ ] Switch to AUTO mode smooth
- [ ] Navigate to single waypoint
- [ ] Navigate multi-waypoint mission
- [ ] GCS telemetry displays correctly
- [ ] Emergency stop functional
- [ ] Battery monitoring accurate
- [ ] All modes transition properly

### Field Tests
- [ ] Outdoor GPS lock reliable
- [ ] Navigate 100m straight line
- [ ] Navigate square pattern
- [ ] Obstacle avoidance in field
- [ ] Mission completion rate >95%
- [ ] Failsafe behavior verified
- [ ] Long-duration test (1+ hour)

---

## 🎓 Benefits of This Architecture

### For Development
✅ Full ROS2 ecosystem access
✅ Easy debugging with ROS2 tools
✅ Gazebo simulation possible
✅ Extensive Nav2 plugins
✅ Active community support

### For Operations
✅ Mission Planner familiarity
✅ Real-time GCS monitoring
✅ Waypoint planning UI
✅ Geo-fencing built-in
✅ Mission logging/replay

### For Safety
✅ Multi-layer failsafe
✅ GPS-based geo-fence
✅ Real-time telemetry
✅ Emergency stop priority
✅ Obstacle avoidance

---

## 📚 References

- **MAVROS:** https://github.com/mavlink/mavros
- **Nav2:** https://navigation.ros.org/
- **ArduRover:** https://ardupilot.org/rover/
- **Mission Planner:** https://ardupilot.org/planner/
- **Robot Localization:** http://docs.ros.org/en/noetic/api/robot_localization/

---

## ✅ Decision Rationale

**Why this architecture?**

1. **Best of both worlds:**
   - ROS2 flexibility + Mission Planner convenience
   - Jetson computational power + Pixhawk reliability

2. **Proven components:**
   - MAVROS is mature and stable
   - Nav2 is industry-standard for ROS2
   - ArduRover has extensive field testing

3. **Scalable:**
   - Easy to add new sensors (Lidar, cameras)
   - Nav2 plugins for different behaviors
   - MAVROS supports all MAVLink devices

4. **Maintainable:**
   - Standard ROS2 packages (no custom hacks)
   - Well-documented components
   - Large community for support

5. **Future-proof:**
   - Can add vision-based navigation
   - Swarm coordination via ROS2
   - ML integration straightforward

---

**Status:** ✅ Ready to implement
**Next Step:** Install MAVROS and start Phase 1
**Timeline:** 3 weeks to full autonomous navigation capability

---

*Architecture decision made: November 9, 2025*
*Implementation starts: Next session*
