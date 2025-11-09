# Mini Pixhawk Integration Architecture Options

**Document Version:** 1.0
**Date:** November 9, 2025
**Status:** Pending Decision

---

## 📋 Executive Summary

This document outlines three architectural approaches for integrating the Mini Pixhawk flight controller running ArduRover firmware into the VETER_NEXT robotic platform. Each option has different trade-offs in terms of complexity, reliability, and feature availability.

**Hardware Context:**
- **Mini Pixhawk** - Flight controller with ArduRover firmware
- **GPS/IMU/Compass** - Navigation sensors connected to Pixhawk
- **Jetson Orin Nano** - Main compute running ROS2 Humble
- **2x ESP32-S3** - Motor control + sensor hub (DroneCAN)
- **2x VESC 75200** - Motor controllers (DroneCAN capable)
- **ExpressLRS RX** - RC receiver for manual control

---

## 🎯 Option A: Mini Pixhawk as Master Controller (Classic)

### Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                     CONTROL HIERARCHY                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────┐                                          │
│  │ ExpressLRS   │                                          │
│  │   Receiver   │                                          │
│  └──────┬───────┘                                          │
│         │ CRSF (420k baud)                                 │
│         ↓                                                   │
│  ┌──────────────────────────────────┐                      │
│  │     Mini Pixhawk (ArduRover)     │ ← Master             │
│  │  - RC input processing           │                      │
│  │  - GPS/IMU/Compass               │                      │
│  │  - Mode management               │                      │
│  │  - Failsafe coordination         │                      │
│  └─────┬──────────────────┬─────────┘                      │
│        │                  │                                 │
│        │ PWM/CAN         │ MAVLink (USB/UART)             │
│        ↓                  ↓                                 │
│  ┌──────────┐      ┌──────────────┐                        │
│  │ VESC L/R │      │ Jetson Orin  │ ← Companion Computer   │
│  │          │      │   (ROS2)     │                        │
│  │ Motors   │      │ - MAVROS     │                        │
│  └──────────┘      │ - Navigation │                        │
│                    │ - Vision     │                        │
│                    │ - AI/ML      │                        │
│                    └──────────────┘                        │
└─────────────────────────────────────────────────────────────┘
```

### Data Flow

**Manual Mode:**
```
RC → Pixhawk → PWM/CAN → VESC → Motors
                    ↓
              MAVLink → Jetson (telemetry only)
```

**Auto Mode (GUIDED):**
```
Jetson → MAVLink → Pixhawk → PWM/CAN → VESC → Motors
  ↑                   ↓
GPS/IMU          Telemetry
```

### Pros ✅

1. **Standard ArduRover workflow** - Well documented, proven
2. **Mature failsafe system** - ArduRover handles all safety
3. **GPS/IMU integration** - Native ArduRover support
4. **Mode switching** - MANUAL/AUTO/GUIDED modes built-in
5. **RC failsafe** - Automatic RTL (Return to Launch)
6. **Extensive tuning tools** - Mission Planner, QGroundControl
7. **Battery monitoring** - Voltage/current sensing
8. **Geo-fencing** - Built-in safety boundaries

### Cons ❌

1. **Limited Jetson control** - Must go through MAVLink
2. **Latency** - Extra hop: Jetson → MAVLink → Pixhawk → Motors
3. **ESP32 underutilized** - Just RC receiver passthrough
4. **Complex wiring** - Pixhawk needs many connections
5. **PWM limitations** - Lower resolution than DroneCAN
6. **Single point of failure** - Pixhawk crash = dead robot

### Required Components

- [x] Mini Pixhawk with ArduRover firmware
- [x] GPS module (connected to Pixhawk)
- [x] IMU/Compass (built into Pixhawk)
- [ ] MAVROS (ROS2 package)
- [ ] MAVLink over USB or UART
- [ ] PWM to VESC wiring OR DroneCAN configuration
- [ ] Power module for battery monitoring
- [ ] RC receiver connection to Pixhawk

### Implementation Steps

1. Install MAVROS on Jetson
2. Configure ArduRover parameters
3. Connect Pixhawk to VESC (PWM or CAN)
4. Set up MAVLink communication (USB preferred)
5. Configure RC input (CRSF or PPM)
6. Tune ArduRover PID loops
7. Create ROS2 nodes for waypoint navigation
8. Test failsafe scenarios

### Estimated Effort
⏱️ **High** - 2-3 weeks for full integration and tuning

---

## 🎯 Option B: Hybrid Control (Recommended)

### Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                  HYBRID CONTROL FLOW                        │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────┐                                          │
│  │ ExpressLRS   │                                          │
│  │   Receiver   │                                          │
│  └──────┬───────┘                                          │
│         │ CRSF (420k)                                      │
│         ↓                                                   │
│  ┌──────────────────────────┐                              │
│  │  ESP32 Motor Controller  │ ← RC Processing              │
│  │   (Node ID 10)           │                              │
│  │  - CRSF decode           │                              │
│  │  - Mode switch detection │                              │
│  │  - Manual mixing         │                              │
│  └────┬────────────┬────────┘                              │
│       │            │                                        │
│   Manual Mode   Auto Mode                                  │
│       │            │                                        │
│       ↓            ↓                                        │
│  ┌─────────────────────────────┐                           │
│  │   DroneCAN Bus (1 Mbps)     │                           │
│  └┬────────┬─────────┬─────────┘                           │
│   │        │         │                                      │
│   ↓        ↓         ↓                                      │
│ VESC L  VESC R   ┌──────────────┐                          │
│                  │ Jetson Orin  │ ← High-Level Control     │
│                  │  (ROS2)      │                          │
│                  │ - Navigation │                          │
│                  │ - cmd_vel    │                          │
│                  └──────┬───────┘                          │
│                         ↑                                   │
│                         │ MAVLink (telemetry)              │
│                         │                                   │
│                  ┌──────┴───────┐                          │
│                  │ Mini Pixhawk │ ← Navigation Module      │
│                  │ (ArduRover)  │                          │
│                  │ - GPS        │                          │
│                  │ - IMU        │                          │
│                  │ - Compass    │                          │
│                  │ - Heading    │                          │
│                  └──────────────┘                          │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Control Modes

**Mode 1: MANUAL (RC Direct)**
```
RC → ESP32 → DroneCAN → VESC
     (mixing)
```

**Mode 2: AUTO (Jetson Control)**
```
Jetson → cmd_vel → DroneCAN Bridge → DroneCAN → VESC
   ↑
   │ GPS/IMU/Heading
   │
Mini Pix (MAVLink)
```

**Mode 3: FAILSAFE**
```
Priority chain:
1. Hardware E-stop (immediate motor stop)
2. RC signal loss → ESP32 → stop motors
3. Jetson crash → timeout → ESP32 → stop motors
4. Mini Pix failsafe → MAVLink → Jetson → stop
```

### Pros ✅

1. **Low latency** - Direct paths for both modes
2. **Best of both worlds** - Manual AND autonomous
3. **Jetson has full control** in auto mode
4. **ESP32 fully utilized** - Intelligence at edge
5. **Redundant failsafe** - Multiple layers
6. **Flexible** - Easy to add new modes
7. **DroneCAN benefits** - High-speed, reliable bus
8. **GPS/IMU available** - Via MAVLink to Jetson

### Cons ❌

1. **More complex** - Custom mode switching logic
2. **Testing required** - Mode transitions must be smooth
3. **Custom code** - Not standard ArduRover workflow
4. **Coordination needed** - Between ESP32/Jetson/Pixhawk

### Required Components

- [x] Mini Pixhawk (GPS/IMU provider only)
- [x] ESP32 Motor Controller (enhanced with mode switching)
- [x] ROS2 DroneCAN Bridge (already implemented)
- [ ] MAVROS (for GPS/IMU data)
- [ ] Mode switching logic in ESP32
- [ ] Failsafe coordination protocol
- [ ] State machine implementation

### Implementation Steps

1. **ESP32 enhancement:**
   - Add mode switch detection (RC channel)
   - Implement mode state machine
   - Add timeout watchdog for Jetson commands

2. **Pixhawk setup:**
   - Configure as GPS/IMU provider
   - Disable motor outputs (or use for backup)
   - Stream MAVLink to Jetson (USB)

3. **ROS2 integration:**
   - Install MAVROS for GPS/IMU
   - Create mode manager node
   - Implement failsafe coordinator

4. **Testing:**
   - Test mode transitions
   - Verify failsafe triggers
   - Tune timeout values

### Estimated Effort
⏱️ **Medium** - 1-2 weeks for implementation and testing

---

## 🎯 Option C: Jetson as Master

### Architecture Diagram

```
┌─────────────────────────────────────────────────────────────┐
│                  JETSON-CENTRIC CONTROL                     │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌──────────────┐                                          │
│  │ ExpressLRS   │                                          │
│  │   Receiver   │                                          │
│  └──────┬───────┘                                          │
│         │ CRSF                                             │
│         ↓                                                   │
│  ┌──────────────────┐                                      │
│  │  ESP32 Motor     │ ← RC Interface                       │
│  │  Controller      │                                      │
│  └────────┬─────────┘                                      │
│           │ DroneCAN                                       │
│           ↓                                                 │
│  ┌─────────────────────────────────┐                       │
│  │      Jetson Orin (Master)       │                       │
│  │  ┌───────────────────────────┐  │                       │
│  │  │   ROS2 Control Stack      │  │                       │
│  │  │  - RC input handler       │  │                       │
│  │  │  - Mode manager           │  │                       │
│  │  │  - cmd_vel generator      │  │                       │
│  │  │  - Navigation (Nav2)      │  │                       │
│  │  │  - Collision avoidance    │  │                       │
│  │  └───────────────────────────┘  │                       │
│  │           ↑                      │                       │
│  │           │ MAVLink              │                       │
│  │  ┌────────┴──────────┐          │                       │
│  │  │  Mini Pix         │          │                       │
│  │  │  - GPS            │          │                       │
│  │  │  - IMU            │          │                       │
│  │  │  - Compass        │          │                       │
│  │  └───────────────────┘          │                       │
│  └──────────────┬──────────────────┘                       │
│                 │ DroneCAN                                  │
│                 ↓                                           │
│        ┌────────┴────────┐                                 │
│        ↓                 ↓                                  │
│    ┌────────┐      ┌────────┐                              │
│    │ VESC L │      │ VESC R │                              │
│    └────────┘      └────────┘                              │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

### Data Flow

**All Modes through Jetson:**
```
RC → ESP32 → DroneCAN → Jetson → Processing → cmd_vel → VESC
                                      ↑
                                      │
                              Mini Pix (GPS/IMU)
```

### Pros ✅

1. **Unified control** - Everything goes through ROS2
2. **Maximum flexibility** - Full software control
3. **Easy debugging** - All logic in one place
4. **ROS2 ecosystem** - Full Nav2 stack available
5. **Data fusion** - Easy to combine all sensors
6. **Logging** - Complete system state in rosbag
7. **Simulation** - Easy to test in Gazebo

### Cons ❌

1. **Single point of failure** - Jetson crash = dead robot
2. **Higher latency** - All through Linux/ROS2
3. **Computational load** - Jetson does everything
4. **Less robust** - No hardware-level failsafe
5. **Power consumption** - Jetson always running full
6. **Complex startup** - ROS2 must boot before operation

### Required Components

- [x] Jetson Orin Nano (main compute)
- [x] ROS2 DroneCAN Bridge
- [x] ESP32 as simple CAN gateway
- [ ] MAVROS for Pixhawk
- [ ] ROS2 teleop node for RC input
- [ ] Emergency stop handler
- [ ] Watchdog system

### Implementation Steps

1. Create ROS2 teleop node
2. Implement mode manager
3. Set up MAVROS for GPS/IMU
4. Create failsafe watchdog
5. Implement emergency stop handler
6. Set up rosbag logging
7. Configure Nav2 stack

### Estimated Effort
⏱️ **Medium-High** - 1.5-2 weeks

---

## 📊 Comparison Matrix

| Feature | Option A<br/>Pixhawk Master | Option B<br/>Hybrid | Option C<br/>Jetson Master |
|---------|:--------------------------:|:------------------:|:-------------------------:|
| **Latency** | Medium | Low | Medium-High |
| **Reliability** | High | High | Medium |
| **Flexibility** | Low | High | Very High |
| **Complexity** | Medium | High | Medium |
| **ArduRover Features** | Full | Partial | Minimal |
| **ROS2 Integration** | Limited | Good | Excellent |
| **Failsafe Robustness** | Excellent | Good | Fair |
| **Development Time** | 2-3 weeks | 1-2 weeks | 1.5-2 weeks |
| **Debugging Ease** | Medium | Medium | Easy |
| **Power Efficiency** | Good | Good | Fair |
| **Scalability** | Low | High | Very High |
| **Hardware Utilization** | Medium | High | Medium |

---

## 🎓 Recommendations

### For Development/Prototyping Phase
**Recommended: Option C (Jetson Master)**

**Rationale:**
- Fastest to implement with existing codebase
- Easy debugging through ROS2 tools
- Maximum flexibility for experimentation
- Mini Pix as simple GPS/IMU provider
- Can transition to other options later

### For Production/Field Deployment
**Recommended: Option B (Hybrid)**

**Rationale:**
- Best balance of reliability and flexibility
- Low-latency manual control
- Hardware-level failsafe (ESP32)
- Jetson retains full auto-mode control
- Redundant safety layers
- Scalable architecture

### For Mission-Critical Applications
**Recommended: Option A (Pixhawk Master)**

**Rationale:**
- Proven ArduRover reliability
- Mature failsafe system
- GPS/IMU/compass integration
- Industry-standard approach
- Extensive community support

---

## 🛠️ Migration Path

### Phase 1: Start with Option C
- Quick implementation
- Get system working end-to-end
- Learn integration challenges
- Build confidence

### Phase 2: Evolve to Option B
- Add ESP32 mode switching
- Implement hybrid control
- Move manual mode to ESP32
- Keep Jetson for auto mode

### Phase 3: Consider Option A (if needed)
- If ArduRover features required
- If ultimate reliability needed
- Migrate gradually from Option B

---

## ❓ Decision Checklist

### Questions to Answer

**Project Goals:**
- [ ] Is this primarily a development platform? → Option C
- [ ] Is this for production deployment? → Option B
- [ ] Is this safety-critical? → Option A

**Required Features:**
- [ ] Need ArduRover AUTO modes? → Option A
- [ ] Need low-latency manual control? → Option B
- [ ] Need maximum ROS2 flexibility? → Option C

**Team Capabilities:**
- [ ] Comfortable with ArduRover tuning? → Option A
- [ ] Prefer custom embedded code? → Option B
- [ ] Strong ROS2 expertise? → Option C

**Risk Tolerance:**
- [ ] Low risk tolerance? → Option A
- [ ] Medium risk tolerance? → Option B
- [ ] High risk tolerance? → Option C

---

## 📝 Next Steps After Decision

### If Option A Chosen:
1. Install MAVROS on Jetson
2. Configure ArduRover firmware
3. Set up Pixhawk → VESC connection
4. Implement MAVROS ROS2 nodes
5. Test mode switching

### If Option B Chosen:
1. Enhance ESP32 with mode switching
2. Configure Pixhawk as GPS/IMU provider
3. Install MAVROS for telemetry
4. Create mode manager ROS2 node
5. Implement failsafe coordinator

### If Option C Chosen:
1. Create ROS2 teleop node for RC input
2. Install MAVROS for GPS/IMU
3. Implement mode manager
4. Set up emergency stop handler
5. Configure Nav2 stack

---

## 📚 References

- **ArduRover Documentation:** https://ardupilot.org/rover/
- **MAVROS Documentation:** https://github.com/mavlink/mavros
- **DroneCAN Specification:** https://dronecan.github.io/
- **ROS2 Navigation:** https://navigation.ros.org/
- **VESC Documentation:** https://vesc-project.com/

---

*Document prepared for architectural decision - November 9, 2025*
*Review and decide, then proceed with implementation*
