# Fire Suppression System with Visual Tracking

**Date Created:** November 13, 2025
**Status:** Design Complete - Ready for Implementation
**Principle:** Visual Servoing + Object Tracking

---

## 📋 Table of Contents

1. [Overview](#overview)
2. [System Architecture](#system-architecture)
3. [Working Principle](#working-principle)
4. [Hardware Components](#hardware-components)
5. [Software Architecture](#software-architecture)
6. [Tracking Algorithms](#tracking-algorithms)
7. [Mechanical Design](#mechanical-design)
8. [Electrical Connections](#electrical-connections)
9. [ROS2 Integration](#ros2-integration)
10. [ESP32 Firmware](#esp32-firmware)
11. [Testing Procedures](#testing-procedures)
12. [Safety Features](#safety-features)
13. [Calibration](#calibration)

---

## Overview

The VETER Fire Suppression System uses **visual servoing** to automatically aim a water nozzle at detected fires. The key innovation is mounting the camera **directly on the nozzle**, creating a "follow-me" tracking system where keeping the fire centered in the camera's view automatically aims the water stream correctly.

### Key Features

- ✅ **Automatic Fire Detection** - YOLOv8n-based fire/smoke detection @ 22.7 FPS
- ✅ **Real-time Tracking** - CSRT tracker with periodic re-detection
- ✅ **Visual Servoing** - Camera and nozzle move together (no calibration needed)
- ✅ **PID Control** - Smooth, accurate targeting with minimal overshoot
- ✅ **Predictive Tracking** - Kalman filter for motion prediction
- ✅ **Safety Interlocks** - Emergency stop, timeout, confidence thresholds
- ✅ **DroneCAN Integration** - Uses existing CAN bus infrastructure

### Design Philosophy

**"The camera IS the targeting system"** - By rigidly mounting the camera to the nozzle, we eliminate complex coordinate transformations and calibration. If the fire is centered in the image, the water will hit it.

---

## System Architecture

```
┌──────────────────────────────────────────────────────────────────┐
│                        DETECTION LAYER                            │
└──────────────────────────────────────────────────────────────────┘

Sony IMX477 Camera (1920x1080 @ 15 Hz)
    │ Mounted on nozzle assembly
    ↓ /camera/image_raw
    │
┌───┴─────────────────────────────────────────────────────────────┐
│  YOLOv8n Fire Detection (Jetson TensorRT @ 22.7 FPS)            │
│  - Detects fire/smoke in image                                   │
│  - Returns bounding box: (x1, y1, x2, y2)                        │
│  - Confidence score: 0.0 - 1.0                                   │
└───┬─────────────────────────────────────────────────────────────┘
    ↓ Fire bbox
    │
┌───┴─────────────────────────────────────────────────────────────┐
│  CSRT Object Tracker (OpenCV)                                    │
│  - Fast tracking between detections (5ms vs 44ms YOLO)          │
│  - Re-initializes every 30 frames or on tracking loss           │
│  - Smooth motion prediction                                      │
└───┬─────────────────────────────────────────────────────────────┘
    ↓ Fire position (x, y)
    │
┌───┴─────────────────────────────────────────────────────────────┐
│  Fire Tracking Controller (ROS2 Node)                            │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ 1. Position Smoothing (Moving Average)                     │ │
│  │    - Filters jitter from detection noise                   │ │
│  │    - 5-frame history buffer                                │ │
│  └────────────────────────────────────────────────────────────┘ │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ 2. Error Calculation                                       │ │
│  │    error_x = fire_x - image_center_x                       │ │
│  │    error_y = fire_y - image_center_y                       │ │
│  └────────────────────────────────────────────────────────────┘ │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ 3. PID Control                                             │ │
│  │    delta_angle = Kp*error + Ki*integral + Kd*derivative   │ │
│  └────────────────────────────────────────────────────────────┘ │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ 4. Servo Command Generation                                │ │
│  │    pan_angle ← pan_angle + delta_pan                       │ │
│  │    tilt_angle ← tilt_angle + delta_tilt                    │ │
│  └────────────────────────────────────────────────────────────┘ │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ 5. Valve Control Logic                                     │ │
│  │    IF |error_x| < threshold AND |error_y| < threshold:     │ │
│  │        OPEN water valve                                    │ │
│  │    ELSE:                                                   │ │
│  │        CLOSE water valve                                   │ │
│  └────────────────────────────────────────────────────────────┘ │
└───┬─────────────────────────────────────────────────────────────┘
    ↓ ROS2 Topics
    ├─ /fire_suppression/servo/pan (std_msgs/Int32)
    ├─ /fire_suppression/servo/tilt (std_msgs/Int32)
    └─ /fire_suppression/water_valve (std_msgs/Bool)
    │
┌───┴─────────────────────────────────────────────────────────────┐
│  ROS2 DroneCAN Bridge (Node ID 20)                              │
│  - Converts ROS2 messages → DroneCAN messages                   │
└───┬─────────────────────────────────────────────────────────────┘
    ↓ CAN Bus @ 1 Mbps (twisted pair, EMI-resistant)
    │
┌───┴─────────────────────────────────────────────────────────────┐
│  ESP32 Fire Suppression Controller (Node ID 12)                 │
│  ┌────────────────────────────────────────────────────────────┐ │
│  │ • Receives DroneCAN commands                               │ │
│  │ • Controls pan servo (GPIO 38)                             │ │
│  │ • Controls tilt servo (GPIO 39)                            │ │
│  │ • Controls water valve via MOSFET (GPIO 40)                │ │
│  │ • Emergency stop monitoring                                │ │
│  │ • Heartbeat transmission                                   │ │
│  └────────────────────────────────────────────────────────────┘ │
└───┬─────────────────────────────────────────────────────────────┘
    ↓
┌───┴─────────────────────────────────────────────────────────────┐
│                    ACTUATOR LAYER                                │
└──────────────────────────────────────────────────────────────────┘

├─ Pan Stepper (NEMA 23 + редуктор 5:1, 64 kg·cm) → Rotates nozzle left/right
├─ Tilt Stepper (NEMA 17 + редуктор 10:1, 60 kg·cm) → Tilts nozzle up/down
└─ Solenoid Valve (12V/24V, 1/2") → Opens/closes water flow

    ↓ Water Flow
    │
[Fire Extinguished! 🔥→💧]
```

---

## Working Principle

### Visual Servoing Concept

**Traditional Approach (Complex):**
```
1. Detect fire in camera at position (x, y)
2. Transform image coordinates to world coordinates
3. Calculate angles from robot base to fire
4. Account for camera offset from nozzle
5. Account for robot movement and vibration
6. Calculate servo angles to aim nozzle
7. Send commands and hope it's accurate
```

**Our Approach (Simple):**
```
1. Detect fire in camera at position (x, y)
2. Calculate error from image center: error = fire_position - center
3. Move servos to reduce error to zero
4. When error ≈ 0: fire is centered → FIRE WATER!
5. Camera and nozzle move together → water hits fire

✨ THAT'S IT! No coordinate transforms, no calibration!
```

### Why This Works

The camera and nozzle are **rigidly mounted together**, so:
- If fire is in **center** of image → nozzle points **at fire**
- If fire is **left** of center → nozzle points **right of fire** → rotate left
- If fire is **right** of center → nozzle points **left of fire** → rotate right
- If fire is **above** center → nozzle points **below fire** → tilt up
- If fire is **below** center → nozzle points **above fire** → tilt down

The system is **self-correcting**: the visual feedback loop automatically compensates for errors.

### Control Loop

```
1. Capture frame from camera (15 Hz)
2. Track/detect fire position
3. Calculate error from center
4. Apply PID control to compute servo adjustments
5. Send servo commands via CAN
6. Wait for next frame
7. GOTO 1

Closed-loop frequency: 15 Hz (limited by camera frame rate)
Open-loop response time: ~100ms (servo movement + CAN latency)
```

---

## Hardware Components

### Computing Platform

**NVIDIA Jetson Orin Nano Super (8GB)**
- YOLOv8n fire detection @ 22.7 FPS (TensorRT optimized)
- ROS2 Humble
- DroneCAN Bridge
- Fire Tracking Controller

### Camera System

**Sony IMX477 12MP Camera**
- Resolution: 1920×1080 @ 15 Hz (current), upgradeable to 30 Hz
- FOV: 62.2° horizontal × 48.8° vertical
- Interface: MIPI CSI to Jetson CAM0
- Mount: Rigidly attached to nozzle assembly
- Protection: Waterproof housing (IP65 minimum)

### Stepper Motor System with Gearboxes

**⚡ ВАЖНО: Используются ШАГОВЫЕ ДВИГАТЕЛИ, а не сервомоторы!**

**Преимущества шаговых двигателей для пожарного гидранта:**
- ✅ Высокий крутящий момент (до 50+ кг·см с редуктором)
- ✅ Высочайшая точность (до 0.01° с микрошагами)
- ✅ Жёсткая фиксация позиции (нет дрожания)
- ✅ Неограниченный угол поворота
- ✅ Обратная связь через энкодеры (опционально)
- ✅ Нет износа потенциометра

**Pan Motor: NEMA 23 с планетарным редуктором 5:1**
- Двигатель: NEMA 23 (57×57мм)
- Момент двигателя: 1.26 Nm (12.8 kg·cm)
- Редуктор: Планетарный 5:1 или 10:1
- Момент после редуктора: 64 kg·cm (5:1) или 128 kg·cm (10:1)
- Шаг: 1.8° (200 шагов/оборот)
- С микрошагами 1/16: 3200 шагов/оборот = 0.1125° точность
- Ток: 2.8A
- Энкодер: Опционально (для обратной связи)
- Драйвер: TB6600 или DM542T (4.5A)
- Управление: STEP/DIR (от ESP32)

**Tilt Motor: NEMA 17 с планетарным редуктором 10:1**
- Двигатель: NEMA 17 (42×42мм)
- Момент двигателя: 0.59 Nm (6.0 kg·cm)
- Редуктор: Планетарный 10:1
- Момент после редуктора: 60 kg·cm
- Шаг: 1.8° (200 шагов/оборот)
- С микрошагами 1/16: 3200 шагов/оборот = 0.1125° точность
- Ток: 1.7A
- Энкодер: Опционально
- Драйвер: A4988 или DRV8825
- Управление: STEP/DIR (от ESP32)

**Драйверы:**
- **TB6600** для NEMA 23 (4.5A, микрошаги до 1/16)
- **A4988** или **DRV8825** для NEMA 17 (2A, микрошаги до 1/16)

**Питание:**
- Напряжение: 24V DC (для шаговиков)
- Ток Pan: 2.8A × 24V = 67W
- Ток Tilt: 1.7A × 24V = 41W
- Итого: ~110W (пиковая нагрузка)
- БП: 24V 5A минимум

**Endstops (концевики):**
- Оптические или механические концевики для калибровки
- Pan: 2 концевика (минимум и максимум)
- Tilt: 2 концевика (вверх и вниз)

### Water System

**Solenoid Valve**
- Type: Normally Closed (NC) - fails safe
- Voltage: 12V DC (alternative: 24V DC)
- Port size: 1/2" NPT (or 3/4" for higher flow)
- Flow rate: 10-20 liters/minute @ 2-4 bar
- Response time: <50ms open/close
- Protection: Flyback diode (1N4007) across coil

**Water Nozzle**
- Type: Adjustable fog/stream nozzle
- Material: Brass or aluminum
- Connection: 1/2" female NPT
- Range: 5-10 meters @ 3 bar pressure

**Hose**
- Flexible reinforced hose from valve to nozzle
- Inner diameter: 12mm (1/2")
- Minimum bend radius: 50mm
- Material: PVC or rubber reinforced with fabric

**Water Source**
- Onboard tank: 20-50 liters
- Pressure: 2-4 bar (pump or pressurized tank)
- Alternative: External hydrant connection

### ESP32 Fire Controller

**ESP32-S3-DevKitC-1 v1.0**
- Flash: 16MB
- PSRAM: 8MB
- DroneCAN Node ID: 12
- CAN Interface: ESP32 TWAI native (GPIO4 TX, GPIO5 RX)
- CAN Transceiver: WCMCU-230 (SN65HVD230) 3.3V
- Power: 5V @ 500mA (USB or buck converter)

### Power System

**Servo Power Supply**
- Voltage: 6V regulated (5V works but less torque)
- Current: 10A minimum (servos can draw 2A each under load)
- Type: Buck converter from main battery (57.6V → 6V)
- Protection: Fuse, reverse polarity protection

**Valve Power**
- Voltage: 12V (from main battery via DC-DC converter)
- Current: 1A (typical solenoid: 10-15W)
- Switching: N-channel MOSFET (IRF540N or IRLZ44N)

**ESP32 Power**
- Voltage: 5V regulated
- Current: 500mA
- Source: USB power from servo power supply

---

## Software Architecture

### ROS2 Packages

**veter_perception** (existing, extended)
```
veter_perception/
├── veter_perception/
│   ├── yolo_detector.py              # Existing YOLO node
│   ├── fire_detector.py              # NEW: Fire-specific detection
│   ├── fire_tracking_controller.py   # NEW: Visual servoing + tracking
│   └── fire_tracking_kalman.py       # NEW: With Kalman filter (advanced)
├── launch/
│   ├── fire_detection.launch.py      # NEW: Detection only
│   ├── fire_tracking.launch.py       # NEW: Full tracking system
│   └── fire_test.launch.py           # NEW: Test without water
├── config/
│   └── fire_params.yaml              # NEW: Tuning parameters
└── models/
    └── yolov8n-fire.pt               # NEW: Fire detection model
```

### ROS2 Topics

**Subscribed:**
- `/camera/image_raw` (sensor_msgs/Image) - Camera feed from IMX477

**Published:**
- `/fire_detection/bbox` (vision_msgs/Detection2D) - Fire bounding box
- `/fire_detection/target` (geometry_msgs/Point) - Fire center + confidence
- `/fire_suppression/servo/pan` (std_msgs/Int32) - Pan servo angle (0-180°)
- `/fire_suppression/servo/tilt` (std_msgs/Int32) - Tilt servo angle (0-90°)
- `/fire_suppression/water_valve` (std_msgs/Bool) - Water valve state
- `/fire_suppression/status` (std_msgs/String) - System status messages
- `/fire_suppression/debug_image` (sensor_msgs/Image) - Debug visualization

### Parameters (fire_params.yaml)

```yaml
fire_tracking:
  ros__parameters:
    # Camera parameters
    image_width: 1920
    image_height: 1080
    fov_horizontal: 62.2  # degrees
    fov_vertical: 48.8    # degrees

    # Detection parameters
    detection_confidence: 0.6  # YOLO confidence threshold
    redetect_interval: 30      # Frames between re-detections

    # Tracking parameters
    tracker_type: "CSRT"       # CSRT, KCF, or MOSSE
    position_history_size: 5   # Smoothing window

    # PID gains (tune these!)
    kp_pan: 0.05
    ki_pan: 0.001
    kd_pan: 0.02

    kp_tilt: 0.05
    ki_tilt: 0.001
    kd_tilt: 0.02

    # Servo limits
    pan_min: 0
    pan_max: 180
    tilt_min: 0
    tilt_max: 90

    # Targeting parameters
    targeting_threshold: 50    # pixels from center
    valve_activation_delay: 0.5  # seconds (ensure stable aim)

    # Safety parameters
    max_water_time: 30.0       # seconds (auto shutoff)
    min_confidence: 0.7        # minimum to activate valve
    emergency_stop_enabled: true
```

---

## Tracking Algorithms

### 1. Detection-Only Tracking (Simple)

**Method:** Run YOLOv8n every frame

**Pros:**
- Simple implementation
- Always has latest fire position
- No tracking loss issues

**Cons:**
- Slower (44ms per frame)
- Can jitter if detection boxes vary
- High GPU load

**When to use:** Testing, or if tracking proves unreliable

### 2. Hybrid Detection + Tracking (Recommended) ⭐

**Method:** CSRT tracker with periodic re-detection

**Algorithm:**
```python
tracker = cv2.TrackerCSRT_create()
frame_count = 0

while True:
    if tracker is None or frame_count % 30 == 0:
        # Run YOLO detection
        bbox = yolo_detect_fire(frame)
        if bbox:
            tracker.init(frame, bbox)
            frame_count = 0
    else:
        # Use fast tracking
        success, bbox = tracker.update(frame)
        if not success:
            tracker = None  # Force re-detection next frame

    frame_count += 1

    # Use bbox for targeting
    fire_center = get_center(bbox)
    aim_at(fire_center)
```

**Pros:**
- Fast (5ms tracking vs 44ms detection)
- Smooth motion (tracker interpolates between detections)
- Periodic re-detection corrects drift
- Falls back to detection if tracking fails

**Cons:**
- Slightly more complex
- Can lose track on fast motion (mitigated by re-detection)

**Tracking Performance:**
- CSRT: Best accuracy, slower (5-10ms)
- KCF: Fast, good accuracy (2-5ms)
- MOSSE: Fastest, lower accuracy (1-2ms)

**Recommendation:** CSRT for fire tracking (accuracy matters more than speed)

### 3. Kalman Filter Tracking (Advanced) ⭐⭐

**Method:** Predict fire motion using Kalman filter

**State Vector:**
```
x = [x_position, y_position, x_velocity, y_velocity]
```

**Process Model:**
```
x(k+1) = x(k) + vx(k) * dt
y(k+1) = y(k) + vy(k) * dt
vx(k+1) = vx(k)  # Assume constant velocity
vy(k+1) = vy(k)
```

**Algorithm:**
```python
from filterpy.kalman import KalmanFilter

kf = KalmanFilter(dim_x=4, dim_z=2)

# State transition matrix
kf.F = np.array([[1, 0, dt, 0],
                 [0, 1, 0, dt],
                 [0, 0, 1, 0],
                 [0, 0, 0, 1]])

# Measurement matrix (we measure x, y only)
kf.H = np.array([[1, 0, 0, 0],
                 [0, 1, 0, 0]])

while True:
    # Predict next state
    kf.predict()

    # Measure (from detection or tracking)
    fire_x, fire_y = get_fire_position()
    kf.update([fire_x, fire_y])

    # Use smoothed and predicted position
    smoothed_x, smoothed_y = kf.x[0], kf.x[1]
    predicted_x = smoothed_x + kf.x[2] * predict_ahead_time
    predicted_y = smoothed_y + kf.x[3] * predict_ahead_time

    # Aim at predicted position (compensates for lag!)
    aim_at(predicted_x, predicted_y)
```

**Pros:**
- Very smooth tracking
- Predicts future position (compensates for servo lag)
- Handles temporary occlusions
- Filters out detection noise

**Cons:**
- More complex implementation
- Requires tuning process noise and measurement noise
- Assumes constant velocity (fire doesn't move predictably)

**When to use:** If fire moves (e.g., spreading across fuel), or if servo response is slow

---

## Mechanical Design

### Assembly Overview

```
                    Top View

        ┌─────────────────────────┐
        │   Mounting Plate        │
        │   (Aluminum 5mm)        │
        └──────────┬──────────────┘
                   │
        ┌──────────┴──────────┐
        │                     │
   [Camera]              [Nozzle]
   IMX477                1/2" pipe
   In waterproof         Brass
   housing (IP65)
        │                     │
        └─────── RIGID ───────┘  ← Critical: Both point SAME direction!
                    │
            Alignment pins/
            laser calibration
                    │
        ┌───────────┴───────────┐
        │   Pan Servo Bracket   │
        │   (U-shaped aluminum) │
        │                       │
        │   ┌───────────────┐   │
        │   │   DS3218      │   │ ← Pan servo (rotates ↻)
        │   │   Pan Servo   │   │
        │   └───────────────┘   │
        └───────────┬───────────┘
                    │
                    │ Servo horn
                    │
        ┌───────────┴───────────┐
        │   Tilt Servo Bracket  │
        │   (Side-mounted)      │
        │                       │
        │   ┌───────────────┐   │
        │   │   DS3218      │   │ ← Tilt servo (tilts ⤴⤵)
        │   │   Tilt Servo  │   │
        │   └───────────────┘   │
        └───────────┬───────────┘
                    │
            ┌───────┴────────┐
            │  Robot Chassis │
            └────────────────┘
```

### Side View

```
                Camera
                  ↓
            ┌────────┐
            │  📷    │
            └───┬────┘
                │         Nozzle
                │           ↓
                │      ┌────🚿
                │      │
    ┌───────────┴──────┴────┐
    │  Mounting Plate        │ ← Must be RIGID!
    └───────────┬────────────┘
                │
        ┌───────┴────────┐
        │                │
        │  Pan Servo     │ ⟲
        │  (Horizontal)  │
        └───────┬────────┘
                │
        ┌───────┴────────┐
        │                │
        │  Tilt Servo    │ ⟳
        │  (Vertical)    │
        └────────────────┘
```

### Critical Design Requirements

**1. Rigid Camera-Nozzle Coupling**
- Material: 5mm aluminum plate or 3D printed PETG/ABS
- No flex under servo motion or water pressure
- Use bolts (M4 or M5), not just 3D printed joints
- Alternative: Welded metal bracket

**2. Alignment**
- Camera and nozzle must point in **exactly** the same direction
- Use laser pointer for initial alignment:
  - Mount laser pointer to camera
  - Aim at target 5 meters away
  - Adjust nozzle until water hits same spot
- Check alignment at multiple angles (pan: 0°, 90°, 180°; tilt: 30°, 60°, 90°)

**3. Weight Distribution**
- Keep center of mass close to pan servo axis
- Heavy items (camera housing, water-filled hose) should be balanced
- May need counterweight if very unbalanced

**4. Servo Strength**
- Calculate maximum load:
  ```
  Weight of assembly = camera (200g) + housing (300g) +
                       nozzle (100g) + plate (200g) +
                       water in hose (500g) = 1300g

  Arm length = 15cm (distance from pan servo to center of mass)

  Torque required = 1.3 kg × 15 cm = 19.5 kg·cm

  → Need servo with ≥20 kg·cm (DS3218 = 20 kg·cm is minimum!)
  ```

**5. Water Hose Management**
- Use flexible hose from valve to nozzle
- Leave slack for full range of motion
- Use cable ties or spiral wrap to keep hose tidy
- Avoid sharp bends (reduces flow)

**6. Protection**
- Camera: Waterproof housing (IP65 minimum)
- Servos: Shield from water spray with rubber boot
- ESP32: Waterproof enclosure near base
- Connectors: Use IP67-rated connectors or silicone sealant

---

## Electrical Connections

### ESP32 Fire Controller Pinout

```
ESP32-S3-DevKitC-1 v1.0 (Node ID 12)

// Шаговые двигатели (STEP/DIR управление)
GPIO 38 ──────> Pan Stepper STEP
GPIO 39 ──────> Pan Stepper DIR
GPIO 21 ──────> Tilt Stepper STEP
GPIO 47 ──────> Tilt Stepper DIR
GPIO 42 ──────> Pan Endstop MIN (optional)
GPIO 45 ──────> Pan Endstop MAX (optional)
GPIO 46 ──────> Tilt Endstop MIN (optional)
GPIO 48 ──────> Tilt Endstop MAX (optional)
GPIO 40 ──────> MOSFET Gate (Water Valve Control)

GPIO 4 (TX) ───> WCMCU-230 TX (TWAI CAN)
GPIO 5 (RX) ───> WCMCU-230 RX (TWAI CAN)

GPIO 23 ──────> Emergency Stop Button (INPUT_PULLUP)
GPIO 48 ──────> Status LED (WS2812B) - optional

5V ────────────> VCC (USB or buck converter)
GND ───────────> GND (common with all systems)
```

### Servo Connections

```
Pan Stepper Motor (NEMA 23 + TB6600):
  STEP ────────> ESP32 GPIO 38
  DIR ─────────> ESP32 GPIO 39
  Endstop MIN ─> ESP32 GPIO 42
  Endstop MAX ─> ESP32 GPIO 45
  VCC (+) ─────> 24V Power Supply (+)
  GND (−) ─────> Common GND

Tilt Stepper Motor (NEMA 17 + A4988):
  STEP ────────> ESP32 GPIO 21
  DIR ─────────> ESP32 GPIO 47
  Endstop MIN ─> ESP32 GPIO 46
  Endstop MAX ─> ESP32 GPIO 48
  VCC (+) ─────> 24V Power Supply (+)
  GND (−) ─────> Common GND

NOTE: Stepper drivers draw up to 5A @ 24V (120W peak)!
      Use thick wires (18 AWG / 1mm²)
```

### Water Valve Circuit

```
                +12V (from battery via DC-DC converter)
                  │
                  │
           [Solenoid Valve Coil]
                  │
                  ├───── Flyback Diode (1N4007) ──┐
                  │      (Cathode to +12V)        │
                  │                                │
             Drain│                                │
            ┌─────┴─────────┐                     │
            │               │                     │
Gate ───────┤  MOSFET       │                     │
(GPIO40)    │  IRF540N or   │                     │
via 220Ω    │  IRLZ44N      │                     │
resistor    │               │                     │
            └─────┬─────────┘                     │
             Source│                               │
                  │                                │
                 GND ──────────────────────────────┘
                  │
                  │ 10kΩ pull-down resistor
                  └───────[10kΩ]────────> GND

Components:
- MOSFET: IRF540N (N-channel, 100V, 33A) or IRLZ44N (logic-level)
- Diode: 1N4007 (1A, 1000V) protects against inductive kickback
- Gate resistor: 220Ω (limits current from ESP32)
- Pull-down resistor: 10kΩ (ensures MOSFET off when ESP32 not driving)
```

### Power Distribution

```
Main Battery (18S LiFePO4, 57.6V)
    │
    ├───> Buck Converter #1 (57.6V → 6V @ 10A)
    │         └──> Servo Power (Pan + Tilt)
    │
    ├───> Buck Converter #2 (57.6V → 12V @ 2A)
    │         └──> Solenoid Valve Power
    │
    └───> Buck Converter #3 (57.6V → 5V @ 1A)
              └──> ESP32 Fire Controller

All converters: Common GND!

Recommended converters:
- LM2596 modules (cheap, reliable, handle high input voltage)
- Efficiency: ~85%
- Heat: Add heatsinks if running hot
```

### CAN Bus Connection

```
ESP32 Fire Controller (Node 12)
    │
    ├─ GPIO4 (TX) ──> WCMCU-230 TX
    ├─ GPIO5 (RX) ──> WCMCU-230 RX
    │
WCMCU-230 Transceiver
    │
    ├─ CANH ──────────┐
    │                 │
    ├─ CANL ──────────┤
    │                 │
    └─ GND ───────────┤
                      │
        Twisted Pair  │
        (can be       │
        several       │
        meters long)  │
                      │
    ┌─────────────────┘
    │
CAN Bus (Main)
    │
    ├─ Jetson CAN Interface (can0)
    ├─ ESP32 Motor Controller (Node 10)
    ├─ ESP32 Sensor Hub (Node 11)
    ├─ ESP32 Fire Controller (Node 12) ← NEW
    └─ VESC × 2

Termination Resistors:
- 120Ω at Jetson end
- 120Ω at far end of bus
- DO NOT terminate at ESP32 Fire Controller (middle of bus)
```

---

## ROS2 Integration

### Launch File

**fire_tracking.launch.py:**

```python
#!/usr/bin/env python3
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Launch arguments
        DeclareLaunchArgument(
            'debug',
            default_value='false',
            description='Enable debug visualization'
        ),

        DeclareLaunchArgument(
            'test_mode',
            default_value='false',
            description='Test mode (no water valve control)'
        ),

        # Fire detection node
        Node(
            package='veter_perception',
            executable='fire_detector',
            name='fire_detector',
            output='screen',
            parameters=[{
                'model_path': '/path/to/yolov8n-fire.pt',
                'confidence_threshold': 0.6,
            }]
        ),

        # Fire tracking controller
        Node(
            package='veter_perception',
            executable='fire_tracking_controller',
            name='fire_tracking',
            output='screen',
            parameters=[
                'config/fire_params.yaml',
                {
                    'debug': LaunchConfiguration('debug'),
                    'test_mode': LaunchConfiguration('test_mode'),
                }
            ]
        ),

        # DroneCAN bridge (if not already running)
        Node(
            package='veter_dronecan_bridge',
            executable='dronecan_bridge_node',
            name='dronecan_bridge',
            output='screen',
        ),
    ])
```

**Usage:**
```bash
# Normal operation
ros2 launch veter_perception fire_tracking.launch.py

# Debug mode (publishes visualization)
ros2 launch veter_perception fire_tracking.launch.py debug:=true

# Test mode (no water!)
ros2 launch veter_perception fire_tracking.launch.py test_mode:=true
```

### Manual Control

**Testing servo movement:**
```bash
# Center position
ros2 topic pub /fire_suppression/servo/pan std_msgs/Int32 "{data: 90}"
ros2 topic pub /fire_suppression/servo/tilt std_msgs/Int32 "{data: 45}"

# Pan left
ros2 topic pub /fire_suppression/servo/pan std_msgs/Int32 "{data: 0}"

# Pan right
ros2 topic pub /fire_suppression/servo/pan std_msgs/Int32 "{data: 180}"

# Tilt down (point forward)
ros2 topic pub /fire_suppression/servo/tilt std_msgs/Int32 "{data: 0}"

# Tilt up (point down at ground)
ros2 topic pub /fire_suppression/servo/tilt std_msgs/Int32 "{data: 90}"

# Open water valve (CAREFUL!)
ros2 topic pub /fire_suppression/water_valve std_msgs/Bool "{data: true}"

# Close water valve
ros2 topic pub /fire_suppression/water_valve std_msgs/Bool "{data: false}"
```

### Monitoring

```bash
# Watch fire detection
ros2 topic echo /fire_detection/target

# Watch servo commands
ros2 topic echo /fire_suppression/servo/pan
ros2 topic echo /fire_suppression/servo/tilt

# Watch valve state
ros2 topic echo /fire_suppression/water_valve

# Watch system status
ros2 topic echo /fire_suppression/status
```

---

## ESP32 Firmware

### Firmware Structure

```
firmware/esp32_fire_controller/
├── platformio.ini
├── include/
│   ├── config.h
│   └── dronecan_interface.h
├── src/
│   ├── main.cpp
│   └── dronecan_interface.cpp
└── README.md
```

### platformio.ini

```ini
[env:esp32-s3-devkitc-1]
platform = espressif32@6.10.0
board = esp32-s3-devkitc-1
framework = arduino

board_build.flash_size = 16MB
board_build.psram_size = 8MB
monitor_speed = 115200
monitor_filters = direct

build_flags =
    -DCORE_DEBUG_LEVEL=3
    -DARDUINO_USB_MODE=1
    -DARDUINO_USB_CDC_ON_BOOT=1

lib_deps =
    waspinator/AccelStepper@^1.64
```

### config.h

```cpp
// ESP32 Fire Suppression Controller Configuration
// DroneCAN Node ID: 12

#pragma once

// CAN Bus (TWAI)
#define CAN_TX_PIN          4
#define CAN_RX_PIN          5
#define CAN_BITRATE         1000000  // 1 Mbps

// DroneCAN
#define DRONECAN_NODE_ID    12
#define HEARTBEAT_INTERVAL  100   // ms

// Servo Control
#define SERVO_PAN_PIN       38
#define SERVO_TILT_PIN      39
#define SERVO_PWM_MIN       500   // μs
#define SERVO_PWM_MAX       2500  // μs

// Servo Limits
#define PAN_MIN_ANGLE       0
#define PAN_MAX_ANGLE       180
#define TILT_MIN_ANGLE      0
#define TILT_MAX_ANGLE      90

// Water Valve
#define WATER_VALVE_PIN     40
#define VALVE_ACTIVE        HIGH  // or LOW depending on MOSFET

// Safety
#define EMERGENCY_STOP_PIN  23
#define E_STOP_ACTIVE       LOW   // Active low (pullup)

// Status LED (optional)
#define STATUS_LED_PIN      48
```

### main.cpp (Basic Version)

```cpp
#include <Arduino.h>
#include <AccelStepper.h>
#include "config.h"
#include "dronecan_interface.h"

// Stepper motor objects
// AccelStepper(interface, stepPin, dirPin)
AccelStepper stepperPan(AccelStepper::DRIVER, PIN_PAN_STEP, PIN_PAN_DIR);
AccelStepper stepperTilt(AccelStepper::DRIVER, PIN_TILT_STEP, PIN_TILT_DIR);

// Endstops
const int PIN_PAN_ENDSTOP_MIN = 42;
const int PIN_PAN_ENDSTOP_MAX = 45;
const int PIN_TILT_ENDSTOP_MIN = 46;
const int PIN_TILT_ENDSTOP_MAX = 48;

// Calibration state
bool panCalibrated = false;
bool tiltCalibrated = false;
long panZeroPosition = 0;
long tiltZeroPosition = 0;

// Stepper parameters (with 1/16 microstepping and 5:1 gearbox)
const float STEPS_PER_DEGREE_PAN = (200 * 16 * 5) / 360.0;  // ~44.4 steps/degree
const float STEPS_PER_DEGREE_TILT = (200 * 16 * 10) / 360.0; // ~88.9 steps/degree

// Current positions (in degrees)
float currentPanAngle = 0.0;   // Center (after calibration)
float currentTiltAngle = 0.0;  // Horizontal (after calibration)

// Water valve state
bool waterValveOpen = false;

// Emergency stop
bool emergencyStop = false;

void setup() {
    Serial.begin(115200);
    Serial.println("ESP32 Fire Suppression Controller v1.0");
    Serial.println("Node ID: 12");

    // Initialize endstops
    pinMode(PIN_PAN_ENDSTOP_MIN, INPUT_PULLUP);
    pinMode(PIN_PAN_ENDSTOP_MAX, INPUT_PULLUP);
    pinMode(PIN_TILT_ENDSTOP_MIN, INPUT_PULLUP);
    pinMode(PIN_TILT_ENDSTOP_MAX, INPUT_PULLUP);

    // Configure stepper motors
    stepperPan.setMaxSpeed(2000);       // steps/sec (conservative)
    stepperPan.setAcceleration(1000);   // steps/sec²
    stepperTilt.setMaxSpeed(2000);
    stepperTilt.setAcceleration(1000);

    // Calibrate steppers (home to endstops)
    Serial.println("Calibrating Pan axis...");
    calibratePan();
    Serial.println("Calibrating Tilt axis...");
    calibrateTilt();

    // Move to neutral position
    movePanToAngle(0);    // Center
    moveTiltToAngle(0);   // Horizontal

    // Initialize water valve
    pinMode(WATER_VALVE_PIN, OUTPUT);
    digitalWrite(WATER_VALVE_PIN, !VALVE_ACTIVE);  // Closed

    // Initialize emergency stop
    pinMode(EMERGENCY_STOP_PIN, INPUT_PULLUP);

    // Initialize DroneCAN
    dronecan_init(DRONECAN_NODE_ID, CAN_BITRATE);

    Serial.println("Initialization complete");
    Serial.println("Waiting for commands...");
}

void loop() {
    // Check emergency stop
    emergencyStop = (digitalRead(EMERGENCY_STOP_PIN) == E_STOP_ACTIVE);

    if (emergencyStop) {
        // E-stop active: close valve immediately
        if (waterValveOpen) {
            digitalWrite(WATER_VALVE_PIN, !VALVE_ACTIVE);
            waterValveOpen = false;
            Serial.println("E-STOP ACTIVE! Valve closed");
        }
    }

    // Process DroneCAN messages
    dronecan_process();

    // Check for servo commands
    // Update steppers (MUST be called frequently!)
    stepperPan.run();
    stepperTilt.run();

    // Process DroneCAN commands
    if (dronecan_has_pan_command()) {
        float angle = dronecan_get_pan_angle();

        // Clamp to limits
        angle = constrain(angle, PAN_MIN_ANGLE, PAN_MAX_ANGLE);

        // Move stepper
        movePanToAngle(angle);
        currentPanAngle = angle;

        Serial.printf("Pan: %.1f°\n", angle);
    }

    if (dronecan_has_tilt_command()) {
        float angle = dronecan_get_tilt_angle();

        // Clamp to limits
        angle = constrain(angle, TILT_MIN_ANGLE, TILT_MAX_ANGLE);

        // Move stepper
        moveTiltToAngle(angle);
        currentTiltAngle = angle;

        Serial.printf("Tilt: %.1f°\n", angle);
    }

    // Check for valve command
    if (dronecan_has_valve_command()) {
        bool commanded_state = dronecan_get_valve_state();

        // Only open valve if E-stop is not active
        if (commanded_state && !emergencyStop) {
            digitalWrite(WATER_VALVE_PIN, VALVE_ACTIVE);
            waterValveOpen = true;
            Serial.println("Valve OPEN 💦");
        } else {
            digitalWrite(WATER_VALVE_PIN, !VALVE_ACTIVE);
            waterValveOpen = false;
            if (emergencyStop) {
                Serial.println("Valve CLOSED (E-stop)");
            } else {
                Serial.println("Valve closed");
            }
        }
    }

    // Send heartbeat
    static unsigned long lastHeartbeat = 0;
    if (millis() - lastHeartbeat >= HEARTBEAT_INTERVAL) {
        dronecan_send_heartbeat();
        lastHeartbeat = millis();
    }

    delay(1);  // Small delay for stability
}
```

---

## Testing Procedures

### 1. Electrical Testing (Power OFF)

**Checklist:**
- [ ] Check all power supply voltages with multimeter
  - Servo power: 5-6V DC
  - Valve power: 12V or 24V DC
  - ESP32 power: 5V DC
- [ ] Check GND continuity between all systems
- [ ] Check CAN bus continuity (CANH, CANL)
- [ ] Check for shorts (power to GND)
- [ ] Inspect all solder joints
- [ ] Check wire gauge (servos: 18 AWG minimum)

### 2. ESP32 Firmware Test (No Servos)

**Procedure:**
1. Flash firmware to ESP32
2. Open serial monitor (115200 baud)
3. Check startup messages
4. Send test CAN messages from Jetson
5. Verify ESP32 receives and processes commands

**Expected Output:**
```
ESP32 Fire Suppression Controller v1.0
Node ID: 12
TWAI initialized at 1000000 bps
Servos attached
Initialization complete
Waiting for commands...
```

### 3. Servo Test (No Load)

**Procedure:**
1. Connect servos (no mechanical load yet)
2. Power on system
3. Send manual commands via ROS2:
```bash
ros2 topic pub /fire_suppression/servo/pan std_msgs/Int32 "{data: 0}"
ros2 topic pub /fire_suppression/servo/pan std_msgs/Int32 "{data: 90}"
ros2 topic pub /fire_suppression/servo/pan std_msgs/Int32 "{data: 180}"
```
4. Observe smooth motion, no jitter
5. Repeat for tilt servo

**Pass Criteria:**
- [ ] Servos move smoothly
- [ ] No excessive heat
- [ ] No strange noises
- [ ] Correct angle correspondence

### 4. Valve Test (Dry - No Water)

**Procedure:**
1. Connect solenoid valve (no water pressure)
2. Send open command:
```bash
ros2 topic pub /fire_suppression/water_valve std_msgs/Bool "{data: true}"
```
3. Listen for valve click (should hear solenoid activate)
4. Send close command
5. Verify valve closes (click)

**Pass Criteria:**
- [ ] Audible click on open
- [ ] Audible click on close
- [ ] No excessive heat from MOSFET
- [ ] Flyback diode installed correctly

### 5. Mechanical Assembly Test

**Procedure:**
1. Mount camera and nozzle to plate
2. Attach to servo assembly
3. Manually move through full range of motion
4. Check for:
   - Binding or friction points
   - Servo strain (should move easily by hand when powered off)
   - Clearances (nothing hits chassis)
   - Cable management (no pinched wires)

### 6. Alignment Test (Laser Pointer)

**Procedure:**
1. Mount laser pointer to camera
2. Place target 5 meters away
3. Center servos (pan=90°, tilt=45°)
4. Adjust nozzle until laser and water stream converge on target
5. Test at different angles:
   - Pan: 0°, 45°, 90°, 135°, 180°
   - Tilt: 0°, 30°, 45°, 60°, 90°
6. Mark any offsets and adjust mounting

**Pass Criteria:**
- [ ] Camera and nozzle point in same direction (±2° tolerance)
- [ ] Alignment holds across all angles
- [ ] No mechanical flex under servo motion

### 7. Water Test (Low Pressure)

**Procedure:**
1. Connect water supply at **LOW pressure** (1 bar)
2. Point nozzle in safe direction (away from electronics!)
3. Open valve manually:
```bash
ros2 topic pub /fire_suppression/water_valve std_msgs/Bool "{data: true}"
```
4. Observe:
   - Water flow (should be steady stream)
   - Leaks (check all connections)
   - Servo strength (should hold position under water pressure)
5. Close valve
6. Verify no dripping

**Pass Criteria:**
- [ ] No leaks
- [ ] Servos maintain position (don't droop)
- [ ] Valve opens/closes reliably
- [ ] No water spray on electronics

### 8. Fire Detection Test (Static Image)

**Procedure:**
1. Show fire image or video to camera
2. Run fire detection node:
```bash
ros2 launch veter_perception fire_detection.launch.py
```
3. Check for detection:
```bash
ros2 topic echo /fire_detection/target
```
4. Move fire image around
5. Verify tracking follows fire

**Pass Criteria:**
- [ ] Fire detected with confidence >0.7
- [ ] Bounding box accurate
- [ ] Detection stable (not flickering)

### 9. Tracking Test (Test Mode - No Water)

**Procedure:**
1. Start full system in test mode:
```bash
ros2 launch veter_perception fire_tracking.launch.py test_mode:=true
```
2. Show fire image to camera
3. Move fire around
4. Observe servos track fire
5. Monitor status:
```bash
ros2 topic echo /fire_suppression/status
```

**Pass Criteria:**
- [ ] Servos move to center fire in image
- [ ] Smooth motion (no oscillation)
- [ ] "ON TARGET" message when centered
- [ ] Valve remains CLOSED (test mode)

### 10. Full System Test (Real Fire - OUTDOOR ONLY!)

**⚠️ WARNING: Only perform in safe outdoor environment with fire extinguisher ready!**

**Procedure:**
1. Set up controlled test fire:
   - Small fire in metal container
   - Clear area around (5 meter radius)
   - Fire extinguisher ready
   - Safety observer present
2. Start full system:
```bash
ros2 launch veter_perception fire_tracking.launch.py
```
3. Robot detects fire
4. System aims nozzle
5. System opens valve automatically
6. Verify fire is extinguished
7. System closes valve

**Pass Criteria:**
- [ ] Fire detected reliably
- [ ] Servos aim at fire center
- [ ] Valve opens when on target
- [ ] Water stream hits fire
- [ ] Fire extinguished within 5-10 seconds
- [ ] System shuts off valve after timeout

---

## Safety Features

### 1. Emergency Stop

**Implementation:**
- Hardware button connected to ESP32 GPIO 23
- Active LOW with internal pullup
- Checked every loop iteration
- Immediately closes water valve
- Disables all servo motion

**Code:**
```cpp
if (digitalRead(EMERGENCY_STOP_PIN) == LOW) {
    digitalWrite(WATER_VALVE_PIN, !VALVE_ACTIVE);  // Close valve
    // Don't process any servo commands
    return;
}
```

### 2. Water Timeout

**Purpose:** Prevent wasting water or flooding

**Implementation:**
```python
MAX_WATER_TIME = 30.0  # seconds

if valve_open:
    if water_start_time is None:
        water_start_time = time.time()

    elapsed = time.time() - water_start_time

    if elapsed > MAX_WATER_TIME:
        close_valve()
        self.get_logger().error('⚠️ MAX WATER TIME EXCEEDED - Auto shutoff')
        water_start_time = None
```

### 3. Confidence Threshold

**Purpose:** Don't activate on false detections

**Implementation:**
```python
MIN_CONFIDENCE = 0.7

if fire_confidence < MIN_CONFIDENCE:
    close_valve()
    return  # Don't aim
```

### 4. Targeting Threshold

**Purpose:** Only spray when accurately aimed

**Implementation:**
```python
TARGETING_THRESHOLD = 50  # pixels

error_x = abs(fire_x - center_x)
error_y = abs(fire_y - center_y)

if error_x < TARGETING_THRESHOLD and error_y < TARGETING_THRESHOLD:
    # Well aimed, okay to open valve
    open_valve()
else:
    # Still aiming, don't spray yet
    close_valve()
```

### 5. Valve Activation Delay

**Purpose:** Ensure stable aim before spraying

**Implementation:**
```python
VALVE_ACTIVATION_DELAY = 0.5  # seconds

if on_target():
    if target_stable_start_time is None:
        target_stable_start_time = time.time()

    if time.time() - target_stable_start_time > VALVE_ACTIVATION_DELAY:
        open_valve()
else:
    target_stable_start_time = None
    close_valve()
```

### 6. Watchdog Timer

**Purpose:** Detect if ROS2 node crashes

**Implementation (ESP32):**
```cpp
#define COMMAND_TIMEOUT 2000  // ms

unsigned long lastCommandTime = 0;

void loop() {
    if (millis() - lastCommandTime > COMMAND_TIMEOUT) {
        // No commands received for 2 seconds
        digitalWrite(WATER_VALVE_PIN, !VALVE_ACTIVE);  // Close valve
        Serial.println("WATCHDOG: No commands, valve closed");
    }
}

// In command processing:
void process_command() {
    lastCommandTime = millis();  // Reset watchdog
    // ... process command ...
}
```

### 7. Servo Limits

**Purpose:** Prevent mechanical damage

**Implementation:**
```cpp
float commanded_angle = dronecan_get_pan_angle();
commanded_angle = constrain(commanded_angle, PAN_MIN_ANGLE, PAN_MAX_ANGLE);
movePanToAngle(commanded_angle);  // Non-blocking
stepperPan.run();  // Update stepper position
```

**Helper Functions for Stepper Control:**

```cpp
// Calibrate Pan axis (home to MIN endstop)
void calibratePan() {
    stepperPan.setSpeed(-500);  // Move backwards slowly

    while (digitalRead(PIN_PAN_ENDSTOP_MIN) == HIGH) {
        stepperPan.runSpeed();
    }

    // Found MIN endstop
    stepperPan.setCurrentPosition(0);
    panZeroPosition = 0;
    panCalibrated = true;

    Serial.println("Pan calibrated at MIN position");

    // Move off endstop slightly
    movePanToAngle(5);
    while (stepperPan.distanceToGo() != 0) {
        stepperPan.run();
    }
}

// Calibrate Tilt axis (home to MIN endstop)
void calibrateTilt() {
    stepperTilt.setSpeed(-500);  // Move backwards slowly

    while (digitalRead(PIN_TILT_ENDSTOP_MIN) == HIGH) {
        stepperTilt.runSpeed();
    }

    // Found MIN endstop
    stepperTilt.setCurrentPosition(0);
    tiltZeroPosition = 0;
    tiltCalibrated = true;

    Serial.println("Tilt calibrated at MIN position");

    // Move off endstop slightly
    moveTiltToAngle(5);
    while (stepperTilt.distanceToGo() != 0) {
        stepperTilt.run();
    }
}

// Move Pan to angle (in degrees, non-blocking)
void movePanToAngle(float angle) {
    long targetSteps = (long)(angle * STEPS_PER_DEGREE_PAN);
    stepperPan.moveTo(targetSteps);
}

// Move Tilt to angle (in degrees, non-blocking)
void moveTiltToAngle(float angle) {
    long targetSteps = (long)(angle * STEPS_PER_DEGREE_TILT);
    stepperTilt.moveTo(targetSteps);
}

// Check if Pan is at target
bool isPanAtTarget() {
    return stepperPan.distanceToGo() == 0;
}

// Check if Tilt is at target
bool isTiltAtTarget() {
    return stepperTilt.distanceToGo() == 0;
}
```

### 8. Power Loss Handling

**Hardware:**
- Solenoid valve is **Normally Closed (NC)**
- If power is lost, valve closes automatically
- No software needed - fail-safe by design!

---

## Calibration

### Camera-Nozzle Alignment

**Goal:** Ensure camera and nozzle point in same direction

**Method 1: Laser Pointer**

1. Mount laser pointer parallel to camera lens
2. Place target 5 meters away
3. Turn on laser
4. Open water valve (low pressure)
5. Adjust nozzle mount until water stream hits laser dot
6. Tighten all bolts
7. Re-test at different pan/tilt angles

**Method 2: Test Fire**

1. Create controlled test fire (small flame in metal container)
2. Center servos (pan=90°, tilt=45°)
3. Manually aim camera at fire (visually centered in frame)
4. Open water valve
5. Observe where water lands
6. If water misses fire:
   - Measure offset (left/right, up/down)
   - Adjust nozzle mount
   - Re-test

**Target Accuracy:** ±5cm at 5 meters (±1° angular accuracy)

### PID Tuning

**Goal:** Fast, smooth aiming without overshoot

**Initial Values (Conservative):**
```yaml
kp_pan: 0.02
ki_pan: 0.0
kd_pan: 0.0

kp_tilt: 0.02
ki_tilt: 0.0
kd_tilt: 0.0
```

**Tuning Procedure:**

1. **Proportional (Kp) Only:**
   - Start with Kp = 0.01
   - Increase by 0.01 until servo responds
   - If oscillates, reduce by 50%
   - Target: Quick response, slight overshoot okay

2. **Add Derivative (Kd):**
   - Start with Kd = 0.01
   - Increase until overshoot is eliminated
   - Target: Smooth stop at target, no oscillation

3. **Add Integral (Ki) If Needed:**
   - Only add if there's steady-state error (doesn't reach center)
   - Start with Ki = 0.001
   - Increase slowly (too much causes oscillation)
   - Target: Eventually reaches exact center

**Recommended Final Values:**
```yaml
# Aggressive (fast but may oscillate slightly)
kp_pan: 0.05
ki_pan: 0.001
kd_pan: 0.02

# Conservative (slower but very smooth)
kp_pan: 0.02
ki_pan: 0.0005
kd_pan: 0.01
```

**Tuning Tips:**
- Tune pan and tilt separately (may need different gains)
- Test with and without water pressure (adds inertia)
- Watch for servo strain (grinding noise = gains too high)

### Tracking Performance

**Metrics to Monitor:**
```bash
# Average time to aim at new fire
ros2 topic echo /fire_suppression/status | grep "targeting_time"

# Percentage of time on target
ros2 topic echo /fire_suppression/status | grep "on_target_percent"

# Servo oscillation (should be <±5°)
ros2 topic echo /fire_suppression/servo/pan
```

**Target Performance:**
- Time to aim: <2 seconds
- Steady-state error: <50 pixels (±3°)
- Oscillation: None or <±2°
- On-target time: >80%

---

## Troubleshooting

### Fire Not Detected

**Symptoms:** No bounding boxes, no tracking

**Checks:**
- [ ] Camera working? (`ros2 topic echo /camera/image_raw`)
- [ ] YOLOv8 model loaded? (check logs)
- [ ] Fire in view? (check with rviz2 or debug image)
- [ ] Lighting conditions? (too dark/bright affects detection)
- [ ] Model confidence threshold too high? (lower to 0.5)

**Solutions:**
- Re-train YOLOv8 model with more fire images
- Adjust detection confidence: `detection_confidence: 0.5`
- Add infrared camera for smoke detection

### Servos Not Moving

**Symptoms:** Commands sent but servos don't move

**Checks:**
- [ ] Power supply voltage? (should be 5-6V)
- [ ] Power supply current capability? (need 10A for dual servos)
- [ ] Servo connections correct? (signal, VCC, GND)
- [ ] ESP32 receiving commands? (check serial monitor)
- [ ] DroneCAN messages reaching ESP32? (`candump can0`)

**Solutions:**
- Check power supply under load (voltage may drop)
- Use thicker wires (18 AWG minimum)
- Add capacitors (1000μF) near servos
- Check servo limits in code

### Servos Oscillate

**Symptoms:** Servos vibrate or oscillate around target

**Cause:** PID gains too high (especially Kp)

**Solutions:**
- Reduce Kp by 50%
- Add Kd (derivative term) to dampen
- Check for mechanical binding (adds friction/vibration)
- Increase position history window (more smoothing)

### Water Doesn't Spray

**Symptoms:** Valve command sent but no water flow

**Checks:**
- [ ] Water supply connected?
- [ ] Water pressure adequate? (need 2-4 bar)
- [ ] Valve opening? (listen for click)
- [ ] MOSFET switching? (measure voltage at valve)
- [ ] Hose kinked or blocked?

**Solutions:**
- Check 12V power at solenoid valve
- Test MOSFET with multimeter (Vgs >4V when on)
- Test valve manually (apply 12V directly)
- Check flyback diode orientation

### Water Misses Fire

**Symptoms:** System aims but water doesn't hit fire

**Cause:** Camera-nozzle misalignment

**Solutions:**
- Re-do laser alignment procedure
- Check for mechanical flex under water pressure
- Add offset compensation in software (last resort):
```python
# If water consistently misses to the right by 10cm at 5m:
# That's ~1° offset
offset_pan = -1  # degrees
target_pan = calculated_pan + offset_pan
```

### Tracking Lost

**Symptoms:** Tracker loses fire, servos stop moving

**Cause:** Fire moves too fast, or obscured temporarily

**Solutions:**
- Reduce re-detection interval (30 → 15 frames)
- Switch to KCF tracker (faster than CSRT)
- Add Kalman filter for motion prediction
- Lower detection confidence for re-initialization

---

## Future Enhancements

### 1. Infrared Camera

- Add FLIR Lepton or similar thermal camera
- Detect hot spots invisible to RGB camera
- Works through smoke
- Better range (10+ meters)

### 2. Multi-Target

- Track multiple fires simultaneously
- Prioritize by size, proximity, or intensity
- Spray each fire in sequence

### 3. Depth Estimation

- Add stereo camera or LiDAR
- Estimate distance to fire
- Adjust water pressure based on distance
- Calculate optimal spray angle (ballistic trajectory)

### 4. Autonomous Navigation

- Integrate with ROS2 Nav2
- Robot drives to fire autonomously
- Maintains optimal distance (5-7 meters)
- Avoids obstacles

### 5. Voice Control

- "Robot, extinguish fire at 3 o'clock"
- Whisper STT already integrated
- Add fire suppression commands to vocabulary

### 6. Swarm Coordination

- Multiple robots coordinate
- Surround fire from different angles
- Share fire location via mesh network

---

## References

### Papers & Articles

- **Visual Servoing**: "A Tutorial on Visual Servo Control" - Hutchinson et al. (IEEE 1996)
- **Fire Detection**: "Fire Detection in Video Using Deep Learning" - various recent papers
- **Object Tracking**: OpenCV documentation on tracking algorithms

### Hardware Datasheets

- Sony IMX477: https://www.sony-semicon.com/files/62/pdf/p-15_IMX477-AACK_Flyer.pdf
- DS3218 Servo: https://www.electronicoscaldas.com/datasheet/MG996R_Tower-Pro.pdf
- IRF540N MOSFET: https://www.infineon.com/dgdl/irf540n.pdf?fileId=5546d462533600a40153560b3a9f220d
- WCMCU-230 Transceiver: https://www.waveshare.com/w/upload/e/ed/SN65HVD230.pdf

### Code Examples

- OpenCV Tracking: https://docs.opencv.org/4.x/d9/df8/group__tracking.html
- AccelStepper Library: https://www.airspayce.com/mikem/arduino/AccelStepper/
- YOLOv8: https://docs.ultralytics.com/

---

**Document Status:** Design Complete ✅
**Next Steps:** Begin implementation with electrical testing
**Estimated Development Time:** 2-3 weeks (hardware assembly + software integration)

*Document created: November 13, 2025*
*Author: Claude Code + Human collaboration*
