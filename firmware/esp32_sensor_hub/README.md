# ESP32 Sensor Hub Firmware

## Overview

This firmware runs on an ESP32-S3 module and serves as the sensor and actuator controller for VETER_NEXT. It manages all environmental sensors, collision avoidance, camera control, and lighting, publishing sensor data via DroneCAN.

**DroneCAN Node ID**: 11

## Features

- **4x HC-SR04 Ultrasonic Sensors**: Collision avoidance (front, rear, left, right)
- **BME280 Environmental Sensor**: Temperature, humidity, and atmospheric pressure (I2C)
- **Camera Servo Control**: Pan/tilt servos for camera positioning (PWM)
- **LED Lighting**: 4-channel PWM lighting control (front/rear left/right)
- **DroneCAN Communication**: Sensor data publishing and command reception at 1 Mbps
- **Collision Detection**: Real-time obstacle detection with warning/stop thresholds
- **Emergency Stop**: Hardware button integration
- **Status LED**: WS2812B LED indicates system state
- **Heartbeat**: DroneCAN heartbeat every 100ms

## Hardware Connections

### ESP32-S3 Pinout

```
CAN Bus (MCP2515):
- CS:    GPIO 5
- INT:   GPIO 4
- MOSI:  GPIO 11 (SPI)
- MISO:  GPIO 13 (SPI)
- SCK:   GPIO 12 (SPI)

Ultrasonic Sensors (HC-SR04):
Front:
- TRIG:  GPIO 14
- ECHO:  GPIO 15

Rear:
- TRIG:  GPIO 16
- ECHO:  GPIO 17

Left:
- TRIG:  GPIO 18
- ECHO:  GPIO 21

Right:
- TRIG:  GPIO 47
- ECHO:  GPIO 48

BME280 (I2C):
- SDA:   GPIO 8
- SCL:   GPIO 9
- ADDR:  0x76 (or 0x77)

Camera Servos (PWM 50Hz):
- PAN:   GPIO 38 (horizontal rotation)
- TILT:  GPIO 39 (vertical tilt)

LED Lighting (PWM 5kHz):
- FL:    GPIO 40 (front left)
- FR:    GPIO 41 (front right)
- RL:    GPIO 42 (rear left)
- RR:    GPIO 45 (rear right)

Safety:
- E-STOP: GPIO 23 (INPUT_PULLUP, active LOW)

Status:
- LED:   GPIO 19 (WS2812B data pin)
```

### HC-SR04 Ultrasonic Sensors

Connect HC-SR04 modules to ESP32 for collision avoidance:
- VCC: 5V (use level shifter for ECHO if needed)
- GND: Ground
- TRIG: GPIO output pins
- ECHO: GPIO input pins (5V tolerant or use voltage divider)

Maximum range: 400cm
Minimum range: 2cm

### BME280 Sensor

I2C environmental sensor for temperature, humidity, and pressure:
- VCC: 3.3V
- GND: Ground
- SDA/SCL: I2C bus (GPIO 8/9)
- I2C address: 0x76 (default) or 0x77

### Camera Servos

Standard hobby servos (50Hz PWM):
- VCC: 5V (external power recommended for high-torque servos)
- GND: Ground (common with ESP32)
- Signal: PWM pins (GPIO 38/39)

Servo range: 0-180 degrees
Center position: 90 degrees

### LED Lighting

PWM-controlled LED strips or modules:
- 4 independent channels (FL, FR, RL, RR)
- PWM frequency: 5 kHz
- 8-bit resolution (0-255 brightness)
- Use MOSFETs for high-power LEDs

## Building and Flashing

### Prerequisites

- PlatformIO Core installed
- ESP32-S3 board with 16MB flash and 8MB PSRAM
- USB cable for programming

### Build

```bash
cd firmware/esp32_sensor_hub
pio run
```

### Upload

```bash
pio run -t upload
```

### Monitor Serial Output

```bash
pio device monitor
```

## Configuration

All configuration is in `include/config.h`:

### Pin Definitions
- CAN, ultrasonic, BME280, servo, LED, E-stop pins

### DroneCAN Settings
- Node ID: 11
- Heartbeat interval: 100ms
- Sensor publish rate: 100ms (10 Hz)

### Ultrasonic Sensor Parameters
- Max distance: 400cm
- Min distance: 2cm
- Warning threshold: 50cm
- Stop threshold: 20cm
- Update rate: 10 Hz

### BME280 Parameters
- Sampling: 16x oversampling (temp, pressure, humidity)
- Filter: 16x IIR filter
- Standby: 500ms
- Read interval: 1 second

### Servo Parameters
- PWM frequency: 50 Hz
- Min pulse width: 500 µs
- Max pulse width: 2500 µs
- Pan range: 0-180°, center: 90°
- Tilt range: 30-150°, center: 90°

### LED Parameters
- PWM frequency: 5 kHz
- Resolution: 8-bit (0-255)
- Default mode: Auto
- Default brightness: 128

## Operation

### Startup Sequence

1. **Boot**: LED turns BLUE
2. **Initialize**: Serial, pins, LED, DroneCAN
3. **Sensor Init**: BME280, ultrasonic sensors, servos, LED lighting
4. **Ready**: LED turns GREEN

### Normal Operation

- Reads ultrasonic sensors at 10 Hz
- Reads BME280 sensor at 1 Hz
- Publishes sensor data via DroneCAN at 10 Hz
- Receives servo/LED commands from DroneCAN
- Detects collisions and publishes warnings
- Sends DroneCAN heartbeat every 100ms
- Updates status LED based on system state

### LED Status Indicators

| Color | State |
|-------|-------|
| BLUE | Booting |
| GREEN (solid) | Normal operation |
| YELLOW (blinking) | No CAN signal |
| RED (fast blink) | Emergency stop activated |
| PURPLE (fast blink) | Collision warning |
| ORANGE (blinking) | Sensor error |
| MAGENTA | CAN bus error |

### Collision Detection

**Warning Levels:**
- **Info**: Distance > 50cm (no action)
- **Warning**: Distance 20-50cm (publish warning)
- **Critical**: Distance < 20cm (publish stop command, set COLLISION failsafe)

**Actions:**
- Publish `CollisionWarning` DroneCAN message with direction and severity
- Set failsafe mode to FAILSAFE_COLLISION
- Update status LED to PURPLE (fast blink)

**Recovery:**
- Obstacle cleared (distance > 50cm): Clear failsafe, resume normal operation

## DroneCAN Messages

### Transmitted

**NodeStatus (ID: 341 / 0x155)**
- Sent every 100ms
- Contains: uptime, health, mode, vendor-specific data

**RangeSensor (ID: 1050 / 0x41A)**
- Sent every 100ms
- Contains: 4x ultrasonic distances (mm, 16-bit each)

**AirData (ID: 1060 / 0x424)**
- Sent every 100ms (when BME280 available)
- Contains: temperature (0.01°C), humidity (0.01%), pressure (0.01 hPa)

**CollisionWarning (ID: 1070 / 0x42E)**
- Sent on collision detection
- Contains: direction (0-3), distance (mm), severity (0-2)

### Received

**ServoCommand (ID: 1152 / 0x480)**
- Camera servo control
- Contains: pan angle (deg), tilt angle (deg)

**LEDCommand (ID: 1168 / 0x490)**
- LED lighting control
- Contains: mode, brightness, individual channel values

## Testing

### Test Ultrasonic Sensors

Enable debug output in `config.h`:

```cpp
#define DEBUG_SONAR 1
```

Monitor serial output to see distance readings for all 4 sensors.

### Test BME280 Sensor

Enable debug output:

```cpp
#define DEBUG_BME280 1
```

Monitor serial output to see temperature, humidity, and pressure readings.

### Test DroneCAN Output

Use `candump` on Jetson to monitor CAN traffic:

```bash
candump can0 | grep 41A  # RangeSensor
candump can0 | grep 424  # AirData
candump can0 | grep 42E  # CollisionWarning
candump can0 | grep 155  # Heartbeat
```

### Test Collision Detection

Place obstacle in front of sensor:
- 50-20cm: LED turns PURPLE (blinking), warning published
- <20cm: LED turns PURPLE (fast blink), critical warning published

### Test Servo Control

Send DroneCAN servo command (from Jetson or another node):
- Servos should move to commanded positions
- Servos return to center after timeout (1 second)

### Test LED Lighting

Send DroneCAN LED command:
- LEDs should respond to mode and brightness settings
- Test modes: OFF, AUTO, ON, BLINK

## Troubleshooting

### No CAN Output

- Check MCP2515 wiring (SPI pins, CS, INT)
- Verify CAN bus termination resistors (120Ω)
- Check serial output for initialization errors

### Ultrasonic Sensors Not Reading

- Verify VCC (5V) and GND connections
- Check TRIG/ECHO pin connections
- Ensure sensors are not blocked or damaged
- Try increasing timeout or max distance

### BME280 Not Found

- Check I2C wiring (SDA, SCL, VCC, GND)
- Verify I2C address (0x76 or 0x77)
- Use I2C scanner to detect sensor
- Check for conflicts with other I2C devices

### Servos Not Moving

- Check servo power (5V, sufficient current)
- Verify signal pin connections
- Ensure servo PWM frequency is 50Hz
- Test servos with known-good controller

### LEDs Not Working

- Verify PWM pin connections
- Check LED power supply
- Ensure MOSFETs (if used) are correctly wired
- Test with different brightness values

### LED Always RED

- Check if E-stop button is stuck or shorted
- Try different GPIO pin for E-stop
- Verify INPUT_PULLUP configuration

## Future Enhancements

- [ ] IMU integration (accelerometer, gyroscope, magnetometer)
- [ ] GPS module for outdoor navigation
- [ ] Ambient light sensor for automatic LED brightness
- [ ] Additional ultrasonic/IR sensors for better coverage
- [ ] Camera image processing on ESP32
- [ ] DroneCAN parameter server
- [ ] OTA firmware updates via DroneCAN
- [ ] Battery voltage monitoring
- [ ] Temperature-based fan control

## License

Part of VETER_NEXT project.

## Authors

- Claude Code (AI Assistant)
- Eugene Melnik

---

*Last Updated: November 2025*
