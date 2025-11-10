# ESP32 Motor Controller Firmware

> **See Also**: [DEVELOPMENT_LOG.md](DEVELOPMENT_LOG.md) - Detailed development notes and incremental testing process

## Overview

This firmware runs on an **ESP32-S3-DevKitC-1 v1.0** (N16R8: 16MB Flash + 8MB PSRAM) and serves as the primary motor controller for VETER_NEXT. It receives CRSF commands from an ExpressLRS receiver and converts them to DroneCAN ESC commands for VESC motor controllers.

**DroneCAN Node ID**: 10

**Current Status** (Nov 10, 2025):
- ✅ Base platform stable (ESP32, Serial, SPI, LED)
- ✅ CRSF/ExpressLRS input initialized
- ✅ MCP2515 CAN controller initialized
- ⏳ DroneCAN protocol stack (in development)
- ⏳ Motor control logic (pending)

## Features

- **CRSF/ExpressLRS Input**: Receives RC commands at 420000 baud
- **DroneCAN Output**: Sends motor commands at 1 Mbps via CAN bus
- **Differential Steering**: Mixes throttle and steering into left/right motor commands
- **Emergency Stop**: Hardware button with debouncing (GPIO 23)
- **Failsafe**: Automatic motor stop on signal loss or E-stop
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

CRSF (ExpressLRS):
- RX:    GPIO 16 (Serial1)
- TX:    GPIO 17 (Serial1)

Safety:
- E-STOP: GPIO 23 (INPUT_PULLUP, active LOW)

Status:
- LED:   GPIO 48 (WS2812B RGB LED - onboard on v1.0)
```

**Note**: ESP32-S3-DevKitC-1 v1.0 has RGB LED on GPIO48, v1.1 uses GPIO38.

### ExpressLRS Connection

Connect ExpressLRS receiver TX pin to ESP32 RX pin (GPIO 16).
Baudrate: 420000 bps (8N1)

### MCP2515 CAN Module

SPI-based CAN controller connected to ESP32 SPI pins.
CAN bus bitrate: 1 Mbps (matches DroneCAN specification)

## Building and Flashing

### Prerequisites

- PlatformIO Core installed
- ESP32-S3 board with 16MB flash and 8MB PSRAM
- USB cable for programming

### Build

```bash
cd firmware/esp32_motor_controller
pio run
```

### Upload

```bash
pio run -t upload
```

### Monitor Serial Output

**Via SSH (recommended)**:
```bash
python3 /home/jetson/jetson-robot-project/scripts/esp32_monitor.py
```

**Local terminal**:
```bash
pio device monitor
```

**Note**: `pio device monitor` may not work over SSH with ESP32-S3 USB CDC. Use the Python script instead.

## Configuration

All configuration is in `include/config.h`:

### Pin Definitions
- CAN, CRSF, E-stop, LED pins

### DroneCAN Settings
- Node ID: 10
- Heartbeat interval: 100ms
- ESC command timeout: 1000ms

### CRSF Channel Mapping
- Channel 0: Steering (left/right)
- Channel 1: Throttle (forward/backward)
- Channel 4: Mode switch
- Channel 5: Emergency stop toggle

### Motor Parameters
- Throttle scale: 1.0
- Steering scale: 0.7
- Max differential: 0.8
- Max speed: 100%

### Failsafe
- RC timeout: 1000ms
- Min packets before accepting input: 3

## Operation

### Startup Sequence

1. **Boot**: LED turns BLUE
2. **Initialize**: Serial, pins, LED, CRSF, DroneCAN
3. **Ready**: LED turns GREEN (if RC connected) or YELLOW (no RC)

### Normal Operation

- Receives CRSF packets from ExpressLRS
- Mixes throttle and steering into differential motor commands
- Sends DroneCAN ESC RawCommand messages to VESC controllers
- Sends DroneCAN heartbeat every 100ms
- Updates status LED based on system state

### LED Status Indicators

| Color | State |
|-------|-------|
| BLUE | Booting (during initialization) |
| GREEN (blinking) | Normal operation |
| YELLOW | No RC signal |
| RED | Emergency stop activated |
| MAGENTA | CAN bus error |

**Current Implementation** (Nov 10, 2025):
- 🔴 RED (solid): E-Stop active
- 🟡 YELLOW (solid): No RC signal after initial connection
- 🟢 GREEN (blinking 1 Hz): Normal operation

### Failsafe Behavior

**Triggers:**
- No CRSF signal for >1 second
- Emergency stop button pressed
- CAN bus communication error

**Action:**
- Stop all motors (send zero command)
- Set node health to ERROR or CRITICAL
- Update LED to indicate failsafe state

**Recovery:**
- RC signal restored: Resume normal operation after 3 valid packets
- E-stop released: Resume immediately

## DroneCAN Messages

### Transmitted

**NodeStatus (ID: 341)**
- Sent every 100ms
- Contains: uptime, health, mode, vendor-specific data

**ESC RawCommand (ID: 1030)**
- Sent every 10ms (when not in failsafe)
- Contains: Array of int14 values for left and right motors
- Range: -8191 (full reverse) to +8191 (full forward)

### Received

Currently, this firmware does not process incoming DroneCAN messages.
Future: Parameter configuration, firmware updates, etc.

## Testing

### Test CRSF Input

With ExpressLRS receiver connected and bound to transmitter:

```cpp
#define DEBUG_CRSF 1  // in config.h
```

Monitor serial output to see channel values.

### Test DroneCAN Output

Use `candump` on Jetson to monitor CAN traffic:

```bash
candump can0 | grep 406  # ESC commands
candump can0 | grep 155  # Heartbeat
```

### Test Emergency Stop

Press E-stop button (connect GPIO 23 to GND):
- LED should turn RED (fast blink)
- Motors should stop
- Serial output: "[ESTOP] Emergency stop activated!"

### Test Failsafe

Disconnect ExpressLRS receiver or turn off transmitter:
- After 1 second, LED turns YELLOW (blinking)
- Motors stop
- Serial output: "[FAILSAFE] RC signal lost!"

## Troubleshooting

### No CAN Output

- Check MCP2515 wiring (SPI pins, CS, INT)
- Verify CAN bus termination resistors (120Ω)
- Check serial output for initialization errors

### No CRSF Data

- Verify ExpressLRS receiver is powered
- Check RX/TX pin connection (swap if needed)
- Verify baudrate (420000 for ExpressLRS)
- Ensure receiver is bound to transmitter

### Motors Not Responding

- Check VESC CAN configuration (UAVCAN mode, ESC index 0 and 1)
- Verify CAN bus wiring (CAN_H, CAN_L, GND)
- Monitor CAN traffic with `candump can0`
- Check VESC status in VESC Tool

### LED Always RED

- Check if E-stop button is stuck or shorted
- Try different GPIO pin for E-stop
- Verify INPUT_PULLUP configuration

## Future Enhancements

- [ ] Full CRSF protocol implementation (all 16 channels)
- [ ] CRC verification for CRSF packets
- [ ] DroneCAN parameter server
- [ ] Telemetry transmission to RC
- [ ] OTA firmware updates via DroneCAN
- [ ] Advanced mixing modes (arcade, tank, etc.)
- [ ] Exponential curves for smooth control
- [ ] Rate limiting for acceleration

## License

Part of VETER_NEXT project.

## Authors

- Claude Code (AI Assistant)
- Eugene Melnik

---

*Last Updated: November 10, 2025*

*Board Confirmed: ESP32-S3-DevKitC-1 v1.0 (RGB LED on GPIO48)*
