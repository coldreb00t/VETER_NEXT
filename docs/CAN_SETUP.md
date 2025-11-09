# CAN Interface Setup Guide

## Overview

VETER_NEXT uses SocketCAN interface for DroneCAN communication between the Jetson and all peripheral devices (VESC motor controllers, ESP32 nodes, ArduRover).

**CAN Bus Specification:**
- **Protocol**: DroneCAN (formerly UAVCAN)
- **Bitrate**: 1 Mbps (1,000,000 bps)
- **Interface**: can0 (native Jetson CAN or USB-CAN adapter)
- **Termination**: 120Ω resistors at each end of the bus

## Hardware Setup

### CAN Bus Wiring

```
[Jetson can0] ----CAN_H----+----+----+----+---- [120Ω termination]
                            |    |    |    |
                     [VESC] [VESC] [ESP32] [ArduRover]
                            |    |    |    |
[Jetson can0] ----CAN_L----+----+----+----+---- [120Ω termination]
```

### Pin Connections

**Jetson Orin Nano CAN pins** (verify in your hardware docs):
- CAN0_TX: Pin X (check schematic)
- CAN0_RX: Pin Y (check schematic)
- GND: Common ground

**Alternative: USB-CAN Adapter**
If native CAN is not available:
- PEAK-System PCAN-USB
- Kvaser Leaf Light
- CANable USB adapter

## Software Configuration

### Manual Configuration

Run the configuration script:
```bash
sudo bash scripts/setup/configure_can.sh
```

This script will:
1. Load kernel modules (can, can_raw, mttcan)
2. Configure can0 at 1 Mbps
3. Set TX queue length to 1000
4. Bring up the interface

### Verify Configuration

```bash
# Check interface status
ip -details link show can0

# Should show:
# can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP
#     can state ERROR-ACTIVE restart-ms 0
#     bitrate 1000000 sample-point 0.875
```

### Automatic Start on Boot

Install systemd service:
```bash
sudo cp scripts/startup/can-interface.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable can-interface
sudo systemctl start can-interface
```

Check service status:
```bash
sudo systemctl status can-interface
```

## Testing CAN Interface

### Monitor CAN Traffic

```bash
# Monitor all CAN messages
candump can0

# Monitor with timestamps
candump can0 -t a

# Filter specific CAN ID (e.g., 341 for DroneCAN heartbeat)
candump can0 | grep "341"
```

### Send Test Message

```bash
# Send a test frame (ID: 123, Data: DEADBEEF)
cansend can0 123#DEADBEEF
```

### Using Python (pydronecan)

```python
import dronecan

# Connect to CAN interface
node = dronecan.make_node('can0', node_id=100, bitrate=1000000)

# Monitor heartbeat messages
def heartbeat_callback(event):
    print(f"Heartbeat from node {event.transfer.source_node_id}")

node.add_handler(dronecan.uavcan.protocol.NodeStatus, heartbeat_callback)

# Spin forever
node.spin()
```

## DroneCAN Node Configuration

### Expected Nodes on Bus

| Node ID | Device | Function |
|---------|--------|----------|
| 1 | VESC Left Motor | ESC for left track |
| 2 | VESC Right Motor | ESC for right track |
| 10 | ESP32 Motor Controller | CRSF receiver & motor commands |
| 11 | ESP32 Sensor Hub | Sensors & peripherals |
| 50 | ArduRover | GPS navigation & IMU |
| 100 | Jetson (DroneCAN Bridge) | High-level control |

### VESC Configuration

In VESC Tool, configure:
```
App Configuration > General
  App to use: No App

CAN Configuration
  CAN Mode: UAVCAN
  VESC ID: 1 (left motor) or 2 (right motor)
  UAVCAN ESC Index: 0 (left) or 1 (right)
  CAN Baud Rate: CAN_BAUD_1M
  CAN Status Message Mode: CAN_STATUS_1_2_3_4_5
```

### ArduRover Parameters

```
CAN_P1_DRIVER = 1
CAN_D1_PROTOCOL = 1  # DroneCAN
CAN_D1_UC_ESC_BM = 3  # Enable ESC 0 and 1
```

## Troubleshooting

### Interface Not Found

**Problem:** `can0` interface doesn't exist

**Solutions:**
1. Check kernel modules: `lsmod | grep can`
2. Load modules manually:
   ```bash
   sudo modprobe can
   sudo modprobe can_raw
   sudo modprobe mttcan
   ```
3. Verify hardware: `dmesg | grep can`
4. Consider USB-CAN adapter if native CAN unavailable

### No CAN Traffic

**Problem:** `candump can0` shows nothing

**Checks:**
1. Verify termination resistors (120Ω at each end)
2. Check CAN_H and CAN_L connections
3. Verify all nodes are powered
4. Check bitrate matches (1 Mbps)
5. Use oscilloscope to verify signals

### Bus Errors

**Problem:** Interface shows `ERROR-PASSIVE` or `BUS-OFF`

**Solutions:**
1. Check termination resistors
2. Verify cable quality and length (<40m for 1 Mbps)
3. Check for shorts between CAN_H and CAN_L
4. Restart interface:
   ```bash
   sudo ip link set can0 down
   sudo ip link set can0 up
   ```

### DroneCAN Nodes Not Visible

**Problem:** No heartbeat messages from expected nodes

**Checks:**
1. Verify node is powered and boots correctly
2. Check node configuration (CAN bitrate, node ID)
3. Monitor raw CAN traffic: `candump can0 -x`
4. Use DroneCAN GUI tool:
   ```bash
   dronecan_gui_tool can0
   ```

## Performance Tuning

### Increase RX Buffer

For high traffic loads:
```bash
sudo ifconfig can0 txqueuelen 10000
```

### Enable CAN Error Frames

```bash
ip link set can0 type can berr-reporting on
```

### Monitor Bus Load

```python
import dronecan
import time

node = dronecan.make_node('can0', node_id=100)

def monitor_load():
    stats = node.can_driver.get_stats()
    print(f"RX: {stats.rx_count}, TX: {stats.tx_count}, Errors: {stats.errors}")

while True:
    monitor_load()
    time.sleep(1)
```

## Safety Considerations

1. **Always use termination resistors** - prevents signal reflections
2. **Maximum cable length** - 40m at 1 Mbps, longer at lower rates
3. **EMI protection** - use shielded twisted pair cables
4. **Power isolation** - CAN bus doesn't carry power
5. **Ground loops** - ensure common ground between all devices

## References

- [DroneCAN Specification](https://dronecan.github.io/)
- [SocketCAN Documentation](https://www.kernel.org/doc/html/latest/networking/can.html)
- [VESC CAN Configuration](https://vesc-project.com/node/234)
- [ArduPilot DroneCAN](https://ardupilot.org/copter/docs/common-uavcan-setup-advanced.html)

---

*Last Updated: November 2025*
