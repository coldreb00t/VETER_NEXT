#!/bin/bash
# VETER_NEXT - CAN Interface Configuration Script
# Configures SocketCAN interface for DroneCAN communication at 1 Mbps

set -e  # Exit on error

CAN_INTERFACE="can0"
CAN_BITRATE=1000000  # 1 Mbps for DroneCAN
TX_QUEUE_LEN=1000

echo "============================================================"
echo "VETER_NEXT - CAN Interface Configuration"
echo "============================================================"
echo ""

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Error: This script must be run as root (use sudo)"
    exit 1
fi

echo "[1/5] Loading kernel modules..."
modprobe can 2>/dev/null || echo "  - can module already loaded"
modprobe can_raw 2>/dev/null || echo "  - can_raw module already loaded"
modprobe mttcan 2>/dev/null || echo "  - mttcan module already loaded"
echo "✓ Kernel modules loaded"
echo ""

echo "[2/5] Checking if $CAN_INTERFACE exists..."
if ip link show $CAN_INTERFACE &> /dev/null; then
    echo "✓ $CAN_INTERFACE found"
else
    echo "✗ $CAN_INTERFACE not found"
    echo ""
    echo "Available network interfaces:"
    ip link show | grep -E "^[0-9]+:"
    echo ""
    echo "Note: CAN interface may not be available on all Jetson models."
    echo "Check hardware documentation or use USB-CAN adapter."
    exit 1
fi
echo ""

echo "[3/5] Bringing down $CAN_INTERFACE (if up)..."
ip link set $CAN_INTERFACE down 2>/dev/null || true
echo "✓ Interface down"
echo ""

echo "[4/5] Configuring $CAN_INTERFACE..."
echo "  - Bitrate: $CAN_BITRATE bps (1 Mbps)"
echo "  - TX queue length: $TX_QUEUE_LEN"
ip link set $CAN_INTERFACE type can bitrate $CAN_BITRATE
ip link set $CAN_INTERFACE txqueuelen $TX_QUEUE_LEN
echo "✓ Configuration applied"
echo ""

echo "[5/5] Bringing up $CAN_INTERFACE..."
ip link set $CAN_INTERFACE up
echo "✓ Interface up"
echo ""

# Verify configuration
echo "============================================================"
echo "CAN Interface Status:"
echo "============================================================"
ip -details link show $CAN_INTERFACE
echo ""

echo "============================================================"
echo "✓ CAN interface configured successfully!"
echo "============================================================"
echo ""
echo "Test CAN interface:"
echo "  candump $CAN_INTERFACE          # Monitor CAN traffic"
echo "  cansend $CAN_INTERFACE 123#DEADBEEF  # Send test frame"
echo ""
echo "To make this permanent, install systemd service:"
echo "  sudo cp scripts/startup/can-interface.service /etc/systemd/system/"
echo "  sudo systemctl enable can-interface"
echo "  sudo systemctl start can-interface"
echo ""
