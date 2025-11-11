#!/bin/bash
# Enable maximum performance mode on Jetson Orin Nano
# This reduces latency by ~10-20ms by maximizing CPU/GPU clocks

echo "========================================"
echo "Enabling Maximum Performance Mode"
echo "========================================"

# Check if running as root
if [ "$EUID" -ne 0 ]; then
    echo "Please run as root (sudo)"
    exit 1
fi

# Set to maximum power mode (mode 0)
echo "Setting power mode to MAXN..."
nvpmodel -m 0

# Enable jetson_clocks to maximize all clocks
echo "Enabling jetson_clocks..."
jetson_clocks

# Show current status
echo ""
echo "========================================"
echo "Performance Mode Status"
echo "========================================"
nvpmodel -q
echo ""
echo "Jetson clocks enabled"
echo "========================================"
echo "System is now in maximum performance mode"
echo "This will increase power consumption and heat"
echo "========================================"
