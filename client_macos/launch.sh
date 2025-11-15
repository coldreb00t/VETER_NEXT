#!/bin/bash
# VETER Robot Control Client Launcher (ROS2 version)

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
YELLOW='\033[0;33m'
NC='\033[0m' # No Color

echo -e "${BLUE}"
echo "╔════════════════════════════════════════╗"
echo "║   VETER Robot Control Client v2.0     ║"
echo "║        ROS2 DDS Edition                ║"
echo "╚════════════════════════════════════════╝"
echo -e "${NC}"

# Check ROS2 installation
if ! command -v ros2 &> /dev/null; then
    echo -e "${RED}[ERROR]${NC} ROS2 not found!"
    echo -e "${BLUE}[INFO]${NC} Please install ROS2 Humble first:"
    echo ""
    echo "  Option 1 (Homebrew):"
    echo "    brew tap ros/ros2"
    echo "    brew install ros-humble-desktop"
    echo ""
    echo "  Option 2 (Binary):"
    echo "    https://docs.ros.org/en/humble/Installation/macOS-Install-Binary.html"
    echo ""
    echo "  Then source ROS2 environment:"
    echo "    source /opt/ros/humble/setup.bash"
    echo ""
    exit 1
fi

echo -e "${GREEN}[OK]${NC} ROS2 found: $(ros2 --version 2>&1 | head -1)"

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo -e "${BLUE}[INFO]${NC} Virtual environment not found. Creating..."
    python3 -m venv venv

    echo -e "${BLUE}[INFO]${NC} Installing Python dependencies..."
    source venv/bin/activate
    pip install --upgrade pip
    pip install -r requirements.txt
else
    echo -e "${GREEN}[OK]${NC} Virtual environment found"
    source venv/bin/activate
fi

# Check VLC installation
if ! command -v vlc &> /dev/null && [ ! -f "/Applications/VLC.app/Contents/MacOS/VLC" ]; then
    echo -e "${RED}[ERROR]${NC} VLC Media Player not found!"
    echo -e "${BLUE}[INFO]${NC} Install VLC with: brew install --cask vlc"
    exit 1
fi

echo -e "${GREEN}[OK]${NC} VLC Media Player found"

# Check ROS_DOMAIN_ID
if [ -z "$ROS_DOMAIN_ID" ]; then
    echo -e "${YELLOW}[WARN]${NC} ROS_DOMAIN_ID not set, using default: 0"
    export ROS_DOMAIN_ID=0
else
    echo -e "${GREEN}[OK]${NC} ROS_DOMAIN_ID: $ROS_DOMAIN_ID"
fi

# Launch client
echo -e "${BLUE}[INFO]${NC} Launching VETER Control Client (ROS2 DDS)..."
echo ""
python3 veter_control.py

# Cleanup on exit
deactivate
