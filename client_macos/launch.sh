#!/bin/bash
# VETER Robot Control Client Launcher

# Colors
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

echo -e "${BLUE}"
echo "╔════════════════════════════════════════╗"
echo "║   VETER Robot Control Client v1.0     ║"
echo "╚════════════════════════════════════════╝"
echo -e "${NC}"

# Check if virtual environment exists
if [ ! -d "venv" ]; then
    echo -e "${BLUE}[INFO]${NC} Virtual environment not found. Creating..."
    python3 -m venv venv

    echo -e "${BLUE}[INFO]${NC} Installing dependencies..."
    source venv/bin/activate
    pip install --upgrade pip
    pip install -r requirements.txt
else
    echo -e "${GREEN}[OK]${NC} Virtual environment found"
    source venv/bin/activate
fi

# Check VLC installation
if ! command -v vlc &> /dev/null; then
    echo -e "${RED}[ERROR]${NC} VLC Media Player not found!"
    echo -e "${BLUE}[INFO]${NC} Install VLC with: brew install --cask vlc"
    exit 1
fi

echo -e "${GREEN}[OK]${NC} VLC Media Player found"

# Launch client
echo -e "${BLUE}[INFO]${NC} Launching VETER Control Client..."
python3 veter_control.py

# Cleanup on exit
deactivate
