# VETER_NEXT - Installation Guide

## System Check Results

**Date**: November 9, 2025
**Platform**: NVIDIA Jetson Orin Nano Super Developer Kit
**OS**: Ubuntu 22.04 (Jammy)
**Kernel**: Linux 5.15.148-tegra

### System Resources
- **Disk Space**: 467GB total, 425GB available (91% free)
- **RAM**: 7.4GB total, 6.1GB available
- **Swap**: 3.7GB

### Current Software Status

#### ✗ Missing Components
- **ROS2 Humble**: Not installed
- **pip3**: Not installed
- **PlatformIO**: Not installed
- **can-utils**: Not installed
- **Python Libraries**:
  - pydronecan
  - ultralytics
  - transformers
  - python-telegram-bot

#### ✓ Installed Components
- **Python**: 3.10.12

## Installation Plan

### Phase 1: System Prerequisites
1. Update package lists
2. Install pip3
3. Install build tools and dependencies

### Phase 2: ROS2 Humble
1. Add ROS2 repository
2. Install ROS2 Humble Desktop
3. Install ROS2 development tools
4. Configure environment

### Phase 3: CAN Bus Support
1. Install can-utils
2. Load kernel modules
3. Configure CAN interface
4. Create systemd service for auto-start

### Phase 4: PlatformIO
1. Install PlatformIO Core
2. Configure udev rules for ESP32
3. Test ESP32 connectivity

### Phase 5: Python Libraries
1. Install pydronecan for DroneCAN support
2. Install ultralytics for YOLO
3. Install transformers for AI models
4. Install python-telegram-bot
5. Install additional dependencies

---

## Detailed Installation Steps

### Prerequisites Installation

```bash
# Update package lists
sudo apt update

# Install pip3
sudo apt install -y python3-pip

# Install build tools
sudo apt install -y \
    build-essential \
    cmake \
    git \
    wget \
    curl \
    software-properties-common
```

### ROS2 Humble Installation

```bash
# Add ROS2 repository
sudo apt install -y software-properties-common
sudo add-apt-repository universe
sudo apt update

# Add ROS2 GPG key
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Update and install
sudo apt update
sudo apt install -y ros-humble-desktop

# Install development tools
sudo apt install -y \
    ros-humble-ros-base \
    ros-humble-navigation2 \
    ros-humble-teleop-twist-keyboard \
    python3-colcon-common-extensions \
    python3-rosdep

# Initialize rosdep
sudo rosdep init
rosdep update

# Add to bashrc
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### CAN Bus Support Installation

```bash
# Install can-utils
sudo apt install -y can-utils

# Load kernel modules
sudo modprobe can
sudo modprobe can_raw
sudo modprobe mttcan

# Configure CAN interface (1 Mbps for DroneCAN)
sudo ip link set can0 type can bitrate 1000000
sudo ip link set up can0
sudo ip link set can0 txqueuelen 1000
```

### PlatformIO Installation

```bash
# Install PlatformIO Core
pip3 install --user platformio

# Add to PATH
echo 'export PATH=$PATH:~/.local/bin' >> ~/.bashrc
source ~/.bashrc

# Install udev rules for USB devices
curl -fsSL https://raw.githubusercontent.com/platformio/platformio-core/master/platformio/assets/system/99-platformio-udev.rules | sudo tee /etc/udev/rules.d/99-platformio-udev.rules

# Reload udev rules
sudo udevadm control --reload-rules
sudo udevadm trigger

# Add user to dialout group (for USB serial access)
sudo usermod -a -G dialout $USER
```

### Python Libraries Installation

```bash
# Install DroneCAN support
pip3 install --user pydronecan

# Install computer vision libraries
pip3 install --user ultralytics

# Install AI/ML libraries
pip3 install --user transformers
pip3 install --user torch torchvision torchaudio

# Install communication libraries
pip3 install --user python-telegram-bot

# Install additional utilities
pip3 install --user \
    pyserial \
    numpy \
    opencv-python \
    pyyaml \
    pytest
```

---

## Post-Installation Verification

### Verify ROS2
```bash
ros2 --version
ros2 topic list
```

### Verify CAN Interface
```bash
ip link show can0
candump can0
```

### Verify PlatformIO
```bash
pio --version
pio system info
```

### Verify Python Libraries
```bash
python3 -c "import dronecan; print('pydronecan:', dronecan.__version__)"
python3 -c "import ultralytics; print('ultralytics OK')"
python3 -c "import transformers; print('transformers OK')"
python3 -c "import telegram; print('python-telegram-bot OK')"
```

---

## Known Issues and Solutions

### Issue: ROS2 Package Not Found
**Solution**: Make sure universe repository is enabled
```bash
sudo add-apt-repository universe
sudo apt update
```

### Issue: CAN Interface Not Found
**Solution**: Check if mttcan module is loaded
```bash
lsmod | grep can
sudo modprobe mttcan
```

### Issue: PlatformIO Permission Denied
**Solution**: Add user to dialout group and relogin
```bash
sudo usermod -a -G dialout $USER
# Then logout and login again
```

### Issue: Python Module Import Error
**Solution**: Check if user site-packages is in PATH
```bash
python3 -m site --user-site
# Add to ~/.bashrc if needed:
export PYTHONPATH=$PYTHONPATH:$(python3 -m site --user-site)
```

---

## Installation Log

All installation steps are logged in: `docs/installation_log.txt`

To review the log:
```bash
cat docs/installation_log.txt
```

---

## Next Steps After Installation

1. **Configure CAN interface** - Run setup script
   ```bash
   sudo bash scripts/setup/configure_can.sh
   ```

2. **Build ROS2 workspace**
   ```bash
   cd ros2_ws
   colcon build --symlink-install
   source install/setup.bash
   ```

3. **Flash ESP32 firmware**
   ```bash
   cd firmware/esp32_motor_controller
   pio run -t upload
   ```

4. **Test DroneCAN communication**
   ```bash
   python3 scripts/diagnostics/can_monitor.py
   ```

---

*Last Updated: November 9, 2025*
*Status: Initial setup - awaiting installation*
