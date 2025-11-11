# VETER Camera Setup Guide

## Overview

Sony IMX477 12MP camera connected to Jetson Orin Nano CSI CAM0 port.

**Date:** November 11, 2025
**Status:** ✅ Operational

## Hardware

- **Camera:** Sony IMX477 12MP
- **Connection:** MIPI CSI to CAM0 (closest to heatsink)
- **Supported Modes:**
  - 3840 x 2160 @ 30 fps (4K)
  - 1920 x 1080 @ 60 fps (Full HD)

## Software Stack

- **GStreamer Plugin:** nvarguscamerasrc (NVIDIA Argus API)
- **ROS2 Integration:** gscam (ros-humble-gscam)
- **Format:** RGB8 (1920x1080)
- **Publishing Rate:** ~15 Hz

## Installation

### 1. Configure Camera in Device Tree

```bash
# Run jetson-io utility
sudo /opt/nvidia/jetson-io/jetson-io.py

# Select: Configure for compatible hardware → IMX477 Camera
# Save and reboot
sudo reboot
```

### 2. Verify Camera Detection

```bash
# Check video device
ls -la /dev/video0

# Test with GStreamer
gst-launch-1.0 nvarguscamerasrc sensor-id=0 num-buffers=30 ! \
  'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=30/1' ! \
  nvvidconv ! fakesink
```

### 3. Install ROS2 gscam

```bash
sudo apt update
sudo apt install -y ros-humble-gscam
```

## Usage

### Launch Camera Node

```bash
# Source ROS2 environment
cd /home/jetson/jetson-robot-project/ros2_ws
source /opt/ros/humble/setup.bash
source install/setup.bash

# Launch camera
ros2 launch veter_bringup camera.launch.py
```

### Check Topics

```bash
# List camera topics
ros2 topic list | grep camera
# Output:
# /camera/camera_info
# /camera/image_raw

# Check publishing rate
ros2 topic hz /camera/image_raw
# Output: ~15 Hz

# View image info
ros2 topic echo /camera/image_raw --once
```

### Manual Launch (without launch file)

```bash
# Set GStreamer pipeline
export GSCAM_CONFIG="nvarguscamerasrc sensor-id=0 ! \
  video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12,framerate=30/1 ! \
  nvvidconv ! video/x-raw,format=BGRx ! videoconvert"

# Run gscam node
ros2 run gscam gscam_node
```

## Configuration

### Change Resolution/Framerate

Edit `/home/jetson/jetson-robot-project/ros2_ws/src/veter_bringup/launch/camera.launch.py`:

```python
# For 4K @ 30fps
gstreamer_pipeline = (
    'nvarguscamerasrc sensor-id=0 ! '
    'video/x-raw(memory:NVMM),width=3840,height=2160,format=NV12,framerate=30/1 ! '
    'nvvidconv ! video/x-raw,format=BGRx ! videoconvert'
)

# For 1080p @ 60fps
gstreamer_pipeline = (
    'nvarguscamerasrc sensor-id=0 ! '
    'video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12,framerate=60/1 ! '
    'nvvidconv ! video/x-raw,format=BGRx ! videoconvert'
)
```

Then rebuild:
```bash
colcon build --packages-select veter_bringup --symlink-install
```

### Camera Calibration

Create calibration file (optional):
```bash
mkdir -p ~/.ros/camera_info
# Add your calibration YAML here
nano ~/.ros/camera_info/camera.yaml
```

## Troubleshooting

### Camera Not Detected

```bash
# Check if camera is physically detected
ls -la /dev/video0

# If not found, re-run jetson-io and reboot
sudo /opt/nvidia/jetson-io/jetson-io.py
sudo reboot
```

### "Failed to create CaptureSession" Error

Camera is already in use by another process:
```bash
# Kill all camera processes
pkill -9 gscam_node
pkill -9 nvarguscamerasrc

# Wait and restart
sleep 2
ros2 launch veter_bringup camera.launch.py
```

### Low Frame Rate

- Default: ~15 Hz (sufficient for most applications)
- GStreamer captures @ 30fps but ROS2 publishing overhead reduces rate
- For higher rates, consider:
  - Reducing resolution
  - Using compressed image transport
  - Increasing system resources

## Integration Examples

### With Nav2 (for obstacle detection)

```bash
# Launch camera + Nav2
ros2 launch veter_bringup camera.launch.py &
ros2 launch nav2_bringup navigation_launch.py
```

### View in RViz2

```bash
# Launch camera
ros2 launch veter_bringup camera.launch.py &

# Launch RViz2
rviz2

# Add: Displays → By topic → /camera/image_raw → Image
```

### Record Video

```bash
# Record to ROS2 bag
ros2 bag record /camera/image_raw /camera/camera_info

# Play back
ros2 bag play <bag_file>
```

## Performance

- **Resolution:** 1920x1080 (2.1 MP)
- **Format:** RGB8 (24-bit color)
- **Data Rate:** ~15 Hz × 1920 × 1080 × 3 bytes ≈ 93 MB/s
- **Latency:** < 100ms (camera to ROS2 topic)
- **CPU Usage:** ~15-20% (one core)

## Next Steps

1. **Web Streaming:** Add RTSP/WebRTC for remote viewing
2. **Computer Vision:** Integrate YOLOv8n for object detection
3. **Calibration:** Perform camera calibration for accurate measurements
4. **Compression:** Use image_transport_plugins for bandwidth optimization

## References

- [gscam ROS2 Documentation](http://wiki.ros.org/gscam)
- [NVIDIA Argus Camera API](https://docs.nvidia.com/jetson/l4t-multimedia/group__LibargusAPI.html)
- [IMX477 Datasheet](https://www.sony-semicon.com/en/products/is/industry/img_sensor.html)

---

**Last Updated:** November 11, 2025
**Maintainer:** Eugene Melnik (eugene.a.melnik@gmail.com)
