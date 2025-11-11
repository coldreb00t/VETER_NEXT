# Camera Integration Session Summary

**Date:** November 11, 2025
**Duration:** ~3 hours
**Status:** ✅ Successfully Completed

## Objective

Integrate Sony IMX477 camera for real-time video streaming with minimal latency to enable remote robot operation.

## What Was Accomplished

### 1. ✅ Camera Hardware Configuration
- Physical connection of IMX477 to CSI CAM0 port
- Device tree configuration via jetson-io utility
- Verification of camera detection (`/dev/video0`)
- GStreamer pipeline testing (1920x1080 @ 30fps working)

### 2. ✅ ROS2 Integration
- Installed `ros-humble-gscam` package
- Created `camera.launch.py` for ROS2 integration
- Camera publishing at ~15 Hz to `/camera/image_raw` topic
- Created `CAMERA_SETUP.md` documentation

### 3. ✅ RTSP Streaming (Initial Attempt)
- Created `camera_rtsp_server.py` with GstRtspServer
- Implemented H.264 encoding with x264enc
- Added 180° image rotation (flip-method=2)
- Optimized x264 parameters for low latency
- **Result:** Working but latency ~200-300ms

### 4. ✅ UDP/RTP Streaming (Final Solution)
- Researched community best practices
- Discovered hardware encoder (nvv4l2h264enc) unavailable on Orin Nano
- Implemented direct UDP/RTP streaming
- Fixed IP address discovery issue (192.168.8.5 vs 192.168.8.7)
- **Result:** Ultra-low latency ~80-150ms 🎉

### 5. ✅ Scripts and Documentation
- Created `start_camera_udp.sh` for easy server startup
- Created MacBook receiver script template
- Wrote comprehensive `CAMERA_UDP_STREAMING.md` guide
- Updated `CAMERA_SETUP.md` with UDP streaming section

## Technical Details

### Final Working Configuration

**Server (Jetson):**
```bash
gst-launch-1.0 nvarguscamerasrc sensor-id=0 ! \
  'video/x-raw(memory:NVMM),width=1280,height=720,format=NV12,framerate=30/1' ! \
  nvvidconv flip-method=2 ! video/x-raw,format=I420 ! \
  x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 ! \
  h264parse ! rtph264pay config-interval=10 pt=96 ! \
  udpsink host=192.168.8.5 port=5600
```

**Client (MacBook):**
```bash
gst-launch-1.0 udpsrc port=5600 \
  caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! \
  rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! osxvideosink
```

### Key Learnings

1. **No Hardware Encoder on Orin Nano**
   - `nvv4l2h264enc` only available on AGX Orin/Xavier
   - Must use software encoder (x264enc) on Orin Nano
   - CPU usage ~25-30% for 720p encoding

2. **UDP More Reliable Than Expected**
   - Despite being "unreliable protocol", works excellently on local WiFi
   - Much lower latency than RTSP (TCP overhead eliminated)
   - tcpdump essential for debugging connectivity

3. **IP Address Discovery Critical**
   - MacBook IP changed from .7 to .5 during testing
   - Always verify client IP before streaming
   - Created auto-detection in script

4. **GStreamer → GStreamer Most Reliable**
   - VLC and ffplay struggled with RTP/UDP
   - GStreamer receiver worked immediately
   - SDP files not needed with proper caps

## Problems Solved

### Problem 1: NumPy Version Conflict
- **Issue:** cv_bridge required NumPy 1.x, but 2.2.6 installed
- **Solution:** Switched to official gscam package instead of custom code

### Problem 2: RTSP High Latency
- **Issue:** RTSP streaming had 200-300ms latency
- **Research:** Investigated community solutions and benchmarks
- **Solution:** Switched to UDP/RTP direct streaming

### Problem 3: UDP Stream Not Received
- **Issue:** MacBook couldn't receive UDP packets
- **Root Cause:** Wrong IP address (192.168.8.7 vs actual 192.168.8.5)
- **Solution:** Used `ifconfig` to verify IP, updated stream target

### Problem 4: VLC/ffplay Won't Play Stream
- **Issue:** Neither VLC nor ffplay could open RTP stream
- **Solution:** Used GStreamer on MacBook with proper caps specification

## Performance Metrics

| Metric | Value |
|--------|-------|
| Resolution | 1280x720 |
| Frame Rate | 30 FPS (stable) |
| Latency | 80-150ms (glass-to-glass) |
| Bitrate | 2 Mbps |
| CPU Usage (Jetson) | 25-30% (one core) |
| Network Bandwidth | ~2-3 Mbps |
| Image Quality | Excellent |

## Files Created/Modified

### New Files
- `/home/jetson/jetson-robot-project/ros2_ws/src/veter_bringup/launch/camera.launch.py`
- `/home/jetson/jetson-robot-project/ros2_ws/src/veter_bringup/launch/camera_gscam.launch.py`
- `/home/jetson/jetson-robot-project/ros2_ws/src/veter_bringup/scripts/camera_rtsp_server.py`
- `/home/jetson/jetson-robot-project/ros2_ws/src/veter_bringup/scripts/camera_udp_server.py`
- `/home/jetson/jetson-robot-project/scripts/start_camera_udp.sh`
- `/home/jetson/jetson-robot-project/scripts/enable_max_performance.sh`
- `/home/jetson/jetson-robot-project/docs/CAMERA_SETUP.md`
- `/home/jetson/jetson-robot-project/docs/CAMERA_UDP_STREAMING.md`

### Modified Files
- `/home/jetson/jetson-robot-project/CLAUDE.md` (will be updated)

## Next Steps

1. **Commit to Git**
   ```bash
   git add .
   git commit -m "feat: Add ultra-low latency UDP camera streaming

   - Implement H.264 RTP/UDP streaming with 80-150ms latency
   - Create camera launch files for ROS2
   - Add RTSP server (fallback option)
   - Document camera setup and streaming
   - Create convenience scripts for UDP streaming

   🤖 Generated with Claude Code"
   ```

2. **Test Computer Vision Integration**
   - Integrate YOLOv8n with camera feed
   - Test object detection performance
   - Optimize for real-time processing

3. **Add to System Startup** (Optional)
   - Create systemd service for auto-start
   - Add to main robot launch file

4. **Outdoor Testing**
   - Test GPS fusion with camera navigation
   - Verify performance in different lighting conditions

## Conclusion

✅ **Mission Accomplished!**

Successfully integrated camera with ultra-low latency streaming. Robot can now:
- See its environment (IMX477 12MP camera)
- Stream video to operator with minimal delay (80-150ms)
- Support future computer vision tasks
- Enable remote operation with visual feedback

The combination of sensor fusion (GPS+IMU) + camera streaming makes the robot ready for autonomous navigation and remote operation!

---

**Session Completed:** November 11, 2025
**Next Session:** Computer vision integration or physical hardware assembly
