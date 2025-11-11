# Global Camera Streaming via VPS

**Date:** November 11, 2025
**Status:** ✅ Working
**Access:** From anywhere in the world via RTSP

## Overview

Camera streaming accessible globally through VPS reverse SSH tunnel. Robot can use any communication channel (Starlink, 4G, WiFi) and video will be accessible via stable VPS endpoint.

## Architecture

```
Jetson (Robot)              VPS (81.200.157.230)         Your Device
    ↓                              ↓                          ↓
IMX477 Camera          SSH Reverse Tunnel           GStreamer/VLC
    ↓                              ↓                          ↓
RTSP Server            Port Forwarding              RTSP Client
(localhost:8554)  →    (public:8554)         ←     (anywhere)
```

## Key Features

- ✅ **Global Access**: View from anywhere (not limited to local network)
- ✅ **Multi-Channel**: Works over Starlink, 4G, WiFi, fiber
- ✅ **Persistent**: VPS provides stable endpoint even when robot changes networks
- ✅ **Auto-Reconnect**: SSH tunnel automatically reconnects if connection drops
- ✅ **Auto-Start**: Both RTSP server and SSH tunnel start on boot

## Quick Start

### On Jetson (Robot)

Services start automatically on boot. To check status:

```bash
./scripts/start_camera_global.sh
```

### On Remote Device (Anywhere in the World)

**Stream URL:**
```
rtsp://81.200.157.230:8554/camera
```

**View with GStreamer (recommended):**
```bash
gst-launch-1.0 rtspsrc location=rtsp://81.200.157.230:8554/camera latency=200 ! \
  rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink
```

**View with VLC:**
```bash
vlc rtsp://81.200.157.230:8554/camera
```

**View with FFplay:**
```bash
ffplay -rtsp_transport tcp rtsp://81.200.157.230:8554/camera
```

## Performance

| Metric | Value |
|--------|-------|
| Resolution | 1280x720 |
| Frame Rate | 30 FPS |
| Latency | 200-400ms (depends on internet connection) |
| Bitrate | ~2.5 Mbps |
| Protocol | RTSP over TCP (reliable) |
| Bandwidth | ~3-4 Mbps including overhead |

## System Services

### RTSP Server Service

**Service:** `camera-rtsp.service`
**Status:** Auto-starts on boot
**Port:** localhost:8554

```bash
# Check status
sudo systemctl status camera-rtsp.service

# Restart
sudo systemctl restart camera-rtsp.service

# View logs
sudo journalctl -u camera-rtsp.service -f
```

### SSH Tunnel Service

**Service:** `ssh-tunnel-jetson-robot.service`
**Status:** Auto-starts on boot
**Forwards:**
- Port 2223 → SSH access
- Port 8554 → RTSP camera stream

```bash
# Check status
sudo systemctl status ssh-tunnel-jetson-robot.service

# Restart
sudo systemctl restart ssh-tunnel-jetson-robot.service

# View logs
sudo journalctl -u ssh-tunnel-jetson-robot.service -f
```

## VPS Configuration

**VPS:** 81.200.157.230
**OS:** Ubuntu/Debian (assumed)
**Firewall:** ufw

### Ports Open

- **2223/tcp** - SSH access to Jetson
- **8554/tcp** - RTSP camera stream

### SSH Tunnel

Reverse tunnel from Jetson:
```
-R 2223:localhost:22     # SSH access
-R 8554:localhost:8554   # RTSP stream
```

## Troubleshooting

### No Video Stream

1. **Check RTSP server on Jetson:**
   ```bash
   sudo systemctl status camera-rtsp.service
   ss -tlnp | grep 8554
   ```

2. **Check SSH tunnel:**
   ```bash
   sudo systemctl status ssh-tunnel-jetson-robot.service
   ```

3. **Check VPS port forwarding:**
   ```bash
   ssh root@81.200.157.230 "ss -tlnp | grep 8554"
   ```

4. **Check VPS firewall:**
   ```bash
   ssh root@81.200.157.230 "ufw status | grep 8554"
   ```

### High Latency

- **Expected:** 200-400ms over internet
- **If >500ms:**
  - Check robot's internet connection quality
  - Try lower resolution (640x480)
  - Check VPS server load

### Stream Freezes

- **RTSP over TCP** is reliable but can freeze on poor connections
- Solution: Wait for buffering or restart stream
- For unreliable connections: Consider SRT protocol (see below)

## Alternative: SRT for Unreliable Networks

For better performance over unstable connections (moving vehicle, poor 4G):

### Server (Jetson):
```bash
gst-launch-1.0 nvarguscamerasrc ! ... ! x264enc ! mpegtsmux ! \
  srtsink uri=srt://81.200.157.230:9000
```

### Client (Remote):
```bash
gst-launch-1.0 srtsrc uri=srt://81.200.157.230:9000 ! tsdemux ! \
  h264parse ! avdec_h264 ! videoconvert ! autovideosink
```

**SRT Benefits:**
- Automatic packet retransmission
- Adaptive bitrate
- Built-in encryption
- Better for packet loss

## Comparison: Local vs Global Streaming

| Feature | UDP (Local) | RTSP (Global) |
|---------|-------------|---------------|
| Latency | 80-150ms | 200-400ms |
| Range | Local network only | Worldwide |
| Reliability | Packet loss visible | TCP retransmits |
| Setup | Simple | Requires VPS |
| Firewall | Not needed | Port forwarding |
| Best For | Low latency required | Remote operation |

## Security Considerations

⚠️ **Current Setup:**
- No authentication on RTSP stream
- No encryption (plain video over internet)
- VPS firewall protects against port scanning

🔒 **Recommendations for Production:**
1. Add RTSP authentication (username/password)
2. Use RTSPS (RTSP over TLS)
3. Implement IP whitelisting on VPS
4. Consider VPN instead of public RTSP

## Integration with Multi-Channel System

Robot has multiple communication channels (see `CLAUDE.md`):
1. Fiber optic (3km)
2. Starlink Mini
3. 4G/5G
4. ExpressLRS
5. WiFi

**Camera streaming works over channels #1-5** as long as internet connectivity exists. VPS provides consistent endpoint regardless of which channel robot uses.

## Future Improvements

1. **WebRTC**: Browser-based viewing (no GStreamer install needed)
2. **Adaptive Bitrate**: Adjust quality based on connection
3. **Multi-Camera**: Support for additional cameras
4. **Recording**: Automatic DVR on VPS
5. **Authentication**: RTSP username/password
6. **Encryption**: RTSPS with TLS

## Files

- `/home/jetson/jetson-robot-project/scripts/start_camera_global.sh` - Status check and info
- `/home/jetson/jetson-robot-project/scripts/camera-rtsp.service` - RTSP systemd service
- `/home/jetson/jetson-robot-project/scripts/ssh-tunnel-jetson-robot.service` - SSH tunnel service
- `/home/jetson/jetson-robot-project/ros2_ws/src/veter_bringup/scripts/camera_rtsp_server.py` - RTSP server

## References

- [RTSP Protocol Specification](https://datatracker.ietf.org/doc/html/rfc2326)
- [GStreamer RTSP Server](https://gstreamer.freedesktop.org/documentation/rtsp_server/)
- [SSH Tunneling Guide](https://www.ssh.com/academy/ssh/tunneling)

---

**Last Updated:** November 11, 2025
**Maintainer:** Eugene Melnik (eugene.a.melnik@gmail.com)
**Status:** Production Ready ✅
