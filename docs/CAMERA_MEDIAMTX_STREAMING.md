# Global Camera Streaming via MediaMTX

Production-ready solution for streaming VETER robot camera globally via VPS.

## Architecture

```
Jetson (Robot) → UDP/RTP (H.264) → VPS:9000 → FFmpeg relay → MediaMTX:8555 → Clients (anywhere)
```

**Key advantages:**
- ✅ Pure UDP from Jetson to VPS (low latency)
- ✅ Global access from anywhere in the world
- ✅ Works through any NAT/firewall
- ✅ Production-grade reliability (MediaMTX)
- ✅ ~200-300ms glass-to-glass latency (through Germany VPS)

## Components

### 1. Jetson (Camera Publisher)
- Captures video from Sony IMX477 (1920x1080 @ 20fps)
- Encodes H.264 with x264enc (software encoder - see Hardware Limitations below)
- Sends UDP/RTP stream to VPS:9000

### 2. VPS (Stream Relay)
- **Location**: Germany (81.200.157.230)
- **FFmpeg**: Receives UDP/RTP on port 9000
- **MediaMTX**: RTSP server on port 8555
- **Services**:
  - `camera-udp-to-mediamtx.service` - FFmpeg relay
  - `mediamtx.service` - RTSP/media server

### 3. Clients (Viewers)
- Connect via RTSP: `rtsp://81.200.157.230:8555/camera`
- Uses TCP interleaved mode (RTP over TCP)
- Works from anywhere without port forwarding

## Usage

### Start Streaming (on Jetson)

```bash
# Start camera stream to MediaMTX
cd /home/jetson/jetson-robot-project
./scripts/start_camera_mediamtx.sh

# Optional parameters: resolution fps bitrate
./scripts/start_camera_mediamtx.sh 1920x1080 20 1200
```

**Optimized settings (recommended):**
- Resolution: 1920x1080 (Full HD)
- Framerate: 20 fps
- Bitrate: 1200 kbps
- Codec: H.264 (baseline profile, x264enc software encoder)
- CPU Usage: ~79% (optimal for Orin Nano)

**Note**: These settings provide the best balance between quality and CPU usage on Jetson Orin Nano, which lacks hardware H.264 encoder. See "Hardware Limitations" section below.

### View Stream (from anywhere)

#### Option 1: VLC (easiest)
```bash
vlc rtsp://81.200.157.230:8555/camera
```

#### Option 2: GStreamer
```bash
gst-launch-1.0 -v \
  rtspsrc location=rtsp://81.200.157.230:8555/camera latency=100 ! \
  rtph264depay ! \
  h264parse ! \
  avdec_h264 ! \
  videoconvert ! \
  autovideosink
```

#### Option 3: FFmpeg
```bash
ffplay -fflags nobuffer -flags low_delay rtsp://81.200.157.230:8555/camera
```

#### Option 4: Script (on MacBook)
```bash
# Download script from Jetson
scp -P 2223 jetson@81.200.157.230:/home/jetson/jetson-robot-project/scripts/view_mediamtx_stream.sh ~/

# Run
chmod +x ~/view_mediamtx_stream.sh
~/view_mediamtx_stream.sh
```

## VPS Configuration

### MediaMTX Config (`/etc/mediamtx.yml`)
```yaml
# MediaMTX configuration for VETER robot camera
logLevel: info
logDestinations: [stdout]

# API
api: yes
apiAddress: :9997

# Metrics
metrics: yes
metricsAddress: :9998

# RTSP server
rtspAddress: :8555
protocols: [tcp, udp]
encryption: "no"
rtpAddress: :8000
rtcpAddress: :8001

# Paths
paths:
  camera:
    source: publisher
    runOnReady: echo "Camera stream ready"
```

### FFmpeg Relay Service (`/etc/systemd/system/camera-udp-to-mediamtx.service`)
```ini
[Unit]
Description=Camera UDP to MediaMTX Relay
After=network.target mediamtx.service
Wants=mediamtx.service

[Service]
Type=simple
User=root
Restart=always
RestartSec=5

# FFmpeg: receive UDP/RTP → publish to MediaMTX via RTSP
ExecStart=/usr/bin/ffmpeg -hide_banner -loglevel warning \
    -protocol_whitelist file,udp,rtp \
    -i /root/camera.sdp \
    -c copy \
    -f rtsp -rtsp_transport tcp rtsp://localhost:8555/camera

StandardOutput=journal
StandardError=journal

[Install]
WantedBy=multi-user.target
```

### SDP File (`/root/camera.sdp`)
```
v=0
o=- 0 0 IN IP4 0.0.0.0
s=VETER Camera
c=IN IP4 0.0.0.0
t=0 0
m=video 9000 RTP/AVP 96
a=rtpmap:96 H264/90000
a=fmtp:96 packetization-mode=1
```

## Firewall Configuration

### VPS Firewall (UFW)
```bash
# MediaMTX RTSP
sudo ufw allow 8555/tcp comment 'MediaMTX RTSP'

# MediaMTX RTP/RTCP (for UDP mode)
sudo ufw allow 8000:8001/udp comment 'MediaMTX RTP/RTCP'

# Camera UDP input
sudo ufw allow 9000/udp comment 'Camera UDP stream'

# API (optional, for monitoring)
sudo ufw allow 9997/tcp comment 'MediaMTX API'
```

## Monitoring

### Check Stream Status
```bash
# On VPS - check if MediaMTX receives stream
curl http://localhost:9997/v3/paths/get/camera

# Expected output:
# {"ready":true,"tracks":["H264"],"bytesReceived":...}
```

### Check Services
```bash
# On VPS
sudo systemctl status mediamtx
sudo systemctl status camera-udp-to-mediamtx

# View logs
sudo journalctl -u mediamtx -f
sudo journalctl -u camera-udp-to-mediamtx -f
```

### Check Stream on Jetson
```bash
# Check if GStreamer is running
ps aux | grep gst-launch

# View publishing log
tail -f /tmp/mediamtx_udp.log
```

## Performance

### Measured Latency
- **Local network (WiFi)**: ~80-150ms
- **Global (through Germany VPS)**: ~200-300ms
- **Protocol overhead**: Minimal (UDP to VPS, TCP interleaved to client)

### Bandwidth Usage
- **Video (1080p @ 20fps @ 1.2 Mbps)**: ~1.2 Mbps
- **Overhead**: ~5-10% (RTP headers)
- **Total**: ~1.3 Mbps

### CPU Usage (Optimized Settings)
- **Jetson (encoding)**: ~79% CPU with 1920x1080 @ 20fps
- **VPS (relay)**: ~5% CPU, 50MB RAM
- **Client (decoding)**: Depends on device

**Note**: CPU usage on Jetson Orin Nano is relatively high due to lack of hardware H.264 encoder. See "Hardware Limitations" section.

## Hardware Limitations

### ⚠️ Jetson Orin Nano: No Hardware H.264 Encoder

**Critical**: The Jetson Orin Nano does **NOT** have a hardware H.264 encoder (NVENC). This is a key difference from higher-end Jetson models.

| Model | Hardware H.264 Encoder |
|-------|----------------------|
| Jetson Orin Nano | ❌ **No** (software only) |
| Jetson Orin NX | ✅ Yes (NVENC) |
| Jetson Orin AGX | ✅ Yes (NVENC) |
| Jetson Xavier NX | ✅ Yes (NVENC) |

**Source**: [NVIDIA Jetson Orin Nano Developer Guide - Software Encode](https://docs.nvidia.com/jetson/archives/r35.3.1/DeveloperGuide/text/SD/Multimedia/SoftwareEncodeInOrinNano.html)

### Why This Matters

Without hardware encoding, the Jetson Orin Nano must use **software encoding** (x264enc), which is CPU-intensive:

- **Hardware encoding** (NVENC): ~5-10% CPU, dedicated video encoder
- **Software encoding** (x264enc): ~70-80% CPU, uses general-purpose CPU

### Performance Benchmarks

According to NVIDIA benchmarks for Jetson Orin Nano with x264enc:

| Resolution | FPS | Preset | CPU Usage |
|-----------|-----|--------|-----------|
| 1080p | 64.7 | ultrafast | ~52% |
| 1080p | 22.5 | medium | ~75% |
| 1080p | 4.6 | veryslow | ~95% |

**Our configuration** (1920x1080 @ 20fps @ 1.2 Mbps):
- **CPU**: ~79%
- **Preset**: ultrafast (for low latency)
- **Quality**: Excellent (1080p Full HD)

This is **optimal** for Orin Nano - we get high quality at acceptable CPU usage.

### Why Not Hardware Encoder?

Tested configurations that **did not work**:

1. **avenc_h264_omx** (OpenMAX IL):
   - Status: ❌ Failed with "not-negotiated" error
   - Reason: libav wrapper doesn't support OMX on Orin Nano

2. **omxh264enc** (native OMX):
   - Status: ❌ Plugin not available
   - Reason: Not present in GStreamer on Orin Nano

3. **nvv4l2h264enc** (V4L2 wrapper):
   - Status: ❌ Not available
   - Reason: No hardware encoder to wrap

**Conclusion**: Software encoding (x264enc) is the **only option** for Orin Nano.

### Optimization Strategies

Given hardware limitations, we optimized for best quality/performance ratio:

1. **1080p @ 20fps** instead of 720p @ 30fps:
   - ✅ Better resolution (2.25x more pixels)
   - ✅ Smoother for slow robot movement
   - ✅ Lower bitrate (1.2 Mbps vs 2.0 Mbps)

2. **x264enc ultrafast preset**:
   - ✅ Low latency encoding
   - ✅ ~79% CPU (acceptable)
   - ⚠️ Slightly lower compression efficiency

3. **Baseline profile**:
   - ✅ Wide compatibility
   - ✅ Lower decoder complexity
   - ✅ Works on all devices

### Alternative Solutions (if CPU is a concern)

If you need lower CPU usage, consider these options:

| Configuration | CPU | Quality | Latency |
|--------------|-----|---------|---------|
| **1920x1080 @ 20fps** (current) | 79% | ⭐⭐⭐⭐⭐ | ~200ms |
| 1920x1080 @ 15fps | ~65% | ⭐⭐⭐⭐ | ~250ms |
| 1280x720 @ 30fps | ~50% | ⭐⭐⭐ | ~200ms |
| 1280x720 @ 20fps | ~40% | ⭐⭐⭐ | ~250ms |

**Recommendation**: Stick with **1920x1080 @ 20fps** for best quality. The Jetson has enough headroom for other tasks (ROS2, YOLO, etc.).

## Troubleshooting

### Stream Not Connecting

1. **Check Jetson is streaming:**
   ```bash
   ps aux | grep gst-launch | grep udpsink
   ```

2. **Check VPS receives UDP:**
   ```bash
   sudo tcpdump -i any port 9000
   ```

3. **Check MediaMTX status:**
   ```bash
   curl http://81.200.157.230:9997/v3/paths/get/camera
   ```

### Video Freezes/Stutters

1. **Increase latency on client:**
   ```bash
   # GStreamer: increase latency to 200ms
   rtspsrc location=rtsp://... latency=200 ! ...
   ```

2. **Check network stability:**
   ```bash
   ping -c 100 81.200.157.230
   ```

3. **Lower bitrate on Jetson:**
   ```bash
   ./scripts/start_camera_mediamtx.sh 1280x720 30 1500
   ```

### "Could not receive UDP packets" Error

This means your client's firewall blocks incoming UDP. **Solution: Use TCP mode (default):**

```bash
# Remove protocols=udp, let it auto-select TCP interleaved
gst-launch-1.0 -v \
  rtspsrc location=rtsp://81.200.157.230:8555/camera ! ...
```

TCP interleaved mode works through any NAT/firewall.

## Advanced: Multiple Clients

MediaMTX supports unlimited simultaneous viewers:

```bash
# Client 1
vlc rtsp://81.200.157.230:8555/camera

# Client 2
vlc rtsp://81.200.157.230:8555/camera

# Client 3...
```

Each client gets their own stream from MediaMTX.

## Comparison with Other Solutions

| Solution | Latency | NAT Friendly | Reliability | Complexity |
|----------|---------|--------------|-------------|------------|
| **MediaMTX** | ⭐⭐⭐⭐ (200-300ms) | ✅ Yes | ⭐⭐⭐⭐⭐ | ⭐⭐⭐ Medium |
| Direct UDP | ⭐⭐⭐⭐⭐ (80ms) | ❌ No | ⭐⭐⭐ | ⭐⭐ Easy |
| SRT | ⭐⭐⭐⭐ (200ms) | ⚠️ Partial | ⭐⭐⭐⭐⭐ | ⭐⭐⭐⭐ Hard |
| WebRTC | ⭐⭐⭐⭐⭐ (100ms) | ✅ Yes | ⭐⭐⭐⭐ | ⭐⭐⭐⭐⭐ Very Hard |

**MediaMTX chosen for:**
- ✅ Best balance of latency/reliability
- ✅ Works everywhere without configuration
- ✅ Production-tested, mature
- ✅ Easy to deploy and maintain

## Related Documentation

- [Local UDP Streaming](CAMERA_UDP_STREAMING.md) - Ultra-low latency for local network
- [Global RTSP](CAMERA_GLOBAL_STREAMING.md) - Simple RTSP via SSH tunnel
- [SRT Streaming](CAMERA_SRT_STREAMING.md) - Experimental SRT solution
- [Camera Setup](CAMERA_SETUP.md) - Basic camera configuration

## Updates

- **November 11, 2025**: Initial production release (720p @ 30fps)
- **November 15, 2025**: Optimized to 1080p @ 20fps, added hardware limitations documentation

**Status**: ✅ Production-ready, tested and optimized for Jetson Orin Nano
