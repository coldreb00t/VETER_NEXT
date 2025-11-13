# Camera Streaming - Quick Start

## 🚀 Production (Recommended)

### Start on Jetson:
```bash
./scripts/start_camera_mediamtx.sh
```

### View from MacBook/PC:
```bash
vlc rtsp://81.200.157.230:8555/camera
```

**That's it!** Works from anywhere in the world.

---

## 📋 All Streaming Options

| Method | Use Case | Latency | Command |
|--------|----------|---------|---------|
| **MediaMTX** | 🌍 Global (production) | 200-300ms | `./scripts/start_camera_mediamtx.sh` |
| UDP Direct | 🏠 Local network only | 80-150ms | `./scripts/start_camera_udp.sh <IP>` |
| RTSP Tunnel | 🌍 Global (simple) | 300-400ms | `./scripts/start_camera_global.sh` |
| SRT | 🧪 Experimental | 200-300ms | `./scripts/start_camera_srt.sh` |

---

## 📺 Viewing Options

### VLC (Easiest)
```bash
vlc rtsp://81.200.157.230:8555/camera
```

### GStreamer
```bash
gst-launch-1.0 -v rtspsrc location=rtsp://81.200.157.230:8555/camera latency=100 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink
```

### FFmpeg
```bash
ffplay -fflags nobuffer -flags low_delay rtsp://81.200.157.230:8555/camera
```

---

## 🔍 Monitoring

### Check stream status:
```bash
# On Jetson
ps aux | grep gst-launch

# Check VPS
curl http://81.200.157.230:9997/v3/paths/get/camera
```

### Expected output:
```json
{"ready":true,"tracks":["H264"],"bytesReceived":...}
```

---

## 🔧 Troubleshooting

### No video?
1. Check Jetson stream is running: `ps aux | grep gst-launch`
2. Check VPS status: `curl http://81.200.157.230:9997/v3/paths/get/camera`
3. Try increasing latency: `latency=200` or `latency=300`

### Choppy video?
Lower bitrate on Jetson:
```bash
./scripts/start_camera_mediamtx.sh 1280x720 30 1500
```

---

## 📖 Full Documentation

`docs/CAMERA_MEDIAMTX_STREAMING.md`
