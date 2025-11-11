# Ultra-Low Latency Camera Streaming via UDP

**Date:** November 11, 2025
**Status:** ✅ Working
**Latency:** 80-150ms (excellent for local network)

## Overview

Successfully implemented ultra-low latency H.264 video streaming from Jetson Orin Nano to MacBook using UDP/RTP protocol.

## Hardware

- **Camera:** Sony IMX477 12MP
- **Connection:** MIPI CSI CAM0 port
- **Resolution:** 1280x720 @ 30fps
- **Encoder:** x264enc (software, CPU-based)

## Why UDP Instead of RTSP?

- **Lower latency:** 80-150ms vs 200-300ms with RTSP
- **Simpler protocol:** No TCP overhead
- **Better for local networks:** Direct RTP streaming

**Note:** NVIDIA hardware encoder (nvv4l2h264enc) is NOT available on Orin Nano, only on AGX Orin/Xavier.

## Server Side (Jetson)

### Quick Start

```bash
# Auto-detect MacBook IP and start streaming
./scripts/start_camera_udp.sh 192.168.8.5
```

### Manual GStreamer Command

```bash
gst-launch-1.0 \
  nvarguscamerasrc sensor-id=0 ! \
  'video/x-raw(memory:NVMM),width=1280,height=720,format=NV12,framerate=30/1' ! \
  nvvidconv flip-method=2 ! \
  video/x-raw,format=I420 ! \
  x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 ! \
  h264parse ! \
  rtph264pay config-interval=10 pt=96 ! \
  udpsink host=<MACBOOK_IP> port=5600
```

### Key Parameters

- `flip-method=2` - Rotate image 180 degrees
- `tune=zerolatency` - x264 low-latency mode
- `speed-preset=ultrafast` - Fastest encoding speed
- `bitrate=2000` - 2 Mbps (lower = faster encoding)
- `config-interval=10` - Send SPS/PPS every 10 seconds
- `pt=96` - RTP payload type for H.264

## Client Side (MacBook)

### Requirements

```bash
brew install gstreamer gst-plugins-base gst-plugins-good gst-plugins-bad gst-libav
```

### Receive Stream

```bash
gst-launch-1.0 \
  udpsrc port=5600 caps="application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96" ! \
  rtph264depay ! \
  h264parse ! \
  avdec_h264 ! \
  videoconvert ! \
  osxvideosink
```

### Troubleshooting

**No video appears:**

1. Check MacBook IP address:
   ```bash
   ifconfig | grep "inet " | grep -v 127.0.0.1
   ```

2. Verify UDP packets are arriving:
   ```bash
   sudo tcpdump -i any -n udp port 5600
   ```
   Should show packets from Jetson (192.168.8.6) to MacBook.

3. Temporarily disable firewall (for testing):
   ```bash
   sudo /usr/libexec/ApplicationFirewall/socketfilterfw --setglobalstate off
   ```

**Video is upside down:**
- Add `flip-method=2` to nvvidconv on Jetson side

**High latency:**
- Reduce resolution (640x480)
- Lower bitrate (1500 kbps)
- Check WiFi signal strength

## Network Requirements

- **Local network only:** UDP doesn't work well over internet
- **Stable connection:** WiFi with low packet loss
- **Bandwidth:** ~2-3 Mbps minimum

## Performance

- **Latency:** 80-150ms glass-to-glass
- **CPU Usage (Jetson):** ~25-30% (one core for x264enc)
- **Network Bandwidth:** ~2 Mbps
- **Frame Rate:** Stable 30 FPS

## Limitations

1. **No hardware encoder** on Orin Nano (CPU-based encoding only)
2. **UDP is unreliable** - packet loss can cause artifacts
3. **Local network only** - doesn't work well over internet
4. **No encryption** - unencrypted video stream

## Future Improvements

1. **WebRTC:** For browser-based viewing with lower latency
2. **Hardware encoding:** Upgrade to AGX Orin for nvv4l2h264enc
3. **Adaptive bitrate:** Adjust quality based on network conditions
4. **Multicast:** Stream to multiple clients simultaneously

## References

- [GStreamer H.264 RTP Streaming](https://gist.github.com/esrever10/7d39fe2d4163c5b2d7006495c3c911bb)
- [NVIDIA Jetson GStreamer Guide](https://developer.ridgerun.com/wiki/index.php/NVIDIA_Jetson_Orin/GStreamer_Pipelines)
- [Low Latency Streaming on Jetson](https://forums.developer.nvidia.com/t/trying-to-get-ultra-low-live-streaming-latency-100ms-on-the-drone-using-nano/83264)

---

**Last Updated:** November 11, 2025
**Maintainer:** Eugene Melnik (eugene.a.melnik@gmail.com)
