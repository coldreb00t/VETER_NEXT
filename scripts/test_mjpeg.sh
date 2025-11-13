#!/bin/bash
# Test MJPEG streaming with nvjpegenc (GPU hardware encoder)

VPS_HOST="81.200.157.230"
VPS_PORT="9000"

echo "============================================"
echo "Testing MJPEG Streaming (GPU nvjpegenc)"
echo "============================================"
echo "Target: udp://$VPS_HOST:$VPS_PORT"
echo "Resolution: 1280x720 @ 30fps"
echo "Codec: MJPEG (nvjpegenc GPU)"
echo "Quality: 80%"
echo "Protocol: UDP/RTP"
echo ""
echo "Expected CPU usage: 10-20% (vs 75% for x264enc)"
echo "Expected bandwidth: 6-8 Mbps (vs 2 Mbps for H.264)"
echo ""
echo "============================================"
echo "Starting MJPEG stream..."
echo ""

# Kill any existing streams
pkill -9 gst-launch 2>/dev/null
sleep 1

# GStreamer pipeline: camera → JPEG encode (GPU) → RTP → UDP send
gst-launch-1.0 -v \
  nvarguscamerasrc sensor-id=0 ! \
  "video/x-raw(memory:NVMM),width=1280,height=720,format=NV12,framerate=30/1" ! \
  nvvidconv flip-method=2 ! \
  "video/x-raw(memory:NVMM)" ! \
  nvjpegenc quality=80 ! \
  rtpjpegpay ! \
  udpsink host=$VPS_HOST port=$VPS_PORT sync=false async=false

echo ""
echo "MJPEG stream stopped."
