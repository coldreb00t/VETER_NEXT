#!/bin/bash
# Publish camera to MediaMTX via UDP/RTP with MJPEG codec (GPU nvjpegenc)

VPS_HOST="81.200.157.230"
VPS_PORT="9001"
RESOLUTION=${1:-"1280x720"}
FRAMERATE=${2:-"30"}
QUALITY=${3:-"80"}

echo "============================================"
echo "Publishing Camera to MediaMTX (MJPEG)"
echo "============================================"
echo "Target: udp://$VPS_HOST:$VPS_PORT"
echo "Resolution: $RESOLUTION @ ${FRAMERATE}fps"
echo "Codec: MJPEG (nvjpegenc GPU)"
echo "Quality: $QUALITY%"
echo "Protocol: UDP/RTP"
echo ""
echo "Expected CPU usage: ~30% (GPU accelerated)"
echo "Expected bandwidth: 6-8 Mbps"
echo ""
echo "View from anywhere:"
echo "  rtsp://81.200.157.230:8555/camera_mjpeg"
echo ""
echo "============================================"

# Parse resolution
WIDTH=$(echo $RESOLUTION | cut -d'x' -f1)
HEIGHT=$(echo $RESOLUTION | cut -d'x' -f2)

# Kill any existing streams
pkill -9 gst-launch 2>/dev/null
sleep 1

echo "Publishing MJPEG via UDP/RTP..."
echo ""

# GStreamer pipeline: camera → JPEG encode (GPU) → RTP → UDP send
# Payload type 26 = MJPEG (RFC 2435)
gst-launch-1.0 -v \
  nvarguscamerasrc sensor-id=0 ! \
  video/x-raw\(memory:NVMM\),width=$WIDTH,height=$HEIGHT,format=NV12,framerate=$FRAMERATE/1 ! \
  nvvidconv flip-method=2 ! \
  video/x-raw,format=I420 ! \
  nvjpegenc quality=$QUALITY ! \
  rtpjpegpay pt=26 ! \
  udpsink host=$VPS_HOST port=$VPS_PORT sync=false async=false

echo ""
echo "Stream stopped."
