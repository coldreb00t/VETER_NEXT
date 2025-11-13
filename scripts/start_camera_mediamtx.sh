#!/bin/bash
# Publish camera to MediaMTX via UDP/RTP
# Most reliable solution for global access

VPS_HOST="81.200.157.230"
VPS_PORT="9000"
RESOLUTION=${1:-"1280x720"}
FRAMERATE=${2:-"30"}
BITRATE=${3:-"2000"}

echo "============================================"
echo "Publishing Camera to MediaMTX (via UDP)"
echo "============================================"
echo "Target: udp://$VPS_HOST:$VPS_PORT"
echo "Resolution: $RESOLUTION @ ${FRAMERATE}fps"
echo "Codec: H.264 (x264enc CPU)"
echo "Bitrate: ${BITRATE} kbps"
echo "Protocol: UDP/RTP"
echo ""
echo "Features:"
echo "  • Pure UDP (no TCP overhead)"
echo "  • Global access via MediaMTX"
echo "  • Production-grade reliability"
echo ""
echo "View from anywhere:"
echo "  rtsp://81.200.157.230:8555/camera"
echo ""
echo "============================================"

# Parse resolution
WIDTH=$(echo $RESOLUTION | cut -d'x' -f1)
HEIGHT=$(echo $RESOLUTION | cut -d'x' -f2)

# Kill any existing streams
pkill -9 gst-launch 2>/dev/null
sleep 1

echo "Publishing via UDP/RTP..."
echo ""

# GStreamer pipeline: camera → H.264 encode → RTP → UDP send
# Using x264enc (CPU) as JetPack 6 lacks GPU H.264 encoder
gst-launch-1.0 -v \
  nvarguscamerasrc sensor-id=0 ! \
  "video/x-raw(memory:NVMM),width=$WIDTH,height=$HEIGHT,format=NV12,framerate=$FRAMERATE/1" ! \
  nvvidconv flip-method=2 ! \
  "video/x-raw,format=I420" ! \
  x264enc tune=zerolatency speed-preset=ultrafast bitrate=$BITRATE key-int-max=30 bframes=0 ! \
  "video/x-h264,profile=baseline,stream-format=byte-stream" ! \
  h264parse ! \
  rtph264pay config-interval=1 pt=96 ! \
  queue max-size-buffers=10 max-size-time=0 max-size-bytes=0 ! \
  udpsink host=$VPS_HOST port=$VPS_PORT sync=false async=false

echo ""
echo "Stream stopped."
