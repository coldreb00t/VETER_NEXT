#!/bin/bash
# Start UDP camera streaming to VPS for global access
# Pure UDP for lowest latency

VPS_HOST="81.200.157.230"
VPS_PORT="9000"
RESOLUTION=${1:-"1280x720"}
FRAMERATE=${2:-"30"}
BITRATE=${3:-"2000"}

echo "============================================"
echo "Starting UDP Camera Stream to VPS"
echo "============================================"
echo "Target: $VPS_HOST:$VPS_PORT"
echo "Resolution: $RESOLUTION @ ${FRAMERATE}fps"
echo "Codec: H.264 (x264enc low-latency)"
echo "Bitrate: ${BITRATE} kbps"
echo "Protocol: UDP (RTP/H.264)"
echo ""
echo "View from anywhere:"
echo "  udp://81.200.157.230:9001"
echo ""
echo "============================================"

# Parse resolution
WIDTH=$(echo $RESOLUTION | cut -d'x' -f1)
HEIGHT=$(echo $RESOLUTION | cut -d'x' -f2)

# Kill any existing streams
pkill -9 gst-launch 2>/dev/null
sleep 1

echo "Starting UDP stream..."
echo ""

# Start UDP stream
# Pipeline: camera → x264 encode → RTP payload → UDP send
gst-launch-1.0 -v \
  nvarguscamerasrc sensor-id=0 ! \
  "video/x-raw(memory:NVMM),width=$WIDTH,height=$HEIGHT,format=NV12,framerate=$FRAMERATE/1" ! \
  nvvidconv flip-method=2 ! \
  "video/x-raw,format=I420" ! \
  x264enc tune=zerolatency speed-preset=ultrafast bitrate=$BITRATE key-int-max=30 bframes=0 ! \
  "video/x-h264,profile=baseline,stream-format=byte-stream" ! \
  h264parse ! \
  rtph264pay config-interval=1 pt=96 ! \
  udpsink host=$VPS_HOST port=$VPS_PORT sync=false async=false

echo ""
echo "UDP stream stopped."
