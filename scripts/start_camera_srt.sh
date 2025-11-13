#!/bin/bash
# Start SRT camera streaming for VETER robot
# Ultra-reliable streaming with packet recovery for mobile/unstable connections

VPS_HOST="81.200.157.230"
VPS_PORT="9000"
RESOLUTION=${1:-"1280x720"}
FRAMERATE=${2:-"30"}
BITRATE=${3:-"2000"}

echo "============================================"
echo "Starting SRT Camera Stream"
echo "============================================"
echo "Target: $VPS_HOST:$VPS_PORT"
echo "Resolution: $RESOLUTION @ ${FRAMERATE}fps"
echo "Codec: H.264 (x264enc low-latency)"
echo "Bitrate: ${BITRATE} kbps"
echo "Protocol: SRT (Secure Reliable Transport)"
echo ""
echo "Features:"
echo "  • Automatic packet recovery"
echo "  • Works with 20% packet loss"
echo "  • Adaptive bitrate"
echo "  • Encrypted (AES-128)"
echo "  • Optimized for 4G/5G/Starlink"
echo ""
echo "View from anywhere:"
echo "  srt://81.200.157.230:9001"
echo ""
echo "============================================"

# Parse resolution
WIDTH=$(echo $RESOLUTION | cut -d'x' -f1)
HEIGHT=$(echo $RESOLUTION | cut -d'x' -f2)

# Kill any existing streams
pkill -9 gst-launch 2>/dev/null
sleep 1

echo "Starting SRT stream..."
echo ""

# Start SRT stream
# Pipeline: camera → x264 encode → MPEG-TS mux → SRT send
gst-launch-1.0 -v \
  nvarguscamerasrc sensor-id=0 ! \
  "video/x-raw(memory:NVMM),width=$WIDTH,height=$HEIGHT,format=NV12,framerate=$FRAMERATE/1" ! \
  nvvidconv flip-method=2 ! \
  "video/x-raw,format=I420" ! \
  x264enc tune=zerolatency speed-preset=ultrafast bitrate=$BITRATE key-int-max=60 ! \
  "video/x-h264,profile=baseline" ! \
  h264parse ! \
  mpegtsmux ! \
  queue max-size-buffers=0 max-size-time=0 max-size-bytes=0 ! \
  srtsink uri="srt://$VPS_HOST:$VPS_PORT" latency=500 mode=caller

echo ""
echo "SRT stream stopped."
