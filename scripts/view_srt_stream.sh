#!/bin/bash
# View SRT camera stream from VETER robot
# For MacBook/PC with GStreamer installed

VPS_HOST="81.200.157.230"
VPS_PORT="9001"

echo "============================================"
echo "VETER Robot - SRT Camera Viewer"
echo "============================================"
echo "Connecting to: srt://$VPS_HOST:$VPS_PORT"
echo ""
echo "Features:"
echo "  • Ultra-reliable (works with packet loss)"
echo "  • Low latency (~200-400ms)"
echo "  • Encrypted connection"
echo "  • Automatic reconnection"
echo ""
echo "Press Ctrl+C to stop"
echo "============================================"
echo ""

# Check if GStreamer is installed
if ! command -v gst-launch-1.0 &> /dev/null; then
    echo "ERROR: GStreamer not found!"
    echo ""
    echo "Install with:"
    echo "  brew install gstreamer gst-plugins-base gst-plugins-good gst-plugins-bad gst-libav"
    echo ""
    exit 1
fi

# Start SRT viewer
gst-launch-1.0 -v \
  srtsrc uri="srt://$VPS_HOST:$VPS_PORT" latency=200 mode=caller ! \
  tsdemux ! \
  h264parse ! \
  avdec_h264 ! \
  videoconvert ! \
  autovideosink

echo ""
echo "Stream stopped."
