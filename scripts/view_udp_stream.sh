#!/bin/bash
# View UDP camera stream from VETER robot
# For MacBook/PC with GStreamer installed

VPS_HOST="81.200.157.230"
VPS_PORT="9001"

echo "============================================"
echo "VETER Robot - UDP Camera Viewer"
echo "============================================"
echo "Connecting to: udp://$VPS_HOST:$VPS_PORT"
echo ""
echo "Features:"
echo "  • Ultra-low latency (~80-150ms)"
echo "  • Pure UDP (no TCP overhead)"
echo "  • H.264 video over RTP"
echo "  • 1280x720 @ 30fps"
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

# Start UDP viewer
gst-launch-1.0 -v \
  udpsrc uri="udp://$VPS_HOST:$VPS_PORT" caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H264" ! \
  rtph264depay ! \
  h264parse ! \
  avdec_h264 ! \
  videoconvert ! \
  autovideosink

echo ""
echo "Stream stopped."
