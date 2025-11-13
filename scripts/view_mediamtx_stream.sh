#!/bin/bash
# View camera stream from VETER robot via MediaMTX
# For MacBook/PC with GStreamer installed

VPS_HOST="81.200.157.230"
VPS_PORT="8555"

echo "============================================"
echo "VETER Robot - MediaMTX Camera Viewer"
echo "============================================"
echo "Connecting to: rtsp://$VPS_HOST:$VPS_PORT/camera"
echo ""
echo "Features:"
echo "  • Global access from anywhere"
echo "  • UDP transport (low latency ~100-200ms)"
echo "  • H.264 video, 1280x720 @ 30fps"
echo "  • Auto-reconnect on network issues"
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

# Start RTSP viewer with UDP transport
gst-launch-1.0 -v \
  rtspsrc location=rtsp://$VPS_HOST:$VPS_PORT/camera protocols=udp latency=100 ! \
  rtph264depay ! \
  h264parse ! \
  avdec_h264 ! \
  videoconvert ! \
  autovideosink

echo ""
echo "Stream stopped."
