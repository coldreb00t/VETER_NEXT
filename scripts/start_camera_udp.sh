#!/bin/bash
# Start UDP camera streaming for VETER robot
# Ultra-low latency H.264 RTP streaming via UDP

# Get MacBook IP (or set manually)
MACBOOK_IP=${1:-"192.168.8.5"}
PORT=${2:-"5600"}

echo "============================================"
echo "Starting UDP Camera Stream"
echo "============================================"
echo "Target: $MACBOOK_IP:$PORT"
echo "Resolution: 1280x720 @ 30fps"
echo "Codec: H.264 (x264enc ultra-low latency)"
echo "Expected latency: 80-150ms"
echo ""
echo "To view on MacBook run:"
echo "gst-launch-1.0 udpsrc port=$PORT caps=\"application/x-rtp, media=(string)video, clock-rate=(int)90000, encoding-name=(string)H264, payload=(int)96\" ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! osxvideosink"
echo "============================================"

# Kill any existing streams
pkill -9 gst-launch 2>/dev/null
sleep 1

# Start UDP stream
gst-launch-1.0 \
  nvarguscamerasrc sensor-id=0 ! \
  'video/x-raw(memory:NVMM),width=1280,height=720,format=NV12,framerate=30/1' ! \
  nvvidconv flip-method=2 ! \
  video/x-raw,format=I420 ! \
  x264enc tune=zerolatency speed-preset=ultrafast bitrate=2000 ! \
  h264parse ! \
  rtph264pay config-interval=10 pt=96 ! \
  udpsink host=$MACBOOK_IP port=$PORT
