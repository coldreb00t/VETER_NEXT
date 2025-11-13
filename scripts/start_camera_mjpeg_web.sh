#!/bin/bash
# MJPEG streamer for web interface
# Транскодирует камеру в MJPEG через ffmpeg

echo "🎥 Starting MJPEG stream for web interface..."
echo "   Resolution: 640x480 @ 15fps"
echo "   Port: 8081"
echo "   URL: http://localhost:8081/stream.mjpg"

# Остановить старые процессы
pkill -f 'ffmpeg.*mjpeg'
sleep 1

# GStreamer -> ffmpeg -> MJPEG HTTP
gst-launch-1.0 -v \
  nvarguscamerasrc sensor-id=0 ! \
  "video/x-raw(memory:NVMM),width=640,height=480,format=NV12,framerate=15/1" ! \
  nvvidconv flip-method=2 ! \
  "video/x-raw,format=I420" ! \
  videoconvert ! \
  "video/x-raw,format=YUY2" ! \
  fdsink fd=1 | \
ffmpeg -f rawvideo -pix_fmt yuyv422 -s 640x480 -r 15 -i - \
  -f mjpeg -q:v 5 \
  -listen 1 -f mjpeg \
  http://0.0.0.0:8081/stream.mjpg
