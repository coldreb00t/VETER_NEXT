#!/bin/bash
# Start global camera streaming through VPS
# Accessible from anywhere in the world via RTSP

echo "=========================================="
echo "VETER Robot - Global Camera Streaming"
echo "=========================================="
echo ""
echo "📡 Stream accessible from ANYWHERE via:"
echo ""
echo "   rtsp://81.200.157.230:8554/camera"
echo ""
echo "=========================================="
echo "View commands:"
echo "=========================================="
echo ""
echo "GStreamer (recommended):"
echo "  gst-launch-1.0 rtspsrc location=rtsp://81.200.157.230:8554/camera latency=200 ! \\"
echo "    rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink"
echo ""
echo "VLC:"
echo "  vlc rtsp://81.200.157.230:8554/camera"
echo ""
echo "FFplay:"
echo "  ffplay -rtsp_transport tcp rtsp://81.200.157.230:8554/camera"
echo ""
echo "=========================================="
echo "System Status:"
echo "=========================================="
echo ""

# Check RTSP server status
if systemctl is-active --quiet camera-rtsp.service; then
    echo "✅ RTSP Server: RUNNING"
else
    echo "❌ RTSP Server: STOPPED"
    echo "   Starting RTSP server..."
    sudo systemctl start camera-rtsp.service
    sleep 2
    if systemctl is-active --quiet camera-rtsp.service; then
        echo "✅ RTSP Server: STARTED"
    else
        echo "❌ Failed to start RTSP server"
        exit 1
    fi
fi

# Check SSH tunnel status
if systemctl is-active --quiet ssh-tunnel-jetson-robot.service; then
    echo "✅ SSH Tunnel: CONNECTED"
else
    echo "❌ SSH Tunnel: DISCONNECTED"
    echo "   Restarting SSH tunnel..."
    sudo systemctl restart ssh-tunnel-jetson-robot.service
    sleep 3
    if systemctl is-active --quiet ssh-tunnel-jetson-robot.service; then
        echo "✅ SSH Tunnel: CONNECTED"
    else
        echo "❌ Failed to connect SSH tunnel"
        exit 1
    fi
fi

echo ""
echo "=========================================="
echo "Performance Tips:"
echo "=========================================="
echo ""
echo "  • Expected latency: 200-400ms"
echo "  • Resolution: 1280x720 @ 30fps"
echo "  • Bitrate: ~2.5 Mbps"
echo "  • Protocol: RTSP over TCP (reliable)"
echo ""
echo "For local network (lower latency):"
echo "  ./start_camera_udp.sh <YOUR_IP>"
echo ""
echo "=========================================="
echo "✅ Global camera streaming is ACTIVE"
echo "=========================================="
