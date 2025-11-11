#!/usr/bin/env python3
"""
RTSP Server for VETER IMX477 Camera
Streams camera over RTSP for remote viewing
Access: rtsp://<jetson_ip>:8554/camera
"""

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GstRtspServer', '1.0')
from gi.repository import Gst, GstRtspServer, GLib
import sys

class CameraRTSPServer:
    def __init__(self, port=8554, width=1920, height=1080, framerate=30):
        """Initialize RTSP server for IMX477 camera"""

        # Initialize GStreamer
        Gst.init(None)

        self.port = port
        self.width = width
        self.height = height
        self.framerate = framerate

        # Create RTSP server
        self.server = GstRtspServer.RTSPServer()
        self.server.set_service(str(port))

        # Get mount points
        self.factory = GstRtspServer.RTSPMediaFactory()

        # Build GStreamer pipeline for IMX477 with H.264 encoding
        # nvarguscamerasrc → nvvidconv (flip 180) → x264enc (ultra-low latency) → rtph264pay
        # Optimized for minimum latency:
        # - bframes=0: No B-frames (they add latency for reordering)
        # - threads=0: Use all CPU cores
        # - sliced-threads=true: Parallel encoding within frames
        # - rc-lookahead=0: Disable rate control lookahead
        # - sync-lookahead=0: Disable sync lookahead
        # - vbv-buf-capacity=0: Minimum video buffer verifier size
        # - aud=false: Disable access unit delimiters (small optimization)
        pipeline = (
            f'nvarguscamerasrc sensor-id=0 ! '
            f'video/x-raw(memory:NVMM),width={width},height={height},'
            f'format=NV12,framerate={framerate}/1 ! '
            f'nvvidconv flip-method=2 ! '  # flip-method=2 = rotate 180 degrees
            f'video/x-raw,format=I420 ! '
            f'x264enc '
            f'tune=zerolatency '
            f'speed-preset=ultrafast '
            f'bitrate=2500 '
            f'threads=0 '
            f'bframes=0 '
            f'sliced-threads=true '
            f'rc-lookahead=0 '
            f'sync-lookahead=0 '
            f'vbv-buf-capacity=0 '
            f'key-int-max=30 '
            f'aud=false ! '
            f'video/x-h264,profile=baseline,stream-format=byte-stream ! '
            f'h264parse ! '
            f'rtph264pay config-interval=1 name=pay0 pt=96'
        )

        print(f"RTSP Pipeline: {pipeline}")
        self.factory.set_launch(pipeline)

        # Allow multiple clients
        self.factory.set_shared(True)

        # Mount the factory at /camera
        mounts = self.server.get_mount_points()
        mounts.add_factory("/camera", self.factory)

        # Attach server to default main context
        self.server.attach(None)

        print(f"\n{'='*60}")
        print(f"RTSP Camera Server Started")
        print(f"{'='*60}")
        print(f"Stream URL: rtsp://YOUR_JETSON_IP:{port}/camera")
        print(f"Resolution: {width}x{height} @ {framerate} fps")
        print(f"Codec: H.264")
        print(f"\nView with:")
        print(f"  VLC:     vlc rtsp://YOUR_JETSON_IP:{port}/camera")
        print(f"  FFplay:  ffplay rtsp://YOUR_JETSON_IP:{port}/camera")
        print(f"  GStreamer: gst-launch-1.0 rtspsrc location=rtsp://YOUR_JETSON_IP:{port}/camera ! decodebin ! autovideosink")
        print(f"\nPress Ctrl+C to stop")
        print(f"{'='*60}\n")

    def run(self):
        """Run the server main loop"""
        loop = GLib.MainLoop()
        try:
            loop.run()
        except KeyboardInterrupt:
            print("\n\nShutting down RTSP server...")
            loop.quit()

def main():
    """Main entry point"""
    # Parse arguments
    import argparse
    parser = argparse.ArgumentParser(description='RTSP server for IMX477 camera')
    parser.add_argument('--port', type=int, default=8554, help='RTSP port (default: 8554)')
    parser.add_argument('--width', type=int, default=1920, help='Video width (default: 1920)')
    parser.add_argument('--height', type=int, default=1080, help='Video height (default: 1080)')
    parser.add_argument('--framerate', type=int, default=30, help='Framerate (default: 30)')
    args = parser.parse_args()

    # Create and run server
    try:
        server = CameraRTSPServer(
            port=args.port,
            width=args.width,
            height=args.height,
            framerate=args.framerate
        )
        server.run()
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(1)

if __name__ == '__main__':
    main()
