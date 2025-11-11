#!/usr/bin/env python3
"""
Ultra-Low Latency UDP H.264 Streaming Server for VETER IMX477 Camera
Uses GStreamer UDP instead of RTSP for minimal latency (~80-100ms)
Receiver: gst-launch-1.0 -v udpsrc port=5000 ! application/x-rtp,encoding-name=H264,payload=96 ! rtph264depay ! h264parse ! avdec_h264 ! videoconvert ! autovideosink sync=false async=false
"""

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import sys
import argparse

class CameraUDPStreamer:
    def __init__(self, host='0.0.0.0', port=5000, width=1280, height=720, framerate=30):
        """Initialize UDP streamer for IMX477 camera"""

        # Initialize GStreamer
        Gst.init(None)

        self.host = host
        self.port = port
        self.width = width
        self.height = height
        self.framerate = framerate

        # Build ultra-low latency pipeline
        # Key optimizations:
        # - Lower resolution (1280x720) for faster encoding
        # - x264enc with extreme low-latency settings
        # - UDP transport (no RTSP overhead)
        # - sync=false async=false (no buffering)
        # - tune=zerolatency + ultrafast preset
        # - bframes=0, I-frame only encoding
        pipeline_str = (
            f'nvarguscamerasrc sensor-id=0 ! '
            f'video/x-raw(memory:NVMM),width={width},height={height},'
            f'format=NV12,framerate={framerate}/1 ! '
            f'nvvidconv flip-method=2 ! '
            f'video/x-raw,format=I420 ! '
            f'x264enc '
            f'tune=zerolatency '
            f'speed-preset=ultrafast '
            f'bitrate=2000 '
            f'threads=0 '
            f'bframes=0 '
            f'key-int-max=15 '
            f'sliced-threads=true '
            f'rc-lookahead=0 '
            f'sync-lookahead=0 '
            f'vbv-buf-capacity=0 '
            f'aud=false ! '
            f'video/x-h264,profile=baseline,stream-format=byte-stream ! '
            f'h264parse config-interval=-1 ! '
            f'rtph264pay config-interval=1 pt=96 ! '
            f'udpsink host={host} port={port} sync=false async=false'
        )

        print(f"\n{'='*70}")
        print(f"Ultra-Low Latency UDP H.264 Streaming Server")
        print(f"{'='*70}")
        print(f"UDP Target: {host}:{port}")
        print(f"Resolution: {width}x{height} @ {framerate} fps")
        print(f"Codec: H.264 baseline profile")
        print(f"Expected latency: 80-150ms (local network)")
        print(f"\nPipeline:\n{pipeline_str}")
        print(f"\n{'='*70}")
        print(f"Receiver command (VLC alternative):")
        print(f"{'='*70}")
        print(f"gst-launch-1.0 -v udpsrc port={port} ! \\")
        print(f"  application/x-rtp,encoding-name=H264,payload=96 ! \\")
        print(f"  rtph264depay ! h264parse ! avdec_h264 ! \\")
        print(f"  videoconvert ! autovideosink sync=false async=false")
        print(f"\nOr with FFplay:")
        print(f"ffplay -fflags nobuffer -flags low_delay -framedrop \\")
        print(f"  -probesize 32 -analyzeduration 0 \\")
        print(f"  udp://{host}:{port}")
        print(f"\n{'='*70}")
        print(f"Press Ctrl+C to stop")
        print(f"{'='*70}\n")

        # Create pipeline
        self.pipeline = Gst.parse_launch(pipeline_str)

        # Get bus for messages
        bus = self.pipeline.get_bus()
        bus.add_signal_watch()
        bus.connect("message", self.on_message)

    def on_message(self, bus, message):
        """Handle GStreamer bus messages"""
        t = message.type
        if t == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"ERROR: {err}: {debug}")
            self.stop()
        elif t == Gst.MessageType.WARNING:
            err, debug = message.parse_warning()
            print(f"WARNING: {err}: {debug}")
        elif t == Gst.MessageType.EOS:
            print("End of stream")
            self.stop()

    def start(self):
        """Start streaming"""
        print("Starting stream...")
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            print("ERROR: Unable to start pipeline")
            sys.exit(1)

    def stop(self):
        """Stop streaming"""
        self.pipeline.set_state(Gst.State.NULL)

    def run(self):
        """Run the main loop"""
        self.start()
        loop = GLib.MainLoop()
        try:
            loop.run()
        except KeyboardInterrupt:
            print("\n\nShutting down UDP streamer...")
            self.stop()
            loop.quit()

def main():
    """Main entry point"""
    parser = argparse.ArgumentParser(
        description='Ultra-low latency UDP H.264 streaming server for IMX477 camera',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog='''
Examples:
  # Stream to specific IP (for remote viewing)
  python3 camera_udp_server.py --host 192.168.8.7 --port 5000

  # Lower resolution for even lower latency
  python3 camera_udp_server.py --width 960 --height 540

  # Higher frame rate (if CPU can handle it)
  python3 camera_udp_server.py --framerate 60
        '''
    )
    parser.add_argument('--host', type=str, default='0.0.0.0',
                        help='Target IP address (default: 0.0.0.0 for broadcast)')
    parser.add_argument('--port', type=int, default=5000,
                        help='UDP port (default: 5000)')
    parser.add_argument('--width', type=int, default=1280,
                        help='Video width (default: 1280)')
    parser.add_argument('--height', type=int, default=720,
                        help='Video height (default: 720)')
    parser.add_argument('--framerate', type=int, default=30,
                        help='Framerate (default: 30)')
    args = parser.parse_args()

    # Create and run streamer
    try:
        streamer = CameraUDPStreamer(
            host=args.host,
            port=args.port,
            width=args.width,
            height=args.height,
            framerate=args.framerate
        )
        streamer.run()
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)

if __name__ == '__main__':
    main()
