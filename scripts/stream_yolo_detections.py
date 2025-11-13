#!/usr/bin/env python3
"""
Stream YOLO Detections to MediaMTX
Subscribes to /detections/image ROS2 topic and streams annotated video to MediaMTX
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import sys
import argparse

class YOLOStreamer(Node):
    """ROS2 Node that streams YOLO detections via GStreamer to MediaMTX"""

    def __init__(self, vps_host='81.200.157.230', vps_port=9000, width=1280, height=720, framerate=30, bitrate=2000):
        super().__init__('yolo_streamer')

        self.bridge = CvBridge()
        self.frame_count = 0

        # GStreamer initialization
        Gst.init(None)

        # Build GStreamer pipeline for H.264 encoding and UDP streaming
        # appsrc → videoconvert → x264enc → h264parse → rtph264pay → udpsink
        pipeline_str = (
            f'appsrc name=source is-live=true block=true format=GST_FORMAT_TIME '
            f'caps=video/x-raw,format=BGR,width={width},height={height},framerate={framerate}/1 ! '
            f'videoconvert ! '
            f'video/x-raw,format=I420 ! '
            f'x264enc tune=zerolatency speed-preset=ultrafast bitrate={bitrate} '
            f'key-int-max=30 bframes=0 ! '
            f'video/x-h264,profile=baseline,stream-format=byte-stream ! '
            f'h264parse ! '
            f'rtph264pay config-interval=1 pt=96 ! '
            f'queue max-size-buffers=10 max-size-time=0 max-size-bytes=0 ! '
            f'udpsink host={vps_host} port={vps_port} sync=false async=false'
        )

        self.get_logger().info(f'GStreamer pipeline: {pipeline_str}')

        try:
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.appsrc = self.pipeline.get_by_name('source')

            # Set pipeline to PLAYING
            ret = self.pipeline.set_state(Gst.State.PLAYING)
            if ret == Gst.StateChangeReturn.FAILURE:
                self.get_logger().error('Unable to set pipeline to PLAYING state')
                sys.exit(1)

            self.get_logger().info('GStreamer pipeline started successfully')

        except Exception as e:
            self.get_logger().error(f'Failed to create GStreamer pipeline: {e}')
            sys.exit(1)

        # Subscribe to YOLO annotated images
        self.subscription = self.create_subscription(
            Image,
            '/detections/image',
            self.image_callback,
            10
        )

        self.get_logger().info('='*60)
        self.get_logger().info('YOLO Detection Streamer Started')
        self.get_logger().info('='*60)
        self.get_logger().info(f'Subscribing to: /detections/image')
        self.get_logger().info(f'Streaming to: udp://{vps_host}:{vps_port}')
        self.get_logger().info(f'Resolution: {width}x{height} @ {framerate} fps')
        self.get_logger().info(f'Bitrate: {bitrate} kbps')
        self.get_logger().info(f'View at: rtsp://{vps_host}:8555/camera')
        self.get_logger().info('='*60)

    def image_callback(self, msg):
        """Process incoming annotated images from YOLO"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Resize if needed (to match streaming resolution)
            # The annotated image from YOLO is 1920x1080, but we stream at 1280x720
            if cv_image.shape[1] != 1280 or cv_image.shape[0] != 720:
                cv_image = cv2.resize(cv_image, (1280, 720))

            # Convert to GStreamer buffer
            data = cv_image.tobytes()
            buf = Gst.Buffer.new_allocate(None, len(data), None)
            buf.fill(0, data)

            # Set timestamp
            buf.pts = self.frame_count * (Gst.SECOND // 30)  # 30 fps
            buf.duration = Gst.SECOND // 30

            # Push buffer to pipeline
            ret = self.appsrc.emit('push-buffer', buf)
            if ret != Gst.FlowReturn.OK:
                self.get_logger().warning(f'Push buffer failed: {ret}')

            self.frame_count += 1

            # Log progress every 30 frames
            if self.frame_count % 30 == 0:
                self.get_logger().info(f'Streamed {self.frame_count} frames')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')

    def cleanup(self):
        """Clean up GStreamer pipeline"""
        self.get_logger().info('Stopping GStreamer pipeline...')
        if hasattr(self, 'pipeline'):
            self.appsrc.emit('end-of-stream')
            self.pipeline.set_state(Gst.State.NULL)
        self.get_logger().info('Pipeline stopped')

def main(args=None):
    """Main entry point"""
    parser = argparse.ArgumentParser(description='Stream YOLO detections to MediaMTX')
    parser.add_argument('--vps-host', type=str, default='81.200.157.230', help='VPS host address')
    parser.add_argument('--vps-port', type=int, default=9000, help='VPS UDP port')
    parser.add_argument('--width', type=int, default=1280, help='Stream width')
    parser.add_argument('--height', type=int, default=720, help='Stream height')
    parser.add_argument('--framerate', type=int, default=30, help='Stream framerate')
    parser.add_argument('--bitrate', type=int, default=2000, help='Bitrate in kbps')

    parsed_args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)

    try:
        node = YOLOStreamer(
            vps_host=parsed_args.vps_host,
            vps_port=parsed_args.vps_port,
            width=parsed_args.width,
            height=parsed_args.height,
            framerate=parsed_args.framerate,
            bitrate=parsed_args.bitrate
        )

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if 'node' in locals():
            node.cleanup()
        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
