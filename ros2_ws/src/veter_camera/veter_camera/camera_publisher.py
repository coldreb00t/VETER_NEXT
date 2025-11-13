#!/usr/bin/env python3
"""
VETER Camera Publisher Node
Publishes IMX477 camera stream to ROS2 topics using GStreamer directly
Based on official GStreamer Python appsink documentation
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
import numpy as np
import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst

class CameraPublisher(Node):
    """ROS2 node for publishing camera images from IMX477"""

    def __init__(self):
        super().__init__('camera_publisher')

        # Declare parameters
        self.declare_parameter('width', 1920)
        self.declare_parameter('height', 1080)
        self.declare_parameter('framerate', 30)
        self.declare_parameter('sensor_id', 0)

        # Get parameters
        self.width = self.get_parameter('width').value
        self.height = self.get_parameter('height').value
        self.framerate = self.get_parameter('framerate').value
        self.sensor_id = self.get_parameter('sensor_id').value

        # Create publishers
        self.image_pub = self.create_publisher(Image, 'camera/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, 'camera/camera_info', 10)

        # Frame counter
        self.frame_count = 0

        # Initialize GStreamer
        Gst.init(None)

        # Build GStreamer pipeline (hardware accelerated)
        # Use nvvidconv (hardware) for format conversion, no software videoconvert
        pipeline_str = (
            f'nvarguscamerasrc sensor-id={self.sensor_id} ! '
            f'video/x-raw(memory:NVMM),width={self.width},height={self.height},'
            f'format=NV12,framerate={self.framerate}/1 ! '
            f'nvvidconv ! '
            f'video/x-raw,format=BGRx ! '
            f'appsink name=appsink emit-signals=true max-buffers=2 drop=true'
        )

        self.get_logger().info(f'Creating pipeline: {pipeline_str}')

        # Create pipeline
        self.pipeline = Gst.parse_launch(pipeline_str)

        # Get appsink element
        self.appsink = self.pipeline.get_by_name('appsink')

        # Connect to new-sample signal
        self.appsink.connect('new-sample', self.on_new_sample)

        # Start pipeline
        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error('Failed to start pipeline!')
            raise RuntimeError('Camera initialization failed')

        self.get_logger().info(f'Camera pipeline started: {self.width}x{self.height}@{self.framerate}fps')

    def on_new_sample(self, appsink):
        """Callback when new frame is available"""
        try:
            # Pull sample from appsink (this is the correct way per GStreamer docs)
            sample = appsink.emit('pull-sample')

            # Check if sample is valid
            if not isinstance(sample, Gst.Sample):
                return Gst.FlowReturn.ERROR

            # Get buffer from sample
            buf = sample.get_buffer()

            # Extract frame data
            success, map_info = buf.map(Gst.MapFlags.READ)
            if not success:
                self.get_logger().warn('Failed to map buffer')
                return Gst.FlowReturn.ERROR

            # Use BGRx directly without conversion - much faster!
            # YOLOv8 will handle BGRA/BGRx automatically

            # Create ROS2 Image message with BGRx (4 channels)
            img_msg = Image()
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = 'camera_link'
            img_msg.height = self.height
            img_msg.width = self.width
            img_msg.encoding = 'bgra8'  # BGRx = BGRA with unused alpha
            img_msg.is_bigendian = 0
            img_msg.step = self.width * 4  # 4 bytes per pixel
            # ULTIMATE FIX: Direct memoryview cast - zero copy!
            # Cast to uint8 array to make it compatible with ROS2 message
            img_msg.data = memoryview(map_info.data).cast('B')

            # Publish image
            self.image_pub.publish(img_msg)

            # Create and publish camera info
            camera_info = self.create_camera_info()
            camera_info.header = img_msg.header
            self.camera_info_pub.publish(camera_info)

            self.frame_count += 1

            if self.frame_count % 100 == 0:
                self.get_logger().info(f'Published {self.frame_count} frames')

            # Unmap buffer
            buf.unmap(map_info)

            return Gst.FlowReturn.OK

        except Exception as e:
            self.get_logger().error(f'Error processing frame: {e}')
            return Gst.FlowReturn.ERROR

    def create_camera_info(self):
        """Create CameraInfo message with basic parameters"""
        camera_info = CameraInfo()
        camera_info.width = self.width
        camera_info.height = self.height
        camera_info.distortion_model = 'plumb_bob'

        # These are placeholder values - should be calibrated
        camera_info.d = [0.0, 0.0, 0.0, 0.0, 0.0]
        camera_info.k = [
            1000.0, 0.0, float(self.width) / 2.0,
            0.0, 1000.0, float(self.height) / 2.0,
            0.0, 0.0, 1.0
        ]
        camera_info.r = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        camera_info.p = [
            1000.0, 0.0, float(self.width) / 2.0, 0.0,
            0.0, 1000.0, float(self.height) / 2.0, 0.0,
            0.0, 0.0, 1.0, 0.0
        ]

        return camera_info

    def destroy_node(self):
        """Cleanup when node is destroyed"""
        if self.pipeline:
            self.pipeline.set_state(Gst.State.NULL)
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    try:
        camera_publisher = CameraPublisher()
        rclpy.spin(camera_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
