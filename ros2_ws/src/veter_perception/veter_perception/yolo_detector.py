#!/usr/bin/env python3
"""
YOLOv8 Object Detection Node for VETER Robot
Subscribes to camera feed, performs object detection, and publishes results
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from std_msgs.msg import Header
from cv_bridge import CvBridge
import cv2
import numpy as np

try:
    from ultralytics import YOLO
    YOLO_AVAILABLE = True
except ImportError:
    YOLO_AVAILABLE = False
    print("WARNING: ultralytics not installed. Install with: pip3 install ultralytics")


class YOLODetector(Node):
    """ROS2 Node for YOLOv8 object detection"""

    def __init__(self):
        super().__init__('yolo_detector')

        # Parameters
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/detections')
        self.declare_parameter('output_image_topic', '/detections/image')
        self.declare_parameter('device', 'cuda')  # 'cuda' or 'cpu'
        self.declare_parameter('img_size', 640)
        self.declare_parameter('half_precision', True)  # FP16 for Jetson
        self.declare_parameter('publish_annotated', True)

        # Get parameters
        model_path = self.get_parameter('model_path').value
        self.confidence_threshold = self.get_parameter('confidence_threshold').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        output_image_topic = self.get_parameter('output_image_topic').value
        device = self.get_parameter('device').value
        self.img_size = self.get_parameter('img_size').value
        self.half_precision = self.get_parameter('half_precision').value
        self.publish_annotated = self.get_parameter('publish_annotated').value

        # Initialize
        self.bridge = CvBridge()
        self.model = None
        self.model_loaded = False

        # Load YOLO model
        if YOLO_AVAILABLE:
            try:
                self.get_logger().info(f'Loading YOLOv8 model: {model_path}')

                # Detect if TensorRT engine
                is_tensorrt = model_path.endswith('.engine')
                if is_tensorrt:
                    self.get_logger().info('Detected TensorRT engine - using optimized inference')

                self.model = YOLO(model_path)

                # Move to device (only for PyTorch models)
                if not is_tensorrt:
                    self.model.to(device)

                    # Enable half precision for Jetson
                    if self.half_precision and device == 'cuda':
                        self.get_logger().info('Using FP16 (half precision) for faster inference')
                else:
                    self.get_logger().info('TensorRT engine already optimized for FP16 on CUDA')

                self.model_loaded = True
                self.get_logger().info(f'Model loaded successfully')
                self.get_logger().info(f'Confidence threshold: {self.confidence_threshold}')
                self.get_logger().info(f'IOU threshold: {self.iou_threshold}')

            except Exception as e:
                self.get_logger().error(f'Failed to load YOLO model: {e}')
                self.model_loaded = False
        else:
            self.get_logger().error('Ultralytics not available. Please install: pip3 install ultralytics')

        # Publishers
        self.detection_pub = self.create_publisher(Detection2DArray, output_topic, 10)

        if self.publish_annotated:
            self.image_pub = self.create_publisher(Image, output_image_topic, 10)

        # Subscriber
        self.image_sub = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )

        # Statistics
        self.frame_count = 0
        self.detection_count = 0

        self.get_logger().info('YOLODetector node started')
        self.get_logger().info(f'Subscribed to: {input_topic}')
        self.get_logger().info(f'Publishing detections to: {output_topic}')
        if self.publish_annotated:
            self.get_logger().info(f'Publishing annotated images to: {output_image_topic}')

    def image_callback(self, msg):
        """Process incoming camera images"""
        if not self.model_loaded:
            return

        try:
            # OPTIMIZATION: Accept bgra8 directly, drop alpha channel via numpy slicing (zero-copy!)
            # This avoids CPU-based videoconvert in GStreamer pipeline
            if msg.encoding == 'bgra8':
                # Convert to numpy array without any conversion
                np_image = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 4)
                # Drop alpha channel (zero-copy view!) - BGRx[:,:,:3] = BGR
                cv_image = np_image[:, :, :3]  # Shape: (H, W, 3) - BGR format
            else:
                # Fallback for other formats (bgr8, rgb8, etc.)
                cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Run inference
            results = self.model.predict(
                cv_image,
                conf=self.confidence_threshold,
                iou=self.iou_threshold,
                imgsz=self.img_size,
                half=self.half_precision,
                verbose=False
            )

            # Process results
            detections_msg = Detection2DArray()
            detections_msg.header = msg.header

            if len(results) > 0 and results[0].boxes is not None:
                boxes = results[0].boxes

                for box in boxes:
                    # Create Detection2D message
                    detection = Detection2D()
                    detection.header = msg.header

                    # Bounding box
                    x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                    detection.bbox.center.position.x = float((x1 + x2) / 2)
                    detection.bbox.center.position.y = float((y1 + y2) / 2)
                    detection.bbox.size_x = float(x2 - x1)
                    detection.bbox.size_y = float(y2 - y1)

                    # Class and confidence
                    hypothesis = ObjectHypothesisWithPose()
                    hypothesis.hypothesis.class_id = str(int(box.cls[0]))
                    hypothesis.hypothesis.score = float(box.conf[0])

                    detection.results.append(hypothesis)
                    detections_msg.detections.append(detection)

                    self.detection_count += 1

            # Publish detections
            self.detection_pub.publish(detections_msg)

            # Publish annotated image (always, even if no detections)
            if self.publish_annotated:
                if len(results) > 0:
                    # Use YOLO's built-in annotation
                    annotated_frame = results[0].plot()
                else:
                    # No detections - just publish original image
                    annotated_frame = cv_image
                annotated_msg = self.bridge.cv2_to_imgmsg(annotated_frame, encoding='bgr8')
                annotated_msg.header = msg.header
                self.image_pub.publish(annotated_msg)

            # Statistics
            self.frame_count += 1
            if self.frame_count % 30 == 0:  # Every 30 frames
                self.get_logger().info(
                    f'Processed {self.frame_count} frames, '
                    f'{self.detection_count} total detections, '
                    f'avg {self.detection_count/self.frame_count:.1f} det/frame'
                )

        except Exception as e:
            self.get_logger().error(f'Error processing image: {e}')


def main(args=None):
    rclpy.init(args=args)

    try:
        node = YOLODetector()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
