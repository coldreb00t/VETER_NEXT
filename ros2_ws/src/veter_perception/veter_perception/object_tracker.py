#!/usr/bin/env python3
"""
Simple Object Tracker for VETER Robot
Tracks detected objects across frames using IoU matching
"""

import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection2DArray, Detection2D
from std_msgs.msg import String
import json
from collections import defaultdict
import math


class ObjectTracker(Node):
    """Simple object tracker using IoU matching"""

    def __init__(self):
        super().__init__('object_tracker')

        # Parameters
        self.declare_parameter('input_topic', '/detections')
        self.declare_parameter('output_topic', '/tracked_objects')
        self.declare_parameter('iou_threshold', 0.3)
        self.declare_parameter('max_age', 10)  # frames

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.iou_threshold = self.get_parameter('iou_threshold').value
        self.max_age = self.get_parameter('max_age').value

        # Tracking state
        self.tracks = {}  # {track_id: {bbox, class_id, age, last_seen}}
        self.next_track_id = 0
        self.frame_count = 0

        # Publishers & Subscribers
        self.tracked_pub = self.create_publisher(String, output_topic, 10)
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            input_topic,
            self.detection_callback,
            10
        )

        self.get_logger().info('ObjectTracker node started')
        self.get_logger().info(f'IoU threshold: {self.iou_threshold}')
        self.get_logger().info(f'Max age: {self.max_age} frames')

    def compute_iou(self, bbox1, bbox2):
        """Compute IoU between two bounding boxes"""
        # bbox format: {center_x, center_y, width, height}
        x1 = bbox1['center_x'] - bbox1['width'] / 2
        y1 = bbox1['center_y'] - bbox1['height'] / 2
        x2 = bbox1['center_x'] + bbox1['width'] / 2
        y2 = bbox1['center_y'] + bbox1['height'] / 2

        x1_2 = bbox2['center_x'] - bbox2['width'] / 2
        y1_2 = bbox2['center_y'] - bbox2['height'] / 2
        x2_2 = bbox2['center_x'] + bbox2['width'] / 2
        y2_2 = bbox2['center_y'] + bbox2['height'] / 2

        # Intersection
        xi1 = max(x1, x1_2)
        yi1 = max(y1, y1_2)
        xi2 = min(x2, x2_2)
        yi2 = min(y2, y2_2)

        inter_width = max(0, xi2 - xi1)
        inter_height = max(0, yi2 - yi1)
        inter_area = inter_width * inter_height

        # Union
        bbox1_area = bbox1['width'] * bbox1['height']
        bbox2_area = bbox2['width'] * bbox2['height']
        union_area = bbox1_area + bbox2_area - inter_area

        if union_area == 0:
            return 0.0

        return inter_area / union_area

    def detection_callback(self, msg):
        """Process detections and update tracks"""
        self.frame_count += 1

        # Convert detections to internal format
        detections = []
        for det in msg.detections:
            if len(det.results) > 0:
                bbox = {
                    'center_x': det.bbox.center.position.x,
                    'center_y': det.bbox.center.position.y,
                    'width': det.bbox.size_x,
                    'height': det.bbox.size_y
                }
                class_id = det.results[0].hypothesis.class_id
                confidence = det.results[0].hypothesis.score
                detections.append({'bbox': bbox, 'class_id': class_id, 'confidence': confidence})

        # Match detections to existing tracks
        matched_tracks = set()
        matched_detections = set()

        for det_idx, detection in enumerate(detections):
            best_iou = 0
            best_track_id = None

            for track_id, track in self.tracks.items():
                if track['class_id'] != detection['class_id']:
                    continue

                iou = self.compute_iou(detection['bbox'], track['bbox'])
                if iou > best_iou and iou > self.iou_threshold:
                    best_iou = iou
                    best_track_id = track_id

            if best_track_id is not None:
                # Update existing track
                self.tracks[best_track_id]['bbox'] = detection['bbox']
                self.tracks[best_track_id]['last_seen'] = self.frame_count
                self.tracks[best_track_id]['age'] = 0
                self.tracks[best_track_id]['confidence'] = detection['confidence']
                matched_tracks.add(best_track_id)
                matched_detections.add(det_idx)

        # Create new tracks for unmatched detections
        for det_idx, detection in enumerate(detections):
            if det_idx not in matched_detections:
                self.tracks[self.next_track_id] = {
                    'bbox': detection['bbox'],
                    'class_id': detection['class_id'],
                    'confidence': detection['confidence'],
                    'age': 0,
                    'last_seen': self.frame_count
                }
                self.next_track_id += 1

        # Age unmatched tracks
        tracks_to_remove = []
        for track_id, track in self.tracks.items():
            if track_id not in matched_tracks:
                track['age'] += 1
                if track['age'] > self.max_age:
                    tracks_to_remove.append(track_id)

        # Remove old tracks
        for track_id in tracks_to_remove:
            del self.tracks[track_id]

        # Publish tracked objects
        tracked_msg = {
            'frame': self.frame_count,
            'tracks': [
                {
                    'id': track_id,
                    'class': track['class_id'],
                    'confidence': track['confidence'],
                    'bbox': track['bbox'],
                    'age': track['age']
                }
                for track_id, track in self.tracks.items()
            ]
        }

        msg_str = String()
        msg_str.data = json.dumps(tracked_msg)
        self.tracked_pub.publish(msg_str)

        # Log statistics
        if self.frame_count % 30 == 0:
            self.get_logger().info(
                f'Frame {self.frame_count}: {len(self.tracks)} active tracks, '
                f'{len(detections)} detections'
            )


def main(args=None):
    rclpy.init(args=args)

    try:
        node = ObjectTracker()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
