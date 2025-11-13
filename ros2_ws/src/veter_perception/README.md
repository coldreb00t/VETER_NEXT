# VETER Perception Package

Computer vision and object detection for VETER robot using YOLOv8.

## Features

- **YOLOv8 Object Detection**: Real-time object detection optimized for Jetson
- **FP16 Inference**: Half-precision for faster performance on Jetson GPUs
- **Object Tracking**: Simple IoU-based tracker for multi-frame tracking
- **ROS2 Integration**: Full integration with ROS2 ecosystem
- **Annotated Visualization**: Optional output with bounding boxes and labels

## Nodes

### 1. yolo_detector

Performs object detection on camera images using YOLOv8.

**Subscribed Topics:**
- `/camera/image_raw` (sensor_msgs/Image): Input camera feed

**Published Topics:**
- `/detections` (vision_msgs/Detection2DArray): Detected objects
- `/detections/image` (sensor_msgs/Image): Annotated image with bounding boxes

**Parameters:**
- `model_path` (string, default: "yolov8n.pt"): Path to YOLO model
- `confidence_threshold` (float, default: 0.5): Minimum confidence for detections
- `iou_threshold` (float, default: 0.45): IoU threshold for NMS
- `device` (string, default: "cuda"): Device to run inference on
- `img_size` (int, default: 640): Input image size
- `half_precision` (bool, default: true): Use FP16 for faster inference
- `publish_annotated` (bool, default: true): Publish annotated images

### 2. object_tracker

Tracks detected objects across frames using IoU matching.

**Subscribed Topics:**
- `/detections` (vision_msgs/Detection2DArray): Input detections

**Published Topics:**
- `/tracked_objects` (std_msgs/String): Tracked objects as JSON

**Parameters:**
- `iou_threshold` (float, default: 0.3): IoU threshold for matching
- `max_age` (int, default: 10): Maximum frames without detection before removing track

## Installation

### 1. Install Dependencies

```bash
# PyTorch for Jetson (CUDA-enabled)
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu124

# YOLOv8 (Ultralytics)
pip3 install ultralytics

# Additional requirements
pip3 install opencv-python numpy
```

### 2. Build Package

```bash
cd ~/jetson-robot-project/ros2_ws
colcon build --packages-select veter_perception --symlink-install
source install/setup.bash
```

## Usage

### Basic Detection

```bash
# Launch YOLOv8 detector
ros2 launch veter_perception yolo_detection.launch.py

# View detections (separate terminal)
ros2 topic echo /detections

# View annotated video (if published)
ros2 run rqt_image_view rqt_image_view /detections/image
```

### With Object Tracking

```bash
# Launch with tracker
ros2 launch veter_perception yolo_detection.launch.py use_tracker:=true

# View tracked objects
ros2 topic echo /tracked_objects
```

### Different Models

```bash
# Use larger model for better accuracy (slower)
ros2 launch veter_perception yolo_detection.launch.py model:=yolov8s.pt

# Use smallest model for maximum speed
ros2 launch veter_perception yolo_detection.launch.py model:=yolov8n.pt
```

## Performance

Expected performance on Jetson Orin Nano (8GB):

| Model | Resolution | FPS | mAP | Use Case |
|-------|-----------|-----|-----|----------|
| YOLOv8n | 640x640 | 24-30 | 37.3 | Real-time, mobile robots |
| YOLOv8s | 640x640 | 18-22 | 44.9 | Balanced accuracy/speed |
| YOLOv8m | 640x640 | 12-15 | 50.2 | Higher accuracy |

## TensorRT Optimization

For even faster inference, export to TensorRT:

```bash
# Export model to TensorRT engine
python3 -c "from ultralytics import YOLO; model = YOLO('yolov8n.pt'); model.export(format='engine', half=True)"

# Use in launch file
ros2 launch veter_perception yolo_detection.launch.py model:=yolov8n.engine
```

TensorRT can provide 2-3x speedup on Jetson.

## Supported Classes

YOLOv8 pre-trained model detects 80 COCO classes including:
- Person
- Car, truck, bus
- Dog, cat, bird
- Chair, couch, bed
- And 70+ more objects

## Integration Examples

### Security Patrol

```python
# Subscribe to detections
ros2 topic echo /detections --field detections

# Check for people detected
# Trigger alarm if person detected in restricted area
```

### Follow Mode

```python
# Subscribe to tracked objects
# Find person with highest confidence
# Compute center of bounding box
# Publish cmd_vel to follow
```

## Troubleshooting

### CUDA Out of Memory

Reduce image size or use smaller model:
```bash
ros2 launch veter_perception yolo_detection.launch.py model:=yolov8n.pt
ros2 param set /yolo_detector img_size 416
```

### Low FPS

- Ensure using CUDA: `ros2 param get /yolo_detector device`
- Enable half precision: `ros2 param set /yolo_detector half_precision true`
- Export to TensorRT (see above)

### No Detections

- Check confidence threshold: `ros2 param get /yolo_detector confidence_threshold`
- Lower threshold: `ros2 param set /yolo_detector confidence_threshold 0.3`
- Verify camera feed: `ros2 topic hz /camera/image_raw`

## Architecture

```
Camera → yolo_detector → Detection2DArray → object_tracker → Tracked Objects
                ↓
         Annotated Image
```

## Files

```
veter_perception/
├── veter_perception/
│   ├── __init__.py
│   ├── yolo_detector.py      # Main detection node
│   ├── object_tracker.py     # Object tracking node
│   └── models/                # Downloaded models stored here
├── launch/
│   └── yolo_detection.launch.py  # Launch file
├── config/
│   └── yolo_params.yaml       # Configuration parameters
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

## Future Enhancements

- [ ] DeepSORT tracking for better multi-object tracking
- [ ] Custom object classes (train on specific objects)
- [ ] 3D object detection using depth estimation
- [ ] Semantic segmentation for navigation
- [ ] Integration with Nav2 for obstacle avoidance

## License

MIT

## Author

Eugene Melnik - eugene.a.melnik@gmail.com
