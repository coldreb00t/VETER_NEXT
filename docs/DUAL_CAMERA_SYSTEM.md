# Архитектура системы двух камер для VETER_NEXT

**Дата:** 13 ноября 2025
**Статус:** ✅ Утверждено к реализации
**Автор:** Eugene Melnik, Claude Code

---

## 📋 Содержание

1. [Обзор](#обзор)
2. [Обоснование решения](#обоснование-решения)
3. [Архитектура системы](#архитектура-системы)
4. [Железо](#железо)
5. [Режимы работы](#режимы-работы)
6. [Программная реализация](#программная-реализация)
7. [ROS2 интеграция](#ros2-интеграция)
8. [Производительность](#производительность)
9. [Установка и подключение](#установка-и-подключение)
10. [Тестирование](#тестирование)
11. [Troubleshooting](#troubleshooting)
12. [Будущие улучшения](#будущие-улучшения)

---

## Обзор

VETER_NEXT использует **две независимые камеры** для различных задач:

- **Камера 1 (CSI):** Sony IMX477 12MP - навигация и общий обзор
- **Камера 2 (USB):** Logitech C920 (или аналог) - целевая система пожаротушения

**Ключевой принцип:** Только **одна YOLOv8 inference работает в каждый момент времени**, что обеспечивает полную производительность 22.7 FPS.

## Обоснование решения

### Проблема

Jetson Orin Nano Super (8GB, 40 TOPS) не может эффективно запускать две YOLOv8 inference одновременно при полной системной нагрузке:

- **Текущая производительность:** 22.7 FPS (одна камера)
- **Ожидаемая с двумя:** ~10-12 FPS на каждую (неприемлемо)
- **Минимум для безопасности:** 20+ FPS

### Анализ сценариев использования

**Реальная работа пожарного робота:**

1. **Движение к пожару:**
   - Робот ЕДЕТ
   - Нужна навигация (детекция препятствий, людей)
   - Гидрант НЕ работает
   - **Активна:** CSI камера

2. **Прибытие на место:**
   - Робот ОСТАНАВЛИВАЕТСЯ
   - Переключение режима
   - **Переход:** CSI → USB камера

3. **Тушение пожара:**
   - Робот СТОИТ на месте
   - Прицеливание гидранта
   - Подача воды
   - **Активна:** USB камера

4. **Возвращение:**
   - Робот ЕДЕТ домой
   - **Активна:** CSI камера

**Вывод:** Робот либо ЕДЕТ, либо ТУШИТ - никогда одновременно!

### Выбранное решение

**Две камеры + программное переключение режимов:**

✅ Полная производительность (22.7 FPS) в любом режиме
✅ Специализированные модели YOLOv8 для каждой задачи
✅ CSI камера всегда доступна для FPV/телеметрии
✅ Простая программная реализация
✅ Безопасность: компьютерное зрение для навигации

---

## Архитектура системы

```
┌─────────────────────────────────────────────────────────────────┐
│                     JETSON ORIN NANO SUPER                      │
│                        (8GB, 40 TOPS)                           │
├─────────────────────────────────────────────────────────────────┤
│                                                                 │
│  ┌─────────────────┐                 ┌─────────────────┐       │
│  │  CSI CAMERA     │                 │  USB CAMERA     │       │
│  │  Sony IMX477    │                 │  Logitech C920  │       │
│  │  12MP, 1080p60  │                 │  1080p30, H.264 │       │
│  └────────┬────────┘                 └────────┬────────┘       │
│           │                                   │                │
│           │ MIPI CSI (CAM0)                   │ USB 3.0        │
│           ▼                                   ▼                │
│  ┌─────────────────┐                 ┌─────────────────┐       │
│  │  nvarguscamerasrc│                │  v4l2src        │       │
│  │  (GStreamer)    │                 │  (GStreamer)    │       │
│  └────────┬────────┘                 └────────┬────────┘       │
│           │                                   │                │
│           │ /camera/image_raw                 │                │
│           │ (sensor_msgs/Image)               │                │
│           ▼                                   ▼                │
│  ┌────────────────────────────────────────────────────┐        │
│  │         CAMERA MODE MANAGER NODE                   │        │
│  │         (Программное переключение)                 │        │
│  │                                                     │        │
│  │  Режимы:                                            │        │
│  │  1. NAVIGATION    -> CSI камера (YOLOv8n)         │        │
│  │  2. FIRE_SUPPRESSION -> USB камера (YOLOv8n-fire) │        │
│  │  3. STANDBY       -> Обе OFF                       │        │
│  └────────┬──────────────────────────────┬────────────┘        │
│           │                              │                     │
│           ▼                              ▼                     │
│  ┌─────────────────┐          ┌─────────────────┐             │
│  │  YOLOv8n        │          │  YOLOv8n-fire   │             │
│  │  TensorRT       │          │  TensorRT       │             │
│  │  80 классов     │          │  2 класса       │             │
│  │  (general)      │          │  (fire, smoke)  │             │
│  │  22.7 FPS       │          │  22.7 FPS       │             │
│  └─────────────────┘          └─────────────────┘             │
│           │                              │                     │
│           ▼                              ▼                     │
│  ┌────────────────────────────────────────────────────┐        │
│  │              DETECTIONS OUTPUT                     │        │
│  │  /detections/navigation (person, car, obstacle)    │        │
│  │  /detections/fire (fire, smoke)                    │        │
│  └────────────────────────────────────────────────────┘        │
│                                                                 │
└─────────────────────────────────────────────────────────────────┘

          │ /detections/navigation            │ /detections/fire
          ▼                                   ▼
┌──────────────────────┐          ┌──────────────────────┐
│  Navigation Stack    │          │  Fire Tracking       │
│  (Nav2)              │          │  Controller          │
│  - Path planning     │          │  - Visual servoing   │
│  - Obstacle avoid    │          │  - Servo control     │
│  - SLAM              │          │  - Water valve       │
└──────────────────────┘          └──────────────────────┘
```

---

## Железо

### Камера 1: Sony IMX477 (CSI) - Навигация

**Назначение:** Основная навигационная камера

**Характеристики:**
- Матрица: Sony IMX477 12MP
- Подключение: MIPI CSI (CAM0 порт)
- Разрешение: 1920×1080 @ 60 FPS нативно
- Через gscam: ~26 FPS в ROS2
- После YOLOv8: 22.7 FPS детекций
- Поле зрения: Широкое (зависит от объектива)
- Длина кабеля: 15-30 см (жёсткое ограничение MIPI)

**Расположение:**
- Закреплена на **корпусе робота**
- Направлена **вперёд по ходу движения**
- Высота: ~50-80 см от земли (для обзора)
- Угол: 0-10° вниз (захват дороги)

**Задачи:**
- Детекция препятствий (person, car, bicycle)
- SLAM и навигация
- FPV для телеоператора
- Общий мониторинг

**Статус:** ✅ Установлена и работает (November 11, 2025)

---

### Камера 2: Logitech C920 (USB) - Пожаротушение

**Назначение:** Прицельная камера на гидранте

**Характеристики:**
- Модель: Logitech C920 Pro HD Webcam
- Подключение: USB 3.0 (до 5 метров)
- Разрешение: 1920×1080 @ 30 FPS
- Кодек: H.264 встроенный (опционально)
- Формат: UVC (Universal Video Class)
- Автофокус: Да
- Крепление: Стандартная резьба 1/4"

**Расположение:**
- **Жёстко закреплена на стволе гидранта**
- Направление: Совпадает с направлением воды
- Принцип: Fire in center = nozzle aimed correctly

**Задачи:**
- Детекция огня и дыма (YOLOv8n-fire)
- Visual servoing (трекинг очага)
- Прицеливание сервоприводами

**Альтернативы:**
- **Logitech C922** - ~$80, аналогичные характеристики
- **Китайская 1080p USB камера** - ~$20-30 (бюджет)
- **Любая UVC камера** с 1080p и автофокусом

**Кабель:**
- **Активный USB 3.0 кабель** (5 метров)
- Со встроенным усилителем сигнала
- Цена: ~$15-20
- Пример: Amazon Basics Active USB 3.0 Extension

**Статус:** ⏳ Планируется к установке

---

### Требования к установке USB камеры

**Механическое крепление:**
```
┌──────────────────────────────────────┐
│         ВОДЯНОЙ СТВОЛ                │
│  ╔═══════════════════════════════╗   │
│  ║ ┌─────────────────┐           ║   │
│  ║ │  Pan Servo      │←──┬───────╫───┤ GPIO 38 (ESP32)
│  ║ └────────┬────────┘   │       ║   │
│  ║          │            │       ║   │
│  ║    ┌─────▼─────┐      │       ║   │
│  ║    │ Tilt Servo│←─────┴───────╫───┤ GPIO 39 (ESP32)
│  ║    └─────┬─────┘              ║   │
│  ║          │                    ║   │
│  ║    ┌─────▼─────────────┐      ║   │
│  ║    │   NOZZLE HOUSING  │      ║   │
│  ║    │  ┌─────────────┐  │      ║   │
│  ║    │  │ C920 Camera │  │      ║   │
│  ║    │  │  [LENS]     │  │◄─────╫───┤ USB кабель 5м
│  ║    │  └─────────────┘  │      ║   │
│  ║    │         │          │      ║   │
│  ║    │    ┌────▼────┐    │      ║   │
│  ║    │    │ NOZZLE  │    │      ║   │
│  ║    │    └─────────┘    │      ║   │
│  ║    └───────────────────┘      ║   │
│  ╚═══════════════════════════════╝   │
└──────────────────────────────────────┘
```

**Водонепроницаемость:**
- Камера С920 НЕ водонепроницаема!
- Нужен защитный корпус:
  - Прозрачный акриловый кожух
  - Резиновая прокладка
  - Отвод конденсата
- Альтернатива: водонепроницаемая IP67 камера (~$50-80)

**Кабельная трасса:**
```
USB Camera (на гидранте)
    │
    │ USB кабель 5м (активный)
    │
    ▼
Jetson Orin Nano (USB 3.0 порт)
```

---

## Режимы работы

Система имеет **3 основных режима** работы:

### 1. NAVIGATION (Навигация)

**Когда:** Робот движется к месту пожара или возвращается на базу

**Активные компоненты:**
- ✅ CSI камера (Sony IMX477)
- ✅ YOLOv8n general (80 классов)
- ✅ Navigation Stack (Nav2)
- ✅ Obstacle Avoidance
- ❌ USB камера OFF
- ❌ YOLOv8n-fire OFF

**Детектируемые объекты:**
- `person` - люди (КРИТИЧНО!)
- `car`, `truck`, `bus` - транспорт
- `bicycle`, `motorcycle` - двухколёсные
- `dog`, `cat` - животные
- Общие препятствия

**Топики ROS2:**
```bash
/camera/image_raw              # Вход: CSI камера
/detections/navigation         # Выход: детекции YOLOv8n
/cmd_vel                       # Команды движения от Nav2
```

**FPS:** 22.7 (full performance)

---

### 2. FIRE_SUPPRESSION (Пожаротушение)

**Когда:** Робот прибыл на место и готов тушить

**Активные компоненты:**
- ❌ CSI камера (только FPV streaming, без ML)
- ✅ USB камера (Logitech C920)
- ✅ YOLOv8n-fire (2 класса: fire, smoke)
- ✅ Fire Tracking Controller
- ✅ Servo Pan/Tilt Control
- ✅ Water Valve Control
- ❌ Navigation Stack PAUSED

**Детектируемые объекты:**
- `fire` - открытое пламя
- `smoke` - дым

**Топики ROS2:**
```bash
/fire_camera/image_raw         # Вход: USB камера
/detections/fire               # Выход: детекции YOLOv8n-fire
/camera/servo/pan              # Управление pan сервоприводом
/camera/servo/tilt             # Управление tilt сервоприводом
/fire_suppression/valve        # Управление водяным клапаном
```

**FPS:** 22.7 (full performance)

---

### 3. STANDBY (Ожидание)

**Когда:** Робот стоит на месте, задач нет

**Активные компоненты:**
- ✅ CSI камера (только FPV streaming)
- ❌ USB камера OFF
- ❌ Обе YOLOv8 OFF
- ❌ Navigation OFF
- ❌ Fire Suppression OFF

**Назначение:**
- Экономия вычислительных ресурсов
- Снижение энергопотребления
- Телеметрия и мониторинг

**FPS:** N/A (нет inference)

---

### Переключение режимов

**Топик управления:**
```bash
/robot/mode    # std_msgs/Int32
```

**Значения:**
- `1` = NAVIGATION
- `2` = FIRE_SUPPRESSION
- `3` = STANDBY

**Время переключения:** < 500 мс (загрузка модели из кэша)

**Ручное переключение:**
```bash
# Режим навигации
ros2 topic pub --once /robot/mode std_msgs/Int32 "{data: 1}"

# Режим пожаротушения
ros2 topic pub --once /robot/mode std_msgs/Int32 "{data: 2}"

# Режим ожидания
ros2 topic pub --once /robot/mode std_msgs/Int32 "{data: 3}"
```

**Автоматическое переключение:**
- При обнаружении пожара → автопереход в FIRE_SUPPRESSION
- После тушения → возврат в NAVIGATION
- Низкий заряд батареи → STANDBY

---

## Программная реализация

### Camera Mode Manager Node

**Новый ROS2 пакет:** `veter_camera_manager`

**Назначение:** Управление режимами камер и моделями YOLOv8

**Структура:**
```
veter_camera_manager/
├── veter_camera_manager/
│   ├── __init__.py
│   ├── camera_mode_manager.py    # Главная нода
│   ├── yolo_navigator.py          # YOLOv8n general
│   ├── yolo_fire_detector.py      # YOLOv8n-fire
│   └── mode_controller.py         # Логика переключения
├── config/
│   └── camera_params.yaml
├── launch/
│   └── camera_manager.launch.py
├── package.xml
├── setup.py
└── README.md
```

---

### Код: Camera Mode Manager

**Файл:** `veter_camera_manager/camera_mode_manager.py`

```python
#!/usr/bin/env python3
"""
Camera Mode Manager Node for VETER_NEXT
Manages dual camera system with mode switching
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32, String
from vision_msgs.msg import Detection2DArray
from enum import Enum
import cv2
from cv_bridge import CvBridge

class RobotMode(Enum):
    """Robot operating modes"""
    NAVIGATION = 1
    FIRE_SUPPRESSION = 2
    STANDBY = 3

class CameraModeManager(Node):
    """
    Manages dual camera system:
    - CSI Camera (Sony IMX477) for navigation
    - USB Camera (Logitech C920) for fire suppression

    Only ONE YOLOv8 model runs at a time for maximum performance
    """

    def __init__(self):
        super().__init__('camera_mode_manager')

        # Parameters
        self.declare_parameter('initial_mode', RobotMode.NAVIGATION.value)
        self.declare_parameter('model_navigation', 'yolov8n.pt')
        self.declare_parameter('model_fire', 'yolov8n-fire.pt')
        self.declare_parameter('confidence_threshold', 0.5)

        # Current mode
        initial_mode = self.get_parameter('initial_mode').value
        self.current_mode = RobotMode(initial_mode)

        # Bridge for image conversion
        self.bridge = CvBridge()

        # YOLO models (lazy loading)
        self.yolo_navigation = None
        self.yolo_fire = None

        # Subscribers
        self.csi_sub = self.create_subscription(
            Image, '/camera/image_raw',
            self.csi_callback, 10)

        self.usb_sub = self.create_subscription(
            Image, '/fire_camera/image_raw',
            self.usb_callback, 10)

        self.mode_sub = self.create_subscription(
            Int32, '/robot/mode',
            self.mode_callback, 10)

        # Publishers
        self.detections_nav_pub = self.create_publisher(
            Detection2DArray, '/detections/navigation', 10)

        self.detections_fire_pub = self.create_publisher(
            Detection2DArray, '/detections/fire', 10)

        self.status_pub = self.create_publisher(
            String, '/camera_manager/status', 10)

        # Statistics
        self.frame_count = 0
        self.detection_count = 0

        # Timer for status updates
        self.create_timer(1.0, self.publish_status)

        self.get_logger().info(
            f'Camera Mode Manager started in mode: {self.current_mode.name}')

    def mode_callback(self, msg):
        """Handle mode change requests"""
        new_mode = RobotMode(msg.data)

        if new_mode != self.current_mode:
            self.get_logger().info(
                f'Mode change: {self.current_mode.name} → {new_mode.name}')

            self.current_mode = new_mode

            # Load appropriate model
            if new_mode == RobotMode.NAVIGATION:
                self.load_navigation_model()
            elif new_mode == RobotMode.FIRE_SUPPRESSION:
                self.load_fire_model()
            elif new_mode == RobotMode.STANDBY:
                self.unload_models()

    def load_navigation_model(self):
        """Load YOLOv8n general model for navigation"""
        if self.yolo_navigation is None:
            from ultralytics import YOLO
            model_path = self.get_parameter('model_navigation').value
            self.yolo_navigation = YOLO(model_path)
            self.get_logger().info(f'Loaded navigation model: {model_path}')

        # Unload fire model to free memory
        self.yolo_fire = None

    def load_fire_model(self):
        """Load YOLOv8n-fire model for fire detection"""
        if self.yolo_fire is None:
            from ultralytics import YOLO
            model_path = self.get_parameter('model_fire').value
            self.yolo_fire = YOLO(model_path)
            self.get_logger().info(f'Loaded fire model: {model_path}')

        # Unload navigation model to free memory
        self.yolo_navigation = None

    def unload_models(self):
        """Unload all models (STANDBY mode)"""
        self.yolo_navigation = None
        self.yolo_fire = None
        self.get_logger().info('All models unloaded (STANDBY mode)')

    def csi_callback(self, msg):
        """Process CSI camera frames (navigation)"""
        if self.current_mode != RobotMode.NAVIGATION:
            return  # Skip processing if not in navigation mode

        if self.yolo_navigation is None:
            return

        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLOv8 inference
        confidence = self.get_parameter('confidence_threshold').value
        results = self.yolo_navigation.predict(
            cv_image,
            conf=confidence,
            verbose=False
        )

        # Convert results to ROS2 Detection2DArray
        detections = self.results_to_detections(results[0])

        # Publish detections
        self.detections_nav_pub.publish(detections)

        self.frame_count += 1
        self.detection_count += len(detections.detections)

    def usb_callback(self, msg):
        """Process USB camera frames (fire suppression)"""
        if self.current_mode != RobotMode.FIRE_SUPPRESSION:
            return  # Skip processing if not in fire mode

        if self.yolo_fire is None:
            return

        # Convert ROS Image to OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Run YOLOv8-fire inference
        confidence = self.get_parameter('confidence_threshold').value
        results = self.yolo_fire.predict(
            cv_image,
            conf=confidence,
            verbose=False
        )

        # Convert results to ROS2 Detection2DArray
        detections = self.results_to_detections(results[0])

        # Publish detections
        self.detections_fire_pub.publish(detections)

        self.frame_count += 1
        self.detection_count += len(detections.detections)

    def results_to_detections(self, results):
        """Convert YOLO results to ROS2 Detection2DArray"""
        detections_msg = Detection2DArray()
        detections_msg.header.stamp = self.get_clock().now().to_msg()
        detections_msg.header.frame_id = 'camera'

        # TODO: Implement conversion logic
        # boxes = results.boxes
        # for box in boxes:
        #     detection = Detection2D()
        #     ... fill detection fields ...
        #     detections_msg.detections.append(detection)

        return detections_msg

    def publish_status(self):
        """Publish periodic status updates"""
        status = (
            f"Mode: {self.current_mode.name} | "
            f"Frames: {self.frame_count} | "
            f"Detections: {self.detection_count}"
        )

        status_msg = String()
        status_msg.data = status
        self.status_pub.publish(status_msg)

        # Reset counters
        self.frame_count = 0
        self.detection_count = 0

def main(args=None):
    rclpy.init(args=args)
    node = CameraModeManager()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## ROS2 интеграция

### Топики

**Входные (подписки):**
```
/camera/image_raw              sensor_msgs/Image       CSI камера (навигация)
/fire_camera/image_raw         sensor_msgs/Image       USB камера (пожаротушение)
/robot/mode                    std_msgs/Int32          Команда смены режима
```

**Выходные (публикации):**
```
/detections/navigation         vision_msgs/Detection2DArray  Детекции для навигации
/detections/fire               vision_msgs/Detection2DArray  Детекции огня/дыма
/camera_manager/status         std_msgs/String               Статус менеджера
```

### Launch файл

**Файл:** `veter_camera_manager/launch/camera_manager.launch.py`

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument(
            'initial_mode',
            default_value='1',
            description='Initial mode: 1=NAVIGATION, 2=FIRE, 3=STANDBY'
        ),

        # Camera Mode Manager Node
        Node(
            package='veter_camera_manager',
            executable='camera_mode_manager',
            name='camera_mode_manager',
            output='screen',
            parameters=[{
                'initial_mode': LaunchConfiguration('initial_mode'),
                'model_navigation': 'yolov8n.pt',
                'model_fire': 'yolov8n-fire.pt',
                'confidence_threshold': 0.5,
            }]
        ),
    ])
```

**Запуск:**
```bash
# Режим навигации (по умолчанию)
ros2 launch veter_camera_manager camera_manager.launch.py

# Режим пожаротушения
ros2 launch veter_camera_manager camera_manager.launch.py initial_mode:=2
```

---

## Производительность

### Теоретическая производительность

| Режим | Активная камера | YOLOv8 модель | FPS | Latency |
|-------|----------------|---------------|-----|---------|
| **NAVIGATION** | CSI (Sony IMX477) | YOLOv8n (80 классов) | **22.7** | ~44 ms |
| **FIRE_SUPPRESSION** | USB (C920) | YOLOv8n-fire (2 класса) | **22.7** | ~44 ms |
| **STANDBY** | - | - | N/A | N/A |

### Фактические измерения

**Будет обновлено после тестирования**

### Нагрузка на систему

**Режим NAVIGATION:**
```
CPU: ~30-40% (single core @ max clock)
GPU: ~60-70% (TensorRT inference)
RAM: ~2.5 GB (YOLO model + buffers)
Power: ~15W
Temperature: ~55-65°C
```

**Режим FIRE_SUPPRESSION:**
```
CPU: ~25-35% (fire model smaller)
GPU: ~50-60% (2 classes vs 80)
RAM: ~2.0 GB
Power: ~12W
Temperature: ~50-60°C
```

**Режим STANDBY:**
```
CPU: ~5-10% (minimal)
GPU: idle
RAM: ~1.0 GB
Power: ~7W
Temperature: ~40-50°C
```

---

## Установка и подключение

### 1. Установка USB камеры

**Физическое подключение:**

```bash
# Проверка подключения камеры
lsusb | grep Logitech
# Вывод: Bus 001 Device 005: ID 046d:082d Logitech, Inc. HD Pro Webcam C920

# Проверка видео устройства
v4l2-ctl --list-devices
# Вывод:
# Logitech HD Pro Webcam C920 (usb-xhci-hcd.0-2):
#     /dev/video1

# Проверка форматов
v4l2-ctl -d /dev/video1 --list-formats-ext
```

**Настройка GStreamer pipeline:**

```bash
# Тест захвата с USB камеры
gst-launch-1.0 v4l2src device=/dev/video1 ! \
  'video/x-raw,width=1920,height=1080,framerate=30/1' ! \
  videoconvert ! autovideosink

# Тест с H.264 (если поддерживается камерой)
gst-launch-1.0 v4l2src device=/dev/video1 ! \
  'image/jpeg,width=1920,height=1080,framerate=30/1' ! \
  jpegdec ! videoconvert ! autovideosink
```

---

### 2. Установка ROS2 пакета

```bash
cd ~/jetson-robot-project/ros2_ws/src

# Создать пакет (если ещё не создан)
ros2 pkg create veter_camera_manager \
  --build-type ament_python \
  --dependencies rclpy sensor_msgs std_msgs vision_msgs cv_bridge

# Добавить код (см. выше)

# Собрать
cd ~/jetson-robot-project/ros2_ws
colcon build --packages-select veter_camera_manager
source install/setup.bash
```

---

### 3. Установка YOLOv8 моделей

**YOLOv8n general (навигация):**
```bash
# Скачать предобученную модель
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt \
  -O ~/jetson-robot-project/models/yolov8n.pt

# Экспортировать в TensorRT (на Jetson)
cd ~/jetson-robot-project/models
python3 << EOF
from ultralytics import YOLO
model = YOLO('yolov8n.pt')
model.export(format='engine', device=0, half=True, imgsz=640)
EOF

# Результат: yolov8n.engine
```

**YOLOv8n-fire (пожаротушение):**
```bash
# Вариант 1: Скачать готовую fire-модель (если есть)
# wget <URL>/yolov8n-fire.pt -O ~/jetson-robot-project/models/yolov8n-fire.pt

# Вариант 2: Обучить свою модель на датасете пожаров
# Датасеты:
#   - Fire Detection Dataset (Roboflow)
#   - Smoke Detection Dataset (Kaggle)
#   - Custom dataset

# После получения модели - экспорт в TensorRT:
cd ~/jetson-robot-project/models
python3 << EOF
from ultralytics import YOLO
model = YOLO('yolov8n-fire.pt')
model.export(format='engine', device=0, half=True, imgsz=640)
EOF
```

---

### 4. Настройка автозапуска

**Интеграция с существующим systemd service:**

Отредактировать `scripts/startup/veter.service`:

```ini
[Unit]
Description=VETER Robot System with Dual Camera
After=network.target can0.service

[Service]
Type=simple
User=jetson
WorkingDirectory=/home/jetson/jetson-robot-project/ros2_ws
Environment="ROS_DOMAIN_ID=0"
ExecStartPre=/bin/sleep 10
ExecStart=/bin/bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch veter_bringup veter_full_with_cameras.launch.py'
Restart=on-failure
RestartSec=10s

[Install]
WantedBy=multi-user.target
```

**Новый launch file:** `veter_full_with_cameras.launch.py`

```python
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    return LaunchDescription([
        # Existing veter_full.launch.py
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('veter_bringup'),
                '/launch/veter_full.launch.py'
            ])
        ),

        # Camera Manager
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                FindPackageShare('veter_camera_manager'),
                '/launch/camera_manager.launch.py'
            ]),
            launch_arguments={'initial_mode': '1'}.items()
        ),
    ])
```

---

## Тестирование

### Тест 1: Проверка обеих камер

```bash
# Терминал 1: CSI камера
ros2 run gscam gscam_node --ros-args \
  -p gscam_config:="nvarguscamerasrc sensor-id=0 ! video/x-raw(memory:NVMM),width=1920,height=1080,format=NV12,framerate=30/1 ! nvvidconv ! video/x-raw,format=BGRx ! videoconvert" \
  -r image:=/camera/image_raw

# Терминал 2: USB камера
ros2 run gscam gscam_node --ros-args \
  -p gscam_config:="v4l2src device=/dev/video1 ! video/x-raw,width=1920,height=1080,framerate=30/1 ! videoconvert" \
  -r image:=/fire_camera/image_raw

# Терминал 3: Проверка топиков
ros2 topic hz /camera/image_raw
ros2 topic hz /fire_camera/image_raw
```

**Ожидаемый результат:**
- CSI: ~26 Hz
- USB: ~30 Hz

---

### Тест 2: Camera Mode Manager

```bash
# Запустить менеджер
ros2 launch veter_camera_manager camera_manager.launch.py initial_mode:=1

# В другом терминале: проверить статус
ros2 topic echo /camera_manager/status

# Переключить режим на Fire Suppression
ros2 topic pub --once /robot/mode std_msgs/Int32 "{data: 2}"

# Проверить детекции
ros2 topic echo /detections/fire

# Вернуться в Navigation
ros2 topic pub --once /robot/mode std_msgs/Int32 "{data: 1}"
```

---

### Тест 3: Производительность YOLOv8

**Тест навигационной модели:**
```bash
cd ~/jetson-robot-project/models

python3 << EOF
from ultralytics import YOLO
import time

model = YOLO('yolov8n.engine')

# Warm-up
for _ in range(10):
    results = model.predict('test_image.jpg', verbose=False)

# Benchmark
start = time.time()
count = 100
for _ in range(count):
    results = model.predict('test_image.jpg', verbose=False)
end = time.time()

fps = count / (end - start)
print(f"YOLOv8n TensorRT FPS: {fps:.2f}")
EOF
```

**Ожидаемый результат:** 60-90 FPS (inference only, без camera overhead)

---

### Тест 4: End-to-End Performance

```bash
# Запустить всю систему
ros2 launch veter_camera_manager camera_manager.launch.py

# Мониторинг FPS
ros2 topic hz /detections/navigation

# Проверить задержку (latency)
ros2 topic delay /detections/navigation
```

**Ожидаемый результат:** 20-25 FPS, latency < 50 ms

---

## Troubleshooting

### Проблема 1: USB камера не обнаружена

**Симптомы:**
```bash
v4l2-ctl --list-devices
# Нет устройства /dev/video1
```

**Решение:**
```bash
# Проверить USB подключение
lsusb | grep -i camera

# Если устройство есть, но нет /dev/video:
sudo modprobe uvcvideo

# Проверить dmesg
dmesg | tail -30 | grep -i video
```

---

### Проблема 2: Падение FPS в режиме Fire Suppression

**Симптомы:**
- USB камера даёт < 20 FPS
- Задержка > 100 ms

**Возможные причины:**
1. Длинный USB кабель (> 5м без усилителя)
2. USB 2.0 вместо 3.0
3. Недостаточная пропускная способность

**Решение:**
```bash
# Проверить версию USB
lsusb -t | grep -i camera

# Должно быть: 5000M (USB 3.0)
# Если 480M = USB 2.0 - заменить кабель

# Снизить разрешение до 720p
gst-launch-1.0 v4l2src device=/dev/video1 ! \
  'video/x-raw,width=1280,height=720,framerate=30/1' ! \
  videoconvert ! autovideosink
```

---

### Проблема 3: Модель YOLOv8 не загружается

**Симптомы:**
```
[ERROR] Failed to load model: yolov8n-fire.pt
```

**Решение:**
```bash
# Проверить существование файла
ls -lh ~/jetson-robot-project/models/yolov8n-fire.pt

# Проверить права доступа
chmod 644 ~/jetson-robot-project/models/*.pt

# Проверить версию ultralytics
pip3 list | grep ultralytics

# Переустановить если нужно
pip3 install --upgrade ultralytics
```

---

### Проблема 4: Переключение режимов не работает

**Симптомы:**
- Публикация в `/robot/mode` не вызывает изменений
- Логи показывают `Mode change ignored`

**Решение:**
```bash
# Проверить запущена ли нода
ros2 node list | grep camera_mode_manager

# Проверить подписки
ros2 node info /camera_mode_manager

# Проверить топик
ros2 topic info /robot/mode

# Ручная публикация с подтверждением
ros2 topic pub --once /robot/mode std_msgs/Int32 "{data: 2}"
ros2 topic echo /camera_manager/status
```

---

## Будущие улучшения

### 1. Автоматическое переключение режимов

**Идея:** Робот сам переключается между NAVIGATION и FIRE_SUPPRESSION

**Логика:**
```python
# В режиме NAVIGATION:
if detect_fire_in_distance():
    # Приблизиться к пожару
    navigate_to_fire()
    # Остановиться на безопасном расстоянии
    stop_robot()
    # Переключиться на тушение
    switch_mode(RobotMode.FIRE_SUPPRESSION)

# В режиме FIRE_SUPPRESSION:
if no_fire_detected_for(timeout=30):  # 30 секунд без огня
    # Пожар потушен
    switch_mode(RobotMode.NAVIGATION)
    # Продолжить патруль
```

---

### 2. Simultaneous Localization (SLAM) с CSI камерой

Использовать CSI камеру для SLAM даже в режиме FIRE_SUPPRESSION (без YOLOv8):

- Visual Odometry (OpenCV)
- ORB-SLAM3 (лёгкая версия)
- Картирование окружения

**Преимущества:**
- Улучшенная локализация
- Навигация в помещениях без GPS
- Построение карты местности

---

### 3. Thermal Camera (опционально)

**Для пожаротушения:**
- FLIR Lepton 3.5 (80×60, ~$200)
- Подключение: SPI к ESP32 или USB к Jetson
- Детекция горячих точек (fire detection by temperature)

**Преимущества:**
- Видит огонь через дым
- Работает ночью
- Находит скрытые очаги

---

### 4. Dual YOLO Pipeline (экспериментально)

**Для мощных версий Jetson (AGX Orin):**

Попробовать запустить обе YOLOv8 одновременно с оптимизацией:

- Batch inference (2 камеры в одном батче)
- Разные разрешения (CSI 1080p, USB 720p)
- Разные модели (YOLOv8n vs YOLOv8s)
- CUDA streams для параллельной обработки

---

## Ссылки и ресурсы

**Документация проекта:**
- [FIRE_SUPPRESSION_SYSTEM.md](./FIRE_SUPPRESSION_SYSTEM.md)
- [CAMERA_SETUP.md](./CAMERA_SETUP.md)
- [CAMERA_OPTIMIZATION_REPORT_2025-11-12.md](./CAMERA_OPTIMIZATION_REPORT_2025-11-12.md)
- [DEVELOPMENT_STATUS.md](./DEVELOPMENT_STATUS.md)

**Камеры:**
- [Sony IMX477 Datasheet](https://www.sony-semicon.com/en/products/is/industry/img_sensor.html)
- [Logitech C920 Specs](https://www.logitech.com/en-us/products/webcams/c920-pro-hd-webcam.html)

**ROS2:**
- [vision_msgs](http://wiki.ros.org/vision_msgs)
- [gscam](http://wiki.ros.org/gscam)
- [cv_bridge](http://wiki.ros.org/cv_bridge)

**YOLOv8:**
- [Ultralytics Docs](https://docs.ultralytics.com/)
- [TensorRT Export](https://docs.ultralytics.com/integrations/tensorrt/)
- [Fire Detection Datasets](https://universe.roboflow.com/search?q=fire%20detection)

**Jetson:**
- [Jetson Orin Nano Developer Guide](https://developer.nvidia.com/embedded/learn/jetson-orin-nano-devkit-user-guide)
- [GStreamer Guide for Jetson](https://docs.nvidia.com/jetson/archives/r35.3.1/DeveloperGuide/text/SD/Camera/CameraDevelopment.html)

---

## Changelog

| Дата | Версия | Изменения |
|------|--------|-----------|
| 2025-11-13 | 1.0 | Первая версия документа |

---

**Статус:** ✅ Готово к реализации
**Следующий шаг:** Заказ Logitech C920 и активного USB кабеля
**Приоритет:** HIGH - критично для системы пожаротушения

---

*Документ создан для проекта VETER_NEXT*
*Авторы: Eugene Melnik, Claude Code*
*GitHub: https://github.com/coldreb00t/VETER_NEXT*
