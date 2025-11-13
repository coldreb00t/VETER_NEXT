# Установка драйвера Arducam IMX477 для JetPack 6.2.1

**Дата:** 12 ноября 2025
**Платформа:** Jetson Orin Nano Super (JetPack 6.2.1, L4T R36.4.7)
**Драйвер:** arducam-nvidia-l4t-kernel-t234-nx-5.15.148-tegra-36.4.7-20251023170635_arm64_imx477.deb

---

## Проблема

Текущий драйвер IMX477 ограничивает FPS:
- gscam: 15 FPS @ 720p
- Isaac ROS: 8-10 FPS @ 1080p

**Цель:** Получить **60 FPS @ 1080p** согласно спецификации IMX477.

---

## Решение - Установка официального драйвера Arducam

### 1. Драйвер уже скачан

```bash
ls -lh /tmp/arducam_imx477.deb
# -rw-rw-r-- 1 jetson jetson 11M Oct 28 05:03 /tmp/arducam_imx477.deb
```

**Источник:** https://github.com/ArduCAM/MIPI_Camera/releases/download/v0.0.1-orin-nx/arducam-nvidia-l4t-kernel-t234-nx-5.15.148-tegra-36.4.7-20251023170635_arm64_imx477.deb

**Версия:** Свежий драйвер от 23 октября 2025 года

### 2. Проверка текущей версии ядра

```bash
dpkg-query --showformat='${Version}' --show nvidia-l4t-kernel
# Вывод: 5.15.148-tegra-36.4.7-20250918154033
```

**Совпадение:** Драйвер собран для ядра `5.15.148-tegra-36.4.7` ✅

### 3. Установка драйвера

```bash
cd /tmp
sudo dpkg -i arducam_imx477.deb
```

**ВНИМАНИЕ:** Эта операция заменит системные файлы ядра!

### 4. Перезагрузка системы (ОБЯЗАТЕЛЬНО!)

```bash
sudo reboot
```

Драйвер вступит в силу только после перезагрузки.

### 5. Проверка установки после перезагрузки

```bash
# Проверка камеры
v4l2-ctl --list-devices

# Проверка доступных режимов
v4l2-ctl -d /dev/video0 --list-formats-ext

# Тест захвата @ 60 FPS
gst-launch-1.0 nvarguscamerasrc sensor-id=0 sensor-mode=2 ! \
  'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=60/1' ! \
  fpsdisplaysink text-overlay=false video-sink=fakesink sync=false
```

**Ожидается:** 60 FPS @ 1920x1080

---

## Что делает драйвер

Пакет `arducam-nvidia-l4t-kernel-t234-nx-5.15.148-tegra-36.4.7-20251023170635_arm64_imx477.deb` содержит:

1. **Модули ядра:**
   - `imx477.ko` - драйвер сенсора IMX477
   - Device tree overlays для CSI интерфейса

2. **Конфигурация:**
   - Правильные настройки режимов сенсора (mode 0/1/2)
   - Поддержка 60 FPS @ 1080p

3. **Argus API support:**
   - Интеграция с libArgus (если Arducam добавил поддержку)
   - Может улучшить Isaac ROS производительность

---

## Ожидаемый результат

### До установки:
- Mode 0 (4032x3040): ~10 FPS
- Mode 2 (1920x1080): ~15 FPS (gscam)

### После установки:
- Mode 0 (4032x3040): **20-30 FPS** (согласно спецификации)
- Mode 2 (1920x1080): **60 FPS** (согласно спецификации)

### Безопасность робота:

**Текущее (15 FPS):**
- При 20 км/ч (5.56 м/с): 37 см между кадрами ⚠️ КРИТИЧНО

**После драйвера (30 FPS):**
- При 20 км/ч (5.56 м/с): 18 см между кадрами ✅ БЕЗОПАСНО

**С 60 FPS:**
- При 20 км/ч (5.56 м/с): 9 см между кадрами ✅✅ ОЧЕНЬ БЕЗОПАСНО

---

## Риски и откат

### Риски:
- Замена системных файлов ядра
- Возможна несовместимость (маловероятно, драйвер свежий)
- Требуется перезагрузка

### Откат (если что-то пошло не так):

```bash
# Удаление драйвера Arducam
sudo dpkg -r arducam-nvidia-l4t-kernel

# Переустановка оригинального ядра
sudo apt-get install --reinstall nvidia-l4t-kernel

# Перезагрузка
sudo reboot
```

---

## После установки драйвера

### Тестирование FPS:

```bash
# 1. Тест с gscam @ 1080p
cd /home/jetson/jetson-robot-project/ros2_ws
source /opt/ros/humble/setup.bash && source install/setup.bash
ros2 launch veter_camera camera.launch.py width:=1920 height:=1080 &
sleep 3
ros2 topic hz /camera/image_raw

# 2. Тест с Isaac ROS @ 1080p
source /opt/ros/humble/setup.bash
ros2 launch isaac_ros_argus_camera isaac_ros_argus_camera_mono.launch.py &
sleep 5
ros2 topic hz /left/image_raw

# 3. Нативный тест @ 60 FPS
gst-launch-1.0 nvarguscamerasrc sensor-id=0 sensor-mode=2 ! \
  'video/x-raw(memory:NVMM),width=1920,height=1080,framerate=60/1' ! \
  fpsdisplaysink text-overlay=false video-sink=fakesink sync=false
```

### Интеграция с TensorRT YOLOv8:

После подтверждения улучшения FPS:

```bash
# Запуск камеры
ros2 launch veter_camera camera.launch.py width:=1920 height:=1080

# Запуск детектора (в другом терминале)
ros2 run veter_perception yolo_detector \
  --ros-args \
  -p model_path:=yolov8n.engine \
  -p input_topic:=/camera/image_raw \
  -p device:=cuda

# Измерение end-to-end производительности
ros2 topic hz /detections
```

---

## Резюме

**Действие:** Установка драйвера Arducam IMX477 для разблокировки полной производительности камеры

**Цель:** Увеличить FPS с 15 до 30-60 FPS для безопасной работы робота на скорости 20 км/ч

**Статус:** Драйвер скачан в `/tmp/arducam_imx477.deb`, готов к установке

**Следующий шаг:**
```bash
cd /tmp
sudo dpkg -i arducam_imx477.deb
sudo reboot
```

После перезагрузки продолжить тестирование по инструкции выше.
