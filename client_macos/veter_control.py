#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Клиент управления роботом VETER для macOS
Простой и быстрый интерфейс для управления роботом и видеопотока
"""

import sys
import json
import socket
import subprocess
import re
import threading
import time
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QDialog, QGridLayout, QSlider,
    QGroupBox, QTextEdit, QRadioButton, QButtonGroup
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QPoint, QThread, QUrl
from PyQt6.QtGui import QKeyEvent, QPainter, QPen, QColor
import vlc

# Опциональный импорт WebEngine для карты
try:
    from PyQt6.QtWebEngineWidgets import QWebEngineView
    WEBENGINE_AVAILABLE = True
except ImportError:
    WEBENGINE_AVAILABLE = False
    print("[WARNING] PyQt6-WebEngine not available - map widget will be disabled")
    print("[INFO] To enable map: pip install PyQt6-WebEngine")


class PingThread(QThread):
    """Thread for periodically pinging the robot"""

    ping_result = pyqtSignal(float)  # Emits latency in ms, or -1 if failed

    def __init__(self, host):
        super().__init__()
        self.host = host
        self.running = True

    def run(self):
        """Ping the robot every 2 seconds"""
        while self.running:
            try:
                # Run ping command (macOS: ping -c 1 -W 1000)
                # -c 1: send 1 packet
                # -W 1000: timeout 1000ms
                result = subprocess.run(
                    ['ping', '-c', '1', '-W', '1000', self.host],
                    capture_output=True,
                    text=True,
                    timeout=2
                )

                if result.returncode == 0:
                    # Parse output for latency
                    # Example: "time=23.456 ms"
                    match = re.search(r'time=([0-9.]+)\s*ms', result.stdout)
                    if match:
                        latency = float(match.group(1))
                        self.ping_result.emit(latency)
                    else:
                        self.ping_result.emit(-1)
                else:
                    # Ping failed
                    self.ping_result.emit(-1)

            except (subprocess.TimeoutExpired, Exception):
                self.ping_result.emit(-1)

            # Wait 2 seconds before next ping
            time.sleep(2)

    def stop(self):
        """Stop the ping thread"""
        self.running = False


class TelemetryReceiver(QThread):
    """Thread for receiving telemetry data from robot via UDP"""

    telemetry_received = pyqtSignal(dict)  # Emits telemetry data

    def __init__(self, robot_id=1):
        super().__init__()
        self.robot_id = robot_id
        self.port = 9100 + robot_id  # Telemetry port: 9101, 9102, etc.
        self.running = True
        self.sock = None

    def run(self):
        """Listen for telemetry data"""
        try:
            # Create UDP socket for receiving telemetry
            self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.sock.bind(('0.0.0.0', self.port))
            self.sock.settimeout(1.0)  # 1 second timeout

            print(f"[Telemetry] Listening on port {self.port}")

            while self.running:
                try:
                    data, addr = self.sock.recvfrom(2048)

                    # Parse JSON telemetry
                    telemetry = json.loads(data.decode('utf-8'))
                    self.telemetry_received.emit(telemetry)

                except socket.timeout:
                    # No data received, that's OK
                    pass
                except json.JSONDecodeError as e:
                    print(f"[Telemetry] Invalid JSON: {e}")
                except Exception as e:
                    print(f"[Telemetry] Error: {e}")

        except Exception as e:
            print(f"[Telemetry] Failed to start: {e}")

    def stop(self):
        """Stop the telemetry receiver"""
        self.running = False
        if self.sock:
            self.sock.close()


class RobotConnection:
    """UDP connection to robot for sending commands"""

    def __init__(self, robot_id=1):
        self.connected = False
        self.host = None
        self.robot_id = robot_id
        self.port = 9000 + robot_id  # UDP port: 9001, 9002, 9003, etc.
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

    def connect(self, host):
        """Set connection parameters"""
        self.host = host
        self.connected = True
        print(f"[UDP] Robot #{self.robot_id} ready to send commands to {host}:{self.port}")
        return True

    def send_velocity(self, linear_x, angular_z):
        """Send velocity command to robot via UDP"""
        if not self.connected:
            return False

        # Create JSON message
        msg = {
            'linear': float(linear_x),
            'angular': float(angular_z)
        }

        try:
            # Send UDP packet
            data = json.dumps(msg).encode('utf-8')
            self.sock.sendto(data, (self.host, self.port))
            return True
        except Exception as e:
            print(f"[UDP] Send error: {e}")
            return False

    def disconnect(self):
        """Disconnect from robot"""
        self.connected = False
        self.sock.close()
        print("[UDP] Disconnected")


class ConnectDialog(QDialog):
    """Диалог подключения для ввода IP и ID робота"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Подключение к роботу VETER")
        self.setFixedSize(500, 280)

        layout = QGridLayout()

        # === Выбор типа подключения ===
        type_label = QLabel("Тип подключения:")
        type_label.setStyleSheet("font-weight: bold;")
        layout.addWidget(type_label, 0, 0, 1, 2)

        # Радиокнопки для типа подключения
        self.channel_group = QButtonGroup()

        self.direct_radio = QRadioButton("Прямое (Оптика/Ethernet/WiFi)")
        self.internet_radio = QRadioButton("Интернет (Tailscale/Starlink/4G)")

        self.channel_group.addButton(self.direct_radio, 0)
        self.channel_group.addButton(self.internet_radio, 1)

        # По умолчанию Интернет (наиболее частый)
        self.internet_radio.setChecked(True)

        layout.addWidget(self.direct_radio, 1, 0, 1, 2)
        layout.addWidget(self.internet_radio, 2, 0, 1, 2)

        # Подключить сигналы для обновления подсказок
        self.direct_radio.toggled.connect(self.on_channel_changed)
        self.internet_radio.toggled.connect(self.on_channel_changed)

        # Разделитель
        layout.addWidget(QLabel(""), 3, 0, 1, 2)

        # === Конфигурация робота ===
        # ID робота (для поддержки нескольких роботов)
        layout.addWidget(QLabel("ID робота:"), 4, 0)
        self.id_input = QLineEdit()
        self.id_input.setText("1")  # ID робота по умолчанию
        self.id_input.setMaximumWidth(60)
        layout.addWidget(self.id_input, 4, 1)

        # IP робота
        layout.addWidget(QLabel("IP робота:"), 5, 0)
        self.ip_input = QLineEdit()
        layout.addWidget(self.ip_input, 5, 1)

        # Информационная метка (динамическая в зависимости от канала)
        self.info_label = QLabel()
        self.info_label.setStyleSheet("color: gray; font-size: 10px;")
        self.info_label.setWordWrap(True)
        layout.addWidget(self.info_label, 6, 0, 1, 2)

        # Кнопка подключения
        self.connect_btn = QPushButton("Подключиться")
        self.connect_btn.clicked.connect(self.accept)
        layout.addWidget(self.connect_btn, 7, 0, 1, 2)

        self.setLayout(layout)

        # Инициализировать подсказки и значения по умолчанию
        self.on_channel_changed()

        # Установить фокус на поле ID
        self.id_input.setFocus()

    def on_channel_changed(self):
        """Обновить подсказки и значения по умолчанию при смене типа подключения"""
        if self.direct_radio.isChecked():
            # Прямое подключение (локальная сеть)
            self.ip_input.setPlaceholderText("например, 192.168.1.100")
            self.ip_input.setText("")
            self.info_label.setText(
                "Прямое подключение: Локальный IP адрес\n"
                "Видео: RTSP поток напрямую с робота\n"
                "Использовать для: Оптоволокно, Ethernet или WiFi в локальной сети"
            )
        else:
            # Подключение через интернет (Tailscale)
            self.ip_input.setPlaceholderText("например, 100.112.41.76")
            self.ip_input.setText("100.112.41.76")  # IP Tailscale по умолчанию
            self.info_label.setText(
                "Подключение через интернет: IP адрес Tailscale VPN\n"
                "Видео: RTSP поток через VPS (81.200.157.230)\n"
                "Использовать для: Starlink, 4G/5G или любой удаленный доступ"
            )

    def get_connection_info(self):
        """Return connection parameters"""
        try:
            robot_id = int(self.id_input.text())
        except ValueError:
            robot_id = 1

        # Determine connection type
        is_direct = self.direct_radio.isChecked()

        return {
            'robot_id': robot_id,
            'host': self.ip_input.text(),
            'channel_type': 'direct' if is_direct else 'internet'
        }


class HUDOverlay(QWidget):
    """Transparent overlay widget for drawing HUD (crosshair + ping)"""

    def __init__(self, parent=None):
        # Create as frameless tool window that cannot receive focus
        super().__init__(
            parent,
            Qt.WindowType.FramelessWindowHint |
            Qt.WindowType.Tool |
            Qt.WindowType.WindowDoesNotAcceptFocus
        )

        # Make window transparent and click-through
        self.setAttribute(Qt.WidgetAttribute.WA_TransparentForMouseEvents)
        self.setAttribute(Qt.WidgetAttribute.WA_TranslucentBackground)
        self.setAttribute(Qt.WidgetAttribute.WA_NoSystemBackground)
        self.setAttribute(Qt.WidgetAttribute.WA_ShowWithoutActivating)
        self.setWindowOpacity(1.0)
        self.setFocusPolicy(Qt.FocusPolicy.NoFocus)

        # HUD data
        self.ping_ms = -1  # Robot ping
        self.camera_ping_ms = -1  # Camera/VPS ping
        self.rc_signal_db = -999  # RC signal strength in dB
        self.packets_sent = 0

    def set_ping(self, ping_ms):
        """Update robot ping value"""
        self.ping_ms = ping_ms
        self.update()

    def set_camera_ping(self, ping_ms):
        """Update camera ping value"""
        self.camera_ping_ms = ping_ms
        self.update()

    def set_rc_signal(self, signal_db):
        """Update RC signal strength"""
        self.rc_signal_db = signal_db
        self.update()

    def set_packets(self, packets):
        """Update packets sent"""
        self.packets_sent = packets
        self.update()

    def paintEvent(self, event):
        """Draw HUD overlay: crosshair + ping info"""
        painter = QPainter(self)
        painter.setRenderHint(QPainter.RenderHint.Antialiasing)
        painter.setRenderHint(QPainter.RenderHint.TextAntialiasing)

        # Get center of widget
        center_x = self.width() // 2
        center_y = self.height() // 2

        # === DRAW CROSSHAIR ===
        crosshair_size = 20
        crosshair_gap = 5
        crosshair_thickness = 2

        # Set pen (green with transparency)
        pen = QPen(QColor(0, 255, 0, 200))
        pen.setWidth(crosshair_thickness)
        painter.setPen(pen)

        # Draw horizontal line (left and right from center)
        painter.drawLine(
            center_x - crosshair_size, center_y,
            center_x - crosshair_gap, center_y
        )
        painter.drawLine(
            center_x + crosshair_gap, center_y,
            center_x + crosshair_size, center_y
        )

        # Draw vertical line (top and bottom from center)
        painter.drawLine(
            center_x, center_y - crosshair_size,
            center_x, center_y - crosshair_gap
        )
        painter.drawLine(
            center_x, center_y + crosshair_gap,
            center_x, center_y + crosshair_size
        )

        # Draw center dot
        pen.setWidth(3)
        painter.setPen(pen)
        painter.drawPoint(center_x, center_y)

        # === DRAW HUD INFO (top-left corner) ===
        from PyQt6.QtGui import QFont, QBrush

        box_x = 10
        box_y = 10
        box_width = 140
        box_row_height = 25
        padding = 5

        # Цвет пинга робота
        if self.ping_ms < 0:
            ping_color = QColor(200, 0, 0, 180)
            ping_text = "ТАЙМАУТ"
        elif self.ping_ms < 50:
            ping_color = QColor(0, 180, 0, 180)
            ping_text = f"{self.ping_ms:.0f}мс"
        elif self.ping_ms < 150:
            ping_color = QColor(255, 165, 0, 180)
            ping_text = f"{self.ping_ms:.0f}мс"
        else:
            ping_color = QColor(200, 0, 0, 180)
            ping_text = f"{self.ping_ms:.0f}мс"

        # Нарисовать строку пинга робота
        painter.setBrush(QBrush(ping_color))
        painter.setPen(Qt.PenStyle.NoPen)
        painter.drawRoundedRect(box_x, box_y, box_width, box_row_height, 5, 5)

        painter.setPen(QColor(255, 255, 255))
        font_label = QFont("Arial", 10)
        font_value = QFont("Arial", 12, QFont.Weight.Bold)

        painter.setFont(font_label)
        painter.drawText(box_x + padding, box_y + 12, "Робот:")
        painter.setFont(font_value)
        painter.drawText(box_x + 50, box_y + 18, ping_text)

        # Нарисовать строку пинга камеры (VPS)
        camera_y = box_y + box_row_height + 3

        # Цвет пинга камеры
        if self.camera_ping_ms < 0:
            camera_ping_color = QColor(100, 100, 100, 180)
            camera_ping_text = "--мс"
        elif self.camera_ping_ms < 50:
            camera_ping_color = QColor(0, 180, 0, 180)
            camera_ping_text = f"{self.camera_ping_ms:.0f}мс"
        elif self.camera_ping_ms < 150:
            camera_ping_color = QColor(255, 165, 0, 180)
            camera_ping_text = f"{self.camera_ping_ms:.0f}мс"
        else:
            camera_ping_color = QColor(200, 0, 0, 180)
            camera_ping_text = f"{self.camera_ping_ms:.0f}мс"

        painter.setBrush(QBrush(camera_ping_color))
        painter.drawRoundedRect(box_x, camera_y, box_width, box_row_height, 5, 5)

        painter.setPen(QColor(255, 255, 255))
        painter.setFont(font_label)
        painter.drawText(box_x + padding, camera_y + 12, "Камера:")
        painter.setFont(font_value)
        painter.drawText(box_x + 65, camera_y + 18, camera_ping_text)

        # Нарисовать строку сигнала RC
        rc_y = camera_y + box_row_height + 3

        # Цвет сигнала RC (-40 dB = отлично, -100 dB = плохо, -120 dB = потеряно)
        if self.rc_signal_db == -999:
            rc_color = QColor(100, 100, 100, 180)
            rc_text = "--дБ"
        elif self.rc_signal_db > -60:
            rc_color = QColor(0, 180, 0, 180)  # Зеленый - отлично
            rc_text = f"{self.rc_signal_db}дБ"
        elif self.rc_signal_db > -100:
            rc_color = QColor(255, 165, 0, 180)  # Оранжевый - хорошо
            rc_text = f"{self.rc_signal_db}дБ"
        else:
            rc_color = QColor(200, 0, 0, 180)  # Красный - плохо/потеряно
            rc_text = "ПОТЕРЯНО"

        painter.setBrush(QBrush(rc_color))
        painter.drawRoundedRect(box_x, rc_y, box_width, box_row_height, 5, 5)

        painter.setPen(QColor(255, 255, 255))
        painter.setFont(font_label)
        painter.drawText(box_x + padding, rc_y + 12, "RC:")
        painter.setFont(font_value)
        painter.drawText(box_x + 40, rc_y + 18, rc_text)


class VideoWidget(QWidget):
    """VLC-based video streaming widget with HUD overlay"""

    def __init__(self, rtsp_url):
        super().__init__()
        self.rtsp_url = rtsp_url

        # Create VLC instance with RTSP-over-TCP to avoid UDP packet loss
        # --rtsp-tcp forces TCP instead of UDP for RTSP streams
        # --network-caching=300 adds 300ms buffer for smoother playback
        self.instance = vlc.Instance(
            '--no-xlib',
            '--rtsp-tcp',
            '--network-caching=300'
        )
        self.player = self.instance.media_player_new()

        # Create video frame
        self.video_frame = QWidget()
        self.video_frame.setMinimumSize(640, 480)

        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.video_frame)
        self.setLayout(layout)

        # Set VLC output
        if sys.platform.startswith('darwin'):  # macOS
            self.player.set_nsobject(int(self.video_frame.winId()))

        # Create HUD overlay as separate top-level window
        self.hud = HUDOverlay()
        self.hud.setMinimumSize(640, 480)

        # Timer to update overlay position and redraw
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_overlay)
        self.timer.start(50)  # 20 Hz updates

    def showEvent(self, event):
        """Show HUD when video widget is shown"""
        super().showEvent(event)
        self.update_overlay_position()
        self.hud.show()

    def hideEvent(self, event):
        """Hide HUD when video widget is hidden"""
        super().hideEvent(event)
        self.hud.hide()

    def resizeEvent(self, event):
        """Update HUD position and size when resized"""
        super().resizeEvent(event)
        self.update_overlay_position()

    def moveEvent(self, event):
        """Update HUD position when video widget moves"""
        super().moveEvent(event)
        self.update_overlay_position()

    def update_overlay_position(self):
        """Position HUD overlay over video frame"""
        if self.video_frame.isVisible():
            # Get global position of video frame
            global_pos = self.video_frame.mapToGlobal(QPoint(0, 0))
            # Set HUD geometry to match video frame
            self.hud.setGeometry(
                global_pos.x(),
                global_pos.y(),
                self.video_frame.width(),
                self.video_frame.height()
            )
            self.hud.raise_()

    def update_overlay(self):
        """Update HUD position and redraw"""
        self.update_overlay_position()
        self.hud.update()

    def update_ping(self, ping_ms):
        """Update robot ping display on HUD"""
        self.hud.set_ping(ping_ms)

    def update_camera_ping(self, ping_ms):
        """Update camera ping display on HUD"""
        self.hud.set_camera_ping(ping_ms)

    def update_rc_signal(self, signal_db):
        """Update RC signal display on HUD"""
        self.hud.set_rc_signal(signal_db)

    def update_packets(self, packets):
        """Update packets sent on HUD"""
        self.hud.set_packets(packets)

    def start_stream(self):
        """Start RTSP stream playback"""
        media = self.instance.media_new(self.rtsp_url)
        self.player.set_media(media)
        self.player.play()

    def stop_stream(self):
        """Stop stream playback"""
        self.player.stop()
        self.timer.stop()
        self.hud.hide()
        self.hud.close()


class ControlWidget(QWidget):
    """Виджет управления роботом с виртуальным джойстиком"""

    velocity_changed = pyqtSignal(float, float)

    def __init__(self):
        super().__init__()

        self.linear_speed = 0.0
        self.angular_speed = 0.0

        layout = QVBoxLayout()

        # Группа управления
        control_group = QGroupBox("Управление роботом")
        control_layout = QGridLayout()

        # Слайдер линейной скорости
        control_layout.addWidget(QLabel("Вперед/Назад:"), 0, 0)
        self.linear_slider = QSlider(Qt.Orientation.Horizontal)
        self.linear_slider.setMinimum(-100)
        self.linear_slider.setMaximum(100)
        self.linear_slider.setValue(0)
        self.linear_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.linear_slider.setTickInterval(20)
        self.linear_slider.valueChanged.connect(self.update_velocity)
        control_layout.addWidget(self.linear_slider, 0, 1)

        self.linear_label = QLabel("0.0 м/с")
        control_layout.addWidget(self.linear_label, 0, 2)

        # Слайдер угловой скорости
        control_layout.addWidget(QLabel("Влево/Вправо:"), 1, 0)
        self.angular_slider = QSlider(Qt.Orientation.Horizontal)
        self.angular_slider.setMinimum(-100)
        self.angular_slider.setMaximum(100)
        self.angular_slider.setValue(0)
        self.angular_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.angular_slider.setTickInterval(20)
        self.angular_slider.valueChanged.connect(self.update_velocity)
        control_layout.addWidget(self.angular_slider, 1, 1)

        self.angular_label = QLabel("0.0 рад/с")
        control_layout.addWidget(self.angular_label, 1, 2)

        # Кнопка остановки
        stop_btn = QPushButton("СТОП")
        stop_btn.setStyleSheet("background-color: red; color: white; font-weight: bold; font-size: 16px;")
        stop_btn.clicked.connect(self.emergency_stop)
        control_layout.addWidget(stop_btn, 2, 0, 1, 3)

        # Подсказка по клавиатуре
        hint_label = QLabel("Клавиатура: W/S = вперед/назад, A/D = влево/вправо, Пробел = стоп")
        hint_label.setStyleSheet("color: gray; font-size: 10px;")
        control_layout.addWidget(hint_label, 3, 0, 1, 3)

        control_group.setLayout(control_layout)
        layout.addWidget(control_group)

        self.setLayout(layout)

    def update_velocity(self):
        """Обновить скорость из слайдеров"""
        # Масштабировать до разумных значений: линейная 0-2 м/с, угловая 0-2 рад/с
        self.linear_speed = self.linear_slider.value() / 50.0  # от -2.0 до 2.0 м/с
        self.angular_speed = -self.angular_slider.value() / 50.0  # от -2.0 до 2.0 рад/с (инвертировано)

        self.linear_label.setText(f"{self.linear_speed:.2f} м/с")
        self.angular_label.setText(f"{self.angular_speed:.2f} рад/с")

        self.velocity_changed.emit(self.linear_speed, self.angular_speed)

    def emergency_stop(self):
        """Emergency stop - set all to zero"""
        self.linear_slider.setValue(0)
        self.angular_slider.setValue(0)
        self.update_velocity()


class TelemetryWidget(QWidget):
    """Виджет отображения телеметрии"""

    def __init__(self):
        super().__init__()

        layout = QVBoxLayout()

        telemetry_group = QGroupBox("Телеметрия робота")
        telemetry_layout = QGridLayout()
        telemetry_layout.setSpacing(5)

        # Секция батареи
        battery_header = QLabel("🔋 Батарея")
        battery_header.setStyleSheet("font-weight: bold; font-size: 12px;")
        telemetry_layout.addWidget(battery_header, 0, 0, 1, 2)

        self.battery_voltage_label = QLabel("Напряжение: -- В")
        self.battery_percent_label = QLabel("Заряд: --%")
        telemetry_layout.addWidget(self.battery_voltage_label, 1, 0)
        telemetry_layout.addWidget(self.battery_percent_label, 1, 1)

        # Секция GPS
        gps_header = QLabel("📍 GPS")
        gps_header.setStyleSheet("font-weight: bold; font-size: 12px;")
        telemetry_layout.addWidget(gps_header, 2, 0, 1, 2)

        self.gps_lat_label = QLabel("Широта: --")
        self.gps_lon_label = QLabel("Долгота: --")
        telemetry_layout.addWidget(self.gps_lat_label, 3, 0)
        telemetry_layout.addWidget(self.gps_lon_label, 3, 1)

        # Секция скорости
        speed_header = QLabel("🏃 Скорость")
        speed_header.setStyleSheet("font-weight: bold; font-size: 12px;")
        telemetry_layout.addWidget(speed_header, 4, 0, 1, 2)

        self.speed_label = QLabel("-- м/с")
        telemetry_layout.addWidget(self.speed_label, 5, 0, 1, 2)

        # Секция токов моторов
        current_header = QLabel("⚡ Моторы")
        current_header.setStyleSheet("font-weight: bold; font-size: 12px;")
        telemetry_layout.addWidget(current_header, 6, 0, 1, 2)

        self.current_left_label = QLabel("Л: -- А")
        self.current_right_label = QLabel("П: -- А")
        telemetry_layout.addWidget(self.current_left_label, 7, 0)
        telemetry_layout.addWidget(self.current_right_label, 7, 1)

        telemetry_group.setLayout(telemetry_layout)
        layout.addWidget(telemetry_group)
        layout.addStretch()

        self.setLayout(layout)

    def update_telemetry(self, data):
        """Обновить отображение телеметрии из данных робота"""
        # Батарея
        voltage = data.get('battery_voltage', 0.0)
        percent = data.get('battery_percent', 0)
        self.battery_voltage_label.setText(f"Напряжение: {voltage:.1f} В")
        self.battery_percent_label.setText(f"Заряд: {percent}%")

        # GPS - всегда показывать координаты даже без fix
        lat = data.get('latitude', 0.0)
        lon = data.get('longitude', 0.0)

        # Всегда отображать координаты
        self.gps_lat_label.setText(f"Широта: {lat:.6f}°")
        self.gps_lon_label.setText(f"Долгота: {lon:.6f}°")

        # Скорость
        speed = data.get('speed', 0.0)
        self.speed_label.setText(f"{speed:.2f} м/с")

        # Токи моторов
        current_left = data.get('current_left', 0.0)
        current_right = data.get('current_right', 0.0)
        self.current_left_label.setText(f"Л: {current_left:.1f} А")
        self.current_right_label.setText(f"П: {current_right:.1f} А")


class MapWidget(QWidget):
    """Виджет карты с позицией робота"""

    def __init__(self):
        super().__init__()

        layout = QVBoxLayout()
        layout.setContentsMargins(0, 0, 0, 0)

        # Заголовок
        map_group = QGroupBox("🗺️ Яндекс.Карты")
        map_layout = QVBoxLayout()
        map_layout.setContentsMargins(5, 5, 5, 5)

        if not WEBENGINE_AVAILABLE:
            # Заглушка вместо карты
            fallback_label = QLabel(
                "⚠️ Карта недоступна\n\n"
                "PyQt6-WebEngine не установлен.\n\n"
                "Для включения карты:\n"
                "pip install PyQt6-WebEngine\n\n"
                "Координаты робота отображаются\n"
                "в секции телеметрии выше."
            )
            fallback_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
            fallback_label.setStyleSheet(
                "color: gray; font-size: 11px; padding: 40px;"
            )
            fallback_label.setMinimumHeight(200)
            map_layout.addWidget(fallback_label)
            map_group.setLayout(map_layout)
            layout.addWidget(map_group)
            self.setLayout(layout)
            self.webengine_available = False
            return

        self.webengine_available = True

        # WebEngine view для карты
        self.map_view = QWebEngineView()
        self.map_view.setMinimumHeight(300)

        # HTML с Яндекс.Картами
        html = """
        <!DOCTYPE html>
        <html>
        <head>
            <meta charset="utf-8">
            <script src="https://api-maps.yandex.ru/2.1/?apikey=&lang=ru_RU" type="text/javascript"></script>
            <style>
                body { margin: 0; padding: 0; }
                #map { height: 100vh; width: 100%; }
            </style>
        </head>
        <body>
            <div id="map"></div>
            <script>
                ymaps.ready(init);
                var myMap, myPlacemark, myPolyline;
                var trackPoints = [];

                function init() {
                    // Инициализация карты (по умолчанию Москва)
                    myMap = new ymaps.Map("map", {
                        center: [55.751244, 37.618423],
                        zoom: 13,
                        controls: ['zoomControl', 'typeSelector']
                    });

                    // Маркер робота (красный)
                    myPlacemark = new ymaps.Placemark([55.751244, 37.618423], {
                        balloonContent: '<b>Робот VETER</b><br>Ожидание GPS...',
                        iconContent: '🤖'
                    }, {
                        preset: 'islands#redIcon'
                    });
                    myMap.geoObjects.add(myPlacemark);

                    // Линия трека (синяя)
                    myPolyline = new ymaps.Polyline([], {}, {
                        strokeColor: '#0000FF',
                        strokeWidth: 3,
                        strokeOpacity: 0.7
                    });
                    myMap.geoObjects.add(myPolyline);
                }

                // Функция обновления позиции робота
                function updateRobotPosition(lat, lon) {
                    if (lat !== 0 || lon !== 0) {
                        myPlacemark.geometry.setCoordinates([lat, lon]);
                        myPlacemark.properties.set('balloonContent',
                            '<b>Робот VETER</b><br>Широта: ' + lat.toFixed(6) + '°<br>Долгота: ' + lon.toFixed(6) + '°');

                        // Центрировать карту на роботе только если это первое обновление
                        if (!window.mapCentered) {
                            myMap.setCenter([lat, lon], 16);
                            window.mapCentered = true;
                        }
                    }
                }

                function addTrackPoint(lat, lon) {
                    if (lat !== 0 || lon !== 0) {
                        trackPoints.push([lat, lon]);
                        // Ограничить количество точек трека (последние 1000)
                        if (trackPoints.length > 1000) {
                            trackPoints.shift();
                        }
                        myPolyline.geometry.setCoordinates(trackPoints);
                    }
                }

                // Добавить кнопку центрирования
                var CenterButton = function(options) {
                    CenterButton.superclass.constructor.call(this, options);
                };
                ymaps.util.augment(CenterButton, ymaps.control.Button, {
                    onAddToMap: function(map) {
                        CenterButton.superclass.onAddToMap.call(this, map);
                        this.getParent().getChildElement(this).innerHTML = '🎯';
                    }
                });
                var centerButton = new CenterButton({
                    data: {
                        content: '🎯',
                        title: 'Центрировать на роботе'
                    },
                    options: {
                        selectOnClick: false,
                        maxWidth: 30
                    }
                });
                centerButton.events.add('click', function() {
                    var coords = myPlacemark.geometry.getCoordinates();
                    myMap.setCenter(coords, 16);
                });
                myMap.controls.add(centerButton, { float: 'right' });
            </script>
        </body>
        </html>
        """

        self.map_view.setHtml(html)
        map_layout.addWidget(self.map_view)

        map_group.setLayout(map_layout)
        layout.addWidget(map_group)

        self.setLayout(layout)

        # Флаг для отслеживания первого обновления
        self.first_update = True
        self.last_lat = 0.0
        self.last_lon = 0.0

    def update_position(self, lat, lon):
        """Обновить позицию робота на карте"""
        if not self.webengine_available:
            return  # WebEngine недоступен

        if lat == 0.0 and lon == 0.0:
            return  # Игнорировать нулевые координаты

        # Обновить маркер
        self.map_view.page().runJavaScript(f"updateRobotPosition({lat}, {lon});")

        # Добавить точку в трек (только если координаты изменились)
        if abs(lat - self.last_lat) > 0.00001 or abs(lon - self.last_lon) > 0.00001:
            self.map_view.page().runJavaScript(f"addTrackPoint({lat}, {lon});")
            self.last_lat = lat
            self.last_lon = lon


class MainWindow(QMainWindow):
    """Main application window"""

    def __init__(self, connection_info, robot):
        super().__init__()

        self.connection_info = connection_info
        self.robot = robot
        robot_id = connection_info['robot_id']

        self.setWindowTitle(f"Робот VETER #{robot_id} - {connection_info['host']}")
        self.setMinimumSize(1024, 768)

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Main layout
        main_layout = QHBoxLayout()

        # Left side: Video stream
        # RTSP URL depends on connection type
        channel_type = connection_info.get('channel_type', 'internet')

        if channel_type == 'direct':
            # Прямое подключение: поток напрямую с робота
            rtsp_url = f"rtsp://{connection_info['host']}:8554/camera"
            channel_name = "Прямое"
        else:
            # Подключение через интернет: поток через VPS
            rtsp_url = f"rtsp://81.200.157.230:8555/camera"
            channel_name = "Интернет"

        self.video_widget = VideoWidget(rtsp_url)
        main_layout.addWidget(self.video_widget, stretch=2)

        # Right side: Controls and telemetry
        right_layout = QVBoxLayout()

        # Статус подключения
        status_label = QLabel(f"Робот #{robot_id} @ {connection_info['host']} ({channel_name}, UDP)")
        status_label.setStyleSheet("background-color: green; color: white; padding: 10px; font-weight: bold;")
        right_layout.addWidget(status_label)

        # Control widget
        self.control_widget = ControlWidget()
        self.control_widget.velocity_changed.connect(self.on_velocity_changed)
        right_layout.addWidget(self.control_widget)

        # Telemetry widget
        self.telemetry_widget = TelemetryWidget()
        right_layout.addWidget(self.telemetry_widget)

        # Map widget
        self.map_widget = MapWidget()
        right_layout.addWidget(self.map_widget)

        # Add stretch to push everything up
        right_layout.addStretch()

        main_layout.addLayout(right_layout, stretch=1)

        central_widget.setLayout(main_layout)

        # Connect to robot
        self.connect_to_robot()

        # Start video stream
        self.video_widget.start_stream()

        # Timer for sending velocity commands
        self.velocity_timer = QTimer()
        self.velocity_timer.timeout.connect(self.send_velocity)
        self.velocity_timer.start(100)  # 10 Hz command rate

        self.current_linear = 0.0
        self.current_angular = 0.0
        self.packets_sent = 0

        # Start ping thread for robot
        self.ping_thread = PingThread(self.connection_info['host'])
        self.ping_thread.ping_result.connect(self.on_robot_ping_result)
        self.ping_thread.start()

        # Start ping thread for camera/VPS
        # For direct connection: ping robot (video comes from robot)
        # For internet connection: ping VPS (video comes via VPS)
        if channel_type == 'direct':
            camera_ping_host = self.connection_info['host']
        else:
            camera_ping_host = '81.200.157.230'

        self.camera_ping_thread = PingThread(camera_ping_host)
        self.camera_ping_thread.ping_result.connect(self.on_camera_ping_result)
        self.camera_ping_thread.start()

        # Start telemetry receiver thread
        self.telemetry_receiver = TelemetryReceiver(robot_id=robot_id)
        self.telemetry_receiver.telemetry_received.connect(self.on_telemetry_received)
        self.telemetry_receiver.start()

        # Timer for updating stats
        self.stats_timer = QTimer()
        self.stats_timer.timeout.connect(self.update_stats)
        self.stats_timer.start(1000)  # Update stats every second

    def connect_to_robot(self):
        """Connect to robot via UDP"""
        success = self.robot.connect(self.connection_info['host'])

        if success:
            print("[UDP] Connected to robot successfully!")
        else:
            print("[UDP] Failed to connect to robot")

    def on_velocity_changed(self, linear, angular):
        """Handle velocity changes from control widget"""
        self.current_linear = linear
        self.current_angular = angular

    def send_velocity(self):
        """Send velocity command to robot"""
        if self.robot.connected:
            success = self.robot.send_velocity(self.current_linear, self.current_angular)
            if success:
                self.packets_sent += 1

    def on_robot_ping_result(self, latency):
        """Handle robot ping result"""
        # Update HUD overlay on video
        self.video_widget.update_ping(latency)

    def on_camera_ping_result(self, latency):
        """Handle camera/VPS ping result"""
        # Update HUD overlay on video
        self.video_widget.update_camera_ping(latency)

    def on_telemetry_received(self, telemetry):
        """Handle telemetry data from robot"""
        # Debug: print received telemetry
        lat = telemetry.get('latitude', 0.0)
        lon = telemetry.get('longitude', 0.0)
        print(f"[Telemetry] GPS: {lat:.6f}, {lon:.6f} | Speed: {telemetry.get('speed', 0.0):.2f} m/s")

        # Update telemetry widget
        self.telemetry_widget.update_telemetry(telemetry)

        # Update map with robot position
        self.map_widget.update_position(lat, lon)

        # Update RC signal on HUD if available
        if 'rc_signal_db' in telemetry:
            self.video_widget.update_rc_signal(telemetry['rc_signal_db'])

    def update_stats(self):
        """Update statistics display"""
        # Update HUD overlay on video only
        self.video_widget.update_packets(self.packets_sent)

    def keyPressEvent(self, event: QKeyEvent):
        """Handle keyboard control"""
        key = event.key()

        if key == Qt.Key.Key_W:
            self.control_widget.linear_slider.setValue(50)  # Forward
        elif key == Qt.Key.Key_S:
            self.control_widget.linear_slider.setValue(-50)  # Backward
        elif key == Qt.Key.Key_A:
            self.control_widget.angular_slider.setValue(50)  # Left
        elif key == Qt.Key.Key_D:
            self.control_widget.angular_slider.setValue(-50)  # Right
        elif key == Qt.Key.Key_Space:
            self.control_widget.emergency_stop()

    def keyReleaseEvent(self, event: QKeyEvent):
        """Handle key release - return to neutral"""
        key = event.key()

        if key in [Qt.Key.Key_W, Qt.Key.Key_S]:
            self.control_widget.linear_slider.setValue(0)
        elif key in [Qt.Key.Key_A, Qt.Key.Key_D]:
            self.control_widget.angular_slider.setValue(0)

    def closeEvent(self, event):
        """Cleanup on close"""
        self.velocity_timer.stop()
        self.stats_timer.stop()
        self.video_widget.stop_stream()
        self.robot.disconnect()

        # Stop ping thread
        self.ping_thread.stop()
        self.ping_thread.wait()  # Wait for thread to finish

        # Stop telemetry receiver thread
        self.telemetry_receiver.stop()
        self.telemetry_receiver.wait()  # Wait for thread to finish

        event.accept()


def main():
    """Main application entry point"""

    # Create Qt application
    app = QApplication(sys.argv)

    # Show connection dialog
    dialog = ConnectDialog()
    if dialog.exec() == QDialog.DialogCode.Accepted:
        connection_info = dialog.get_connection_info()

        # Create robot connection with robot ID
        robot = RobotConnection(robot_id=connection_info['robot_id'])

        # Create and show main window
        window = MainWindow(connection_info, robot)
        window.show()

        sys.exit(app.exec())
    else:
        sys.exit(0)


if __name__ == '__main__':
    main()
