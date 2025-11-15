#!/usr/bin/env python3
"""
VETER Robot Control Client for macOS
Simple, fast GUI for robot control and video streaming
"""

import sys
import paramiko
import threading
from PyQt6.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QLabel, QLineEdit, QPushButton, QDialog, QGridLayout, QSlider,
    QGroupBox, QTextEdit
)
from PyQt6.QtCore import Qt, QTimer, pyqtSignal, QObject
from PyQt6.QtGui import QKeyEvent
import vlc

class RobotConnection(QObject):
    """Manages SSH connection to robot for sending commands"""

    telemetry_update = pyqtSignal(dict)

    def __init__(self):
        super().__init__()
        self.ssh = None
        self.connected = False
        self.host = None
        self.port = 22
        self.username = "jetson"

    def connect(self, host, port=22, username="jetson", password="cvbp"):
        """Connect to robot via SSH"""
        try:
            self.ssh = paramiko.SSHClient()
            self.ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
            self.ssh.connect(host, port=port, username=username, password=password, timeout=5)
            self.connected = True
            self.host = host
            self.port = port
            self.username = username
            return True
        except Exception as e:
            print(f"Connection error: {e}")
            self.connected = False
            return False

    def send_velocity(self, linear_x, angular_z):
        """Send velocity command to robot via ROS2"""
        if not self.connected:
            return False

        cmd = f"source /opt/ros/humble/setup.bash && source /home/jetson/jetson-robot-project/ros2_ws/install/setup.bash && ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \"{{linear: {{x: {linear_x}, y: 0.0, z: 0.0}}, angular: {{x: 0.0, y: 0.0, z: {angular_z}}}}}\""

        try:
            # Run in background to avoid blocking
            threading.Thread(target=self._execute_command, args=(cmd,), daemon=True).start()
            return True
        except Exception as e:
            print(f"Command error: {e}")
            return False

    def _execute_command(self, cmd):
        """Execute SSH command in background"""
        try:
            stdin, stdout, stderr = self.ssh.exec_command(cmd, timeout=1)
        except:
            pass  # Ignore timeout, command sent

    def disconnect(self):
        """Disconnect from robot"""
        if self.ssh:
            self.ssh.close()
        self.connected = False


class ConnectDialog(QDialog):
    """Connection dialog for entering robot IP and port"""

    def __init__(self):
        super().__init__()
        self.setWindowTitle("Connect to VETER Robot")
        self.setFixedSize(400, 200)

        layout = QGridLayout()

        # Robot IP
        layout.addWidget(QLabel("Robot IP:"), 0, 0)
        self.ip_input = QLineEdit()
        self.ip_input.setText("100.112.41.76")  # Default Tailscale IP
        layout.addWidget(self.ip_input, 0, 1)

        # SSH Port
        layout.addWidget(QLabel("SSH Port:"), 1, 0)
        self.port_input = QLineEdit()
        self.port_input.setText("22")
        layout.addWidget(self.port_input, 1, 1)

        # Username
        layout.addWidget(QLabel("Username:"), 2, 0)
        self.user_input = QLineEdit()
        self.user_input.setText("jetson")
        layout.addWidget(self.user_input, 2, 1)

        # Password
        layout.addWidget(QLabel("Password:"), 3, 0)
        self.pass_input = QLineEdit()
        self.pass_input.setEchoMode(QLineEdit.EchoMode.Password)
        self.pass_input.setText("cvbp")
        layout.addWidget(self.pass_input, 3, 1)

        # Connect button
        self.connect_btn = QPushButton("Connect")
        self.connect_btn.clicked.connect(self.accept)
        layout.addWidget(self.connect_btn, 4, 0, 1, 2)

        self.setLayout(layout)

        # Make IP input focused
        self.ip_input.setFocus()

    def get_connection_info(self):
        """Return connection parameters"""
        return {
            'host': self.ip_input.text(),
            'port': int(self.port_input.text()),
            'username': self.user_input.text(),
            'password': self.pass_input.text()
        }


class VideoWidget(QWidget):
    """VLC-based video streaming widget"""

    def __init__(self, rtsp_url):
        super().__init__()
        self.rtsp_url = rtsp_url

        # Create VLC instance
        self.instance = vlc.Instance('--no-xlib')
        self.player = self.instance.media_player_new()

        # Create video frame
        self.video_frame = QWidget()
        self.video_frame.setMinimumSize(640, 480)

        layout = QVBoxLayout()
        layout.addWidget(self.video_frame)
        self.setLayout(layout)

        # Set VLC output
        if sys.platform.startswith('darwin'):  # macOS
            self.player.set_nsobject(int(self.video_frame.winId()))

    def start_stream(self):
        """Start RTSP stream playback"""
        media = self.instance.media_new(self.rtsp_url)
        media.get_metas()
        self.player.set_media(media)
        self.player.play()

    def stop_stream(self):
        """Stop stream playback"""
        self.player.stop()


class ControlWidget(QWidget):
    """Robot control widget with virtual joystick"""

    velocity_changed = pyqtSignal(float, float)

    def __init__(self):
        super().__init__()

        self.linear_speed = 0.0
        self.angular_speed = 0.0

        layout = QVBoxLayout()

        # Control group
        control_group = QGroupBox("Robot Control")
        control_layout = QGridLayout()

        # Linear speed slider
        control_layout.addWidget(QLabel("Forward/Backward:"), 0, 0)
        self.linear_slider = QSlider(Qt.Orientation.Horizontal)
        self.linear_slider.setMinimum(-100)
        self.linear_slider.setMaximum(100)
        self.linear_slider.setValue(0)
        self.linear_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.linear_slider.setTickInterval(20)
        self.linear_slider.valueChanged.connect(self.update_velocity)
        control_layout.addWidget(self.linear_slider, 0, 1)

        self.linear_label = QLabel("0.0 m/s")
        control_layout.addWidget(self.linear_label, 0, 2)

        # Angular speed slider
        control_layout.addWidget(QLabel("Left/Right:"), 1, 0)
        self.angular_slider = QSlider(Qt.Orientation.Horizontal)
        self.angular_slider.setMinimum(-100)
        self.angular_slider.setMaximum(100)
        self.angular_slider.setValue(0)
        self.angular_slider.setTickPosition(QSlider.TickPosition.TicksBelow)
        self.angular_slider.setTickInterval(20)
        self.angular_slider.valueChanged.connect(self.update_velocity)
        control_layout.addWidget(self.angular_slider, 1, 1)

        self.angular_label = QLabel("0.0 rad/s")
        control_layout.addWidget(self.angular_label, 1, 2)

        # Stop button
        stop_btn = QPushButton("STOP")
        stop_btn.setStyleSheet("background-color: red; color: white; font-weight: bold; font-size: 16px;")
        stop_btn.clicked.connect(self.emergency_stop)
        control_layout.addWidget(stop_btn, 2, 0, 1, 3)

        # Keyboard hint
        hint_label = QLabel("Keyboard: W/S = forward/back, A/D = left/right, Space = stop")
        hint_label.setStyleSheet("color: gray; font-size: 10px;")
        control_layout.addWidget(hint_label, 3, 0, 1, 3)

        control_group.setLayout(control_layout)
        layout.addWidget(control_group)

        self.setLayout(layout)

    def update_velocity(self):
        """Update velocity from sliders"""
        # Scale to reasonable values: linear 0-2 m/s, angular 0-2 rad/s
        self.linear_speed = self.linear_slider.value() / 50.0  # -2.0 to 2.0 m/s
        self.angular_speed = -self.angular_slider.value() / 50.0  # -2.0 to 2.0 rad/s (inverted)

        self.linear_label.setText(f"{self.linear_speed:.2f} m/s")
        self.angular_label.setText(f"{self.angular_speed:.2f} rad/s")

        self.velocity_changed.emit(self.linear_speed, self.angular_speed)

    def emergency_stop(self):
        """Emergency stop - set all to zero"""
        self.linear_slider.setValue(0)
        self.angular_slider.setValue(0)
        self.update_velocity()


class TelemetryWidget(QWidget):
    """Telemetry display widget"""

    def __init__(self):
        super().__init__()

        layout = QVBoxLayout()

        telemetry_group = QGroupBox("Robot Telemetry")
        telemetry_layout = QVBoxLayout()

        self.telemetry_text = QTextEdit()
        self.telemetry_text.setReadOnly(True)
        self.telemetry_text.setMaximumHeight(150)
        self.telemetry_text.setPlainText("Waiting for telemetry data...")

        telemetry_layout.addWidget(self.telemetry_text)
        telemetry_group.setLayout(telemetry_layout)
        layout.addWidget(telemetry_group)

        self.setLayout(layout)

    def update_telemetry(self, data):
        """Update telemetry display"""
        text = "Robot Status:\n"
        for key, value in data.items():
            text += f"{key}: {value}\n"
        self.telemetry_text.setPlainText(text)


class MainWindow(QMainWindow):
    """Main application window"""

    def __init__(self, connection_info):
        super().__init__()

        self.connection_info = connection_info
        self.robot = RobotConnection()

        self.setWindowTitle(f"VETER Robot Control - {connection_info['host']}")
        self.setMinimumSize(1024, 768)

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Main layout
        main_layout = QHBoxLayout()

        # Left side: Video stream
        rtsp_url = f"rtsp://{connection_info['host']}:8555/camera"
        self.video_widget = VideoWidget(rtsp_url)
        main_layout.addWidget(self.video_widget, stretch=2)

        # Right side: Controls and telemetry
        right_layout = QVBoxLayout()

        # Connection status
        status_label = QLabel(f"Connected to: {connection_info['host']}")
        status_label.setStyleSheet("background-color: green; color: white; padding: 10px; font-weight: bold;")
        right_layout.addWidget(status_label)

        # Control widget
        self.control_widget = ControlWidget()
        self.control_widget.velocity_changed.connect(self.on_velocity_changed)
        right_layout.addWidget(self.control_widget)

        # Telemetry widget
        self.telemetry_widget = TelemetryWidget()
        right_layout.addWidget(self.telemetry_widget)

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

    def connect_to_robot(self):
        """Connect to robot via SSH"""
        success = self.robot.connect(
            self.connection_info['host'],
            self.connection_info['port'],
            self.connection_info['username'],
            self.connection_info['password']
        )

        if success:
            print("Connected to robot successfully!")
        else:
            print("Failed to connect to robot")

    def on_velocity_changed(self, linear, angular):
        """Handle velocity changes from control widget"""
        self.current_linear = linear
        self.current_angular = angular

    def send_velocity(self):
        """Send velocity command to robot"""
        if self.robot.connected:
            self.robot.send_velocity(self.current_linear, self.current_angular)

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
        self.video_widget.stop_stream()
        self.robot.disconnect()
        event.accept()


def main():
    """Main application entry point"""
    app = QApplication(sys.argv)

    # Show connection dialog
    dialog = ConnectDialog()
    if dialog.exec() == QDialog.DialogCode.Accepted:
        connection_info = dialog.get_connection_info()

        # Create and show main window
        window = MainWindow(connection_info)
        window.show()

        sys.exit(app.exec())
    else:
        sys.exit(0)


if __name__ == '__main__':
    main()
