"""
CAN Interface Module

Handles low-level CAN bus communication using python-can library.
Provides methods to send and receive CAN frames for DroneCAN protocol.
"""

import can
import struct
from typing import Optional, Tuple
import threading


class CANInterface:
    """CAN bus interface for DroneCAN communication"""

    def __init__(self, interface='can0', bitrate=1000000):
        """
        Initialize CAN interface

        Args:
            interface: CAN interface name (default: 'can0')
            bitrate: CAN bus bitrate in bps (default: 1000000)
        """
        self.interface = interface
        self.bitrate = bitrate
        self.bus: Optional[can.Bus] = None
        self.running = False
        self.receive_thread: Optional[threading.Thread] = None
        self.callbacks = []

    def start(self) -> bool:
        """
        Start CAN interface

        Returns:
            bool: True if successful, False otherwise
        """
        try:
            self.bus = can.Bus(
                interface='socketcan',
                channel=self.interface,
                bitrate=self.bitrate
            )
            self.running = True
            print(f"[CAN] Initialized {self.interface} at {self.bitrate} bps")
            return True
        except Exception as e:
            print(f"[CAN] ERROR: Failed to initialize: {e}")
            return False

    def stop(self):
        """Stop CAN interface"""
        self.running = False
        if self.receive_thread:
            self.receive_thread.join(timeout=1.0)
        if self.bus:
            self.bus.shutdown()
            self.bus = None
        print("[CAN] Interface stopped")

    def send_frame(self, can_id: int, data: bytes) -> bool:
        """
        Send CAN frame

        Args:
            can_id: CAN identifier
            data: Data bytes to send (up to 8 bytes)

        Returns:
            bool: True if successful, False otherwise
        """
        if not self.bus:
            return False

        try:
            msg = can.Message(
                arbitration_id=can_id,
                data=data,
                is_extended_id=True  # DroneCAN uses 29-bit extended IDs
            )
            self.bus.send(msg)
            return True
        except Exception as e:
            print(f"[CAN] ERROR: Failed to send frame: {e}")
            return False

    def receive_frame(self, timeout: float = 0.1) -> Optional[can.Message]:
        """
        Receive CAN frame (blocking)

        Args:
            timeout: Timeout in seconds

        Returns:
            can.Message or None if timeout/error
        """
        if not self.bus:
            return None

        try:
            msg = self.bus.recv(timeout=timeout)
            return msg
        except Exception as e:
            # Timeout is normal, don't print error
            if "Timeout" not in str(e):
                print(f"[CAN] ERROR: Failed to receive frame: {e}")
            return None

    def register_callback(self, callback):
        """
        Register callback for received messages

        Args:
            callback: Function to call with received can.Message
        """
        if callback not in self.callbacks:
            self.callbacks.append(callback)

    def start_receive_thread(self):
        """Start background thread for receiving messages"""
        if self.receive_thread and self.receive_thread.is_alive():
            return

        self.receive_thread = threading.Thread(
            target=self._receive_loop,
            daemon=True
        )
        self.receive_thread.start()

    def _receive_loop(self):
        """Background receive loop"""
        print("[CAN] Receive thread started")
        while self.running:
            msg = self.receive_frame(timeout=0.1)
            if msg:
                # Call all registered callbacks
                for callback in self.callbacks:
                    try:
                        callback(msg)
                    except Exception as e:
                        print(f"[CAN] ERROR in callback: {e}")

        print("[CAN] Receive thread stopped")


class DroneCAN:
    """DroneCAN protocol helper functions"""

    # DroneCAN message type IDs
    MSG_TYPE_HEARTBEAT = 0x155  # NodeStatus (341)
    MSG_TYPE_ESC_COMMAND = 0x406  # ESC RawCommand (1030)
    MSG_TYPE_RANGE_SENSOR = 0x41A  # RangeSensor (1050)
    MSG_TYPE_AIR_DATA = 0x424  # AirData (1060)
    MSG_TYPE_COLLISION_WARNING = 0x42E  # CollisionWarning (1070)
    MSG_TYPE_SERVO_COMMAND = 0x480  # ServoCommand (1152)
    MSG_TYPE_LED_COMMAND = 0x490  # LEDCommand (1168)

    @staticmethod
    def extract_message_info(can_id: int) -> Tuple[int, int]:
        """
        Extract message type and source node ID from DroneCAN CAN ID

        DroneCAN CAN ID format (29 bits):
        [28:24] Priority (5 bits)
        [23:8]  Message Type ID (16 bits)
        [7:1]   Source Node ID (7 bits)
        [0]     Service/Message flag

        Args:
            can_id: 29-bit CAN identifier

        Returns:
            tuple: (message_type, source_node_id)
        """
        priority = (can_id >> 24) & 0x1F
        msg_type = (can_id >> 8) & 0xFFFF
        node_id = (can_id >> 1) & 0x7F
        return msg_type, node_id

    @staticmethod
    def build_can_id(msg_type: int, node_id: int, priority: int = 8) -> int:
        """
        Build DroneCAN CAN ID (29-bit extended identifier)

        DroneCAN CAN ID format (29 bits):
        [28:24] Priority (5 bits) - 0=highest, 31=lowest
        [23:8]  Message Type ID (16 bits)
        [7:1]   Source Node ID (7 bits)
        [0]     Service/Message flag (0 = message broadcast)

        Args:
            msg_type: DroneCAN message type (16-bit)
            node_id: Source node ID (0-127)
            priority: Message priority (0-31, default 8=HIGH)

        Returns:
            int: Complete 29-bit CAN identifier
        """
        can_id = 0
        can_id |= (priority & 0x1F) << 24           # Priority [28:24]
        can_id |= (msg_type & 0xFFFF) << 8          # Message Type [23:8]
        can_id |= (node_id & 0x7F) << 1             # Source Node [7:1]
        can_id |= 0                                  # Message broadcast [0]
        return can_id

    @staticmethod
    def parse_heartbeat(data: bytes) -> dict:
        """
        Parse DroneCAN NodeStatus (heartbeat) message

        Args:
            data: CAN frame data bytes

        Returns:
            dict: Parsed heartbeat data
        """
        if len(data) < 7:
            return None

        uptime = struct.unpack('<I', data[0:4])[0]
        health = data[4] & 0x03
        mode = (data[4] >> 2) & 0x07

        return {
            'uptime_sec': uptime,
            'health': health,
            'mode': mode
        }

    @staticmethod
    def parse_range_sensor(data: bytes) -> dict:
        """
        Parse RangeSensor message (ultrasonic data)

        Args:
            data: CAN frame data bytes

        Returns:
            dict: Parsed range sensor data
        """
        if len(data) < 8:
            return None

        distances = []
        for i in range(4):
            dist_mm = struct.unpack('<H', data[i*2:(i+1)*2])[0]
            if dist_mm == 0xFFFF:
                distances.append(None)  # Invalid reading
            else:
                distances.append(dist_mm / 10.0)  # Convert mm to cm

        return {
            'front': distances[0],
            'rear': distances[1],
            'left': distances[2],
            'right': distances[3]
        }

    @staticmethod
    def parse_air_data(data: bytes) -> dict:
        """
        Parse AirData message (BME280 data)

        Args:
            data: CAN frame data bytes

        Returns:
            dict: Parsed air data
        """
        if len(data) < 8:
            return None

        temp_raw = struct.unpack('<h', data[0:2])[0]
        hum_raw = struct.unpack('<H', data[2:4])[0]
        pres_raw = struct.unpack('<I', data[4:8])[0]

        return {
            'temperature': temp_raw / 100.0,  # 0.01 degC resolution
            'humidity': hum_raw / 100.0,  # 0.01% resolution
            'pressure': pres_raw / 100.0  # 0.01 hPa resolution
        }

    @staticmethod
    def parse_collision_warning(data: bytes) -> dict:
        """
        Parse CollisionWarning message

        Args:
            data: CAN frame data bytes

        Returns:
            dict: Parsed collision warning
        """
        if len(data) < 4:
            return None

        direction = data[0] & 0x03
        dist_mm = struct.unpack('<H', data[1:3])[0]
        severity = data[3]

        direction_names = ['front', 'rear', 'left', 'right']

        return {
            'direction': direction_names[direction] if direction < 4 else 'unknown',
            'distance_cm': dist_mm / 10.0,
            'severity': severity  # 0=info, 1=warning, 2=critical
        }

    # Transfer ID counter for ESC commands (0-31)
    _esc_transfer_id = 0

    @staticmethod
    def build_esc_command(node_id: int, left_cmd: int, right_cmd: int) -> Tuple[int, bytes]:
        """
        Build ESC RawCommand message

        DroneCAN ESC RawCommand payload format:
        [0-1] Left motor command (int16_t, little-endian, -8191 to 8191)
        [2-3] Right motor command (int16_t, little-endian, -8191 to 8191)
        [4]   Tail byte (transfer ID)

        Args:
            node_id: Source node ID
            left_cmd: Left motor command (-8191 to 8191)
            right_cmd: Right motor command (-8191 to 8191)

        Returns:
            tuple: (can_id, data_bytes)
        """
        can_id = DroneCAN.build_can_id(DroneCAN.MSG_TYPE_ESC_COMMAND, node_id)

        # Clamp to valid range
        left_cmd = max(min(left_cmd, 8191), -8191)
        right_cmd = max(min(right_cmd, 8191), -8191)

        # Pack as signed int16_t little-endian (2 bytes per motor)
        motor_data = struct.pack('<hh', left_cmd, right_cmd)

        # Build full payload: motor commands + tail byte
        data = bytearray(motor_data)

        # Tail byte: Start(1) | End(1) | Toggle(0) | TransferID(5 bits)
        tail = 0xC0 | (DroneCAN._esc_transfer_id & 0x1F)
        data.append(tail)

        # Increment transfer ID (0-31 wrap)
        DroneCAN._esc_transfer_id = (DroneCAN._esc_transfer_id + 1) & 0x1F

        return can_id, bytes(data)

    @staticmethod
    def build_servo_command(node_id: int, pan_deg: int, tilt_deg: int) -> Tuple[int, bytes]:
        """
        Build ServoCommand message

        Args:
            node_id: Source node ID
            pan_deg: Pan angle (0-180 degrees)
            tilt_deg: Tilt angle (0-180 degrees)

        Returns:
            tuple: (can_id, data_bytes)
        """
        can_id = DroneCAN.build_can_id(DroneCAN.MSG_TYPE_SERVO_COMMAND, node_id)

        data = struct.pack('<HH', pan_deg, tilt_deg)

        return can_id, data

    @staticmethod
    def build_led_command(node_id: int, mode: int, brightness: int,
                         fl: int, fr: int, rl: int, rr: int) -> Tuple[int, bytes]:
        """
        Build LEDCommand message

        Args:
            node_id: Source node ID
            mode: LED mode
            brightness: Overall brightness (0-255)
            fl, fr, rl, rr: Individual channel brightness (0-255)

        Returns:
            tuple: (can_id, data_bytes)
        """
        can_id = DroneCAN.build_can_id(DroneCAN.MSG_TYPE_LED_COMMAND, node_id)

        data = struct.pack('BBBBBB', mode, brightness, fl, fr, rl, rr)

        return can_id, data
