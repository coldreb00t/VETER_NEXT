#!/usr/bin/env python3
"""
ESP32-S3 Serial Monitor
Monitors serial output from ESP32-S3 Motor Controller

Usage:
    python3 scripts/esp32_monitor.py [port] [baudrate]

Examples:
    python3 scripts/esp32_monitor.py                    # Default: /dev/ttyACM1 115200
    python3 scripts/esp32_monitor.py /dev/ttyACM1      # Custom port
    python3 scripts/esp32_monitor.py /dev/ttyACM1 115200  # Custom port and baud
"""

import serial
import sys
import time
from datetime import datetime

# Default settings
DEFAULT_PORT = '/dev/ttyACM1'
DEFAULT_BAUDRATE = 115200

def print_header(port, baudrate):
    """Print monitor header"""
    print('=' * 70)
    print(f'  ESP32-S3 Motor Controller - Serial Monitor')
    print(f'  Port: {port}')
    print(f'  Baudrate: {baudrate}')
    print(f'  Started: {datetime.now().strftime("%Y-%m-%d %H:%M:%S")}')
    print('=' * 70)
    print('  Press Ctrl+C to exit')
    print('=' * 70)
    print()

def monitor_serial(port=DEFAULT_PORT, baudrate=DEFAULT_BAUDRATE):
    """Monitor serial port and print output"""
    try:
        # Open serial port
        ser = serial.Serial(port, baudrate, timeout=0.1)
        print_header(port, baudrate)

        # Give ESP32 time to start if needed
        time.sleep(1)

        # Read continuously
        while True:
            try:
                if ser.in_waiting:
                    data = ser.read(ser.in_waiting)
                    # Decode and print
                    text = data.decode('utf-8', errors='replace')
                    print(text, end='', flush=True)
            except KeyboardInterrupt:
                print('\n\n' + '=' * 70)
                print('  Serial Monitor Stopped')
                print('=' * 70)
                break
            except Exception as e:
                print(f'\n[Error reading serial: {e}]')
                time.sleep(1)

        ser.close()

    except serial.SerialException as e:
        print(f'Error opening {port}: {e}')
        print(f'\nAvailable ports:')
        import glob
        ports = glob.glob('/dev/ttyACM*') + glob.glob('/dev/ttyUSB*')
        for p in ports:
            print(f'  {p}')
        sys.exit(1)
    except Exception as e:
        print(f'Error: {e}')
        sys.exit(1)

if __name__ == '__main__':
    # Parse arguments
    port = DEFAULT_PORT
    baudrate = DEFAULT_BAUDRATE

    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        try:
            baudrate = int(sys.argv[2])
        except ValueError:
            print(f'Error: Invalid baudrate "{sys.argv[2]}"')
            sys.exit(1)

    # Start monitoring
    monitor_serial(port, baudrate)
