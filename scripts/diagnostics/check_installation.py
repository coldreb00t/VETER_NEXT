#!/usr/bin/env python3
"""
VETER_NEXT - Installation Verification Script
Checks all required software components are properly installed
"""

import sys
import subprocess
import importlib

def check_command(cmd, name):
    """Check if a command-line tool is available"""
    try:
        result = subprocess.run([cmd, '--version'],
                              capture_output=True,
                              text=True,
                              timeout=5)
        version = result.stdout.strip() or result.stderr.strip()
        first_line = version.split('\n')[0]
        print(f"✓ {name}: {first_line}")
        return True
    except (FileNotFoundError, subprocess.TimeoutExpired):
        print(f"✗ {name}: NOT FOUND")
        return False
    except Exception as e:
        print(f"✗ {name}: ERROR - {e}")
        return False

def check_python_module(module_name, import_name=None):
    """Check if a Python module is installed"""
    if import_name is None:
        import_name = module_name

    try:
        mod = importlib.import_module(import_name)
        version = getattr(mod, '__version__', 'installed')
        print(f"✓ {module_name}: {version}")
        return True
    except ImportError:
        print(f"✗ {module_name}: NOT FOUND")
        return False
    except Exception as e:
        print(f"✗ {module_name}: ERROR - {e}")
        return False

def check_can_interface():
    """Check if CAN interface is available"""
    try:
        result = subprocess.run(['ip', 'link', 'show', 'can0'],
                              capture_output=True,
                              text=True,
                              timeout=5)
        if result.returncode == 0:
            print("✓ CAN interface: can0 found")
            return True
        else:
            print("✗ CAN interface: can0 not configured")
            return False
    except Exception as e:
        print(f"✗ CAN interface: ERROR - {e}")
        return False

def main():
    print("=" * 60)
    print("VETER_NEXT - Installation Verification")
    print("=" * 60)
    print()

    results = {
        'commands': True,
        'python_libs': True,
        'can': True
    }

    # Check command-line tools
    print("Command-Line Tools:")
    print("-" * 60)
    results['commands'] &= check_command('python3', 'Python')
    results['commands'] &= check_command('pip3', 'pip3')
    results['commands'] &= check_command('pio', 'PlatformIO')
    results['commands'] &= check_command('candump', 'CAN utilities')
    results['commands'] &= check_command('git', 'Git')
    print()

    # Check Python libraries
    print("Python Libraries:")
    print("-" * 60)
    results['python_libs'] &= check_python_module('pydronecan', 'dronecan')
    results['python_libs'] &= check_python_module('numpy')
    results['python_libs'] &= check_python_module('opencv-python', 'cv2')
    results['python_libs'] &= check_python_module('python-telegram-bot', 'telegram')
    results['python_libs'] &= check_python_module('pytest')
    results['python_libs'] &= check_python_module('pyyaml', 'yaml')
    print()

    # Check CAN interface
    print("Hardware Interfaces:")
    print("-" * 60)
    results['can'] = check_can_interface()
    print()

    # Check ROS2 (optional for now)
    print("Optional Components:")
    print("-" * 60)
    check_command('ros2', 'ROS2 Humble')
    print()

    # Summary
    print("=" * 60)
    print("Summary:")
    print("=" * 60)
    all_ok = all(results.values())

    if all_ok:
        print("✓ All required components are installed!")
        print("  Ready to start development.")
        return 0
    else:
        print("✗ Some components are missing:")
        if not results['commands']:
            print("  - Check command-line tools")
        if not results['python_libs']:
            print("  - Check Python libraries")
        if not results['can']:
            print("  - Check CAN interface configuration")
        print()
        print("  Run setup scripts or check INSTALLATION.md")
        return 1

if __name__ == '__main__':
    sys.exit(main())
