from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'veter_dronecan_bridge'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')),
        # Include config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eugene Melnik',
    maintainer_email='jetson@veter-next.local',
    description='DroneCAN to ROS2 bridge for VETER_NEXT robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'dronecan_bridge = veter_dronecan_bridge.dronecan_bridge_node:main',
        ],
    },
)
