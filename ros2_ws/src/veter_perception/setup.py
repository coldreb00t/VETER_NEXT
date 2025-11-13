from setuptools import setup
import os
from glob import glob

package_name = 'veter_perception'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eugene Melnik',
    maintainer_email='eugene.a.melnik@gmail.com',
    description='Computer vision and object detection for VETER robot using YOLOv8',
    license='MIT',
    entry_points={
        'console_scripts': [
            'yolo_detector = veter_perception.yolo_detector:main',
            'object_tracker = veter_perception.object_tracker:main',
        ],
    },
)
