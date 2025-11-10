from setuptools import setup
import os
from glob import glob

package_name = 'veter_channel_manager'

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
    description='Multi-channel communication manager with dynamic failover for VETER_NEXT',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'channel_manager_node = veter_channel_manager.channel_manager_node:main',
        ],
    },
)
