from setuptools import find_packages, setup

package_name = 'veter_camera'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/camera.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Eugene Melnik',
    maintainer_email='eugene.a.melnik@gmail.com',
    description='VETER Camera Publisher - IMX477 camera stream to ROS2',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'camera_publisher = veter_camera.camera_publisher:main'
        ],
    },
)
