from glob import glob
from setuptools import find_packages, setup
import os

package_name = 'lawnbot_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Sean Feigis',
    maintainer_email='sfeigis@yahoo.com',
    description='ROS2 Package for lawnbot 41x group',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main = src.main_logic_controller:main',
            'motor = src.motor_controller:main',
            'path = src.path_publisher:main',
            'light = src.light_controller:main',
            'pump = src.pump_controller:main',
            'picamera = src.picamera_controller:main',
            'object_detector = src.object_detector:main'
        ],
    },
)