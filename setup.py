from setuptools import find_packages, setup

package_name = 'lawnbot_ros2'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
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
            'talker = src.motor_publisher:main',
            'listener = src.motor_subscriber:main',
        ],
    },
)
