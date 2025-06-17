from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'xiaoyu_motor_node'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Example for launch files if added later:
        # (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools', 'rclpy', 'canopen'],
    zip_safe=True,
    maintainer='user', # Replace with actual maintainer
    maintainer_email='user@example.com', # Replace with actual maintainer email
    description='ROS2 node for Xiaoyu motor device with CANopen using local messages',
    license='Apache License 2.0', # Or your preferred license
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_node = xiaoyu_motor_node.motor_node:main',
        ],
    },
)
