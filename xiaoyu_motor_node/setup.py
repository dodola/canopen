from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'xiaoyu_motor_node'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools', 'rclpy', 'canopen'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='CANopen motor control node for Xiaoyu robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_node = xiaoyu_motor_node.motor_node:main',
        ],
    },
)
