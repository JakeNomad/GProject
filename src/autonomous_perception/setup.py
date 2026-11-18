from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'autonomous_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='GProject Team',
    maintainer_email='todo@todo.todo',
    description='Perception nodes: YOLO camera detector, LiDAR obstacle extraction, radar object extraction.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolo_detector_node = autonomous_perception.yolo_detector_node:main',
            'lidar_processor_node = autonomous_perception.lidar_processor_node:main',
            'radar_processor_node = autonomous_perception.radar_processor_node:main',
        ],
    },
)
