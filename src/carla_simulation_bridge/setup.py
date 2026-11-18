from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'carla_simulation_bridge'

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
    description='Bridge between the CARLA simulator and ROS 2: spawns the ego vehicle and sensors, publishes sensor data, applies control commands.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'carla_bridge_node = carla_simulation_bridge.carla_bridge_node:main',
        ],
    },
)
