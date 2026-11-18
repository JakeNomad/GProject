from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'autonomous_fusion'

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
    description='Multi-sensor fusion and multi-object tracking with a Kalman filter.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kalman_fusion_node = autonomous_fusion.kalman_fusion_node:main',
        ],
    },
)
