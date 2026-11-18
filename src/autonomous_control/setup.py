from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'autonomous_control'

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
    description='Vehicle control: Pure Pursuit lateral and PID longitudinal controllers.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vehicle_controller_node = autonomous_control.vehicle_controller_node:main',
        ],
    },
)
