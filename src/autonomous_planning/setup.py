from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'autonomous_planning'

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
    description='Planning layer: Nav2 global path, behavior planner and local path generation.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'behavior_planner_node = autonomous_planning.behavior_planner_node:main',
            'path_planner_node = autonomous_planning.path_planner_node:main',
        ],
    },
)
