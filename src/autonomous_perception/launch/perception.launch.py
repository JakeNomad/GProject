import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('autonomous_perception'), 'config', 'perception.yaml')
    return LaunchDescription([
        Node(package='autonomous_perception', executable=name, name=name,
             parameters=[params], output='screen')
        for name in ('yolo_detector_node', 'lidar_processor_node', 'radar_processor_node')
    ])
