import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('autonomous_fusion'), 'config', 'fusion.yaml')
    return LaunchDescription([
        Node(package='autonomous_fusion', executable='kalman_fusion_node',
             name='kalman_fusion_node', parameters=[params], output='screen'),
    ])
