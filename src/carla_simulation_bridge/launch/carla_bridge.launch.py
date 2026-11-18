import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('carla_simulation_bridge'), 'config', 'carla_bridge.yaml')
    return LaunchDescription([
        Node(package='carla_simulation_bridge', executable='carla_bridge_node',
             name='carla_bridge_node', parameters=[params], output='screen'),
        # Static sensor extrinsics (base_link -> sensor), matching the CARLA spawn transforms.
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '1.5', '--z', '1.6', '--frame-id', 'base_link',
                        '--child-frame-id', 'camera_front']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--z', '2.4', '--frame-id', 'base_link', '--child-frame-id', 'lidar']),
        Node(package='tf2_ros', executable='static_transform_publisher',
             arguments=['--x', '2.0', '--z', '1.0', '--frame-id', 'base_link',
                        '--child-frame-id', 'radar_front']),
    ])
