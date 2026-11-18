import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    params = os.path.join(
        get_package_share_directory('autonomous_planning'), 'config', 'nav2_params.yaml')
    return LaunchDescription([
        Node(package='nav2_planner', executable='planner_server', name='planner_server',
             parameters=[params], output='screen'),
        Node(package='nav2_lifecycle_manager', executable='lifecycle_manager',
             name='lifecycle_manager_planning', parameters=[params], output='screen'),
        Node(package='autonomous_planning', executable='path_planner_node',
             name='path_planner_node', parameters=[params], output='screen'),
        Node(package='autonomous_planning', executable='behavior_planner_node',
             name='behavior_planner_node', parameters=[params], output='screen'),
    ])
