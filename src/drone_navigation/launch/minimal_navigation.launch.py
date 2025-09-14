from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    config_dir = os.path.join(get_package_share_directory('drone_navigation'), 'config')
    
    return LaunchDescription([
        Node(
            package='drone_navigation',
            executable='navigation_safety_node',
            name='navigation_safety_node',
            parameters=[os.path.join(config_dir, 'navigation_params.yaml')],
            output='screen'
        ),
        
        Node(
            package='drone_navigation',
            executable='gps_navigation_node',
            name='gps_navigation_node',
            parameters=[os.path.join(config_dir, 'navigation_params.yaml')],
            output='screen'
        ),
        
        Node(
            package='drone_navigation',
            executable='emergency_handler_node',
            name='emergency_handler_node',
            parameters=[os.path.join(config_dir, 'navigation_params.yaml')],
            output='screen'
        )
    ])