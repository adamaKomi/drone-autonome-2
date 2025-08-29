from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_interface',
            executable='interface_node',
            name='drone_interface',
            output='screen',
        ),
    ])
