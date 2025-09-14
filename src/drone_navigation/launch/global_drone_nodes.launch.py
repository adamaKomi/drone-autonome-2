from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_navigation',
            executable='goto_position_node',
            name='goto_position_node',
            output='screen'
        ),

        Node(
            package='drone_navigation',
            executable='waypoint_manager_node',
            name='waypoint_manager_node',
            output='screen',
            parameters=[
                {'global_tolerance': 2.0},
                {'default_speed': 5.0},
                {'max_loops': 0}  # 0 = infinite
            ]
        ),
    ])
