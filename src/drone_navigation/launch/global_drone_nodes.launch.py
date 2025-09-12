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
        # Ajoute ici d'autres noeuds si besoin
    ])
