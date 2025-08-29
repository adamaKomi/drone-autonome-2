from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_mission',
            executable='mission_node',
            name='drone_mission',
            output='screen',
            parameters=[{'mission_file': '/home/adama133/ros2_ws/src/drone_mission/missions/mission.yaml'}],
        )
    ])
