from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Lance les noeud du package drone_interface avec configuration pour ROS2 Humble
    """

    arm_disarm_node = Node(
        package='drone_interface',
        executable='arm_disarm_node',
        name='arm_disarm_node',
        output='screen'
    )

    mode_node = Node(
        package='drone_interface',
        executable='mode_node',
        name='mode_node',
        output='screen'
    )

    takeoff_land_node = Node(
        package='drone_interface',
        executable='takeoff_land_node',
        name='takeoff_land_node',
        output='screen'
    )

    # Ajoute ici d'autres noeuds si besoin

    return LaunchDescription([
        arm_disarm_node,
        mode_node,
        takeoff_land_node
    ])
