from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Utilise le chemin absolu vers le fichier de config
    config_file = '/home/adama133/ros2_ws/src/drone_navigation/config/navigation_params.yaml'
    
    return LaunchDescription([
        # Nœud de sécurité
        Node(
            package='drone_navigation',
            executable='navigation_safety_node',
            name='navigation_safety_node',
            parameters=[config_file],
            output='screen'
        ),
        
        # Nœud de navigation GPS
        Node(
            package='drone_navigation',
            executable='gps_navigation_node',
            name='gps_navigation_node',
            parameters=[config_file],
            output='screen'
        ),
        
        # Manager de mission
        Node(
            package='drone_navigation',
            executable='mission_manager_node',
            name='mission_manager_node',
            parameters=[config_file],
            output='screen'
        ),
        
        # Manager de waypoints
        Node(
            package='drone_navigation',
            executable='waypoint_manager_node',
            name='waypoint_manager_node',
            parameters=[config_file],
            output='screen'
        ),
        
        # Gestionnaire d'urgence
        Node(
            package='drone_navigation',
            executable='emergency_handler_node',
            name='emergency_handler_node',
            parameters=[config_file],
            output='screen'
        ),
        
        # Superviseur de navigation
        Node(
            package='drone_navigation',
            executable='navigation_supervisor_node',
            name='navigation_supervisor_node',
            parameters=[config_file],
            output='screen'
        ),

        # Nœud de navigation locale
        Node(
            package='drone_navigation',
            executable='local_navigation_node',
            name='local_navigation_node',
            parameters=[config_file],
            output='screen'
        ),

    ])