#!/usr/bin/env python3
"""
=============================================================================
LAUNCH FILE - Système drone autonome complet
=============================================================================
Lance l'ensemble du système drone pour les missions de pollinisation:
- Interface drone (MAVROS)
- Navigation
- Mission
- Vision
- Collecte de données

Usage:
    ros2 launch drone_vision complete_system.launch.py
=============================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Génère la description de lancement complet"""
    
    # Arguments
    simulation_arg = DeclareLaunchArgument(
        'simulation',
        default_value='true',
        description='Mode simulation (SITL)'
    )
    
    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value='pollination_mission.json',
        description='Fichier de mission à charger'
    )
    
    # MAVROS (interface drone)
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mavros'),
                'launch',
                'px4.launch'
            ])
        ]),
        launch_arguments={
            'fcu_url': 'udp://:14540@127.0.0.1:14557',
            'gcs_url': '',
            'target_system_id': '1',
            'target_component_id': '1'
        }.items()
    )
    
    # Interface drone
    drone_interface_node = Node(
        package='drone_interface',
        executable='drone_interface_node',
        name='drone_interface',
        output='screen'
    )
    
    # Navigation
    navigation_node = Node(
        package='drone_navigation',
        executable='navigation_node',
        name='drone_navigation',
        output='screen'
    )
    
    # Mission
    mission_node = Node(
        package='drone_mission',
        executable='mission_node',
        name='drone_mission',
        output='screen',
        parameters=[{
            'mission_file': LaunchConfiguration('mission_file')
        }]
    )
    
    # Système de vision
    vision_system_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('drone_vision'),
                'launch',
                'vision_system.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_camera': 'true',
            'debug_mode': 'false'
        }.items()
    )
    
    return LaunchDescription([
        # Arguments
        simulation_arg,
        mission_file_arg,
        
        # MAVROS
        mavros_launch,
        
        # Nœuds drone
        drone_interface_node,
        navigation_node,
        mission_node,
        
        # Système de vision
        vision_system_launch,
    ])
