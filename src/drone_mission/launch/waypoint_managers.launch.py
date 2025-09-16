#!/usr/bin/env python3
"""
waypoint_managers.launch.py - Fichier de lancement pour les gestionnaires de waypoints
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Arguments de lancement
        DeclareLaunchArgument(
            'gps_manager',
            default_value='true',
            description='Lancer le gestionnaire de waypoints GPS'
        ),
        DeclareLaunchArgument(
            'local_manager',
            default_value='true',
            description='Lancer le gestionnaire de waypoints locaux'
        ),
        DeclareLaunchArgument(
            'test_mode',
            default_value='false',
            description='Lancer en mode test avec les scripts de test'
        ),
        DeclareLaunchArgument(
            'log_level',
            default_value='info',
            description='Niveau de log (debug, info, warn, error)'
        ),

        # Nœud gestionnaire de waypoints GPS
        Node(
            package='drone_mission',
            executable='gps_waypoint_manager_node.py',
            name='gps_waypoint_manager',
            condition=IfCondition(LaunchConfiguration('gps_manager')),
            parameters=[
                {'auto_continue': True},
                {'max_retries': 3},
                {'retry_delay': 5.0}
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen'
        ),

        # Nœud gestionnaire de waypoints locaux
        Node(
            package='drone_mission',
            executable='local_waypoint_manager_node.py',
            name='local_waypoint_manager',
            condition=IfCondition(LaunchConfiguration('local_manager')),
            parameters=[
                {'auto_continue': True},
                {'max_retries': 3},
                {'retry_delay': 3.0},
                {'hold_time': 2.0}
            ],
            arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
            output='screen'
        ),

        # Groupe pour les nœuds de test (optionnel)
        GroupAction(
            condition=IfCondition(LaunchConfiguration('test_mode')),
            actions=[
                Node(
                    package='drone_mission',
                    executable='test_gps_waypoint_manager.py',
                    name='gps_waypoint_tester',
                    condition=IfCondition(LaunchConfiguration('gps_manager')),
                    output='screen'
                ),
                Node(
                    package='drone_mission',
                    executable='test_local_waypoint_manager.py',
                    name='local_waypoint_tester',
                    condition=IfCondition(LaunchConfiguration('local_manager')),
                    output='screen'
                ),
            ]
        ),
    ])
