#!/usr/bin/env python3
"""
Launch file pour le système de navigation du drone autonome

Démarre le nœud de navigation principal avec configuration paramétrable.
Gère le lancement de tous les composants de navigation avec les paramètres appropriés.

Usage:
    ros2 launch drone_navigation navigation_launch.py [arguments]

Arguments:
    use_sim_time: Utilise le temps de simulation (défaut: True)
    config_file: Chemin vers le fichier de configuration YAML
    namespace: Namespace pour les topics et services (défaut: drone_nav)
    log_level: Niveau de logging (défaut: info)
    respawn_delay: Délai en secondes avant redémarrage en cas de crash (défaut: 2.0)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Chemin vers le package
    pkg_share = FindPackageShare(package='drone_navigation').find('drone_navigation')

    # Arguments de lancement
    declare_use_sim_time_cmd = DeclareLaunchArgument(
        'use_sim_time', default_value='True', description='Utilise le temps de simulation si true'
    )

    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([pkg_share, 'config', 'navigation_params.yaml']),
        description='Chemin vers le fichier de configuration principal YAML'
    )

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace', default_value='drone_nav', description='Namespace pour les topics et services ROS'
    )

    declare_log_level_cmd = DeclareLaunchArgument(
        'log_level', default_value='info', description='Niveau de logging (debug, info, warn, error, fatal)'
    )

    declare_respawn_delay_cmd = DeclareLaunchArgument(
        'respawn_delay', default_value='2.0', description='Délai en secondes avant redémarrage en cas de crash'
    )

    # Nodes
    navigation_node = Node(
        package='drone_navigation',
        executable='navigation_node',
        name='navigation_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=[
            '--ros-args',
            '--log-level', LaunchConfiguration('log_level'),
            '--params-file', LaunchConfiguration('config_file')
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0,
        emulate_tty=True
    )

    return LaunchDescription([
        # Arguments
        declare_use_sim_time_cmd,
        declare_config_file_cmd,
        declare_namespace_cmd,
        declare_log_level_cmd,
        declare_respawn_delay_cmd,

        # Nodes
        navigation_node
    ])
