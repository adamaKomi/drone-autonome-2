#!/usr/bin/env python3
"""
Launch file principal pour le système de navigation autonome
Auteur: Adama Komi
Date: 2025-09-11

Ce launch file orchestre le démarrage de tous les micro-nœuds du système
de navigation dans l'ordre approprié avec leurs dépendances.

Références:
- ROS2 Launch: https://docs.ros.org/en/humble/Tutorials/Intermediate/Launch/Launch-Main.html
- Launch XML: https://docs.ros.org/en/humble/How-To-Guides/Launch-file-different-formats.html
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, OpaqueFunction
from launch.actions import IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, LifecycleNode
from launch_ros.actions import SetParameter, SetParametersFromFile
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource

import os


def generate_launch_description():
    """Génère la description de lancement pour le système complet"""
    
    # Arguments de lancement
    declare_simulation_mode = DeclareLaunchArgument(
        'simulation_mode',
        default_value='true',
        description='Mode simulation (true/false)'
    )
    
    declare_mavros_enabled = DeclareLaunchArgument(
        'mavros_enabled',
        default_value='true',
        description='Activer MAVROS (true/false)'
    )
    
    declare_auto_start = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Démarrage automatique (true/false)'
    )
    
    declare_log_level = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Niveau de log (debug/info/warn/error)'
    )
    
    declare_namespace = DeclareLaunchArgument(
        'namespace',
        default_value='drone_nav',
        description='Namespace ROS2'
    )
    
    # Chemins des fichiers de configuration
    pkg_share = FindPackageShare('drone_navigation')
    navigation_params_file = PathJoinSubstitution([
        pkg_share, 'config', 'navigation_params.yaml'
    ])
    pid_params_file = PathJoinSubstitution([
        pkg_share, 'config', 'pid_tuning.yaml'
    ])
    
    # Paramètres globaux
    global_parameters = [
        SetParameter('use_sim_time', LaunchConfiguration('simulation_mode')),
        SetParameter('log_level', LaunchConfiguration('log_level'))
    ]
    
    # COUCHE 1: INFRASTRUCTURE (Nœuds de base)
    parameter_manager_node = LifecycleNode(
        package='drone_navigation',
        executable='parameter_manager_node',
        name='parameter_manager_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            navigation_params_file,
            {
                'config_files': [
                    str(navigation_params_file.perform(None)),
                    str(pid_params_file.perform(None))
                ],
                'validation_enabled': True,
                'auto_save_enabled': False
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen'
    )
    
    mavros_interface_node = Node(
        package='drone_navigation',
        executable='mavros_interface_node',
        name='mavros_interface_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            navigation_params_file,
            {
                'mavros_namespace': '/mavros',
                'simulation_mode': LaunchConfiguration('simulation_mode'),
                'connection_timeout': 5.0
            }
        ],
        condition=IfCondition(LaunchConfiguration('mavros_enabled')),
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen'
    )
    
    core_navigation_node = LifecycleNode(
        package='drone_navigation',
        executable='core_navigation_node',
        name='core_navigation_node',
        namespace=LaunchConfiguration('namespace'),
        parameters=[
            navigation_params_file,
            {
                'auto_start': LaunchConfiguration('auto_start'),
                'safety_mode_enabled': True
            }
        ],
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
        output='screen'
    )
    
    # COUCHE 2: PLANIFICATION (avec délai pour dépendances)
    trajectory_planner_node = TimerAction(
        period=2.0,  # Attendre 2s que l'infrastructure soit prête
        actions=[
            Node(
                package='drone_navigation',
                executable='trajectory_planner_node',
                name='trajectory_planner_node',
                namespace=LaunchConfiguration('namespace'),
                parameters=[
                    navigation_params_file,
                    {
                        'default_planner': 'astar',
                        'optimization_enabled': True,
                        'auto_planning_enabled': True
                    }
                ],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                output='screen'
            )
        ]
    )
    
    coverage_pattern_node = TimerAction(
        period=2.5,
        actions=[
            Node(
                package='drone_navigation',
                executable='coverage_pattern_node',
                name='coverage_pattern_node',
                namespace=LaunchConfiguration('namespace'),
                parameters=[
                    navigation_params_file,
                    {
                        'default_pattern': 'zigzag',
                        'overlap_percentage': 20.0
                    }
                ],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                output='screen'
            )
        ]
    )
    
    path_optimizer_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='drone_navigation',
                executable='path_optimizer_node',
                name='path_optimizer_node',
                namespace=LaunchConfiguration('namespace'),
                parameters=[
                    navigation_params_file,
                    {
                        'genetic_algorithm_enabled': True,
                        'population_size': 50,
                        'max_generations': 100
                    }
                ],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                output='screen'
            )
        ]
    )
    
    # COUCHE 3: CONTRÔLE (avec délai pour planification)
    position_controller_node = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='drone_navigation',
                executable='position_controller_node',
                name='position_controller_node',
                namespace=LaunchConfiguration('namespace'),
                parameters=[
                    pid_params_file,
                    {
                        'control_frequency': 50.0,
                        'precision_mode_enabled': True
                    }
                ],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                output='screen'
            )
        ]
    )
    
    trajectory_follower_node = TimerAction(
        period=4.5,
        actions=[
            Node(
                package='drone_navigation',
                executable='trajectory_follower_node',
                name='trajectory_follower_node',
                namespace=LaunchConfiguration('namespace'),
                parameters=[
                    navigation_params_file,
                    {
                        'following_frequency': 20.0,
                        'lookahead_distance': 2.0
                    }
                ],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                output='screen'
            )
        ]
    )
    
    # COUCHE 4: SÉCURITÉ (priorité critique)
    geofence_monitor_node = TimerAction(
        period=1.0,  # Démarrage rapide pour la sécurité
        actions=[
            Node(
                package='drone_navigation',
                executable='geofence_monitor_node',
                name='geofence_monitor_node',
                namespace=LaunchConfiguration('namespace'),
                parameters=[
                    navigation_params_file,
                    {
                        'geofence_enabled': True,
                        'violation_action': 'stop',
                        'warning_distance': 5.0
                    }
                ],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                output='screen'
            )
        ]
    )
    
    emergency_manager_node = TimerAction(
        period=1.5,
        actions=[
            Node(
                package='drone_navigation',
                executable='emergency_manager_node',
                name='emergency_manager_node',
                namespace=LaunchConfiguration('namespace'),
                parameters=[
                    navigation_params_file,
                    {
                        'emergency_landing_enabled': True,
                        'return_home_enabled': True,
                        'response_timeout': 2.0
                    }
                ],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                output='screen'
            )
        ]
    )
    
    obstacle_avoidance_node = TimerAction(
        period=3.5,
        actions=[
            Node(
                package='drone_navigation',
                executable='obstacle_avoidance_node',
                name='obstacle_avoidance_node',
                namespace=LaunchConfiguration('namespace'),
                parameters=[
                    navigation_params_file,
                    {
                        'avoidance_method': 'potential_field',
                        'safety_distance': 5.0,
                        'reactive_enabled': True
                    }
                ],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                output='screen'
            )
        ]
    )
    
    # COUCHE 5: INTERFACE (après que le système soit stable)
    mission_manager_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='drone_navigation',
                executable='mission_manager_node',
                name='mission_manager_node',
                namespace=LaunchConfiguration('namespace'),
                parameters=[
                    navigation_params_file,
                    {
                        'mission_validation_enabled': True,
                        'auto_retry_enabled': False
                    }
                ],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                output='screen'
            )
        ]
    )
    
    status_monitor_node = TimerAction(
        period=5.5,
        actions=[
            Node(
                package='drone_navigation',
                executable='status_monitor_node',
                name='status_monitor_node',
                namespace=LaunchConfiguration('namespace'),
                parameters=[
                    navigation_params_file,
                    {
                        'monitoring_frequency': 2.0,
                        'health_check_enabled': True,
                        'diagnostics_enabled': True
                    }
                ],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                output='screen'
            )
        ]
    )
    
    cli_bridge_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='drone_navigation',
                executable='cli_bridge_node',
                name='cli_bridge_node',
                namespace=LaunchConfiguration('namespace'),
                parameters=[
                    navigation_params_file,
                    {
                        'cli_enabled': True,
                        'command_timeout': 30.0
                    }
                ],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                output='screen'
            )
        ]
    )
    
    # COUCHE 6: UTILITAIRES (optionnels)
    calibration_node = TimerAction(
        period=7.0,
        actions=[
            Node(
                package='drone_navigation',
                executable='calibration_node',
                name='calibration_node',
                namespace=LaunchConfiguration('namespace'),
                parameters=[
                    navigation_params_file,
                    {
                        'auto_calibration_enabled': False,
                        'calibration_on_startup': False
                    }
                ],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                output='screen'
            )
        ]
    )
    
    data_logger_node = TimerAction(
        period=7.5,
        actions=[
            Node(
                package='drone_navigation',
                executable='data_logger_node',
                name='data_logger_node',
                namespace=LaunchConfiguration('namespace'),
                parameters=[
                    navigation_params_file,
                    {
                        'logging_enabled': True,
                        'log_all_topics': False,
                        'compression_enabled': True
                    }
                ],
                arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
                output='screen'
            )
        ]
    )
    
    # Lancement conditionnel de MAVROS (si pas déjà démarré)
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mavros'),
                'launch',
                'apm.launch.py'
            ])
        ]),
        launch_arguments={
            'fcu_url': 'udp://127.0.0.1:14550@14555',
            'gcs_url': '',
            'target_system_id': '1',
            'target_component_id': '1'
        }.items(),
        condition=IfCondition(LaunchConfiguration('mavros_enabled'))
    )
    
    # Action pour configurer et activer les nœuds lifecycle
    def configure_lifecycle_nodes(context):
        """Configure et active les nœuds lifecycle"""
        from launch.actions import ExecuteProcess
        import time
        
        # Attendre que les nœuds soient démarrés
        configure_actions = []
        
        # Délai avant configuration
        configure_actions.append(
            TimerAction(
                period=8.0,  # Attendre que tous les nœuds soient démarrés
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'lifecycle', 'set', 
                             f"/{LaunchConfiguration('namespace').perform(context)}/parameter_manager_node",
                             'configure'],
                        output='screen'
                    )
                ]
            )
        )
        
        configure_actions.append(
            TimerAction(
                period=9.0,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'lifecycle', 'set',
                             f"/{LaunchConfiguration('namespace').perform(context)}/core_navigation_node",
                             'configure'],
                        output='screen'
                    )
                ]
            )
        )
        
        # Activation après configuration
        configure_actions.append(
            TimerAction(
                period=10.0,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'lifecycle', 'set',
                             f"/{LaunchConfiguration('namespace').perform(context)}/parameter_manager_node",
                             'activate'],
                        output='screen'
                    )
                ]
            )
        )
        
        configure_actions.append(
            TimerAction(
                period=11.0,
                actions=[
                    ExecuteProcess(
                        cmd=['ros2', 'lifecycle', 'set',
                             f"/{LaunchConfiguration('namespace').perform(context)}/core_navigation_node",
                             'activate'],
                        output='screen'
                    )
                ]
            )
        )
        
        return configure_actions
    
    lifecycle_configuration = OpaqueFunction(function=configure_lifecycle_nodes)
    
    # Action de vérification du système
    system_check = TimerAction(
        period=15.0,  # Vérification après démarrage complet
        actions=[
            Node(
                package='drone_navigation',
                executable='system_health_check',
                name='system_health_check',
                namespace=LaunchConfiguration('namespace'),
                parameters=[{'check_all_nodes': True}],
                arguments=['--ros-args', '--log-level', 'info'],
                output='screen'
            )
        ]
    )
    
    return LaunchDescription([
        # Arguments de lancement
        declare_simulation_mode,
        declare_mavros_enabled,
        declare_auto_start,
        declare_log_level,
        declare_namespace,
        
        # Paramètres globaux
        *global_parameters,
        
        # MAVROS (si activé)
        mavros_launch,
        
        # COUCHE 1: Infrastructure
        parameter_manager_node,
        mavros_interface_node,
        core_navigation_node,
        
        # COUCHE 2: Planification
        trajectory_planner_node,
        coverage_pattern_node,
        path_optimizer_node,
        
        # COUCHE 3: Contrôle
        position_controller_node,
        trajectory_follower_node,
        
        # COUCHE 4: Sécurité
        geofence_monitor_node,
        emergency_manager_node,
        obstacle_avoidance_node,
        
        # COUCHE 5: Interface
        mission_manager_node,
        status_monitor_node,
        cli_bridge_node,
        
        # COUCHE 6: Utilitaires
        calibration_node,
        data_logger_node,
        
        # Configuration lifecycle
        lifecycle_configuration,
        
        # Vérification système
        system_check
    ])


if __name__ == '__main__':
    generate_launch_description()
