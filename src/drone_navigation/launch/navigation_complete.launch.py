#!/usr/bin/env python3
"""
Launch file pour le système complet de navigation drone - Architecture micro-nœuds
Orchestration séquentielle et contrôlée de tous les composants
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, 
    ExecuteProcess, 
    TimerAction, 
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition

import os
from pathlib import Path


def generate_launch_description():
    """Génère la description de lancement complète"""
    
    # === ARGUMENTS DE LANCEMENT ===
    
    # Mode de fonctionnement
    simulation_mode_arg = DeclareLaunchArgument(
        'simulation_mode',
        default_value='true',
        description='Utiliser le mode simulation (true) ou matériel réel (false)'
    )
    
    # Configuration des nœuds
    enable_coverage_pattern_arg = DeclareLaunchArgument(
        'enable_coverage_pattern',
        default_value='true',
        description='Activer le générateur de motifs de couverture'
    )
    
    enable_path_optimizer_arg = DeclareLaunchArgument(
        'enable_path_optimizer',
        default_value='true',
        description='Activer l\'optimiseur de chemins'
    )
    
    enable_auto_tuning_arg = DeclareLaunchArgument(
        'enable_auto_tuning',
        default_value='false',
        description='Activer l\'auto-tuning des contrôleurs PID'
    )
    
    # Paramètres de navigation
    max_velocity_arg = DeclareLaunchArgument(
        'max_velocity',
        default_value='3.0',
        description='Vitesse maximale du drone (m/s)'
    )
    
    control_frequency_arg = DeclareLaunchArgument(
        'control_frequency',
        default_value='50.0',
        description='Fréquence de contrôle (Hz)'
    )
    
    # Fichiers de configuration
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            FindPackageShare('drone_navigation').find('drone_navigation'),
            'config', 'navigation_params.yaml'
        ),
        description='Fichier de configuration principal'
    )
    
    pid_config_file_arg = DeclareLaunchArgument(
        'pid_config_file',
        default_value=os.path.join(
            FindPackageShare('drone_navigation').find('drone_navigation'),
            'config', 'pid_tuning.yaml'
        ),
        description='Fichier de configuration PID'
    )
    
    # Mode debug
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Activer le mode debug avec logs détaillés'
    )
    
    # === RÉCUPÉRATION DES CONFIGURATIONS ===
    
    simulation_mode = LaunchConfiguration('simulation_mode')
    enable_coverage_pattern = LaunchConfiguration('enable_coverage_pattern')
    enable_path_optimizer = LaunchConfiguration('enable_path_optimizer')
    enable_auto_tuning = LaunchConfiguration('enable_auto_tuning')
    max_velocity = LaunchConfiguration('max_velocity')
    control_frequency = LaunchConfiguration('control_frequency')
    config_file = LaunchConfiguration('config_file')
    pid_config_file = LaunchConfiguration('pid_config_file')
    debug_mode = LaunchConfiguration('debug_mode')
    
    # === PHASE 1: INFRASTRUCTURE (0-5s) ===
    
    # 1.1 Parameter Manager Node (Bootstrap)
    parameter_manager_node = Node(
        package='drone_navigation',
        executable='parameter_manager_node',
        name='parameter_manager_node',
        namespace='drone_nav',
        parameters=[
            config_file,
            {
                'config_file_path': config_file,
                'auto_reload': True,
                'validation_enabled': True
            }
        ],
        output='screen' if debug_mode else 'log',
        respawn=True,
        respawn_delay=2.0
    )
    
    # 1.2 MAVROS Interface Node (après Parameter Manager)
    mavros_interface_node = Node(
        package='drone_navigation',
        executable='mavros_interface_node',
        name='mavros_interface_node',
        namespace='drone_nav',
        parameters=[
            config_file,
            {
                'simulation_mode': simulation_mode,
                'mavros_namespace': '/mavros',
                'safety_checks_enabled': True,
                'auto_arm': False
            }
        ],
        output='screen' if debug_mode else 'log',
        respawn=True,
        respawn_delay=5.0
    )
    
    # 1.3 Core Navigation Node (Lifecycle Node)
    core_navigation_node = LifecycleNode(
        package='drone_navigation',
        executable='core_navigation_node',
        name='core_navigation_node',
        namespace='drone_nav',
        parameters=[
            config_file,
            {
                'max_velocity': max_velocity,
                'control_frequency': control_frequency,
                'enable_auto_recovery': True,
                'system_timeout': 30.0
            }
        ],
        output='screen',
        respawn=True,
        respawn_delay=3.0
    )
    
    # === PHASE 2: PLANIFICATION (5-10s) ===
    
    # 2.1 Trajectory Planner Node
    trajectory_planner_node = Node(
        package='drone_navigation',
        executable='trajectory_planner_node',
        name='trajectory_planner_node',
        namespace='drone_nav',
        parameters=[
            config_file,
            {
                'default_algorithm': 'a_star',
                'enable_3d_planning': True,
                'obstacle_inflation_radius': 1.0,
                'planning_timeout': 10.0
            }
        ],
        output='log',
        respawn=True,
        respawn_delay=5.0
    )
    
    # 2.2 Coverage Pattern Node (optionnel)
    coverage_pattern_node = Node(
        package='drone_navigation',
        executable='coverage_pattern_node',
        name='coverage_pattern_node',
        namespace='drone_nav',
        parameters=[
            config_file,
            {
                'default_pattern_type': 'zigzag',
                'default_line_spacing': 2.0,
                'enable_wind_compensation': True,
                'optimization_algorithm': 'genetic'
            }
        ],
        output='log',
        respawn=True,
        respawn_delay=5.0,
        condition=IfCondition(enable_coverage_pattern)
    )
    
    # 2.3 Path Optimizer Node (optionnel)
    path_optimizer_node = Node(
        package='drone_navigation',
        executable='path_optimizer_node',
        name='path_optimizer_node',
        namespace='drone_nav',
        parameters=[
            config_file,
            {
                'default_algorithm': 'genetic',
                'population_size': 50,
                'max_iterations': 1000,
                'enable_real_time_optimization': True
            }
        ],
        output='log',
        respawn=True,
        respawn_delay=5.0,
        condition=IfCondition(enable_path_optimizer)
    )
    
    # === PHASE 3: CONTRÔLE (10-15s) ===
    
    # 3.1 Position Controller Node
    position_controller_node = Node(
        package='drone_navigation',
        executable='position_controller_node',
        name='position_controller_node',
        namespace='drone_nav',
        parameters=[
            pid_config_file,
            {
                'control_frequency': control_frequency,
                'max_velocity': max_velocity,
                'enable_auto_tuning': enable_auto_tuning,
                'enable_adaptation': True
            }
        ],
        output='log',
        respawn=True,
        respawn_delay=3.0
    )
    
    # 3.2 Trajectory Follower Node
    trajectory_follower_node = Node(
        package='drone_navigation',
        executable='trajectory_follower_node',
        name='trajectory_follower_node',
        namespace='drone_nav',
        parameters=[
            config_file,
            {
                'following_algorithm': 'pure_pursuit',
                'lookahead_distance': 2.0,
                'max_velocity': max_velocity,
                'enable_velocity_planning': True
            }
        ],
        output='log',
        respawn=True,
        respawn_delay=3.0
    )
    
    # === PHASE 4: SÉCURITÉ (15-20s) ===
    
    # 4.1 Geofence Monitor Node
    geofence_monitor_node = Node(
        package='drone_navigation',
        executable='geofence_monitor_node',
        name='geofence_monitor_node',
        namespace='drone_nav',
        parameters=[
            config_file,
            {
                'enable_geofence': True,
                'check_frequency': 10.0,
                'violation_action': 'emergency_stop'
            }
        ],
        output='screen',
        respawn=True,
        respawn_delay=2.0
    )
    
    # 4.2 Emergency Manager Node
    emergency_manager_node = Node(
        package='drone_navigation',
        executable='emergency_manager_node',
        name='emergency_manager_node',
        namespace='drone_nav',
        parameters=[
            config_file,
            {
                'emergency_timeout': 5.0,
                'auto_land_enabled': True,
                'return_home_enabled': True
            }
        ],
        output='screen',
        respawn=True,
        respawn_delay=1.0
    )
    
    # 4.3 Obstacle Avoidance Node
    obstacle_avoidance_node = Node(
        package='drone_navigation',
        executable='obstacle_avoidance_node',
        name='obstacle_avoidance_node',
        namespace='drone_nav',
        parameters=[
            config_file,
            {
                'detection_range': 5.0,
                'avoidance_algorithm': 'potential_field',
                'safety_margin': 1.0
            }
        ],
        output='log',
        respawn=True,
        respawn_delay=3.0
    )
    
    # === PHASE 5: INTERFACE (20-25s) ===
    
    # 5.1 Mission Manager Node
    mission_manager_node = Node(
        package='drone_navigation',
        executable='mission_manager_node',
        name='mission_manager_node',
        namespace='drone_nav',
        parameters=[
            config_file,
            {
                'auto_start_missions': False,
                'mission_timeout': 3600.0,  # 1 heure
                'enable_mission_recovery': True
            }
        ],
        output='screen',
        respawn=True,
        respawn_delay=5.0
    )
    
    # 5.2 Status Monitor Node
    status_monitor_node = Node(
        package='drone_navigation',
        executable='status_monitor_node',
        name='status_monitor_node',
        namespace='drone_nav',
        parameters=[
            config_file,
            {
                'monitoring_frequency': 5.0,
                'health_check_timeout': 2.0,
                'enable_diagnostics': True
            }
        ],
        output='log',
        respawn=True,
        respawn_delay=3.0
    )
    
    # 5.3 CLI Bridge Node
    cli_bridge_node = Node(
        package='drone_navigation',
        executable='cli_bridge_node',
        name='cli_bridge_node',
        namespace='drone_nav',
        parameters=[
            config_file,
            {
                'enable_cli_interface': True,
                'command_timeout': 10.0
            }
        ],
        output='log',
        respawn=True,
        respawn_delay=5.0
    )
    
    # === PHASE 6: UTILITAIRES (25-30s) ===
    
    # 6.1 Calibration Node
    calibration_node = Node(
        package='drone_navigation',
        executable='calibration_node',
        name='calibration_node',
        namespace='drone_nav',
        parameters=[
            config_file,
            {
                'auto_calibration': False,
                'calibration_timeout': 30.0
            }
        ],
        output='log',
        respawn=False  # Pas de respawn pour la calibration
    )
    
    # 6.2 Data Logger Node
    data_logger_node = Node(
        package='drone_navigation',
        executable='data_logger_node',
        name='data_logger_node',
        namespace='drone_nav',
        parameters=[
            config_file,
            {
                'log_directory': '/tmp/drone_navigation_logs',
                'log_frequency': 10.0,
                'max_log_size': 100,  # MB
                'enable_bag_recording': True
            }
        ],
        output='log',
        respawn=True,
        respawn_delay=5.0
    )
    
    # === ACTIONS TEMPORISÉES ===
    
    # Démarrage séquentiel des phases
    phase_1_infrastructure = GroupAction([
        LogInfo(msg=\"🚀 Démarrage Phase 1: Infrastructure\"),
        parameter_manager_node,
        TimerAction(
            period=2.0,
            actions=[mavros_interface_node]
        ),
        TimerAction(
            period=4.0,
            actions=[core_navigation_node]
        )
    ])
    
    phase_2_planification = TimerAction(
        period=5.0,
        actions=[
            GroupAction([
                LogInfo(msg=\"🎯 Démarrage Phase 2: Planification\"),
                trajectory_planner_node,
                TimerAction(period=1.0, actions=[coverage_pattern_node]),
                TimerAction(period=2.0, actions=[path_optimizer_node])
            ])
        ]
    )
    
    phase_3_controle = TimerAction(
        period=10.0,
        actions=[
            GroupAction([
                LogInfo(msg=\"🎮 Démarrage Phase 3: Contrôle\"),
                position_controller_node,
                TimerAction(period=2.0, actions=[trajectory_follower_node])
            ])
        ]
    )
    
    phase_4_securite = TimerAction(
        period=15.0,
        actions=[
            GroupAction([
                LogInfo(msg=\"🛡️ Démarrage Phase 4: Sécurité\"),
                geofence_monitor_node,
                TimerAction(period=1.0, actions=[emergency_manager_node]),
                TimerAction(period=2.0, actions=[obstacle_avoidance_node])
            ])
        ]
    )
    
    phase_5_interface = TimerAction(
        period=20.0,
        actions=[
            GroupAction([
                LogInfo(msg=\"💻 Démarrage Phase 5: Interface\"),
                mission_manager_node,
                TimerAction(period=1.0, actions=[status_monitor_node]),
                TimerAction(period=2.0, actions=[cli_bridge_node])
            ])
        ]
    )
    
    phase_6_utilitaires = TimerAction(
        period=25.0,
        actions=[
            GroupAction([
                LogInfo(msg=\"🔧 Démarrage Phase 6: Utilitaires\"),
                calibration_node,
                TimerAction(period=1.0, actions=[data_logger_node])
            ])
        ]
    )
    
    # === GESTION DU LIFECYCLE POUR CORE NAVIGATION ===
    
    # Configuration automatique du nœud lifecycle
    configure_core_navigation = TimerAction(
        period=6.0,  # Après démarrage
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/drone_nav/core_navigation_node', 'configure'],
                output='screen'
            )
        ]
    )
    
    # Activation automatique du nœud lifecycle
    activate_core_navigation = TimerAction(
        period=8.0,  # Après configuration
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/drone_nav/core_navigation_node', 'activate'],
                output='screen'
            )
        ]
    )
    
    # === VÉRIFICATION SYSTÈME ===
    
    # Vérification de la santé du système après démarrage
    system_health_check = TimerAction(
        period=30.0,
        actions=[
            LogInfo(msg=\"🔍 Vérification de la santé du système\"),
            ExecuteProcess(
                cmd=['ros2', 'run', 'drone_navigation', 'system_health_check'],
                output='screen'
            )
        ]
    )
    
    # === LANCEMENT CONDITIONNEL MAVROS (en mode simulation) ===
    
    mavros_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('mavros').find('mavros'),
            '/launch/apm.launch.py'
        ]),
        launch_arguments={
            'fcu_url': 'udp://127.0.0.1:14550@14555',
            'gcs_url': '',
            'target_system_id': '1',
            'target_component_id': '1'
        }.items(),
        condition=IfCondition(simulation_mode)
    )
    
    # === MONITORING ET DIAGNOSTICS ===
    
    # Lancement RQT en mode debug
    rqt_graph = ExecuteProcess(
        cmd=['rqt_graph'],
        output='screen',
        condition=IfCondition(debug_mode)
    )
    
    # Dashboard de monitoring
    rqt_dashboard = TimerAction(
        period=35.0,
        actions=[
            ExecuteProcess(
                cmd=['rqt', '--perspective-file', 
                     os.path.join(FindPackageShare('drone_navigation').find('drone_navigation'),
                                'config', 'navigation_debug.perspective')],
                output='screen',
                condition=IfCondition(debug_mode)
            )
        ]
    )
    
    # === DESCRIPTION DE LANCEMENT FINALE ===
    
    return LaunchDescription([
        # Arguments
        simulation_mode_arg,
        enable_coverage_pattern_arg,
        enable_path_optimizer_arg,
        enable_auto_tuning_arg,
        max_velocity_arg,
        control_frequency_arg,
        config_file_arg,
        pid_config_file_arg,
        debug_mode_arg,
        
        # MAVROS conditionnel
        mavros_launch,
        
        # Messages d'information
        LogInfo(msg=\"🚀 Démarrage du système de navigation drone complet\"),
        LogInfo(msg=[\"📊 Mode simulation: \", simulation_mode]),
        LogInfo(msg=[\"⚡ Vitesse max: \", max_velocity, \" m/s\"]),
        LogInfo(msg=[\"🔄 Fréquence contrôle: \", control_frequency, \" Hz\"]),
        
        # Phases de démarrage séquentiel
        phase_1_infrastructure,
        phase_2_planification,
        phase_3_controle,
        phase_4_securite,
        phase_5_interface,
        phase_6_utilitaires,
        
        # Gestion lifecycle
        configure_core_navigation,
        activate_core_navigation,
        
        # Vérifications
        system_health_check,
        
        # Monitoring (debug)
        TimerAction(period=5.0, actions=[rqt_graph]),
        rqt_dashboard,
        
        # Message final
        TimerAction(
            period=35.0,
            actions=[
                LogInfo(msg=\"✅ Système de navigation drone complètement démarré\"),
                LogInfo(msg=\"🎮 Prêt pour les missions autonomes\")
            ]
        )
    ])
