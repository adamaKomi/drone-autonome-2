#!/usr/bin/env python3
"""
=============================================================================
DRONE INTERFACE LAUNCH FILE
=============================================================================
Auteur: Adama Komi 
Date: 2025-09-03
Version: 3.0.0

Description:
    Fichier de lancement pour le nœud DroneInterface avec configuration complète.
    
Fonctionnalités:
    - Configuration des paramètres par défaut
    - Lancement avec lifecycle management
    - Configuration automatique des QoS
    - Monitoring et diagnostics
    - Gestion des dépendances MAVROS

Utilisation:
    ros2 launch drone_interface drone_interface_launch.py
    ros2 launch drone_interface drone_interface_launch.py configure_on_start:=true
    ros2 launch drone_interface drone_interface_launch.py safety.min_battery_percentage:=25.0

Paramètres configurables:
    - configure_on_start: Configuration automatique au démarrage
    - activate_on_start: Activation automatique au démarrage  
    - safety.*: Paramètres de sécurité
    - comm.*: Paramètres de communication
    - perf.*: Paramètres de performance
=============================================================================
"""

from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo, 
                          TimerAction, RegisterEventHandler)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.substitutions import (LaunchConfiguration, PythonExpression, 
                                PathJoinSubstitution, FindExecutable)
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    """Génère la description de lancement"""
    
    # ========== ARGUMENTS DE LANCEMENT ==========
    
    # Configuration générale
    configure_on_start_arg = DeclareLaunchArgument(
        'configure_on_start',
        default_value='true',
        description='Configure automatically on start'
    )
    
    activate_on_start_arg = DeclareLaunchArgument(
        'activate_on_start', 
        default_value='true',
        description='Activate automatically after configuration'
    )
    
    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Log level (debug, info, warn, error)'
    )
    
    # Paramètres de sécurité
    min_battery_voltage_arg = DeclareLaunchArgument(
        'safety.min_battery_voltage',
        default_value='10.5',
        description='Minimum battery voltage for arming (V)'
    )
    
    min_battery_percentage_arg = DeclareLaunchArgument(
        'safety.min_battery_percentage',
        default_value='20.0',
        description='Minimum battery percentage for arming (%)'
    )
    
    max_altitude_arg = DeclareLaunchArgument(
        'safety.max_altitude',
        default_value='120.0',
        description='Maximum allowed altitude (m)'
    )
    
    min_gps_satellites_arg = DeclareLaunchArgument(
        'safety.min_gps_satellites',
        default_value='6',
        description='Minimum GPS satellites for arming'
    )
    
    # Paramètres de communication
    heartbeat_timeout_arg = DeclareLaunchArgument(
        'comm.heartbeat_timeout',
        default_value='5.0',
        description='Heartbeat timeout (s)'
    )
    
    service_timeout_arg = DeclareLaunchArgument(
        'comm.service_timeout',
        default_value='30.0',
        description='Service call timeout (s)'
    )
    
    retry_attempts_arg = DeclareLaunchArgument(
        'comm.retry_attempts',
        default_value='3',
        description='Number of retry attempts for failed operations'
    )
    
    # Paramètres de performance
    status_rate_arg = DeclareLaunchArgument(
        'perf.status_rate',
        default_value='10.0',
        description='Status publication rate (Hz)'
    )
    
    health_check_rate_arg = DeclareLaunchArgument(
        'perf.health_check_rate',
        default_value='1.0',
        description='Health check rate (Hz)'
    )
    
    diagnostics_rate_arg = DeclareLaunchArgument(
        'perf.diagnostics_rate',
        default_value='0.5',
        description='Diagnostics publication rate (Hz)'
    )
    
    # Paramètres MAVROS
    mavros_namespace_arg = DeclareLaunchArgument(
        'mavros_namespace',
        default_value='mavros',
        description='MAVROS namespace'
    )
    
    fcu_url_arg = DeclareLaunchArgument(
        'fcu_url',
        default_value='udp://:14540@127.0.0.1:14557',
        description='Flight controller connection URL'
    )
    
    # ========== NŒUD PRINCIPAL ==========
    
    # Nœud DroneInterface avec lifecycle
    drone_interface_node = LifecycleNode(
        package='drone_interface',
        executable='interface_node',
        name='drone_interface',
        namespace='',
        parameters=[{
            'safety.min_battery_voltage': LaunchConfiguration('safety.min_battery_voltage'),
            'safety.min_battery_percentage': LaunchConfiguration('safety.min_battery_percentage'),
            'safety.max_altitude': LaunchConfiguration('safety.max_altitude'),
            'safety.min_gps_satellites': LaunchConfiguration('safety.min_gps_satellites'),
            'comm.heartbeat_timeout': LaunchConfiguration('comm.heartbeat_timeout'),
            'comm.service_timeout': LaunchConfiguration('comm.service_timeout'),
            'comm.retry_attempts': LaunchConfiguration('comm.retry_attempts'),
            'perf.status_rate': LaunchConfiguration('perf.status_rate'),
            'perf.health_check_rate': LaunchConfiguration('perf.health_check_rate'),
            'perf.diagnostics_rate': LaunchConfiguration('perf.diagnostics_rate'),
        }],
        remappings=[
            # Remappages MAVROS si namespace différent
            ('/mavros/state', ['/', LaunchConfiguration('mavros_namespace'), '/state']),
            ('/mavros/local_position/pose', ['/', LaunchConfiguration('mavros_namespace'), '/local_position/pose']),
            ('/mavros/battery', ['/', LaunchConfiguration('mavros_namespace'), '/battery']),
            ('/mavros/global_position/global', ['/', LaunchConfiguration('mavros_namespace'), '/global_position/global']),
            ('/mavros/cmd/arming', ['/', LaunchConfiguration('mavros_namespace'), '/cmd/arming']),
            ('/mavros/set_mode', ['/', LaunchConfiguration('mavros_namespace'), '/set_mode']),
            ('/mavros/cmd/takeoff', ['/', LaunchConfiguration('mavros_namespace'), '/cmd/takeoff']),
        ],
        output='both',
        arguments=['--ros-args', '--log-level', LaunchConfiguration('log_level')],
    )
    
    # ========== LIFECYCLE MANAGEMENT ==========
    
    # Configuration automatique
    configure_event_handler = RegisterEventHandler(
        OnProcessStart(
            target_action=drone_interface_node,
            on_start=[
                LogInfo(msg="DroneInterface node started, waiting 2 seconds before configuring..."),
                TimerAction(
                    period=2.0,
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                FindExecutable(name='ros2'),
                                'lifecycle', 'set', '/drone_interface', 'configure'
                            ],
                            output='screen',
                            condition=IfCondition(LaunchConfiguration('configure_on_start'))
                        )
                    ]
                )
            ]
        )
    )
    
    # Activation automatique après configuration
    activate_event_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=drone_interface_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                LogInfo(msg="DroneInterface configured, activating..."),
                TimerAction(
                    period=1.0,
                    actions=[
                        ExecuteProcess(
                            cmd=[
                                FindExecutable(name='ros2'),
                                'lifecycle', 'set', '/drone_interface', 'activate'
                            ],
                            output='screen',
                            condition=IfCondition(LaunchConfiguration('activate_on_start'))
                        )
                    ]
                )
            ]
        )
    )
    
    # Gestion des transitions d'état
    state_transition_handler = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=drone_interface_node,
            start_state='activating',
            goal_state='active',
            entities=[
                LogInfo(msg="✅ DroneInterface is now ACTIVE and ready for operations!")
            ]
        )
    )
    
    # ========== NŒUDS DE SUPPORT ==========
    
    # Nœud de monitoring (optionnel)
    monitor_node = Node(
        package='drone_interface',
        executable='monitor_node.py',
        name='drone_monitor',
        parameters=[{
            'monitor_rate': 1.0,
            'alert_battery_threshold': LaunchConfiguration('safety.min_battery_percentage'),
        }],
        condition=IfCondition('false'),  # Désactivé par défaut
        output='screen'
    )
    
    # Nœud de logging avancé (optionnel)
    logger_node = Node(
        package='drone_interface',
        executable='logger_node.py',
        name='drone_logger',
        parameters=[{
            'log_directory': '/tmp/drone_logs',
            'log_rotation': True,
            'max_log_size': '100MB'
        }],
        condition=IfCondition('false'),  # Désactivé par défaut
        output='screen'
    )
    
    # ========== DIAGNOSTICS ET MONITORING ==========
    
    # Nœud de diagnostic ROS2
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        condition=IfCondition('false'),  # Désactivé par défaut
        output='screen'
    )
    
    # Agrégateur de diagnostics
    diagnostic_aggregator = Node(
        package='diagnostic_aggregator',
        executable='aggregator_node',
        name='diagnostic_aggregator',
        parameters=[{
            'analyzers': {
                'drone_interface': {
                    'type': 'diagnostic_aggregator/GenericAnalyzer',
                    'path': 'DroneInterface',
                    'contains': ['drone_interface']
                },
                'mavros': {
                    'type': 'diagnostic_aggregator/GenericAnalyzer', 
                    'path': 'MAVROS',
                    'contains': ['mavros']
                },
                'system': {
                    'type': 'diagnostic_aggregator/GenericAnalyzer',
                    'path': 'System',
                    'contains': ['system', 'battery', 'gps']
                }
            }
        }],
        condition=IfCondition('false'),  # Désactivé par défaut
        output='screen'
    )
    
    # ========== ACTIONS DE DÉMARRAGE ==========
    
    # Vérification des prérequis
    check_mavros_action = ExecuteProcess(
        cmd=[
            'bash', '-c', 
            'echo "Checking MAVROS availability..." && '
            'timeout 10s ros2 topic list | grep -q mavros || '
            '(echo "⚠️  MAVROS topics not found. Make sure MAVROS is running." && exit 1)'
        ],
        output='screen',
        on_exit=[
            LogInfo(msg="✅ MAVROS check completed")
        ]
    )
    
    # Messages d'information
    startup_info = LogInfo(
        msg=[
            "🚀 Launching DroneInterface v3.0\n",
            "📋 Available services after activation:\n",
            "   - /drone/arm : Arm the drone\n",
            "   - /drone/disarm : Disarm the drone\n", 
            "   - /drone/set_mode : Change flight mode\n",
            "   - /drone/safety_check : Safety verification\n",
            "   - /drone/health_check : System health check\n",
            "   - /drone/emergency_stop : Emergency stop\n\n",
            "📊 Published topics:\n",
            "   - /drone/status : Drone status (JSON)\n",
            "   - /drone/safety_status : Safety events\n",
            "   - /diagnostics : System diagnostics\n\n",
            "🎛️  Lifecycle commands:\n",
            "   - ros2 lifecycle get /drone_interface\n",
            "   - ros2 lifecycle set /drone_interface configure\n",
            "   - ros2 lifecycle set /drone_interface activate\n",
            "   - ros2 lifecycle set /drone_interface deactivate\n"
        ]
    )
    
    # ========== DESCRIPTION DE LANCEMENT ==========
    
    return LaunchDescription([
        # Arguments
        configure_on_start_arg,
        activate_on_start_arg,
        log_level_arg,
        min_battery_voltage_arg,
        min_battery_percentage_arg,
        max_altitude_arg,
        min_gps_satellites_arg,
        heartbeat_timeout_arg,
        service_timeout_arg,
        retry_attempts_arg,
        status_rate_arg,
        health_check_rate_arg,
        diagnostics_rate_arg,
        mavros_namespace_arg,
        fcu_url_arg,
        
        # Messages d'information
        startup_info,
        
        # Vérifications préalables
        # check_mavros_action,  # Commenté car peut causer des problèmes si MAVROS pas encore lancé
        
        # Nœud principal
        drone_interface_node,
        
        # Gestionnaires d'événements lifecycle
        configure_event_handler,
        activate_event_handler,
        state_transition_handler,
        
        # Nœuds de support (optionnels)
        monitor_node,
        logger_node,
        diagnostic_aggregator,
        
        # Informations finales
        TimerAction(
            period=5.0,
            actions=[
                LogInfo(msg="🔍 Use 'ros2 topic echo /drone/status' to monitor drone state"),
                LogInfo(msg="🛠️  Use 'ros2 service call /drone/safety_check std_srvs/srv/Trigger' to check safety"),
                LogInfo(msg="📈 Use 'rqt' to visualize diagnostics and status")
            ]
        )
    ])