#!/usr/bin/env python3
"""
Complete Drone Pollination System Launch File
Launches all components in proper order with dependencies
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from pathlib import Path


def generate_launch_description():
    """Generate launch description for complete drone system"""
    
    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )
    
    enable_vision_arg = DeclareLaunchArgument(
        'enable_vision',
        default_value='true',
        description='Enable vision system'
    )
    
    enable_data_collector_arg = DeclareLaunchArgument(
        'enable_data_collector',
        default_value='true',
        description='Enable data collector'
    )
    
    autostart_mission_arg = DeclareLaunchArgument(
        'autostart_mission',
        default_value='false',
        description='Automatically start a test mission'
    )
    
    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value='test_simple',
        description='Mission file to load (without .json extension)'
    )
    
    # Configuration file path
    config_dir = os.path.join(
        os.path.expanduser('~'), 'ros2_ws', 'config'
    )
    
    # Parameters from configuration
    drone_config_file = os.path.join(config_dir, 'drone_config.yaml')
    
    # 1. System Coordinator (starts first)
    system_coordinator_node = Node(
        package='drone_system',
        executable='system_coordinator',
        name='system_coordinator',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }],
        remappings=[
            ('/system/state', '/system/coordinator_state'),
        ]
    )
    
    # 2. Drone Interface (core system)
    drone_interface_node = TimerAction(
        period=2.0,  # Wait 2 seconds after system coordinator
        actions=[
            Node(
                package='drone_interface',
                executable='interface_node.py',
                name='drone_interface',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'config_file': drone_config_file
                }],
                remappings=[
                    ('/drone/status', '/drone/status_enhanced'),
                ]
            )
        ]
    )
    
    # 3. Navigation System
    navigation_node = TimerAction(
        period=4.0,  # Wait for drone interface to be ready
        actions=[
            Node(
                package='drone_navigation',
                executable='navigation_node.py',
                name='drone_navigation',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'update_rate': 20.0,
                    'position_tolerance': 0.5,
                    'yaw_tolerance': 0.1,
                    'max_velocity': 2.0,
                    'max_altitude': 50.0,
                    'min_altitude': 0.5
                }]
            )
        ]
    )
    
    # 4. Vision System (conditional)
    vision_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='drone_vision',
                executable='vision_node.py',
                name='drone_vision',
                output='screen',
                condition=IfCondition(LaunchConfiguration('enable_vision')),
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'detection_rate': 10.0,
                    'confidence_threshold': 0.5,
                    'target_colors': ['red', 'yellow', 'pink']
                }]
            )
        ]
    )
    
    # 5. Data Collector (conditional)
    data_collector_node = TimerAction(
        period=8.0,
        actions=[
            Node(
                package='drone_vision',
                executable='data_collector_node.py',
                name='data_collector',
                output='screen',
                condition=IfCondition(LaunchConfiguration('enable_data_collector')),
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'auto_start_collection': True,
                    'save_interval': 30.0
                }]
            )
        ]
    )
    
    # 6. Enhanced Mission System
    enhanced_mission_node = TimerAction(
        period=10.0,  # Wait for all core systems
        actions=[
            Node(
                package='drone_mission',
                executable='enhanced_mission_node.py',
                name='enhanced_mission_orchestrator',
                output='screen',
                parameters=[{
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'mission_directory': os.path.join(
                        os.path.expanduser('~'), 'ros2_ws', 'src', 'drone_mission', 'missions'
                    ),
                    'auto_recovery': True,
                    'max_retries': 3
                }]
            )
        ]
    )
    
    # 7. Camera Simulator (for testing)
    camera_simulator = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['python3', os.path.join(
                    os.path.expanduser('~'), 'ros2_ws', 'camera_simulator.py'
                )],
                name='camera_simulator',
                output='screen'
            )
        ]
    )
    
    # 8. Autostart mission (conditional)
    autostart_mission = TimerAction(
        period=15.0,  # Wait for all systems to be ready
        actions=[
            ExecuteProcess(
                condition=IfCondition(LaunchConfiguration('autostart_mission')),
                cmd=[
                    'ros2', 'service', 'call', '/mission/load_enhanced',
                    'std_srvs/srv/SetString',
                    f'{{data: "{LaunchConfiguration("mission_file")}"}}'
                ],
                name='load_mission'
            )
        ]
    )
    
    # 9. Start mission execution (if autostart enabled)
    start_mission = TimerAction(
        period=18.0,
        actions=[
            ExecuteProcess(
                condition=IfCondition(LaunchConfiguration('autostart_mission')),
                cmd=[
                    'ros2', 'service', 'call', '/mission/start_enhanced',
                    'std_srvs/srv/Trigger'
                ],
                name='start_mission'
            )
        ]
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        enable_vision_arg,
        enable_data_collector_arg,
        autostart_mission_arg,
        mission_file_arg,
        
        # Nodes in startup order
        system_coordinator_node,
        drone_interface_node,
        navigation_node,
        vision_node,
        data_collector_node,
        enhanced_mission_node,
        camera_simulator,
        
        # Mission autostart
        autostart_mission,
        start_mission,
    ])
