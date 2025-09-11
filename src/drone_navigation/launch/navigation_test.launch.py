#!/usr/bin/env python3
"""
Launch file minimal pour tests et développement
Version allégée du système de navigation pour debugging et tests unitaires
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, GroupAction, LogInfo, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare

import os


def generate_launch_description():
    """Génère une description de lancement minimale pour tests"""
    
    # === ARGUMENTS ===
    
    test_mode_arg = DeclareLaunchArgument(
        'test_mode',
        default_value='basic',
        choices=['basic', 'planning', 'control', 'full'],
        description='Mode de test: basic, planning, control, ou full'
    )
    
    enable_rviz_arg = DeclareLaunchArgument(
        'enable_rviz',
        default_value='true',
        description='Lancer RViz pour visualisation'
    )
    
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(
            FindPackageShare('drone_navigation').find('drone_navigation'),
            'config', 'navigation_params.yaml'
        ),
        description='Fichier de configuration'
    )
    
    # === CONFIGURATIONS ===
    
    test_mode = LaunchConfiguration('test_mode')
    enable_rviz = LaunchConfiguration('enable_rviz')
    config_file = LaunchConfiguration('config_file')
    
    # === NŒUDS DE BASE (mode basic) ===
    
    # Parameter Manager (toujours nécessaire)
    parameter_manager = Node(
        package='drone_navigation',
        executable='parameter_manager_node',
        name='parameter_manager_node',
        namespace='drone_nav',
        parameters=[config_file],
        output='screen'
    )
    
    # Core Navigation (lifecycle)
    core_navigation = LifecycleNode(
        package='drone_navigation',
        executable='core_navigation_node',
        name='core_navigation_node',
        namespace='drone_nav',
        parameters=[config_file],
        output='screen'
    )
    
    # === NŒUDS DE PLANIFICATION (mode planning) ===
    
    trajectory_planner = Node(
        package='drone_navigation',
        executable='trajectory_planner_node',
        name='trajectory_planner_node',
        namespace='drone_nav',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", test_mode, "' in ['planning', 'full']"])
        )
    )
    
    coverage_pattern = Node(
        package='drone_navigation',
        executable='coverage_pattern_node',
        name='coverage_pattern_node',
        namespace='drone_nav',
        parameters=[config_file],
        output='log',
        condition=IfCondition(
            PythonExpression(["'", test_mode, "' in ['planning', 'full']"])
        )
    )
    
    # === NŒUDS DE CONTRÔLE (mode control) ===
    
    position_controller = Node(
        package='drone_navigation',
        executable='position_controller_node',
        name='position_controller_node',
        namespace='drone_nav',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", test_mode, "' in ['control', 'full']"])
        )
    )
    
    trajectory_follower = Node(
        package='drone_navigation',
        executable='trajectory_follower_node',
        name='trajectory_follower_node',
        namespace='drone_nav',
        parameters=[config_file],
        output='screen',
        condition=IfCondition(
            PythonExpression(["'", test_mode, "' in ['control', 'full']"])
        )
    )
    
    # === VISUALISATION ===
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', os.path.join(
            FindPackageShare('drone_navigation').find('drone_navigation'),
            'config', 'navigation_visualization.rviz'
        )],
        output='log',
        condition=IfCondition(enable_rviz)
    )
    
    # === CONFIGURATION LIFECYCLE ===
    
    configure_lifecycle = TimerAction(
        period=3.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/drone_nav/core_navigation_node', 'configure'],
                output='screen'
            )
        ]
    )
    
    activate_lifecycle = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/drone_nav/core_navigation_node', 'activate'],
                output='screen'
            )
        ]
    )
    
    # === MESSAGES D'INFORMATION ===
    
    info_messages = GroupAction([
        LogInfo(msg="🧪 Mode test de navigation drone"),
        LogInfo(msg=["🔧 Mode: ", test_mode]),
        LogInfo(msg="📊 Nœuds disponibles selon le mode choisi"),
        LogInfo(msg="🎯 Utilisez 'ros2 topic list' pour voir les topics actifs")
    ])
    
    return LaunchDescription([
        # Arguments
        test_mode_arg,
        enable_rviz_arg,
        config_file_arg,
        
        # Messages d'info
        info_messages,
        
        # Nœuds de base (toujours lancés)
        parameter_manager,
        TimerAction(period=1.0, actions=[core_navigation]),
        
        # Nœuds conditionnels
        TimerAction(period=2.0, actions=[trajectory_planner]),
        TimerAction(period=2.5, actions=[coverage_pattern]),
        TimerAction(period=3.0, actions=[position_controller]),
        TimerAction(period=3.5, actions=[trajectory_follower]),
        
        # Lifecycle management
        configure_lifecycle,
        activate_lifecycle,
        
        # Visualisation
        TimerAction(period=4.0, actions=[rviz_node]),
        
        # Message final
        TimerAction(
            period=8.0,
            actions=[
                LogInfo(msg="✅ Test environment ready"),
                LogInfo(msg="🚀 Système prêt pour les tests")
            ]
        )
    ])


if __name__ == '__main__':
    generate_launch_description()
