#!/usr/bin/env python3
"""
=============================================================================
LAUNCH FILE - Système de vision drone complet
=============================================================================
Lance tous les nœuds nécessaires pour le système de vision du drone:
- Nœud de détection de fleurs
- Nœud de collecte de données
- Simulation de caméra (optionnel)

Usage:
    ros2 launch drone_vision vision_system.launch.py
    ros2 launch drone_vision vision_system.launch.py use_sim_camera:=true
=============================================================================
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Génère la description de lancement"""
    
    # Arguments de lancement
    use_sim_camera_arg = DeclareLaunchArgument(
        'use_sim_camera',
        default_value='false',
        description='Utiliser une caméra simulée'
    )
    
    debug_mode_arg = DeclareLaunchArgument(
        'debug_mode',
        default_value='false',
        description='Mode debug avec logs détaillés'
    )
    
    # Nœud de vision principal
    vision_node = Node(
        package='drone_vision',
        executable='vision_node',
        name='drone_vision',
        output='screen',
        parameters=[{
            'debug_mode': LaunchConfiguration('debug_mode')
        }],
        remappings=[
            ('/camera/image_raw', '/camera/image_raw'),
            ('/vision/flowers_detected', '/vision/flowers_detected'),
            ('/vision/debug_image', '/vision/debug_image'),
        ]
    )
    
    # Nœud collecteur de données
    data_collector_node = Node(
        package='drone_vision',
        executable='data_collector_node',
        name='drone_data_collector',
        output='screen',
        parameters=[{
            'data_directory': '~/ros2_ws/mission_data'
        }]
    )
    
    # Caméra simulée (optionnelle)
    sim_camera_node = Node(
        package='usb_cam',
        executable='usb_cam_node_exe',
        name='camera',
        condition=IfCondition(LaunchConfiguration('use_sim_camera')),
        parameters=[{
            'video_device': '/dev/video0',
            'image_width': 640,
            'image_height': 480,
            'pixel_format': 'yuyv',
            'camera_frame_id': 'camera_link',
            'io_method': 'mmap'
        }],
        remappings=[
            ('/image_raw', '/camera/image_raw')
        ]
    )
    
    # Viewer d'image pour debug (optionnel)
    image_viewer = ExecuteProcess(
        cmd=['ros2', 'run', 'rqt_image_view', 'rqt_image_view', '/vision/debug_image'],
        condition=IfCondition(LaunchConfiguration('debug_mode')),
        output='screen'
    )
    
    return LaunchDescription([
        # Arguments
        use_sim_camera_arg,
        debug_mode_arg,
        
        # Nœuds principaux
        vision_node,
        data_collector_node,
        
        # Nœuds optionnels
        sim_camera_node,
        image_viewer,
    ])
