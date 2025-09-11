# Exemple d'intégration : MAVROS Interface + drone_interface

## Vue d'ensemble

Ce document montre comment utiliser le `mavros_interface_node` refactorisé avec les nœuds du package `drone_interface` pour obtenir un système complet de commandes et de surveillance.

## Architecture du système intégré

```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Navigation    │    │  mavros_interface │   │    MAVROS      │
│     Nodes       │◄───│      _node        │◄──│   (Lecture)    │
│                 │    │  (Données seules) │   │                │
└─────────────────┘    └─────────────────┘    └─────────────────┘
         │                       │                       ▲
         │                       │                       │
         ▼                       ▼                       │
┌─────────────────┐    ┌─────────────────┐               │
│ Mission Control │    │   Monitoring    │               │
│   Interface     │    │   Dashboard     │               │
└─────────────────┘    └─────────────────┘               │
         │                                                │
         ▼                                                │
┌─────────────────┐                                      │
│ drone_interface │─────────────────────────────────────┘
│   (Commandes)   │        Services MAVROS
└─────────────────┘
```

## Launch file complet

```python
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Chemins des packages
    drone_interface_launch_dir = os.path.join(
        get_package_share_directory('drone_interface'),
        'launch'
    )
    
    return LaunchDescription([
        # 1. MAVROS (doit être démarré en premier)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                os.path.join(get_package_share_directory('mavros'), 'launch'),
                '/apm.launch.py'
            ]),
            launch_arguments={
                'fcu_url': 'udp://127.0.0.1:14550@14555',
                'gcs_url': '',
                'target_system_id': '1',
                'target_component_id': '1'
            }.items()
        ),
        
        # 2. MAVROS Interface Node (collecte de données)
        Node(
            package='drone_navigation',
            executable='mavros_interface_node',
            name='mavros_interface',
            namespace='drone_nav',
            parameters=[{
                'mavros_namespace': '/mavros',
                'simulation_mode': True,
                'connection_timeout': 5.0,
                'status_frequency': 2.0
            }],
            output='screen'
        ),
        
        # 3. Nœuds drone_interface (commandes)
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                drone_interface_launch_dir, '/interface.launch.py'
            ])
        ),
        
        # 4. Nœud de surveillance (optionnel)
        Node(
            package='drone_navigation',
            executable='monitoring_node',
            name='drone_monitor',
            namespace='drone_nav',
            parameters=[{
                'status_topic': '/drone_nav/drone_status',
                'safety_topic': '/drone_nav/safety_status'
            }],
            output='screen'
        )
    ])
```

## Séquence de démarrage typique

### 1. Démarrage du système

```bash
# Terminal 1: Simulateur ArduPilot (si en simulation)
cd ~/ardupilot
python3 Tools/autotest/sim_vehicle.py -v ArduCopter --console --map --out=udp:127.0.0.1:14550

# Terminal 2: Système ROS2 complet
cd /home/adama133/ros2_ws
source install/setup.bash
ros2 launch drone_navigation integrated_system.launch.py
```

### 2. Vérification du système

```bash
# Vérifier que tous les nœuds sont actifs
ros2 node list

# Résultat attendu:
# /drone_nav/mavros_interface
# /arm_disarm_node  
# /mode_node
# /takeoff_land_node
# /mavros/*

# Vérifier les topics de données
ros2 topic list | grep drone_nav
# /drone_nav/drone_status
# /drone_nav/safety_status
# /drone_nav/position
# /drone_nav/velocity
# /drone_nav/odom

# Vérifier les topics de commandes
ros2 topic list | grep -E "(arm_control|mode_command|takeoff_command)"
# /arm_control/arm_cmd
# /arm_control/disarm_cmd
# /mode_command
# /takeoff_command
# /land_command
```

## Exemple d'utilisation complète

### Script Python pour une mission simple

```python
#!/usr/bin/env python3
"""
Exemple d'utilisation du système intégré
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String, Float64
import json
import time

class DroneController(Node):
    def __init__(self):
        super().__init__('drone_controller_example')
        
        # Publishers pour les commandes (via drone_interface)
        self.arm_pub = self.create_publisher(Bool, '/arm_control/arm_cmd', 10)
        self.disarm_pub = self.create_publisher(Bool, '/arm_control/disarm_cmd', 10)
        self.mode_pub = self.create_publisher(String, '/mode_command', 10)
        self.takeoff_pub = self.create_publisher(Float64, '/takeoff_command', 10)
        self.land_pub = self.create_publisher(Bool, '/land_command', 10)
        
        # Subscriber pour le statut (depuis mavros_interface)
        self.status_sub = self.create_subscription(
            String,
            '/drone_nav/drone_status',
            self.status_callback,
            10
        )
        
        self.drone_status = None
        
    def status_callback(self, msg):
        """Réception du statut depuis mavros_interface_node"""
        try:
            self.drone_status = json.loads(msg.data)
            self.get_logger().info(f"Statut: connecté={self.drone_status['connected']}, "
                                 f"armé={self.drone_status['armed']}, "
                                 f"mode={self.drone_status['mode']}")
        except Exception as e:
            self.get_logger().error(f"Erreur parsing statut: {e}")
    
    def wait_for_connection(self):
        """Attendre la connexion MAVROS"""
        self.get_logger().info("Attente de la connexion MAVROS...")
        while rclpy.ok() and (not self.drone_status or not self.drone_status['connected']):
            rclpy.spin_once(self, timeout_sec=1.0)
        self.get_logger().info("✅ Connexion MAVROS établie")
    
    def change_mode(self, mode):
        """Changer le mode de vol"""
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f"🔄 Changement de mode vers: {mode}")
        
        # Attendre confirmation
        start_time = time.time()
        while (time.time() - start_time < 10.0 and 
               (not self.drone_status or self.drone_status['mode'] != mode)):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.drone_status and self.drone_status['mode'] == mode:
            self.get_logger().info(f"✅ Mode {mode} confirmé")
            return True
        else:
            self.get_logger().error(f"❌ Échec changement vers {mode}")
            return False
    
    def arm_drone(self):
        """Armer le drone"""
        msg = Bool()
        msg.data = True
        self.arm_pub.publish(msg)
        self.get_logger().info("🔧 Armement du drone...")
        
        # Attendre confirmation
        start_time = time.time()
        while (time.time() - start_time < 10.0 and 
               (not self.drone_status or not self.drone_status['armed'])):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.drone_status and self.drone_status['armed']:
            self.get_logger().info("✅ Drone armé")
            return True
        else:
            self.get_logger().error("❌ Échec de l'armement")
            return False
    
    def takeoff(self, altitude=10.0):
        """Décollage à l'altitude spécifiée"""
        msg = Float64()
        msg.data = altitude
        self.takeoff_pub.publish(msg)
        self.get_logger().info(f"🚁 Décollage à {altitude}m...")
        
        # Attendre d'atteindre l'altitude
        start_time = time.time()
        while (time.time() - start_time < 30.0 and 
               (not self.drone_status or self.drone_status['altitude'] < altitude * 0.9)):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.drone_status and self.drone_status['altitude'] >= altitude * 0.9:
            self.get_logger().info(f"✅ Altitude {altitude}m atteinte")
            return True
        else:
            self.get_logger().error("❌ Échec du décollage")
            return False
    
    def land(self):
        """Atterrissage"""
        msg = Bool()
        msg.data = True
        self.land_pub.publish(msg)
        self.get_logger().info("🛬 Atterrissage...")
        
        # Attendre l'atterrissage
        start_time = time.time()
        while (time.time() - start_time < 30.0 and 
               (not self.drone_status or self.drone_status['altitude'] > 0.5)):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.drone_status and self.drone_status['altitude'] <= 0.5:
            self.get_logger().info("✅ Atterrissage réussi")
            return True
        else:
            self.get_logger().error("❌ Échec de l'atterrissage")
            return False
    
    def disarm_drone(self):
        """Désarmer le drone"""
        msg = Bool()
        msg.data = True
        self.disarm_pub.publish(msg)
        self.get_logger().info("🔓 Désarmement du drone...")
        
        # Attendre confirmation
        start_time = time.time()
        while (time.time() - start_time < 10.0 and 
               (not self.drone_status or self.drone_status['armed'])):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        if self.drone_status and not self.drone_status['armed']:
            self.get_logger().info("✅ Drone désarmé")
            return True
        else:
            self.get_logger().error("❌ Échec du désarmement")
            return False

def main():
    rclpy.init()
    
    controller = DroneController()
    
    try:
        # Séquence complète de mission
        controller.wait_for_connection()
        
        if controller.change_mode("GUIDED"):
            if controller.arm_drone():
                if controller.takeoff(10.0):
                    # Vol stationnaire pendant 10 secondes
                    controller.get_logger().info("🕐 Vol stationnaire 10s...")
                    time.sleep(10)
                    
                    # Atterrissage et désarmement
                    controller.land()
                    controller.disarm_drone()
                    
                    controller.get_logger().info("🎉 Mission terminée avec succès!")
                else:
                    controller.get_logger().error("❌ Mission annulée - échec décollage")
            else:
                controller.get_logger().error("❌ Mission annulée - échec armement")
        else:
            controller.get_logger().error("❌ Mission annulée - échec changement mode")
    
    except KeyboardInterrupt:
        controller.get_logger().info("Mission interrompue par l'utilisateur")
    
    finally:
        controller.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Monitoring et diagnostic

### Script de surveillance

```bash
#!/bin/bash
# monitor_system.sh

echo "=== Surveillance du système de drone ==="

echo "1. Vérification des nœuds actifs:"
ros2 node list | grep -E "(mavros|drone|arm|mode|takeoff)"

echo -e "\n2. Statut de connexion MAVROS:"
timeout 3s ros2 topic echo /mavros/state --once | grep -E "(connected|armed|mode)"

echo -e "\n3. Statut du drone (mavros_interface):"
timeout 3s ros2 service call /drone_nav/get_drone_status std_srvs/srv/Trigger

echo -e "\n4. Position actuelle:"
timeout 3s ros2 service call /drone_nav/get_current_position std_srvs/srv/Trigger

echo -e "\n5. Évaluation de sécurité:"
timeout 3s ros2 topic echo /drone_nav/safety_status --once

echo -e "\n6. Fréquences des topics:"
echo "Position:" $(ros2 topic hz /drone_nav/position --window 5 2>/dev/null | grep "average rate" | awk '{print $3}')
echo "Vitesse:" $(ros2 topic hz /drone_nav/velocity --window 5 2>/dev/null | grep "average rate" | awk '{print $3}')
echo "Statut:" $(ros2 topic hz /drone_nav/drone_status --window 5 2>/dev/null | grep "average rate" | awk '{print $3}')
```

## Avantages de cette architecture

1. **Séparation claire des responsabilités** :
   - `mavros_interface_node` : Collecte et distribution de données
   - `drone_interface` : Commandes et actions sur le drone

2. **Robustesse** :
   - Échec d'un nœud de commande n'affecte pas la collecte de données
   - Monitoring continu même si les commandes échouent

3. **Flexibilité** :
   - Possibilité d'utiliser différentes stratégies de commandes
   - Interface de données standardisée pour tous les nœuds de navigation

4. **Maintenance facilitée** :
   - Chaque composant a un rôle spécifique et bien défini
   - Tests et debug plus simples par composant

Cette architecture intégrée offre un système complet, robuste et maintenable pour le contrôle de drones autonomes.
