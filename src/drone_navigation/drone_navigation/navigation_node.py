#!/usr/bin/env python3
"""
=============================================================================
DRONE NAVIGATION NODE - Nœud de navigation et contrôle de position
=============================================================================
Auteur: Adama Komi
Date: 2025-08-29
Version: 1.0.0

Description:
    Nœud ROS2 spécialisé pour la navigation et le contrôle de position du drone.
    Fournit des services de navigation avancés et gère les waypoints.
    
Responsabilités:
    - Navigation autonome vers des waypoints
    - Contrôle de position et d'altitude
    - Gestion des trajectoires
    - Évitement d'obstacles basique
    - Interface avec le nœud drone_interface central

Utilisation:
    ros2 run drone_navigation navigation_node
=============================================================================
"""

import math
import time
import threading
from enum import Enum
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.action import ActionClient
from rclpy.task import Future

# Message types
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TwistStamped, Vector3
from geographic_msgs.msg import GeoPose
from nav_msgs.msg import Path
from std_msgs.msg import Header, String, Bool, Float64
from std_srvs.srv import Trigger, SetBool

# Action types
from drone_interface.action import NavigateToGoal, FollowPath

# Custom service types (à créer avec ros2 interface)
from drone_navigation.srv import (
    SetWaypoint, 
    GetWaypoints, 
    ClearWaypoints,
    SetVelocity,
    SetPosition
)


class NavigationState(Enum):
    """États de navigation possibles"""
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    HOLDING = "HOLDING"
    EMERGENCY = "EMERGENCY"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"


@dataclass
class Waypoint:
    """Structure pour un waypoint de navigation"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    tolerance: float = 0.5
    wait_time: float = 0.0
    name: str = ""


@dataclass
class DronePosition:
    """Position actuelle du drone"""
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    yaw: float = 0.0
    velocity_x: float = 0.0
    velocity_y: float = 0.0
    velocity_z: float = 0.0
    timestamp: float = 0.0


class NavigationController:
    """Contrôleur de navigation pour le drone"""
    
    def __init__(self, node):
        self.node = node
        self.logger = node.get_logger()
        
        # État de navigation
        self.state = NavigationState.IDLE
        self.current_waypoint = None
        self.waypoints: List[Waypoint] = []
        
        # Position actuelle
        self.position = DronePosition()
        
        # Paramètres de navigation
        self.max_velocity = 2.0  # m/s
        self.position_tolerance = 0.3  # m
        self.yaw_tolerance = 0.1  # rad
        
        # Verrou pour accès thread-safe
        self.lock = threading.RLock()
        
        # Callbacks
        self.navigation_callbacks = []
        
    def update_position(self, pose: PoseStamped):
        """Met à jour la position du drone"""
        with self.lock:
            self.position.x = pose.pose.position.x
            self.position.y = pose.pose.position.y
            self.position.z = pose.pose.position.z
            self.position.timestamp = time.time()
            
            # Extraire le yaw de l'orientation (quaternion)
            orientation = pose.pose.orientation
            self.position.yaw = self._quaternion_to_yaw(orientation)
    
    def update_velocity(self, twist: TwistStamped):
        """Met à jour la vitesse du drone"""
        with self.lock:
            self.position.velocity_x = twist.twist.linear.x
            self.position.velocity_y = twist.twist.linear.y
            self.position.velocity_z = twist.twist.linear.z
    
    def add_waypoint(self, waypoint: Waypoint) -> bool:
        """Ajoute un waypoint à la mission"""
        with self.lock:
            self.waypoints.append(waypoint)
            self.logger.info(f"Waypoint ajouté: {waypoint.name} ({waypoint.x}, {waypoint.y}, {waypoint.z})")
            return True
    
    def clear_waypoints(self) -> bool:
        """Efface tous les waypoints"""
        with self.lock:
            self.waypoints.clear()
            self.logger.info("Tous les waypoints effacés")
            return True
    
    def get_waypoints(self) -> List[Waypoint]:
        """Retourne la liste des waypoints"""
        with self.lock:
            return self.waypoints.copy()
    
    def start_navigation(self) -> bool:
        """Démarre la navigation vers les waypoints"""
        with self.lock:
            if not self.waypoints:
                self.logger.warn("Aucun waypoint défini")
                return False
            
            if self.state != NavigationState.IDLE:
                self.logger.warn(f"Navigation déjà en cours: {self.state}")
                return False
            
            self.state = NavigationState.NAVIGATING
            self.current_waypoint = self.waypoints[0]
            self.logger.info(f"Début navigation vers: {self.current_waypoint.name}")
            return True
    
    def stop_navigation(self) -> bool:
        """Arrête la navigation en cours"""
        with self.lock:
            if self.state == NavigationState.IDLE:
                return True
            
            self.state = NavigationState.IDLE
            self.current_waypoint = None
            self.logger.info("Navigation arrêtée")
            return True
    
    def compute_navigation_command(self) -> Optional[Tuple[float, float, float, float]]:
        """Calcule la commande de navigation vers le waypoint actuel"""
        with self.lock:
            if self.state != NavigationState.NAVIGATING or not self.current_waypoint:
                return None
            
            # Calcul des erreurs de position
            dx = self.current_waypoint.x - self.position.x
            dy = self.current_waypoint.y - self.position.y
            dz = self.current_waypoint.z - self.position.z
            dyaw = self.current_waypoint.yaw - self.position.yaw
            
            # Normalisation de l'angle yaw
            while dyaw > math.pi:
                dyaw -= 2 * math.pi
            while dyaw < -math.pi:
                dyaw += 2 * math.pi
            
            # Distance totale
            distance = math.sqrt(dx**2 + dy**2 + dz**2)
            
            # Vérification si le waypoint est atteint
            if distance < self.current_waypoint.tolerance and abs(dyaw) < self.yaw_tolerance:
                self.logger.info(f"Waypoint atteint: {self.current_waypoint.name}")
                self._next_waypoint()
                return None
            
            # Contrôle proportionnel simple
            kp_linear = 0.5
            kp_yaw = 0.8
            
            # Commandes de vitesse
            vx = max(min(dx * kp_linear, self.max_velocity), -self.max_velocity)
            vy = max(min(dy * kp_linear, self.max_velocity), -self.max_velocity)
            vz = max(min(dz * kp_linear, self.max_velocity/2), -self.max_velocity/2)
            vyaw = max(min(dyaw * kp_yaw, 1.0), -1.0)
            
            return vx, vy, vz, vyaw
    
    def _next_waypoint(self):
        """Passe au waypoint suivant"""
        if self.waypoints:
            self.waypoints.pop(0)
            if self.waypoints:
                self.current_waypoint = self.waypoints[0]
                self.logger.info(f"Navigation vers next waypoint: {self.current_waypoint.name}")
            else:
                self.current_waypoint = None
                self.state = NavigationState.COMPLETED
                self.logger.info("✅ Mission de navigation terminée!")
                self._notify_navigation_complete(True)
        else:
            self.current_waypoint = None
            self.state = NavigationState.IDLE
    
    def _quaternion_to_yaw(self, q: Quaternion) -> float:
        """Convertit un quaternion en angle yaw (radians)"""
        # Conversion quaternion to yaw
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def add_navigation_callback(self, callback):
        """Ajoute un callback pour les événements de navigation"""
        self.navigation_callbacks.append(callback)
    
    def _notify_navigation_complete(self, success: bool):
        """Notifie tous les callbacks de la fin de navigation"""
        for callback in self.navigation_callbacks:
            try:
                callback(success)
            except Exception as e:
                self.logger.error(f"Error in navigation callback: {e}")


class DroneNavigation(Node):
    """Nœud principal de navigation pour le drone"""
    
    def __init__(self):
        super().__init__('drone_navigation')
        
        self.logger = self.get_logger()
        self.logger.info("🧭 Initialisation du DroneNavigation...")
        
        # Configuration QoS
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialisation du contrôleur de navigation
        self.nav_controller = NavigationController(self)
        
        # Setup des subscribers
        self._setup_subscribers()
        
        # Setup des publishers
        self._setup_publishers()
        
        # Setup des services
        self._setup_services()
        
        # Setup des actions
        self._setup_actions()
        
        # Timer pour le contrôle de navigation
        self.control_timer = self.create_timer(0.1, self._navigation_control_callback)  # 10Hz
        
        self.logger.info("✅ DroneNavigation initialisé avec succès!")
        self._print_node_info()
    
    def _setup_subscribers(self):
        """Configure les subscribers"""
        self.logger.info("📡 Configuration des subscribers...")
        
        # Position locale du drone
        self.position_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.nav_controller.update_position,
            self.qos_profile
        )
        
        # Vitesse du drone
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self.nav_controller.update_velocity,
            self.qos_profile
        )
        
        # État du drone (optionnel)
        self.state_sub = self.create_subscription(
            String,
            '/drone/status',
            self._drone_status_callback,
            self.qos_profile
        )
    
    def _setup_publishers(self):
        """Configure les publishers"""
        self.logger.info("📤 Configuration des publishers...")
        
        # Commande de vitesse
        self.velocity_pub = self.create_publisher(
            TwistStamped,
            '/mavros/setpoint_velocity/cmd_vel',
            self.qos_profile
        )
        
        # Commande de position
        self.position_pub = self.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            self.qos_profile
        )
        
        # État de navigation
        self.nav_status_pub = self.create_publisher(
            String,
            '/drone/navigation/status',
            self.qos_profile
        )
        
        # Waypoints actuels
        self.waypoints_pub = self.create_publisher(
            String,
            '/drone/navigation/waypoints',
            self.qos_profile
        )
    
    def _setup_services(self):
        """Configure les services"""
        self.logger.info("🔧 Configuration des services...")
        
        # Service pour ajouter un waypoint
        self.set_waypoint_service = self.create_service(
            SetWaypoint,
            '/drone/navigation/set_waypoint',
            self._handle_set_waypoint
        )
        
        # Service pour obtenir les waypoints
        self.get_waypoints_service = self.create_service(
            GetWaypoints,
            '/drone/navigation/get_waypoints',
            self._handle_get_waypoints
        )
        
        # Service pour effacer les waypoints
        self.clear_waypoints_service = self.create_service(
            ClearWaypoints,
            '/drone/navigation/clear_waypoints',
            self._handle_clear_waypoints
        )
        
        # Service pour démarrer la navigation
        self.start_nav_service = self.create_service(
            Trigger,
            '/drone/navigation/start',
            self._handle_start_navigation
        )
        
        # Service pour arrêter la navigation
        self.stop_nav_service = self.create_service(
            Trigger,
            '/drone/navigation/stop',
            self._handle_stop_navigation
        )
    
    def _setup_actions(self):
        """Configure les actions"""
        self.logger.info("⚡ Configuration des actions...")
        
        # Action pour navigation vers un goal
        self.navigate_action = ActionClient(
            self,
            NavigateToGoal,
            '/drone/navigation/navigate_to_goal'
        )
        
        # Action pour suivre un chemin
        self.follow_path_action = ActionClient(
            self,
            FollowPath,
            '/drone/navigation/follow_path'
        )
    
    def _handle_set_waypoint(self, request, response):
        """Gère l'ajout d'un waypoint"""
        try:
            waypoint = Waypoint(
                x=request.x,
                y=request.y,
                z=request.z,
                yaw=request.yaw,
                tolerance=request.tolerance,
                wait_time=request.wait_time,
                name=request.name
            )
            
            success = self.nav_controller.add_waypoint(waypoint)
            response.success = success
            response.message = f"Waypoint {request.name} ajouté" if success else "Échec ajout waypoint"
            
        except Exception as e:
            self.logger.error(f"Erreur set_waypoint: {e}")
            response.success = False
            response.message = f"Erreur: {str(e)}"
        
        return response
    
    def _handle_get_waypoints(self, request, response):
        """Gère la récupération des waypoints"""
        try:
            waypoints = self.nav_controller.get_waypoints()
            response.waypoints = []
            
            for wp in waypoints:
                response.waypoints.append({
                    'x': wp.x,
                    'y': wp.y,
                    'z': wp.z,
                    'yaw': wp.yaw,
                    'tolerance': wp.tolerance,
                    'wait_time': wp.wait_time,
                    'name': wp.name
                })
            
            response.success = True
            response.message = f"{len(waypoints)} waypoints récupérés"
            
        except Exception as e:
            self.logger.error(f"Erreur get_waypoints: {e}")
            response.success = False
            response.message = f"Erreur: {str(e)}"
        
        return response
    
    def _handle_clear_waypoints(self, request, response):
        """Gère l'effacement des waypoints"""
        try:
            success = self.nav_controller.clear_waypoints()
            response.success = success
            response.message = "Waypoints effacés" if success else "Échec effacement"
            
        except Exception as e:
            self.logger.error(f"Erreur clear_waypoints: {e}")
            response.success = False
            response.message = f"Erreur: {str(e)}"
        
        return response
    
    def _handle_start_navigation(self, request, response):
        """Démarre la navigation"""
        try:
            success = self.nav_controller.start_navigation()
            response.success = success
            response.message = "Navigation démarrée" if success else "Échec démarrage navigation"
            
        except Exception as e:
            self.logger.error(f"Erreur start_navigation: {e}")
            response.success = False
            response.message = f"Erreur: {str(e)}"
        
        return response
    
    def _handle_stop_navigation(self, request, response):
        """Arrête la navigation"""
        try:
            success = self.nav_controller.stop_navigation()
            response.success = success
            response.message = "Navigation arrêtée" if success else "Échec arrêt navigation"
            
        except Exception as e:
            self.logger.error(f"Erreur stop_navigation: {e}")
            response.success = False
            response.message = f"Erreur: {str(e)}"
        
        return response
    
    def _navigation_control_callback(self):
        """Callback pour le contrôle de navigation"""
        try:
            # Calculer la commande de navigation
            command = self.nav_controller.compute_navigation_command()
            
            if command:
                vx, vy, vz, vyaw = command
                
                # Publier la commande de vitesse
                twist_msg = TwistStamped()
                twist_msg.header.stamp = self.get_clock().now().to_msg()
                twist_msg.header.frame_id = "map"
                
                twist_msg.twist.linear = Vector3(x=float(vx), y=float(vy), z=float(vz))
                twist_msg.twist.angular = Vector3(x=0.0, y=0.0, z=float(vyaw))
                
                self.velocity_pub.publish(twist_msg)
                
                # Publier l'état de navigation
                self._publish_navigation_status()
            
        except Exception as e:
            self.logger.error(f"Erreur navigation control: {e}")
    
    def _publish_navigation_status(self):
        """Publie l'état de navigation"""
        status_msg = String()
        status_data = {
            'state': self.nav_controller.state.value,
            'current_waypoint': self.nav_controller.current_waypoint.name if self.nav_controller.current_waypoint else None,
            'remaining_waypoints': len(self.nav_controller.waypoints)
        }
        status_msg.data = str(status_data)
        self.nav_status_pub.publish(status_msg)
    
    def _drone_status_callback(self, msg: String):
        """Callback pour l'état du drone"""
        # Peut être utilisé pour réagir aux changements d'état du drone
        pass
    
    def _print_node_info(self):
        """Affiche les informations du nœud"""
        info = [
            "=" * 60,
            "🧭 DRONE NAVIGATION NODE - Contrôle de position",
            "=" * 60,
            "📡 Subscribers:",
            "   - /mavros/local_position/pose",
            "   - /mavros/local_position/velocity_local", 
            "   - /drone/status",
            "",
            "📤 Publishers:",
            "   - /mavros/setpoint_velocity/cmd_vel",
            "   - /mavros/setpoint_position/local",
            "   - /drone/navigation/status",
            "   - /drone/navigation/waypoints",
            "",
            "🔧 Services:",
            "   - /drone/navigation/set_waypoint",
            "   - /drone/navigation/get_waypoints", 
            "   - /drone/navigation/clear_waypoints",
            "   - /drone/navigation/start",
            "   - /drone/navigation/stop",
            "",
            "⚡ Actions:",
            "   - /drone/navigation/navigate_to_goal",
            "   - /drone/navigation/follow_path",
            "=" * 60
        ]
        
        for line in info:
            self.logger.info(line)


def main(args=None):
    """Point d'entrée principal"""
    rclpy.init(args=args)
    
    try:
        node = DroneNavigation()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        node.logger.info("🛑 Arrêt demandé")
    except Exception as e:
        node.logger.error(f"💥 Erreur fatale: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()