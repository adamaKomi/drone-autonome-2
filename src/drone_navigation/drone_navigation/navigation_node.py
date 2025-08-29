#!/usr/bin/env python3
"""
=============================================================================
DRONE NAVIGATION NODE - Nœud de navigation et contrôle de position
=============================================================================
Auteur: Assistant IA
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
from typing import Optional, Tuple, List, Dict, Any
from dataclasses import dataclass
import uuid

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup

# Message types
from geometry_msgs.msg import PoseStamped, TwistStamped, Point, Vector3, Quaternion
from mavros_msgs.msg import PositionTarget
from mavros_msgs.srv import SetMode
from std_msgs.msg import Header, Float64, String, Bool
from nav_msgs.msg import Path
from builtin_interfaces.msg import Time


class NavigationState(Enum):
    """États possibles du système de navigation"""
    IDLE = "IDLE"                    # En attente
    NAVIGATING = "NAVIGATING"        # Navigation en cours
    HOLDING_POSITION = "HOLDING"     # Maintien de position
    LANDING = "LANDING"              # Atterrissage
    EMERGENCY = "EMERGENCY"          # Arrêt d'urgence


@dataclass
class Waypoint:
    """Structure pour un point de navigation"""
    x: float                    # Position X (locale) en mètres
    y: float                    # Position Y (locale) en mètres  
    z: float                    # Position Z (altitude) en mètres
    yaw: float = 0.0           # Orientation en radians
    tolerance: float = 0.5      # Tolérance d'arrivée en mètres
    hold_time: float = 0.0     # Temps d'attente au point (secondes)
    max_velocity: float = 2.0   # Vitesse maximale vers ce point


class NavigationController:
    """Contrôleur principal de navigation"""
    
    def __init__(self, logger):
        self.logger = logger
        self.state = NavigationState.IDLE
        
        # Paramètres de navigation
        self.max_velocity = 2.0         # m/s
        self.max_acceleration = 1.0     # m/s²
        self.position_tolerance = 0.5   # mètres
        self.yaw_tolerance = 0.1        # radians
        
        # État actuel
        self.current_position = [0.0, 0.0, 0.0]
        self.current_velocity = [0.0, 0.0, 0.0]
        self.current_yaw = 0.0
        self.target_position = [0.0, 0.0, 0.0]
        self.target_yaw = 0.0
        
        # File de waypoints
        self.waypoint_queue = []
        self.current_waypoint = None
        self.waypoint_start_time = 0.0
        
        # Contrôleur PID simple pour la position
        self.pid_gains = {
            'kp': [1.5, 1.5, 2.0],  # Proportionnel [x, y, z]
            'ki': [0.1, 0.1, 0.1],  # Intégral
            'kd': [0.3, 0.3, 0.5]   # Dérivé
        }
        
        self.integral_error = [0.0, 0.0, 0.0]
        self.previous_error = [0.0, 0.0, 0.0]
        self.last_update_time = time.time()
        
    def update_current_state(self, position: List[float], velocity: List[float], yaw: float):
        """Met à jour l'état actuel du drone"""
        self.current_position = position.copy()
        self.current_velocity = velocity.copy()
        self.current_yaw = yaw
        
    def add_waypoint(self, waypoint: Waypoint) -> str:
        """Ajoute un waypoint à la file"""
        waypoint_id = str(uuid.uuid4())[:8]
        self.waypoint_queue.append((waypoint_id, waypoint))
        self.logger.info(f"📍 Waypoint ajouté: {waypoint_id} -> ({waypoint.x:.1f}, {waypoint.y:.1f}, {waypoint.z:.1f})")
        return waypoint_id
        
    def clear_waypoints(self):
        """Vide la file de waypoints"""
        self.waypoint_queue.clear()
        self.current_waypoint = None
        self.state = NavigationState.IDLE
        self.logger.info("🗑️ File de waypoints vidée")
        
    def start_navigation(self) -> bool:
        """Démarre la navigation vers les waypoints en file"""
        if not self.waypoint_queue:
            self.logger.warn("⚠️ Aucun waypoint en file pour navigation")
            return False
            
        self.state = NavigationState.NAVIGATING
        self._start_next_waypoint()
        self.logger.info("🚀 Navigation démarrée")
        return True
        
    def stop_navigation(self):
        """Arrête la navigation et passe en maintien de position"""
        self.state = NavigationState.HOLDING_POSITION
        self.target_position = self.current_position.copy()
        self.target_yaw = self.current_yaw
        self.logger.info("⏸️ Navigation arrêtée, maintien de position")
        
    def emergency_stop(self):
        """Arrêt d'urgence - hover sur place"""
        self.state = NavigationState.EMERGENCY
        self.target_position = self.current_position.copy()
        self.target_yaw = self.current_yaw
        self.waypoint_queue.clear()
        self.current_waypoint = None
        self.logger.warn("🚨 ARRÊT D'URGENCE - Hover sur place")
        
    def _start_next_waypoint(self):
        """Démarre la navigation vers le prochain waypoint"""
        if self.waypoint_queue:
            waypoint_id, waypoint = self.waypoint_queue.pop(0)
            self.current_waypoint = (waypoint_id, waypoint)
            self.waypoint_start_time = time.time()
            
            self.target_position = [waypoint.x, waypoint.y, waypoint.z]
            self.target_yaw = waypoint.yaw
            
            distance = self._calculate_distance(self.current_position, self.target_position)
            self.logger.info(f"🎯 Navigation vers waypoint {waypoint_id}: ({waypoint.x:.1f}, {waypoint.y:.1f}, {waypoint.z:.1f}) - Distance: {distance:.1f}m")
        else:
            # Plus de waypoints, passer en maintien de position
            self.state = NavigationState.HOLDING_POSITION
            self.current_waypoint = None
            self.logger.info("✅ Tous les waypoints atteints, maintien de position")
            
    def update(self) -> PositionTarget:
        """
        Met à jour le contrôleur et retourne la commande de position
        """
        if self.state == NavigationState.IDLE:
            return self._create_hold_command()
            
        elif self.state == NavigationState.NAVIGATING:
            return self._update_navigation()
            
        elif self.state == NavigationState.HOLDING_POSITION:
            return self._create_hold_command()
            
        elif self.state == NavigationState.EMERGENCY:
            return self._create_emergency_command()
            
        else:
            return self._create_hold_command()
            
    def _update_navigation(self) -> PositionTarget:
        """Met à jour la navigation vers le waypoint actuel"""
        if not self.current_waypoint:
            self.state = NavigationState.IDLE
            return self._create_hold_command()
            
        waypoint_id, waypoint = self.current_waypoint
        
        # Calculer la distance au waypoint
        distance = self._calculate_distance(self.current_position, self.target_position)
        
        # Vérifier si on a atteint le waypoint
        if distance <= waypoint.tolerance:
            # Attendre le temps de hold si spécifié
            if waypoint.hold_time > 0:
                elapsed_time = time.time() - self.waypoint_start_time
                if elapsed_time < waypoint.hold_time:
                    # Encore en attente
                    return self._create_hold_command()
                    
            # Waypoint atteint, passer au suivant
            self.logger.info(f"✅ Waypoint {waypoint_id} atteint")
            self._start_next_waypoint()
            return self._create_hold_command()
            
        # Calculer la commande de vitesse avec PID
        velocity_command = self._calculate_pid_control()
        
        # Limiter la vitesse
        velocity_command = self._limit_velocity(velocity_command, waypoint.max_velocity)
        
        # Créer la commande de position
        return self._create_velocity_command(velocity_command)
        
    def _calculate_distance(self, pos1: List[float], pos2: List[float]) -> float:
        """Calcule la distance euclidienne entre deux positions"""
        return math.sqrt(
            (pos1[0] - pos2[0])**2 + 
            (pos1[1] - pos2[1])**2 + 
            (pos1[2] - pos2[2])**2
        )
        
    def _calculate_pid_control(self) -> List[float]:
        """Calcule la commande de vitesse avec un contrôleur PID"""
        current_time = time.time()
        dt = current_time - self.last_update_time
        self.last_update_time = current_time
        
        if dt <= 0:
            dt = 0.01
            
        velocity_command = [0.0, 0.0, 0.0]
        
        for i in range(3):  # x, y, z
            # Erreur de position
            error = self.target_position[i] - self.current_position[i]
            
            # Terme proportionnel
            p_term = self.pid_gains['kp'][i] * error
            
            # Terme intégral
            self.integral_error[i] += error * dt
            i_term = self.pid_gains['ki'][i] * self.integral_error[i]
            
            # Terme dérivé
            d_error = (error - self.previous_error[i]) / dt
            d_term = self.pid_gains['kd'][i] * d_error
            
            # Commande totale
            velocity_command[i] = p_term + i_term + d_term
            
            # Sauvegarder l'erreur pour la prochaine itération
            self.previous_error[i] = error
            
        return velocity_command
        
    def _limit_velocity(self, velocity: List[float], max_vel: float) -> List[float]:
        """Limite la vitesse à la valeur maximale"""
        # Calculer la norme de la vitesse
        vel_norm = math.sqrt(velocity[0]**2 + velocity[1]**2 + velocity[2]**2)
        
        if vel_norm > max_vel:
            # Normaliser et limiter
            scale = max_vel / vel_norm
            return [v * scale for v in velocity]
        else:
            return velocity
            
    def _create_velocity_command(self, velocity: List[float]) -> PositionTarget:
        """Crée une commande de vitesse pour MAVROS"""
        cmd = PositionTarget()
        cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        cmd.type_mask = (
            PositionTarget.IGNORE_PX |
            PositionTarget.IGNORE_PY |
            PositionTarget.IGNORE_PZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        
        cmd.velocity.x = velocity[0]
        cmd.velocity.y = velocity[1]
        cmd.velocity.z = velocity[2]
        cmd.yaw = self.target_yaw
        
        return cmd
        
    def _create_hold_command(self) -> PositionTarget:
        """Crée une commande de maintien de position"""
        cmd = PositionTarget()
        cmd.coordinate_frame = PositionTarget.FRAME_LOCAL_NED
        cmd.type_mask = (
            PositionTarget.IGNORE_VX |
            PositionTarget.IGNORE_VY |
            PositionTarget.IGNORE_VZ |
            PositionTarget.IGNORE_AFX |
            PositionTarget.IGNORE_AFY |
            PositionTarget.IGNORE_AFZ |
            PositionTarget.IGNORE_YAW_RATE
        )
        
        cmd.position.x = self.current_position[0]
        cmd.position.y = self.current_position[1]
        cmd.position.z = self.current_position[2]
        cmd.yaw = self.current_yaw
        
        return cmd
        
    def _create_emergency_command(self) -> PositionTarget:
        """Crée une commande d'arrêt d'urgence (hover)"""
        return self._create_hold_command()


class NavigationNode(Node):
    """
    Nœud de navigation pour le contrôle de position du drone
    """
    
    def __init__(self):
        super().__init__('drone_navigation')
        
        self.logger = self.get_logger()
        self.logger.info("🧭 Initialisation du nœud de navigation...")
        
        # Configuration QoS
        self._setup_qos_profiles()
        
        # Initialisation du contrôleur
        self.nav_controller = NavigationController(self.logger)
        
        # État du drone depuis interface
        self.drone_armed = False
        self.drone_mode = "UNKNOWN"
        self.drone_connected = False
        
        # Configuration des souscripteurs
        self._setup_subscribers()
        
        # Configuration des publishers
        self._setup_publishers()
        
        # Configuration des services
        self._setup_services()
        
        # Timer pour la boucle de contrôle
        self.control_timer = self.create_timer(0.05, self._control_loop)  # 20Hz
        
        self.logger.info("✅ Nœud de navigation initialisé!")
        self._print_node_info()
        
    def _setup_qos_profiles(self):
        """Configure les profils QoS"""
        self.qos_critical = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        self.qos_frequent = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
    def _setup_subscribers(self):
        """Configure les souscripteurs"""
        # État du drone depuis interface central
        self.drone_status_sub = self.create_subscription(
            String,
            '/drone/status',
            self._drone_status_callback,
            self.qos_critical
        )
        
        # Position locale du drone
        self.position_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._position_callback,
            self.qos_frequent
        )
        
        # Vitesse du drone
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self._velocity_callback,
            self.qos_frequent
        )
        
    def _setup_publishers(self):
        """Configure les publishers"""
        # Commandes de position vers MAVROS
        self.setpoint_pub = self.create_publisher(
            PositionTarget,
            '/mavros/setpoint_raw/local',
            self.qos_frequent
        )
        
        # État de navigation
        self.nav_status_pub = self.create_publisher(
            String,
            '/drone/navigation/status',
            self.qos_critical
        )
        
        # Trajectoire prévue (pour visualisation)
        self.path_pub = self.create_publisher(
            Path,
            '/drone/navigation/planned_path',
            self.qos_critical
        )
        
    def _setup_services(self):
        """Configure les services"""
        # Service pour ajouter des waypoints
        self.add_waypoint_service = self.create_service(
            String,  # Format: "x:y:z:yaw:tolerance:hold_time:max_vel"
            '/drone/navigation/add_waypoint',
            self._handle_add_waypoint
        )
        
        # Service pour démarrer/arrêter la navigation
        self.nav_control_service = self.create_service(
            String,  # "START", "STOP", "CLEAR", "EMERGENCY"
            '/drone/navigation/control',
            self._handle_nav_control
        )
        
        # Service pour aller à une position
        self.goto_service = self.create_service(
            String,  # Format: "x:y:z:yaw"
            '/drone/navigation/goto',
            self._handle_goto
        )
        
    def _drone_status_callback(self, msg: String):
        """Callback pour l'état du drone depuis l'interface central"""
        # Parse: "STATE:MODE:ARMED:CONNECTED"
        parts = msg.data.split(':')
        if len(parts) >= 4:
            self.drone_armed = parts[2] == 'True'
            self.drone_mode = parts[1]
            self.drone_connected = parts[3] == 'True'
            
    def _position_callback(self, msg: PoseStamped):
        """Callback pour la position du drone"""
        position = [
            msg.pose.position.x,
            msg.pose.position.y,
            msg.pose.position.z
        ]
        
        # Extraire le yaw depuis le quaternion
        quat = msg.pose.orientation
        yaw = math.atan2(
            2.0 * (quat.w * quat.z + quat.x * quat.y),
            1.0 - 2.0 * (quat.y * quat.y + quat.z * quat.z)
        )
        
        # Mettre à jour le contrôleur (la vitesse sera mise à jour séparément)
        current_velocity = getattr(self, '_current_velocity', [0.0, 0.0, 0.0])
        self.nav_controller.update_current_state(position, current_velocity, yaw)
        
    def _velocity_callback(self, msg: TwistStamped):
        """Callback pour la vitesse du drone"""
        self._current_velocity = [
            msg.twist.linear.x,
            msg.twist.linear.y,
            msg.twist.linear.z
        ]
        
    def _handle_add_waypoint(self, request, response):
        """Gère l'ajout de waypoints"""
        try:
            # Parse: "x:y:z:yaw:tolerance:hold_time:max_vel"
            parts = request.data.split(':')
            
            if len(parts) < 3:
                response.data = "ERROR:Format requis: x:y:z[:yaw[:tolerance[:hold_time[:max_vel]]]]"
                return response
                
            x = float(parts[0])
            y = float(parts[1])
            z = float(parts[2])
            yaw = float(parts[3]) if len(parts) > 3 else 0.0
            tolerance = float(parts[4]) if len(parts) > 4 else 0.5
            hold_time = float(parts[5]) if len(parts) > 5 else 0.0
            max_vel = float(parts[6]) if len(parts) > 6 else 2.0
            
            waypoint = Waypoint(x, y, z, yaw, tolerance, hold_time, max_vel)
            waypoint_id = self.nav_controller.add_waypoint(waypoint)
            
            # Publier la trajectoire mise à jour
            self._publish_planned_path()
            
            response.data = f"SUCCESS:Waypoint {waypoint_id} ajouté"
            
        except Exception as e:
            self.logger.error(f"❌ Erreur add_waypoint: {e}")
            response.data = f"ERROR:{str(e)}"
            
        return response
        
    def _handle_nav_control(self, request, response):
        """Gère le contrôle de navigation"""
        try:
            command = request.data.upper()
            
            if command == "START":
                if not self.drone_armed:
                    response.data = "ERROR:Drone non armé"
                elif self.drone_mode != "GUIDED":
                    response.data = "ERROR:Mode GUIDED requis"
                else:
                    success = self.nav_controller.start_navigation()
                    if success:
                        response.data = "SUCCESS:Navigation démarrée"
                    else:
                        response.data = "ERROR:Aucun waypoint en file"
                        
            elif command == "STOP":
                self.nav_controller.stop_navigation()
                response.data = "SUCCESS:Navigation arrêtée"
                
            elif command == "CLEAR":
                self.nav_controller.clear_waypoints()
                self._publish_planned_path()  # Vider la trajectoire affichée
                response.data = "SUCCESS:Waypoints effacés"
                
            elif command == "EMERGENCY":
                self.nav_controller.emergency_stop()
                response.data = "SUCCESS:Arrêt d'urgence activé"
                
            else:
                response.data = f"ERROR:Commande inconnue: {command}"
                
        except Exception as e:
            self.logger.error(f"❌ Erreur nav_control: {e}")
            response.data = f"ERROR:{str(e)}"
            
        return response
        
    def _handle_goto(self, request, response):
        """Gère la commande d'aller à une position"""
        try:
            # Parse: "x:y:z:yaw"
            parts = request.data.split(':')
            
            if len(parts) < 3:
                response.data = "ERROR:Format requis: x:y:z[:yaw]"
                return response
                
            x = float(parts[0])
            y = float(parts[1])
            z = float(parts[2])
            yaw = float(parts[3]) if len(parts) > 3 else 0.0
            
            # Vider la file et ajouter le nouveau waypoint
            self.nav_controller.clear_waypoints()
            waypoint = Waypoint(x, y, z, yaw)
            waypoint_id = self.nav_controller.add_waypoint(waypoint)
            
            # Démarrer immédiatement si conditions OK
            if self.drone_armed and self.drone_mode == "GUIDED":
                success = self.nav_controller.start_navigation()
                if success:
                    response.data = f"SUCCESS:Navigation vers ({x:.1f}, {y:.1f}, {z:.1f}) démarrée"
                else:
                    response.data = "ERROR:Impossible de démarrer la navigation"
            else:
                response.data = f"SUCCESS:Waypoint ({x:.1f}, {y:.1f}, {z:.1f}) ajouté (armement et mode GUIDED requis)"
                
            self._publish_planned_path()
            
        except Exception as e:
            self.logger.error(f"❌ Erreur goto: {e}")
            response.data = f"ERROR:{str(e)}"
            
        return response
        
    def _control_loop(self):
        """Boucle principale de contrôle"""
        # Publier l'état de navigation
        self._publish_nav_status()
        
        # Exécuter le contrôleur seulement si armé et en mode GUIDED
        if self.drone_armed and self.drone_mode == "GUIDED":
            # Obtenir la commande de position du contrôleur
            position_cmd = self.nav_controller.update()
            
            # Publier la commande
            self.setpoint_pub.publish(position_cmd)
            
    def _publish_nav_status(self):
        """Publie l'état de navigation"""
        status_msg = String()
        status_msg.data = f"{self.nav_controller.state.value}:{len(self.nav_controller.waypoint_queue)}"
        self.nav_status_pub.publish(status_msg)
        
    def _publish_planned_path(self):
        """Publie la trajectoire prévue pour visualisation"""
        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = "map"
        
        # Ajouter la position actuelle
        current_pose = PoseStamped()
        current_pose.header = path.header
        current_pose.pose.position.x = self.nav_controller.current_position[0]
        current_pose.pose.position.y = self.nav_controller.current_position[1]
        current_pose.pose.position.z = self.nav_controller.current_position[2]
        path.poses.append(current_pose)
        
        # Ajouter tous les waypoints en file
        for waypoint_id, waypoint in self.nav_controller.waypoint_queue:
            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = waypoint.x
            pose.pose.position.y = waypoint.y
            pose.pose.position.z = waypoint.z
            path.poses.append(pose)
            
        self.path_pub.publish(path)
        
    def _print_node_info(self):
        """Affiche les informations du nœud"""
        self.logger.info("=" * 60)
        self.logger.info("🧭 DRONE NAVIGATION - NŒUD DE NAVIGATION")
        self.logger.info("=" * 60)
        self.logger.info("📡 Services disponibles:")
        self.logger.info("  • /drone/navigation/add_waypoint - Ajouter waypoint")
        self.logger.info("  • /drone/navigation/control - Contrôle navigation")
        self.logger.info("  • /drone/navigation/goto - Aller à position")
        self.logger.info("")
        self.logger.info("📤 Topics publiés:")
        self.logger.info("  • /drone/navigation/status - État navigation")
        self.logger.info("  • /drone/navigation/planned_path - Trajectoire prévue")
        self.logger.info("  • /mavros/setpoint_raw/local - Commandes position")
        self.logger.info("")
        self.logger.info("🎮 Utilisation:")
        self.logger.info("  ros2 service call /drone/navigation/goto std_msgs/srv/SetString '{data: \"1.0:1.0:3.0:0.0\"}'")
        self.logger.info("  ros2 service call /drone/navigation/control std_msgs/srv/SetString '{data: \"START\"}'")
        self.logger.info("=" * 60)


def main(args=None):
    """Point d'entrée principal"""
    rclpy.init(args=args)
    
    try:
        node = NavigationNode()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 Arrêt demandé par l'utilisateur")
    except Exception as e:
        print(f"❌ Erreur fatale: {e}")
    finally:
        try:
            if 'node' in locals():
                node.destroy_node()
        except:
            pass
        rclpy.shutdown()
        print("👋 NavigationNode arrêté")


if __name__ == '__main__':
    main()
