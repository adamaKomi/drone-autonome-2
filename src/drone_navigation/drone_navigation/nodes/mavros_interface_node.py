#!/usr/bin/env python3
"""
MAVROS Interface Node - Interface exclusive avec MAVROS/ArduPilot
Auteur: Adama Komi
Date: 2025-09-11
Version: 1.0.0

Ce nœud gère la communication exclusive avec MAVROS et l'autopilot.
Il abstrait les détails de MAVROS pour le reste du système et fournit
une interface standardisée pour les commandes de vol.

Références:
- MAVROS Documentation: http://wiki.ros.org/mavros
- MAVLink Protocol: https://mavlink.io/en/
- ArduPilot Documentation: https://ardupilot.org/
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Messages ROS2 standards
from geometry_msgs.msg import PoseStamped, TwistStamped, Point, Vector3
from sensor_msgs.msg import NavSatFix, BatteryState
from std_msgs.msg import String, Float64, Header
from nav_msgs.msg import Odometry

# Messages MAVROS
from mavros_msgs.msg import State, ExtendedState, Altitude

# Services ROS2
from std_srvs.srv import Trigger

import threading
import math
import time
from typing import Optional, Dict, Any, Tuple
from dataclasses import dataclass, field
from enum import Enum, auto


class FlightMode(Enum):
    """Modes de vol supportés"""
    MANUAL = "MANUAL"
    STABILIZE = "STABILIZE"
    GUIDED = "GUIDED"
    AUTO = "AUTO"
    LOITER = "LOITER"
    RTL = "RTL"
    LAND = "LAND"


class VehicleState(Enum):
    """États du véhicule"""
    UNKNOWN = "UNKNOWN"
    DISCONNECTED = "DISCONNECTED"
    CONNECTED = "CONNECTED"
    ARMED = "ARMED"
    DISARMED = "DISARMED"
    FLYING = "FLYING"
    LANDED = "LANDED"


@dataclass
class DroneStatus:
    """Structure des données de statut du drone"""
    connected: bool = False
    armed: bool = False
    guided: bool = False
    mode: str = "UNKNOWN"
    system_status: int = 0
    
    # Position et orientation
    position: Optional[Point] = field(default_factory=lambda: Point())
    velocity: Optional[Vector3] = field(default_factory=lambda: Vector3())
    altitude: float = 0.0
    heading: float = 0.0
    
    # GPS
    gps_fix: bool = False
    satellites: int = 0
    
    # Batterie
    battery_voltage: float = 0.0
    battery_percentage: float = 0.0
    
    # Timestamps
    last_heartbeat: float = 0.0
    last_position_update: float = 0.0


class MavrosInterfaceNode(Node):
    """
    Nœud d'interface avec MAVROS - Collecte et distribution de données
    
    Ce nœud est responsable de:
    - Communication unidirectionnelle depuis MAVROS (lecture des données)
    - Abstraction des détails MAVROS pour le système de navigation
    - Conversion et normalisation des messages entre formats
    - Surveillance de l'état de connexion et diagnostic
    - Distribution des données de télémétrie au système
    
    Note: Les commandes de vol (armement, mode, décollage/atterrissage) 
    sont gérées par les nœuds spécialisés du package drone_interface.
    """
    
    def __init__(self):
        super().__init__('mavros_interface_node', namespace='drone_nav')
        
        # Configuration QoS pour les topics critiques
        self.qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.qos_cmd = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.qos_status = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # État du drone
        self.drone_status = DroneStatus()
        self.status_lock = threading.RLock()
        
        # Paramètres du nœud
        self._declare_parameters()
        
        # Subscribers MAVROS
        self._setup_mavros_subscribers()
        
        # Publishers pour le système de navigation
        self._setup_nav_publishers()
        
        # Note: Les services de commandes (armement, mode, décollage) 
        # sont gérés par les nœuds spécialisés du package drone_interface
        
        # Subscriber pour commandes de navigation
        self._setup_nav_subscribers()
        
        # Services pour informations et diagnostic uniquement
        self._setup_info_services()
        
        # Timers
        self.status_timer = self.create_timer(1.0, self._publish_status)
        self.heartbeat_timer = self.create_timer(0.1, self._check_heartbeat)
        
        # État de connexion
        self.connection_timeout = 5.0  # secondes
        self.last_mavros_heartbeat = time.time()
        self.initialization_complete = False
        
        # Compteurs pour diagnostic
        self.message_counters = {
            'state': 0,
            'position': 0,
            'velocity': 0,
            'battery': 0,
            'gps': 0
        }
        
        self.get_logger().info("MAVROS Interface Node initialisé")
    
    def _declare_parameters(self):
        """Déclare les paramètres du nœud"""
        # Namespace MAVROS
        self.declare_parameter('mavros_namespace', '/mavros')
        
        # Timeouts
        self.declare_parameter('connection_timeout', 5.0)
        self.declare_parameter('command_timeout', 10.0)
        
        # Fréquences de publication
        self.declare_parameter('status_frequency', 1.0)
        self.declare_parameter('position_frequency', 50.0)
        
        # Mode de simulation
        self.declare_parameter('simulation_mode', True)
        
        # Frame de référence
        self.declare_parameter('local_frame', 'map')
        self.declare_parameter('global_frame', 'earth')
    
    def _setup_mavros_subscribers(self):
        """Configure les subscribers MAVROS"""
        mavros_ns = self.get_parameter('mavros_namespace').value
        
        # État de la connexion
        self.state_sub = self.create_subscription(
            State,
            f'{mavros_ns}/state',
            self._state_callback,
            self.qos_sensor
        )
        
        # État étendu
        self.extended_state_sub = self.create_subscription(
            ExtendedState,
            f'{mavros_ns}/extended_state',
            self._extended_state_callback,
            self.qos_sensor
        )
        
        # Position locale
        self.local_pose_sub = self.create_subscription(
            PoseStamped,
            f'{mavros_ns}/local_position/pose',
            self._local_pose_callback,
            self.qos_sensor
        )
        
        # Vitesse locale
        self.local_velocity_sub = self.create_subscription(
            TwistStamped,
            f'{mavros_ns}/local_position/velocity_local',
            self._local_velocity_callback,
            self.qos_sensor
        )
        
        # Position GPS
        self.global_position_sub = self.create_subscription(
            NavSatFix,
            f'{mavros_ns}/global_position/global',
            self._global_position_callback,
            self.qos_sensor
        )
        
        # Altitude
        self.altitude_sub = self.create_subscription(
            Altitude,
            f'{mavros_ns}/altitude',
            self._altitude_callback,
            self.qos_sensor
        )
        
        # Batterie
        self.battery_sub = self.create_subscription(
            BatteryState,
            f'{mavros_ns}/battery',
            self._battery_callback,
            self.qos_sensor
        )
    
    def _setup_nav_publishers(self):
        """Configure les publishers pour le système de navigation"""
        # Position du drone
        self.position_pub = self.create_publisher(
            PoseStamped,
            'position',
            self.qos_sensor
        )
        
        # Vitesse du drone
        self.velocity_pub = self.create_publisher(
            TwistStamped,
            'velocity',
            self.qos_sensor
        )
        
        # Statut complet du drone
        self.status_pub = self.create_publisher(
            String,
            'drone_status',
            self.qos_status
        )
        
        # Indicateurs de sécurité
        self.safety_status_pub = self.create_publisher(
            String,
            'safety_status',
            self.qos_status
        )
        
        # Odométrie pour la navigation
        self.odom_pub = self.create_publisher(
            Odometry,
            'odom',
            self.qos_sensor
        )
    
    def _setup_info_services(self):
        """Configure les services d'information et diagnostic uniquement"""
        # Statut du drone
        self.status_service = self.create_service(
            Trigger,
            'get_drone_status',
            self._handle_get_status
        )
        
        # Position actuelle
        self.position_service = self.create_service(
            Trigger,
            'get_current_position',
            self._handle_get_position
        )
        
        # Service de diagnostic
        self.diagnostic_service = self.create_service(
            Trigger,
            'get_diagnostics',
            self._handle_get_diagnostics
        )
    
    def _setup_nav_subscribers(self):
        """Configure les subscribers pour les commandes de navigation"""
        mavros_ns = self.get_parameter('mavros_namespace').value
        
        # Commandes de vitesse
        self.cmd_vel_sub = self.create_subscription(
            TwistStamped,
            'cmd_vel',
            self._cmd_vel_callback,
            self.qos_cmd
        )
        
        # Publisher vers MAVROS pour les commandes de vitesse
        self.mavros_cmd_vel_pub = self.create_publisher(
            TwistStamped,
            f'{mavros_ns}/setpoint_velocity/cmd_vel',
            self.qos_cmd
        )
        
        # Commandes de position
        self.cmd_pose_sub = self.create_subscription(
            PoseStamped,
            'cmd_pose',
            self._cmd_pose_callback,
            self.qos_cmd
        )
        
        # Publisher vers MAVROS pour les commandes de position
        self.mavros_cmd_pose_pub = self.create_publisher(
            PoseStamped,
            f'{mavros_ns}/setpoint_position/local',
            self.qos_cmd
        )
    
    # Callbacks MAVROS
    def _state_callback(self, msg: State):
        """Callback pour l'état MAVROS"""
        try:
            with self.status_lock:
                self.drone_status.connected = msg.connected
                self.drone_status.armed = msg.armed
                self.drone_status.guided = msg.guided
                self.drone_status.mode = msg.mode
                self.drone_status.system_status = msg.system_status
                self.drone_status.last_heartbeat = time.time()
            
            self.last_mavros_heartbeat = time.time()
            self.message_counters['state'] += 1
            
            # Marquer l'initialisation comme complète après la première réception
            if not self.initialization_complete:
                self.initialization_complete = True
                self.get_logger().info("Connexion MAVROS établie")
                
        except Exception as e:
            self.get_logger().error(f"Erreur dans state_callback: {e}")
    
    def _extended_state_callback(self, msg: ExtendedState):
        """Callback pour l'état étendu MAVROS"""
        # Traitement de l'état étendu si nécessaire
        pass
    
    def _local_pose_callback(self, msg: PoseStamped):
        """Callback pour la position locale"""
        try:
            with self.status_lock:
                self.drone_status.position = msg.pose.position
                
                # Calculer le heading depuis le quaternion
                self.drone_status.heading = self._quaternion_to_yaw(msg.pose.orientation)
                self.drone_status.last_position_update = time.time()
            
            self.message_counters['position'] += 1
            
            # Republier pour le système de navigation
            self.position_pub.publish(msg)
            
            # Publier l'odométrie
            self._publish_odometry(msg)
            
        except Exception as e:
            self.get_logger().error(f"Erreur dans local_pose_callback: {e}")
    
    def _local_velocity_callback(self, msg: TwistStamped):
        """Callback pour la vitesse locale"""
        try:
            with self.status_lock:
                self.drone_status.velocity = msg.twist.linear
            
            self.message_counters['velocity'] += 1
            
            # Republier pour le système de navigation
            self.velocity_pub.publish(msg)
            
        except Exception as e:
            self.get_logger().error(f"Erreur dans local_velocity_callback: {e}")
    
    def _global_position_callback(self, msg: NavSatFix):
        """Callback pour la position GPS"""
        try:
            with self.status_lock:
                self.drone_status.gps_fix = msg.status.status >= 0
                # Note: msg.status.service contient le nombre de satellites pour certains systèmes
                if hasattr(msg.status, 'service') and msg.status.service > 0:
                    self.drone_status.satellites = msg.status.service
            
            self.message_counters['gps'] += 1
            
        except Exception as e:
            self.get_logger().error(f"Erreur dans global_position_callback: {e}")
    
    def _altitude_callback(self, msg: Altitude):
        """Callback pour l'altitude"""
        with self.status_lock:
            self.drone_status.altitude = msg.relative
    
    def _battery_callback(self, msg: BatteryState):
        """Callback pour la batterie"""
        try:
            with self.status_lock:
                self.drone_status.battery_voltage = msg.voltage
                # Conversion pourcentage (0.0-1.0 -> 0.0-100.0)
                if msg.percentage >= 0:  # Vérifier que la valeur est valide
                    self.drone_status.battery_percentage = msg.percentage * 100.0
                else:
                    # Estimation basée sur la tension si le pourcentage n'est pas disponible
                    self.drone_status.battery_percentage = self._estimate_battery_percentage(msg.voltage)
            
            self.message_counters['battery'] += 1
            
        except Exception as e:
            self.get_logger().error(f"Erreur dans battery_callback: {e}")
    
    # Callbacks pour commandes de navigation
    def _cmd_vel_callback(self, msg: TwistStamped):
        """Callback pour les commandes de vitesse"""
        # Transférer la commande vers MAVROS
        self.mavros_cmd_vel_pub.publish(msg)
    
    def _cmd_pose_callback(self, msg: PoseStamped):
        """Callback pour les commandes de position"""
        # Transférer la commande vers MAVROS
        self.mavros_cmd_pose_pub.publish(msg)
    
    # Gestionnaires de services (information uniquement)
    def _handle_get_status(self, request, response):
        """Gestionnaire pour obtenir le statut du drone"""
        try:
            import json
            
            with self.status_lock:
                status_dict = {
                    'connected': self.drone_status.connected,
                    'armed': self.drone_status.armed,
                    'guided': self.drone_status.guided,
                    'mode': self.drone_status.mode,
                    'altitude': self.drone_status.altitude,
                    'gps_fix': self.drone_status.gps_fix,
                    'battery_percentage': self.drone_status.battery_percentage,
                    'timestamp': time.time()
                }
            
            response.success = True
            response.message = json.dumps(status_dict)
        
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
        
        return response
    
    def _handle_get_position(self, request, response):
        """Gestionnaire pour obtenir la position actuelle"""
        try:
            import json
            
            with self.status_lock:
                if self.drone_status.position is not None:
                    position_data = {
                        'position': {
                            'x': self.drone_status.position.x,
                            'y': self.drone_status.position.y,
                            'z': self.drone_status.position.z
                        },
                        'heading': self.drone_status.heading,
                        'altitude': self.drone_status.altitude,
                        'timestamp': time.time(),
                        'frame_id': self.get_parameter('local_frame').value
                    }
                    
                    response.success = True
                    response.message = json.dumps(position_data)
                else:
                    response.success = False
                    response.message = "Position non disponible"
                    self.get_logger().warning("Position non disponible")
        
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
            self.get_logger().error(f"Erreur lors de l'obtention de la position: {e}")
        
        return response
    
    def _handle_get_diagnostics(self, request, response):
        """Gestionnaire pour obtenir les diagnostics du nœud"""
        try:
            import json
            
            diagnostics = {
                'node_name': self.get_name(),
                'initialization_complete': self.initialization_complete,
                'connection_timeout': self.connection_timeout,
                'last_heartbeat': self.last_mavros_heartbeat,
                'message_counters': self.message_counters.copy(),
                'parameters': {
                    'mavros_namespace': self.get_parameter('mavros_namespace').value,
                    'simulation_mode': self.get_parameter('simulation_mode').value,
                    'local_frame': self.get_parameter('local_frame').value
                },
                'timestamp': time.time()
            }
            
            response.success = True
            response.message = json.dumps(diagnostics, indent=2)
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur diagnostics: {e}"
            self.get_logger().error(f"Erreur lors des diagnostics: {e}")
        
        return response
    
    # Méthodes utilitaires
    def _quaternion_to_yaw(self, quaternion) -> float:
        """Convertit un quaternion en angle de lacet (yaw)"""
        try:
            # Extraction du yaw depuis le quaternion
            siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
            cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
            return math.atan2(siny_cosp, cosy_cosp)
        except Exception as e:
            self.get_logger().error(f"Erreur conversion quaternion: {e}")
            return 0.0
    
    def _estimate_battery_percentage(self, voltage: float) -> float:
        """Estime le pourcentage de batterie basé sur la tension"""
        # Estimation pour batterie LiPo 4S (14.8V nominal, 16.8V max, 12.8V min)
        voltage_max = 16.8
        voltage_min = 12.8
        
        if voltage <= voltage_min:
            return 0.0
        elif voltage >= voltage_max:
            return 100.0
        else:
            return ((voltage - voltage_min) / (voltage_max - voltage_min)) * 100.0
    
    def _publish_odometry(self, pose_msg: PoseStamped):
        """Publie l'odométrie basée sur la pose et la vitesse"""
        try:
            odom_msg = Odometry()
            
            # Header
            odom_msg.header = pose_msg.header
            odom_msg.header.frame_id = self.get_parameter('local_frame').value
            odom_msg.child_frame_id = "base_link"
            
            # Position et orientation
            odom_msg.pose.pose = pose_msg.pose
            
            # Vitesse
            with self.status_lock:
                if self.drone_status.velocity is not None:
                    odom_msg.twist.twist.linear = self.drone_status.velocity
                    odom_msg.twist.twist.angular.z = 0.0  # Pas de vitesse angulaire pour l'instant
            
            # Covariances (à ajuster selon les capteurs)
            odom_msg.pose.covariance[0] = 0.1   # x
            odom_msg.pose.covariance[7] = 0.1   # y  
            odom_msg.pose.covariance[14] = 0.1  # z
            odom_msg.pose.covariance[35] = 0.1  # yaw
            
            odom_msg.twist.covariance[0] = 0.1   # vx
            odom_msg.twist.covariance[7] = 0.1   # vy
            odom_msg.twist.covariance[14] = 0.1  # vz
            
            self.odom_pub.publish(odom_msg)
            
        except Exception as e:
            self.get_logger().error(f"Erreur publication odométrie: {e}")
    
    def _publish_status(self):
        """Publie le statut du drone périodiquement"""
        try:
            import json
            
            with self.status_lock:
                # Statut complet
                status_dict = {
                    'timestamp': time.time(),
                    'connected': self.drone_status.connected,
                    'armed': self.drone_status.armed,
                    'guided': self.drone_status.guided,
                    'mode': self.drone_status.mode,
                    'system_status': self.drone_status.system_status,
                    'position': {
                        'x': self.drone_status.position.x if self.drone_status.position else 0.0,
                        'y': self.drone_status.position.y if self.drone_status.position else 0.0,
                        'z': self.drone_status.position.z if self.drone_status.position else 0.0
                    } if self.drone_status.position else {'x': 0.0, 'y': 0.0, 'z': 0.0},
                    'velocity': {
                        'x': self.drone_status.velocity.x if self.drone_status.velocity else 0.0,
                        'y': self.drone_status.velocity.y if self.drone_status.velocity else 0.0,
                        'z': self.drone_status.velocity.z if self.drone_status.velocity else 0.0
                    } if self.drone_status.velocity else {'x': 0.0, 'y': 0.0, 'z': 0.0},
                    'altitude': self.drone_status.altitude,
                    'heading': self.drone_status.heading,
                    'gps_fix': self.drone_status.gps_fix,
                    'satellites': self.drone_status.satellites,
                    'battery': {
                        'voltage': self.drone_status.battery_voltage,
                        'percentage': self.drone_status.battery_percentage
                    }
                }
                
                # Statut de sécurité
                safety_status = self._evaluate_safety_status()
            
            # Publier le statut complet
            status_msg = String()
            status_msg.data = json.dumps(status_dict)
            self.status_pub.publish(status_msg)
            
            # Publier le statut de sécurité
            safety_msg = String()
            safety_msg.data = json.dumps(safety_status)
            self.safety_status_pub.publish(safety_msg)
        
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la publication du statut: {e}")
    
    def _evaluate_safety_status(self) -> Dict[str, Any]:
        """Évalue le statut de sécurité du drone"""
        warnings = []
        errors = []
        
        # Vérifier la connexion
        if not self.drone_status.connected:
            errors.append("Drone non connecté")
        
        # Vérifier le GPS
        if not self.drone_status.gps_fix:
            warnings.append("GPS fix non disponible")
        
        if self.drone_status.satellites < 6:
            warnings.append(f"Peu de satellites GPS: {self.drone_status.satellites}")
        
        # Vérifier la batterie
        if self.drone_status.battery_percentage < 20:
            warnings.append(f"Batterie faible: {self.drone_status.battery_percentage:.1f}%")
        
        if self.drone_status.battery_percentage < 10:
            errors.append(f"Batterie critique: {self.drone_status.battery_percentage:.1f}%")
        
        # Vérifier l'altitude
        if self.drone_status.altitude > 100:
            warnings.append(f"Altitude élevée: {self.drone_status.altitude:.1f}m")
        
        # Déterminer le niveau de sécurité
        if errors:
            safety_level = "CRITICAL"
        elif warnings:
            safety_level = "WARNING"
        else:
            safety_level = "NORMAL"
        
        return {
            'level': safety_level,
            'warnings': warnings,
            'errors': errors,
            'timestamp': time.time()
        }
    
    def _check_heartbeat(self):
        """Vérifie la connexion MAVROS"""
        current_time = time.time()
        time_since_heartbeat = current_time - self.last_mavros_heartbeat
        
        if time_since_heartbeat > self.connection_timeout:
            if self.drone_status.connected:
                self.get_logger().warning("Perte de connexion MAVROS détectée")
                with self.status_lock:
                    self.drone_status.connected = False
    
    def get_drone_status(self) -> DroneStatus:
        """Retourne le statut actuel du drone (thread-safe)"""
        with self.status_lock:
            return self.drone_status


def main(args=None):
    """Point d'entrée principal"""
    rclpy.init(args=args)
    
    node = None
    try:
        node = MavrosInterfaceNode()
        
        # Utiliser MultiThreadedExecutor pour les services
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        node.get_logger().info("MAVROS Interface Node démarré")
        executor.spin()
        
    except KeyboardInterrupt:
        if node:
            node.get_logger().info("Arrêt demandé par l'utilisateur")
    except Exception as e:
        if node:
            node.get_logger().error(f"Erreur: {e}")
        else:
            print(f"Erreur: {e}")
    finally:
        if node:
            try:
                node.destroy_node()
            except:
                pass
        
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
