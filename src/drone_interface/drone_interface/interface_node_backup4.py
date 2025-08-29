#!/usr/bin/env python3
"""
=============================================================================
DRONE INTERFACE NODE - Central Interface for ArduPilot SITL Control
=============================================================================
Auteur: Adama Komi
Date: 2025-08-29
Version: 2.0.0

Description:
    Nœud ROS2 central pour l'interface avec MAVROS et ArduPilot.
    Fournit les services de base et publie l'état du drone.
    Architecture modulaire avec des nœuds spécialisés séparés.
    
Architecture:
    - drone_interface: Interface centrale MAVROS (ce nœud)
    - drone_navigation: Navigation et contrôle de position
    - drone_mission: Gestion et exécution de missions
    - drone_vision: Traitement de vision (futur)
    
Responsabilités de ce nœud:
    - Communication avec MAVROS
    - Armement/Désarmement sécurisé
    - Changement de modes de vol
    - Publication de l'état du drone
    - Vérifications de sécurité
    - Services de contrôle de base

Utilisation:
    ros2 run drone_interface interface_node
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
from rcl_interfaces.msg import SetParametersResult

# Message types
from geometry_msgs.msg import PoseStamped, TwistStamped, Point, Vector3, Quaternion
from mavros_msgs.msg import State, ExtendedState
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from sensor_msgs.msg import NavSatFix, BatteryState
from std_msgs.msg import Header, Float64, String, Bool
from std_srvs.srv import Trigger
from nav_msgs.msg import Odometry
from builtin_interfaces.msg import Time

# Importer nos services personnalisés après la génération
try:
    from drone_interface.srv import DroneCommand
except ImportError:
    # Fallback vers Trigger si pas encore généré
    DroneCommand = None


class DroneState(Enum):
    """Énumération des états possibles du drone"""
    UNKNOWN = "UNKNOWN"           # État inconnu
    DISCONNECTED = "DISCONNECTED" # Déconnecté de MAVROS
    CONNECTED = "CONNECTED"       # Connecté mais désarmé
    ARMED = "ARMED"              # Armé et prêt
    FLYING = "FLYING"            # En vol
    LANDING = "LANDING"          # En cours d'atterrissage
    EMERGENCY = "EMERGENCY"      # Situation d'urgence


class FlightMode(Enum):
    """Modes de vol ArduPilot supportés"""
    STABILIZE = "STABILIZE"       # Mode stabilisé (contrôle manuel)
    ACRO = "ACRO"                # Mode acrobatique
    ALT_HOLD = "ALT_HOLD"        # Maintien d'altitude
    AUTO = "AUTO"                # Mode automatique (mission)
    GUIDED = "GUIDED"            # Mode guidé (contrôle via GCS)
    LOITER = "LOITER"            # Vol stationnaire
    RTL = "RTL"                  # Retour au point de départ
    CIRCLE = "CIRCLE"            # Vol en cercle
    LAND = "LAND"                # Atterrissage automatique
    SPORT = "SPORT"              # Mode sport
    FLIP = "FLIP"                # Mode flip
    AUTOTUNE = "AUTOTUNE"        # Auto-ajustement PID
    POSHOLD = "POSHOLD"          # Maintien de position
    BRAKE = "BRAKE"              # Freinage d'urgence
    THROW = "THROW"              # Mode lancer
    AVOID_ADSB = "AVOID_ADSB"    # Évitement ADSB
    GUIDED_NOGPS = "GUIDED_NOGPS" # Guidé sans GPS
    SMART_RTL = "SMART_RTL"      # RTL intelligent
    FLOWHOLD = "FLOWHOLD"        # Maintien flux optique


@dataclass
class DroneStatusData:
    """Structure de données pour l'état complet du drone"""
    state: DroneState = DroneState.UNKNOWN
    mode: str = "UNKNOWN"
    armed: bool = False
    connected: bool = False
    guided: bool = False
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)  # quaternion
    angular_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    battery_voltage: float = 0.0
    battery_percentage: float = 0.0
    battery_current: float = 0.0
    gps_fix: bool = False
    gps_satellites: int = 0
    gps_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # lat, lon, alt
    altitude_barometric: float = 0.0
    altitude_relative: float = 0.0
    home_distance: float = 0.0
    geofence_violation: bool = False
    low_battery_warning: bool = False
    critical_battery_warning: bool = False
    altitude_warning: bool = False
    last_heartbeat: float = 0.0
    last_update: float = 0.0


class SafetyManager:
    """Module de gestion de la sécurité"""
    
    def __init__(self, logger):
        self.logger = logger
        
        # Paramètres de sécurité configurables
        self._safety_checks_enabled = True
        self._min_battery_voltage = 10.5      # Voltage minimum pour ArduPilot
        self._min_battery_percentage = 20.0   # Pourcentage minimum
        self._min_gps_satellites = 6          # Satellites GPS minimum
        self._max_altitude = 120.0            # Altitude maximum en mètres
        self._geofence_radius = 100.0         # Rayon de géofence en mètres
        self._critical_battery_threshold = 20.0  # Seuil batterie critique
        self._low_battery_threshold = 30.0    # Seuil batterie faible
        
    def check_arm_conditions(self, status: DroneStatusData) -> Tuple[bool, str, List[str]]:
        """
        Vérifie les conditions de sécurité avant l'armement
        Returns: (can_arm, reason_if_not, warnings_list)
        """
        if not self._safety_checks_enabled:
            return True, "Safety checks disabled", []
            
        warnings = []
        
        # Vérification de la connexion
        if not status.connected:
            return False, "Drone not connected to flight controller", warnings
            
        # Vérification GPS (si nécessaire selon le mode)
        if not status.gps_fix:
            return False, "GPS fix not available", warnings
            
        if status.gps_satellites < self._min_gps_satellites:
            return False, f"Insufficient GPS satellites ({status.gps_satellites}/{self._min_gps_satellites})", warnings
            
        # Vérification batterie
        if status.battery_voltage > 0 and status.battery_voltage < self._min_battery_voltage:
            return False, f"Battery voltage too low ({status.battery_voltage:.1f}V < {self._min_battery_voltage}V)", warnings
            
        if status.battery_percentage > 0 and status.battery_percentage < self._min_battery_percentage:
            return False, f"Battery percentage too low ({status.battery_percentage:.1f}% < {self._min_battery_percentage}%)", warnings
            
        # Avertissements non bloquants
        if status.battery_percentage < self._low_battery_threshold:
            warnings.append(f"Low battery: {status.battery_percentage:.1f}%")
            
        return True, "All safety checks passed", warnings
        
    def check_flight_safety(self, status: DroneStatusData) -> Tuple[bool, List[str], List[str]]:
        """
        Vérifie la sécurité en vol continu
        Returns: (is_safe, warnings_list, errors_list)
        """
        warnings = []
        errors = []
        
        # Vérification altitude
        if status.position[2] > self._max_altitude:
            errors.append(f"Altitude too high: {status.position[2]:.1f}m > {self._max_altitude}m")
            
        # Vérification distance du home
        distance_from_home = math.sqrt(status.position[0]**2 + status.position[1]**2)
        if distance_from_home > self._geofence_radius:
            errors.append(f"Outside geofence: {distance_from_home:.1f}m > {self._geofence_radius}m")
            
        # Vérification batterie critique
        if status.battery_percentage > 0:
            if status.battery_percentage < self._critical_battery_threshold:
                errors.append(f"Critical battery level: {status.battery_percentage:.1f}%")
            elif status.battery_percentage < self._low_battery_threshold:
                warnings.append(f"Low battery level: {status.battery_percentage:.1f}%")
        
        # Vérification connexion
        if not status.connected:
            errors.append("Lost connection to flight controller")
            
        # Vérification GPS en vol
        if status.armed and not status.gps_fix:
            warnings.append("GPS fix lost during flight")
            
        is_safe = len(errors) == 0
        return is_safe, warnings, errors
        
    def get_safety_limits(self) -> Dict[str, float]:
        """Retourne les limites de sécurité actuelles"""
        return {
            'max_altitude': self._max_altitude,
            'geofence_radius': self._geofence_radius,
            'min_battery_voltage': self._min_battery_voltage,
            'min_battery_percentage': self._min_battery_percentage,
            'min_gps_satellites': self._min_gps_satellites
        }


class StateManager:
    """Module de gestion d'état du drone"""
    
    def __init__(self, logger, safety_manager: SafetyManager):
        self.logger = logger
        self.safety_manager = safety_manager
        self.status = DroneStatusData()
        self._state_callbacks = []
        self._lock = threading.Lock()
        self._home_position = None
        
    def update_mavros_state(self, msg: State):
        """Met à jour l'état depuis MAVROS State"""
        with self._lock:
            self.status.connected = msg.connected
            self.status.armed = msg.armed
            self.status.guided = msg.guided
            self.status.mode = msg.mode
            self.status.last_heartbeat = time.time()
            self.status.last_update = time.time()
            
            # Déterminer l'état global
            if not msg.connected:
                self.status.state = DroneState.DISCONNECTED
            elif not msg.armed:
                self.status.state = DroneState.CONNECTED
            elif msg.mode in ["LAND", "RTL"] and msg.armed:
                self.status.state = DroneState.LANDING
            elif msg.armed:
                self.status.state = DroneState.FLYING if self.status.position[2] > 0.5 else DroneState.ARMED
                
            # Mettre à jour les alertes de sécurité
            self._update_safety_alerts()
            
            # Notifier les callbacks
            self._notify_state_change()
            
    def update_position(self, msg: PoseStamped):
        """Met à jour la position depuis MAVROS local position"""
        with self._lock:
            self.status.position = (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            )
            
            # Mettre à jour l'orientation
            self.status.orientation = (
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            )
            
            # Calculer la distance du home
            if self._home_position is None and self.status.position != (0.0, 0.0, 0.0):
                self._home_position = self.status.position
                self.logger.info(f"Home position set: {self._home_position}")
                
            if self._home_position:
                self.status.home_distance = math.sqrt(
                    (self.status.position[0] - self._home_position[0])**2 +
                    (self.status.position[1] - self._home_position[1])**2
                )
            
    def update_velocity(self, msg: TwistStamped):
        """Met à jour la vélocité depuis MAVROS"""
        with self._lock:
            self.status.velocity = (
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.linear.z
            )
            
            self.status.angular_velocity = (
                msg.twist.angular.x,
                msg.twist.angular.y,
                msg.twist.angular.z
            )
            
    def update_battery(self, msg: BatteryState):
        """Met à jour l'état de la batterie"""
        with self._lock:
            self.status.battery_voltage = msg.voltage
            self.status.battery_percentage = msg.percentage * 100 if msg.percentage >= 0 else 0
            self.status.battery_current = msg.current
            
    def update_gps(self, msg: NavSatFix):
        """Met à jour l'état GPS"""
        with self._lock:
            self.status.gps_fix = msg.status.status >= 0
            self.status.gps_position = (
                msg.latitude,
                msg.longitude,
                msg.altitude
            )
            
    def update_altitude(self, barometric: float, relative: float):
        """Met à jour les données d'altitude"""
        with self._lock:
            self.status.altitude_barometric = barometric
            self.status.altitude_relative = relative
            
    def _update_safety_alerts(self):
        """Met à jour les alertes de sécurité"""
        # Alerte batterie
        if self.status.battery_percentage > 0:
            self.status.critical_battery_warning = self.status.battery_percentage < 20.0
            self.status.low_battery_warning = self.status.battery_percentage < 30.0
        
        # Alerte altitude
        self.status.altitude_warning = self.status.position[2] > 100.0  # 100m d'alerte
        
        # Alerte géofence
        self.status.geofence_violation = self.status.home_distance > 100.0
            
    def get_status(self) -> DroneStatusData:
        """Retourne une copie de l'état actuel"""
        with self._lock:
            # Créer une copie pour éviter les modifications concurrentes
            return DroneStatusData(
                state=self.status.state,
                mode=self.status.mode,
                armed=self.status.armed,
                connected=self.status.connected,
                guided=self.status.guided,
                position=self.status.position,
                velocity=self.status.velocity,
                orientation=self.status.orientation,
                angular_velocity=self.status.angular_velocity,
                battery_voltage=self.status.battery_voltage,
                battery_percentage=self.status.battery_percentage,
                battery_current=self.status.battery_current,
                gps_fix=self.status.gps_fix,
                gps_satellites=self.status.gps_satellites,
                gps_position=self.status.gps_position,
                altitude_barometric=self.status.altitude_barometric,
                altitude_relative=self.status.altitude_relative,
                home_distance=self.status.home_distance,
                geofence_violation=self.status.geofence_violation,
                low_battery_warning=self.status.low_battery_warning,
                critical_battery_warning=self.status.critical_battery_warning,
                altitude_warning=self.status.altitude_warning,
                last_heartbeat=self.status.last_heartbeat,
                last_update=self.status.last_update
            )
            
    def add_state_callback(self, callback):
        """Ajoute un callback appelé lors des changements d'état"""
        self._state_callbacks.append(callback)
        
    def _notify_state_change(self):
        """Notifie tous les callbacks des changements d'état"""
        for callback in self._state_callbacks:
            try:
                callback(self.status)
            except Exception as e:
                self.logger.error(f"Error in state callback: {e}")


class DroneInterface(Node):
    """
    Nœud principal d'interface pour le contrôle du drone
    
    Ce nœud central fournit l'interface avec MAVROS et les services
    de base pour les autres nœuds spécialisés.
    """
    
    def __init__(self):
        super().__init__('drone_interface')
        
        self.logger = self.get_logger()
        self.logger.info("🚁 Initialisation du DroneInterface central...")
        
        # Configuration QoS pour les communications
        self._setup_qos_profiles()
        
        # Initialisation des modules
        self._initialize_modules()
        
        # Configuration des souscripteurs MAVROS
        self._setup_mavros_subscribers()
        
        # Configuration des publishers personnalisés
        self._setup_custom_publishers()
        
        # Configuration des services
        self._setup_services()
        
        # Timer pour la surveillance continue et publication d'état
        self.monitoring_timer = self.create_timer(0.1, self._monitoring_callback)  # 10Hz
        self.status_pub_timer = self.create_timer(0.05, self._publish_status)     # 20Hz
        
        self.logger.info("✅ DroneInterface central initialisé avec succès!")
        self._print_node_info()
        
    def _setup_qos_profiles(self):
        """Configure les profils QoS pour différents types de communication"""
        # QoS pour les topics critiques (état, sécurité)
        self.qos_critical = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # QoS pour les topics fréquents (position, vitesse)
        self.qos_frequent = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # QoS pour les commandes
        self.qos_commands = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
    def _initialize_modules(self):
        """Initialise tous les modules"""
        self.logger.info("🔧 Initialisation des modules...")
        
        # Modules de base
        self.safety_manager = SafetyManager(self.logger)
        self.state_manager = StateManager(self.logger, self.safety_manager)
        
        # Clients de service MAVROS
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        
        self.logger.info("✅ Modules initialisés")
        
    def _setup_mavros_subscribers(self):
        """Configure les souscripteurs aux topics MAVROS"""
        self.logger.info("📡 Configuration des souscripteurs MAVROS...")
        
        # État du drone
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_manager.update_mavros_state,
            self.qos_critical
        )
        
        # Position locale
        self.position_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.state_manager.update_position,
            self.qos_frequent
        )
        
        # Vélocité
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self.state_manager.update_velocity,
            self.qos_frequent
        )
        
        # État de la batterie
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.state_manager.update_battery,
            self.qos_critical
        )
        
        # GPS
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.state_manager.update_gps,
            self.qos_critical
        )
        
        self.logger.info("✅ Souscripteurs MAVROS configurés")
        
    def _setup_custom_publishers(self):
        """Configure les publishers pour les messages personnalisés"""
        self.logger.info("📤 Configuration des publishers personnalisés...")
        
        # Publisher d'état (utilise des messages standard pour compatibilité)
        self.status_pub = self.create_publisher(
            String,
            '/drone/status',
            self.qos_critical
        )
        
        # Publisher d'alertes de sécurité
        self.safety_alerts_pub = self.create_publisher(
            String,
            '/drone/safety_alerts',
            self.qos_critical
        )
        
        # Publishers pour des données spécifiques (plus facile à utiliser par d'autres nœuds)
        self.armed_status_pub = self.create_publisher(
            Bool,
            '/drone/armed',
            self.qos_critical
        )
        
        self.flight_mode_pub = self.create_publisher(
            String,
            '/drone/flight_mode',
            self.qos_critical
        )
        
        self.battery_pub = self.create_publisher(
            Float64,
            '/drone/battery_percentage',
            self.qos_critical
        )
        
        self.logger.info("✅ Publishers personnalisés configurés")
        
    def _setup_services(self):
        """Configure les services fournis par ce nœud"""
        self.logger.info("🔧 Configuration des services...")
        
        # Service de contrôle principal - utilise Trigger pour simplicité
        self.control_service = self.create_service(
            Trigger,
            '/drone/control',
            self._handle_control_request
        )
        
        # Service de vérification de sécurité
        self.safety_service = self.create_service(
            Trigger,
            '/drone/safety_check',
            self._handle_safety_request
        )
        
        # Service d'armement
        self.arm_service = self.create_service(
            Trigger,
            '/drone/arm',
            self._handle_arm_request
        )
        
        # Service de désarmement
        self.disarm_service = self.create_service(
            Trigger,
            '/drone/disarm',
            self._handle_disarm_request
        )
        
        self.logger.info("✅ Services configurés")
        
    def _handle_control_request(self, request, response):
        """Gère les requêtes de contrôle du drone générique"""
        try:
            # Pour l'instant, utilise ARM par défaut avec Trigger
            success, message = self._handle_arm_command()
            
            response.success = success
            response.message = message
            
        except Exception as e:
            self.logger.error(f"❌ Erreur dans handle_control_request: {e}")
            response.message = f"Internal error: {str(e)}"
            response.success = False
            
        return response
        
    def _handle_arm_request(self, request, response):
        """Gère spécifiquement l'armement"""
        try:
            success, message = self._handle_arm_command()
            response.success = success
            response.message = message
        except Exception as e:
            response.success = False
            response.message = f"Arm error: {str(e)}"
        return response
        
    def _handle_disarm_request(self, request, response):
        """Gère spécifiquement le désarmement"""
        try:
            success, message = self._handle_disarm_command()
            response.success = success
            response.message = message
        except Exception as e:
            response.success = False
            response.message = f"Disarm error: {str(e)}"
        return response
        
    def _handle_safety_request(self, request, response):
        """Gère les requêtes de vérification de sécurité"""
        try:
            status = self.state_manager.get_status()
            
            # Vérifier les conditions d'armement
            can_arm, reason, warnings = self.safety_manager.check_arm_conditions(status)
            
            # Vérifier la sécurité de vol
            is_safe, flight_warnings, flight_errors = self.safety_manager.check_flight_safety(status)
            
            # Préparer la réponse
            safety_summary = f"Can arm: {can_arm}, Safe: {is_safe}, Warnings: {len(warnings + flight_warnings)}, Errors: {len(flight_errors)}"
            
            response.message = safety_summary
            response.success = can_arm and is_safe and len(flight_errors) == 0
            
        except Exception as e:
            self.logger.error(f"❌ Erreur dans handle_safety_request: {e}")
            response.message = f"ERROR:{str(e)}"
            response.success = False
            
        return response
        
    def _handle_arm_command(self) -> Tuple[bool, str]:
        """Traite la commande d'armement"""
        # Vérifications de sécurité
        status = self.state_manager.get_status()
        can_arm, reason, warnings = self.safety_manager.check_arm_conditions(status)
        
        if not can_arm:
            return False, reason
            
        # Vérifier la disponibilité du service
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            return False, "MAVROS arming service not available"
            
        # Envoyer la commande d'armement
        request = CommandBool.Request()
        request.value = True
        
        try:
            future = self.arm_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.logger.info("✅ Drone armé avec succès")
                    return True, "Armed successfully"
                else:
                    return False, f"Arming failed: {response.result}"
            else:
                return False, "Arming request timed out"
                
        except Exception as e:
            return False, f"Arming error: {str(e)}"
            
    def _handle_disarm_command(self) -> Tuple[bool, str]:
        """Traite la commande de désarmement"""
        # Vérifier si on peut désarmer en sécurité
        status = self.state_manager.get_status()
        if status.state == DroneState.FLYING and status.position[2] > 1.0:
            return False, "Cannot disarm while flying (altitude > 1m)"
            
        # Vérifier la disponibilité du service
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            return False, "MAVROS arming service not available"
            
        # Envoyer la commande de désarmement
        request = CommandBool.Request()
        request.value = False
        
        try:
            future = self.arm_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.logger.info("✅ Drone désarmé avec succès")
                    return True, "Disarmed successfully"
                else:
                    return False, f"Disarming failed: {response.result}"
            else:
                return False, "Disarming request timed out"
                
        except Exception as e:
            return False, f"Disarming error: {str(e)}"
            
    def _handle_mode_command(self, mode: str) -> Tuple[bool, str]:
        """Traite la commande de changement de mode"""
        # Vérifier que le mode est valide
        try:
            FlightMode(mode)
        except ValueError:
            self.logger.warn(f"⚠️ Mode {mode} non reconnu, tentative quand même...")
            
        # Vérifier la disponibilité du service
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            return False, "MAVROS mode service not available"
            
        # Envoyer la commande de changement de mode
        request = SetMode.Request()
        request.custom_mode = mode
        
        try:
            future = self.mode_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                response = future.result()
                if response.mode_sent:
                    self.logger.info(f"✅ Mode changé vers {mode}")
                    return True, f"Mode changed to {mode}"
                else:
                    return False, f"Mode change to {mode} failed"
            else:
                return False, "Mode change request timed out"
                
        except Exception as e:
            return False, f"Mode change error: {str(e)}"
            
    def _handle_takeoff_command(self, altitude: float) -> Tuple[bool, str]:
        """Traite la commande de décollage"""
        # Vérifications préalables
        status = self.state_manager.get_status()
        
        if not status.connected:
            return False, "Drone not connected"
            
        if not status.armed:
            return False, "Drone not armed"
            
        # Vérifier la disponibilité du service
        if not self.takeoff_client.wait_for_service(timeout_sec=5.0):
            return False, "MAVROS takeoff service not available"
            
        # Envoyer la commande de décollage
        request = CommandTOL.Request()
        request.altitude = altitude
        
        try:
            future = self.takeoff_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.logger.info(f"✅ Décollage vers {altitude}m commandé")
                    return True, f"Takeoff to {altitude}m commanded"
                else:
                    return False, f"Takeoff failed: {response.result}"
            else:
                return False, "Takeoff request timed out"
                
        except Exception as e:
            return False, f"Takeoff error: {str(e)}"
            
    def _handle_land_command(self) -> Tuple[bool, str]:
        """Traite la commande d'atterrissage"""
        # Vérifier la disponibilité du service
        if not self.land_client.wait_for_service(timeout_sec=5.0):
            return False, "MAVROS land service not available"
            
        # Envoyer la commande d'atterrissage
        request = CommandTOL.Request()
        
        try:
            future = self.land_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.logger.info("✅ Atterrissage commandé")
                    return True, "Landing commanded"
                else:
                    return False, f"Landing failed: {response.result}"
            else:
                return False, "Landing request timed out"
                
        except Exception as e:
            return False, f"Landing error: {str(e)}"
            
    def _handle_emergency_command(self) -> Tuple[bool, str]:
        """Traite la commande d'urgence"""
        self.logger.warn("🚨 PROCÉDURE D'URGENCE ACTIVÉE")
        
        # Essayer RTL d'abord
        success, msg = self._handle_mode_command("RTL")
        if success:
            return True, "Emergency RTL activated"
            
        # Si RTL échoue, essayer LAND
        success, msg = self._handle_mode_command("LAND")
        if success:
            return True, "Emergency LAND activated"
            
        return False, "Emergency procedure failed"
        
    def _publish_status(self):
        """Publie l'état du drone sur les topics personnalisés"""
        status = self.state_manager.get_status()
        
        # Publier l'état général (format JSON simple)
        status_msg = String()
        status_msg.data = f"{status.state.value}:{status.mode}:{status.armed}:{status.connected}"
        self.status_pub.publish(status_msg)
        
        # Publier l'état d'armement
        armed_msg = Bool()
        armed_msg.data = status.armed
        self.armed_status_pub.publish(armed_msg)
        
        # Publier le mode de vol
        mode_msg = String()
        mode_msg.data = status.mode
        self.flight_mode_pub.publish(mode_msg)
        
        # Publier le pourcentage de batterie
        battery_msg = Float64()
        battery_msg.data = status.battery_percentage
        self.battery_pub.publish(battery_msg)
        
        # Publier les alertes de sécurité si nécessaire
        is_safe, warnings, errors = self.safety_manager.check_flight_safety(status)
        if warnings or errors:
            alerts = warnings + errors
            alert_msg = String()
            alert_msg.data = "|".join(alerts)
            self.safety_alerts_pub.publish(alert_msg)
            
    def _monitoring_callback(self):
        """Callback de surveillance continue"""
        status = self.state_manager.get_status()
        
        # Vérification de la connexion
        if not status.connected:
            if hasattr(self, '_last_disconnect_warning'):
                if time.time() - self._last_disconnect_warning > 10.0:  # Avertir toutes les 10s
                    self.logger.warn("⚠️ Drone déconnecté de MAVROS")
                    self._last_disconnect_warning = time.time()
            else:
                self.logger.warn("⚠️ Drone déconnecté de MAVROS")
                self._last_disconnect_warning = time.time()
            return
            
        # Vérifications de sécurité en vol
        if status.armed:
            is_safe, warnings, errors = self.safety_manager.check_flight_safety(status)
            
            # Logger les erreurs critiques
            for error in errors:
                self.logger.error(f"🚨 SÉCURITÉ: {error}")
                
            # Logger les avertissements
            for warning in warnings:
                self.logger.warn(f"⚠️ SÉCURITÉ: {warning}")
                
            # Déclenchement automatique d'urgence si erreurs critiques
            if errors and "Critical battery" in str(errors):
                self.logger.error("🚨 URGENCE AUTOMATIQUE: Batterie critique")
                self._handle_emergency_command()
                
        # Affichage périodique d'informations (toutes les 10 secondes)
        if hasattr(self, '_last_info_time'):
            if time.time() - self._last_info_time > 10.0:
                self._print_periodic_info(status)
                self._last_info_time = time.time()
        else:
            self._last_info_time = time.time()
            
    def _print_periodic_info(self, status: DroneStatusData):
        """Affiche des informations périodiques sur l'état du drone"""
        if status.connected:
            mode_str = f"Mode: {status.mode}"
            arm_str = "ARMÉ" if status.armed else "DÉSARMÉ"
            pos_str = f"Pos: ({status.position[0]:.1f}, {status.position[1]:.1f}, {status.position[2]:.1f})"
            bat_str = f"Bat: {status.battery_voltage:.1f}V ({status.battery_percentage:.1f}%)"
            
            self.logger.info(f"📊 {mode_str} | {arm_str} | {pos_str} | {bat_str}")
            
    def _print_node_info(self):
        """Affiche les informations du nœud"""
        self.logger.info("=" * 60)
        self.logger.info("🚁 DRONE INTERFACE - NŒUD CENTRAL")
        self.logger.info("=" * 60)
        self.logger.info("📡 Services disponibles:")
        self.logger.info("  • /drone/control - Contrôle général")
        self.logger.info("  • /drone/arm - Armement")
        self.logger.info("  • /drone/disarm - Désarmement")
        self.logger.info("  • /drone/safety_check - Vérifications de sécurité")
        self.logger.info("")
        self.logger.info("📤 Topics publiés:")
        self.logger.info("  • /drone/status - État général")
        self.logger.info("  • /drone/armed - État d'armement")
        self.logger.info("  • /drone/flight_mode - Mode de vol")
        self.logger.info("  • /drone/battery_percentage - Batterie")
        self.logger.info("  • /drone/safety_alerts - Alertes de sécurité")
        self.logger.info("")
        self.logger.info("🎮 Utilisation:")
        self.logger.info("  ros2 service call /drone/arm std_srvs/srv/Trigger")
        self.logger.info("  ros2 service call /drone/disarm std_srvs/srv/Trigger")
        self.logger.info("  ros2 service call /drone/safety_check std_srvs/srv/Trigger")
        self.logger.info("=" * 60)


def main(args=None):
    """
    Point d'entrée principal du nœud
    """
    # Initialiser ROS2
    rclpy.init(args=args)
    
    try:
        # Créer le nœud d'interface drone
        node = DroneInterface()
        
        # Exécuter le nœud
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 Arrêt demandé par l'utilisateur")
    except Exception as e:
        print(f"❌ Erreur fatale: {e}")
    finally:
        # Nettoyage
        try:
            if 'node' in locals():
                node.destroy_node()
        except:
            pass
        rclpy.shutdown()
        print("👋 DroneInterface arrêté")


if __name__ == '__main__':
    main()
