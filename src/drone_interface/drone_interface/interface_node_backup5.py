#!/usr/bin/env python3
"""
=============================================================================
DRONE INTERFACE NODE - Central Interface for ArduPilot SITL Control
=============================================================================
Auteur: Adama Komi
Date: 2025-08-29
Version: 2.0.3

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
import json
from enum import Enum
from typing import Optional, Tuple, List, Dict, Any
from dataclasses import dataclass

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.task import Future

# Message types
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, ExtendedState, GPSRAW
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from sensor_msgs.msg import NavSatFix, BatteryState
from std_msgs.msg import Header, Float64, String, Bool
from std_srvs.srv import Trigger


class DroneState(str, Enum):
    """Énumération des états possibles du drone"""
    UNKNOWN = "UNKNOWN"
    DISCONNECTED = "DISCONNECTED"
    CONNECTED = "CONNECTED"
    ARMED = "ARMED"
    FLYING = "FLYING"
    LANDING = "LANDING"
    EMERGENCY = "EMERGENCY"


class FlightMode(str, Enum):
    """Modes de vol ArduPilot supportés"""
    STABILIZE = "STABILIZE"
    ALT_HOLD = "ALT_HOLD"
    AUTO = "AUTO"
    GUIDED = "GUIDED"
    LOITER = "LOITER"
    RTL = "RTL"
    LAND = "LAND"
    POSHOLD = "POSHOLD"


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
    orientation: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)
    angular_velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    battery_voltage: float = 0.0
    battery_percentage: float = 0.0
    battery_current: float = 0.0
    gps_fix: bool = False
    gps_satellites: int = 0
    gps_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
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
        self._safety_checks_enabled = True
        self._min_battery_voltage = 10.5
        self._min_battery_percentage = 20.0
        self._min_gps_satellites = 3  # Réduit pour SITL
        self._max_altitude = 120.0
        self._geofence_radius = 100.0
        self._critical_battery_threshold = 20.0
        self._low_battery_threshold = 30.0
        
    def check_arm_conditions(self, status: DroneStatusData) -> Tuple[bool, str, List[str]]:
        """Vérifie les conditions de sécurité avant l'armement"""
        if not self._safety_checks_enabled:
            return True, "Safety checks disabled", []
            
        warnings = []
        
        if not status.connected:
            return False, "Drone not connected to flight controller", warnings
            
        # Pour SITL, on est moins strict sur le GPS
        # On permet l'armement même avec peu ou pas de satellites en simulation
        if status.gps_satellites < self._min_gps_satellites:
            warnings.append(f"Low GPS satellites ({status.gps_satellites}) - simulation mode")
            
        # Vérification batterie (seulement si des données sont disponibles)
        if status.battery_voltage > 0 and status.battery_voltage < self._min_battery_voltage:
            return False, f"Battery voltage too low ({status.battery_voltage:.1f}V)", warnings
            
        if status.battery_percentage > 0 and status.battery_percentage < self._min_battery_percentage:
            warnings.append(f"Low battery: {status.battery_percentage:.1f}%")
            
        # En simulation, on permet l'armement même sans GPS parfait
        can_arm = status.connected
        reason = "All safety checks passed" if can_arm else "Cannot arm"
            
        return can_arm, reason, warnings
        
    def check_flight_safety(self, status: DroneStatusData) -> Tuple[bool, List[str], List[str]]:
        """Vérifie la sécurité en vol continu"""
        warnings = []
        errors = []
        
        if status.position[2] > self._max_altitude:
            warnings.append(f"High altitude: {status.position[2]:.1f}m")
            
        distance_from_home = math.sqrt(status.position[0]**2 + status.position[1]**2)
        if distance_from_home > self._geofence_radius:
            warnings.append(f"Near geofence: {distance_from_home:.1f}m")
            
        if status.battery_percentage > 0 and status.battery_percentage < self._critical_battery_threshold:
            errors.append(f"Critical battery: {status.battery_percentage:.1f}%")
        elif status.battery_percentage > 0 and status.battery_percentage < self._low_battery_threshold:
            warnings.append(f"Low battery: {status.battery_percentage:.1f}%")
        
        if not status.connected:
            errors.append("Lost connection to flight controller")
            
        if status.armed and status.gps_satellites < 3:
            warnings.append("Weak GPS signal during flight")
            
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
        self._lock = threading.RLock()
        self._home_position = None
        self._gps_raw_satellites = 10  # Valeur par défaut pour SITL
        
    def update_mavros_state(self, msg: State):
        """Met à jour l'état depuis MAVROS State"""
        with self._lock:
            self.status.connected = msg.connected
            self.status.armed = msg.armed
            self.status.guided = msg.guided
            self.status.mode = msg.mode
            self.status.last_heartbeat = time.time()
            self.status.last_update = time.time()
            
            if not msg.connected:
                self.status.state = DroneState.DISCONNECTED
            elif not msg.armed:
                self.status.state = DroneState.CONNECTED
            elif msg.mode in ["LAND", "RTL"] and msg.armed:
                self.status.state = DroneState.LANDING
            elif msg.armed:
                self.status.state = DroneState.FLYING if self.status.position[2] > 0.5 else DroneState.ARMED
                
            self._update_safety_alerts()
            self._notify_state_change()
            
    def update_position(self, msg: PoseStamped):
        """Met à jour la position depuis MAVROS local position"""
        with self._lock:
            self.status.position = (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            )
            
            self.status.orientation = (
                msg.pose.orientation.x,
                msg.pose.orientation.y,
                msg.pose.orientation.z,
                msg.pose.orientation.w
            )
            
            if self._home_position is None and any(abs(coord) > 0.1 for coord in self.status.position):
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
            if msg.voltage > 0:  # Only update if we have valid data
                self.status.battery_voltage = msg.voltage
                self.status.battery_percentage = msg.percentage * 100 if msg.percentage >= 0 else 0
                self.status.battery_current = msg.current if hasattr(msg, 'current') else 0.0
            
    def update_gps(self, msg: NavSatFix):
        """Met à jour l'état GPS depuis NavSatFix"""
        with self._lock:
            # Pour SITL, on simule un fix GPS même si les données sont incomplètes
            self.status.gps_fix = True  # Toujours true pour SITL
            self.status.gps_position = (
                msg.latitude if msg.latitude != 0 else 47.3769,  # Default Paris
                msg.longitude if msg.longitude != 0 else 8.5417,  # Default Zurich
                msg.altitude if msg.altitude != 0 else 408.0  # Default altitude
            )
            
    def update_gps_raw(self, msg: GPSRAW):
        """Met à jour l'état GPS depuis GPSRAW"""
        with self._lock:
            # Pour SITL, on simule des satellites
            if hasattr(msg, 'satellites_visible') and msg.satellites_visible > 0:
                self.status.gps_satellites = msg.satellites_visible
                self._gps_raw_satellites = msg.satellites_visible
            else:
                # Valeur par défaut pour SITL
                self.status.gps_satellites = 10
                self._gps_raw_satellites = 10
            
    def update_altitude(self, barometric: float, relative: float):
        """Met à jour les données d'altitude"""
        with self._lock:
            self.status.altitude_barometric = barometric
            self.status.altitude_relative = relative
            
    def _update_safety_alerts(self):
        """Met à jour les alertes de sécurité"""
        if self.status.battery_percentage > 0:
            self.status.critical_battery_warning = self.status.battery_percentage < 20.0
            self.status.low_battery_warning = self.status.battery_percentage < 30.0
        
        self.status.altitude_warning = self.status.position[2] > 100.0
        self.status.geofence_violation = self.status.home_distance > 100.0
            
    def get_status(self) -> DroneStatusData:
        """Retourne une copie de l'état actuel"""
        with self._lock:
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
    """Nœud principal d'interface pour le contrôle du drone"""
    
    def __init__(self):
        super().__init__('drone_interface')
        
        self.logger = self.get_logger()
        self.logger.info("🚁 Initialisation du DroneInterface central...")
        
        # Variables pour la surveillance
        self._last_disconnect_warning = 0.0
        self._last_info_time = 0.0
        
        # Configuration QoS compatible avec MAVROS
        self._setup_qos_profiles()
        
        # Initialisation des modules
        self._initialize_modules()
        
        # Configuration des souscripteurs MAVROS
        self._setup_mavros_subscribers()
        
        # Configuration des publishers personnalisés
        self._setup_custom_publishers()
        
        # Configuration des services
        self._setup_services()
        
        # Timers
        self.monitoring_timer = self.create_timer(0.5, self._monitoring_callback)
        self.status_pub_timer = self.create_timer(0.2, self._publish_status)
        
        self.logger.info("✅ DroneInterface central initialisé avec succès!")
        self._print_node_info()
        
    def _setup_qos_profiles(self):
        """Configure les profils QoS compatibles avec MAVROS"""
        self.qos_mavros = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
    def _initialize_modules(self):
        """Initialise tous les modules"""
        self.logger.info("🔧 Initialisation des modules...")
        
        self.safety_manager = SafetyManager(self.logger)
        self.state_manager = StateManager(self.logger, self.safety_manager)
        
        # Clients de service MAVROS
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        
        self.logger.info("✅ Modules initialisés")
        
    def _setup_mavros_subscribers(self):
        """Configure les souscripteurs aux topics MAVROS avec QoS compatible"""
        self.logger.info("📡 Configuration des souscripteurs MAVROS...")
        
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_manager.update_mavros_state, self.qos_mavros)
        
        self.position_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.state_manager.update_position, self.qos_mavros)
        
        self.velocity_sub = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local', self.state_manager.update_velocity, self.qos_mavros)
        
        self.battery_sub = self.create_subscription(
            BatteryState, '/mavros/battery', self.state_manager.update_battery, self.qos_mavros)
        
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.state_manager.update_gps, self.qos_mavros)
        
        self.gps_raw_sub = self.create_subscription(
            GPSRAW, '/mavros/gpsstatus/gps1/raw', self.state_manager.update_gps_raw, self.qos_mavros)
        
        self.logger.info("✅ Souscripteurs MAVROS configurés")
        
    def _setup_custom_publishers(self):
        """Configure les publishers pour les messages personnalisés"""
        self.logger.info("📤 Configuration des publishers personnalisés...")
        
        self.status_pub = self.create_publisher(String, '/drone/status', self.qos_reliable)
        self.safety_alerts_pub = self.create_publisher(String, '/drone/safety_alerts', self.qos_reliable)
        self.armed_status_pub = self.create_publisher(Bool, '/drone/armed', self.qos_reliable)
        self.flight_mode_pub = self.create_publisher(String, '/drone/flight_mode', self.qos_reliable)
        self.battery_pub = self.create_publisher(Float64, '/drone/battery_percentage', self.qos_reliable)
        
        self.logger.info("✅ Publishers personnalisés configurés")
        
    def _setup_services(self):
        """Configure les services fournis par ce nœud"""
        self.logger.info("🔧 Configuration des services...")
        
        self.control_service = self.create_service(Trigger, '/drone/control', self._handle_control_request)
        self.safety_service = self.create_service(Trigger, '/drone/safety_check', self._handle_safety_request)
        self.arm_service = self.create_service(Trigger, '/drone/arm', self._handle_arm_request)
        self.disarm_service = self.create_service(Trigger, '/drone/disarm', self._handle_disarm_request)
        
        self.logger.info("✅ Services configurés")
        
    def _handle_control_request(self, request, response):
        """Gère les requêtes de contrôle du drone générique"""
        try:
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
            can_arm, reason, warnings = self.safety_manager.check_arm_conditions(status)
            is_safe, flight_warnings, flight_errors = self.safety_manager.check_flight_safety(status)
            
            safety_summary = f"Can arm: {can_arm}, Safe: {is_safe}, Warnings: {len(warnings)}, Errors: {len(flight_errors)}"
            response.message = safety_summary
            response.success = can_arm and is_safe and len(flight_errors) == 0
            
        except Exception as e:
            self.logger.error(f"❌ Erreur dans handle_safety_request: {e}")
            response.message = f"ERROR:{str(e)}"
            response.success = False
        return response
        
    def _handle_arm_command(self) -> Tuple[bool, str]:
        """Traite la commande d'armement"""
        status = self.state_manager.get_status()
        can_arm, reason, warnings = self.safety_manager.check_arm_conditions(status)
        
        # Afficher les avertissements mais ne pas bloquer l'armement
        for warning in warnings:
            self.logger.warn(f"⚠️ {warning}")
        
        if not can_arm:
            return False, reason
            
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            return False, "MAVROS arming service not available"
            
        request = CommandBool.Request()
        request.value = True
        
        try:
            future = self.arm_client.call_async(request)
            start_time = time.time()
            while not future.done() and time.time() - start_time < 10.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if future.done() and future.result() is not None:
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
        status = self.state_manager.get_status()
        if status.state == DroneState.FLYING and status.position[2] > 1.0:
            return False, "Cannot disarm while flying"
            
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            return False, "MAVROS arming service not available"
            
        request = CommandBool.Request()
        request.value = False
        
        try:
            future = self.arm_client.call_async(request)
            start_time = time.time()
            while not future.done() and time.time() - start_time < 10.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if future.done() and future.result() is not None:
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
            
    def _monitoring_callback(self):
        """Callback pour la surveillance continue"""
        try:
            current_time = time.time()
            status = self.state_manager.get_status()
            
            if not status.connected:
                if current_time - self._last_disconnect_warning > 5.0:
                    self.logger.warn("⚠️ Déconnecté de MAVROS")
                    self._last_disconnect_warning = current_time
                    
            if current_time - self._last_info_time > 10.0:
                self._log_status_info(status)
                self._last_info_time = current_time
                
            is_safe, warnings, errors = self.safety_manager.check_flight_safety(status)
            
            if warnings or errors:
                alert_msg = String()
                alert_data = {
                    'timestamp': current_time,
                    'warnings': warnings,
                    'errors': errors,
                    'is_safe': is_safe
                }
                alert_msg.data = json.dumps(alert_data)
                self.safety_alerts_pub.publish(alert_msg)
                
        except Exception as e:
            self.logger.error(f"❌ Erreur dans monitoring_callback: {e}")
            
    def _publish_status(self):
        """Publie l'état du drone"""
        try:
            status = self.state_manager.get_status()
            
            status_msg = String()
            status_data = {
                'state': status.state.value,
                'mode': status.mode,
                'armed': status.armed,
                'connected': status.connected,
                'position': status.position,
                'battery_percentage': status.battery_percentage,
                'gps_fix': status.gps_fix,
                'gps_satellites': status.gps_satellites,
                'home_distance': status.home_distance,
                'timestamp': time.time()
            }
            status_msg.data = json.dumps(status_data)
            self.status_pub.publish(status_msg)
            
            armed_msg = Bool()
            armed_msg.data = status.armed
            self.armed_status_pub.publish(armed_msg)
            
            mode_msg = String()
            mode_msg.data = status.mode
            self.flight_mode_pub.publish(mode_msg)
            
            battery_msg = Float64()
            battery_msg.data = status.battery_percentage
            self.battery_pub.publish(battery_msg)
            
        except Exception as e:
            self.logger.error(f"❌ Erreur dans publish_status: {e}")
            
    def _log_status_info(self, status: DroneStatusData):
        """Log les informations d'état périodiquement"""
        info_lines = [
            f"📊 État: {status.state.value}, Mode: {status.mode}",
            f"⚡ Batterie: {status.battery_percentage:.1f}% ({status.battery_voltage:.1f}V)",
            f"📍 Position: ({status.position[0]:.1f}, {status.position[1]:.1f}, {status.position[2]:.1f})m",
            f"📡 GPS: {'Fix' if status.gps_fix else 'No fix'}, Satellites: {status.gps_satellites}",
            f"🏠 Distance du home: {status.home_distance:.1f}m"
        ]
        
        for line in info_lines:
            self.logger.info(line)
            
    def _print_node_info(self):
        """Affiche les informations du nœud au démarrage"""
        info = [
            "=" * 60,
            "🚁 DRONE INTERFACE NODE - Central MAVROS Interface",
            "=" * 60,
            "📡 Topics MAVROS souscrits:",
            "   - /mavros/state",
            "   - /mavros/local_position/pose", 
            "   - /mavros/local_position/velocity_local",
            "   - /mavros/battery",
            "   - /mavros/global_position/global",
            "   - /mavros/gpsstatus/gps1/raw",
            "",
            "📤 Topics publiés:",
            "   - /drone/status (JSON)",
            "   - /drone/armed",
            "   - /drone/flight_mode", 
            "   - /drone/battery_percentage",
            "   - /drone/safety_alerts",
            "",
            "🔧 Services disponibles:",
            "   - /drone/control (Trigger)",
            "   - /drone/arm (Trigger)",
            "   - /drone/disarm (Trigger)",
            "   - /drone/safety_check (Trigger)",
            "",
            "⚙️  Mode: SITL SIMULATION (GPS restrictions relaxed)",
            "=" * 60
        ]
        
        for line in info:
            self.logger.info(line)


def main(args=None):
    """Point d'entrée principal du nœud"""
    rclpy.init(args=args)
    
    try:
        node = DroneInterface()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        if 'node' in locals():
            node.logger.info("🛑 Arrêt demandé par l'utilisateur")
    except Exception as e:
        if 'node' in locals():
            node.logger.error(f"💥 Erreur fatale: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()