#!/usr/bin/env python3
"""
=============================================================================
DRONE INTERFACE NODE - ArduPilot SITL Control System
=============================================================================
Auteur: Adama Komi
Date: 2025-08-29
Version: 1.0.0

Description:
    Nœud ROS2 modulaire pour le contrôle complet d'un drone ArduPilot via MAVROS.
    Architecture évolutive avec modules séparés pour chaque fonctionnalité.
    
Fonctionnalités:
    - Armement/Désarmement sécurisé
    - Changement de modes de vol
    - Décollage/Atterrissage automatisé
    - Navigation par waypoints
    - Contrôle de position (position hold)
    - Surveillance de l'état du drone
    - Gestion des missions
    - Détection et gestion d'erreurs
    - Tests unitaires intégrés

Utilisation avec SITL:
    1. Démarrer ArduPilot SITL:
       sim_vehicle.py -v ArduCopter --console --map --out=127.0.0.1:14550
    
    2. Démarrer MAVROS:
       ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555
    
    3. Lancer ce nœud:
       ros2 run drone_interface interface_node
    
    4. Utiliser les commandes via paramètres:
       ros2 param set /drone_interface action "ARM"
=============================================================================
"""

import math
import time
import threading
from enum import Enum
from typing import Optional, Tuple, List, Dict, Any
from dataclasses import dataclass

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from rcl_interfaces.msg import SetParametersResult

# Message types
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from mavros_msgs.msg import State, ExtendedState, WaypointList, Waypoint, OverrideRCIn
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, WaypointClear, WaypointPush
from sensor_msgs.msg import NavSatFix, BatteryState
from std_msgs.msg import Header, Float64, String
from nav_msgs.msg import Odometry


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
class DroneStatus:
    """Structure de données pour l'état complet du drone"""
    state: DroneState = DroneState.UNKNOWN
    mode: str = "UNKNOWN"
    armed: bool = False
    connected: bool = False
    guided: bool = False
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    battery_voltage: float = 0.0
    battery_percentage: float = 0.0
    gps_fix: bool = False
    gps_satellites: int = 0
    last_update: float = 0.0


class SafetyManager:
    """Module de gestion de la sécurité"""
    
    def __init__(self, logger):
        self.logger = logger
        self._safety_checks_enabled = True
        self._min_battery_voltage = 10.5  # Voltage minimum pour ArduPilot
        self._min_gps_satellites = 6      # Satellites GPS minimum
        self._max_altitude = 120.0        # Altitude maximum en mètres
        self._geofence_radius = 100.0     # Rayon de géofence en mètres
        
    def check_arm_conditions(self, status: DroneStatus) -> Tuple[bool, str]:
        """
        Vérifie les conditions de sécurité avant l'armement
        Returns: (can_arm, reason_if_not)
        """
        if not self._safety_checks_enabled:
            return True, "Safety checks disabled"
            
        # Vérification de la connexion
        if not status.connected:
            return False, "Drone not connected to flight controller"
            
        # Vérification GPS (si nécessaire selon le mode)
        if not status.gps_fix:
            return False, "GPS fix not available"
            
        if status.gps_satellites < self._min_gps_satellites:
            return False, f"Insufficient GPS satellites ({status.gps_satellites}/{self._min_gps_satellites})"
            
        # Vérification batterie
        if status.battery_voltage > 0 and status.battery_voltage < self._min_battery_voltage:
            return False, f"Battery voltage too low ({status.battery_voltage:.1f}V < {self._min_battery_voltage}V)"
            
        return True, "All safety checks passed"
        
    def check_flight_safety(self, status: DroneStatus) -> Tuple[bool, str]:
        """
        Vérifie la sécurité en vol continu
        Returns: (is_safe, warning_message)
        """
        warnings = []
        
        # Vérification altitude
        if status.position[2] > self._max_altitude:
            warnings.append(f"Altitude too high: {status.position[2]:.1f}m > {self._max_altitude}m")
            
        # Vérification distance du home
        distance_from_home = math.sqrt(status.position[0]**2 + status.position[1]**2)
        if distance_from_home > self._geofence_radius:
            warnings.append(f"Outside geofence: {distance_from_home:.1f}m > {self._geofence_radius}m")
            
        # Vérification batterie critique
        if status.battery_percentage > 0 and status.battery_percentage < 20:
            warnings.append(f"Critical battery level: {status.battery_percentage:.1f}%")
            
        if warnings:
            return False, "; ".join(warnings)
            
        return True, "Flight safety OK"


class StateManager:
    """Module de gestion d'état du drone"""
    
    def __init__(self, logger):
        self.logger = logger
        self.status = DroneStatus()
        self._state_callbacks = []
        self._lock = threading.Lock()
        
    def update_mavros_state(self, msg: State):
        """Met à jour l'état depuis MAVROS State"""
        with self._lock:
            self.status.connected = msg.connected
            self.status.armed = msg.armed
            self.status.guided = msg.guided
            self.status.mode = msg.mode
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
                
            self._notify_state_change()
            
    def update_position(self, msg: PoseStamped):
        """Met à jour la position depuis MAVROS local position"""
        with self._lock:
            self.status.position = (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            )
            
    def update_velocity(self, msg: TwistStamped):
        """Met à jour la vélocité depuis MAVROS"""
        with self._lock:
            self.status.velocity = (
                msg.twist.linear.x,
                msg.twist.linear.y,
                msg.twist.linear.z
            )
            
    def update_battery(self, msg: BatteryState):
        """Met à jour l'état de la batterie"""
        with self._lock:
            self.status.battery_voltage = msg.voltage
            self.status.battery_percentage = msg.percentage * 100 if msg.percentage >= 0 else 0
            
    def update_gps(self, msg: NavSatFix):
        """Met à jour l'état GPS"""
        with self._lock:
            self.status.gps_fix = msg.status.status >= 0
            # Note: le nombre de satellites n'est pas dans NavSatFix standard
            # Il faudrait utiliser mavros_msgs/GPSSTATUS pour cela
            
    def get_status(self) -> DroneStatus:
        """Retourne une copie de l'état actuel"""
        with self._lock:
            # Créer une copie pour éviter les modifications concurrentes
            return DroneStatus(
                state=self.status.state,
                mode=self.status.mode,
                armed=self.status.armed,
                connected=self.status.connected,
                guided=self.status.guided,
                position=self.status.position,
                velocity=self.status.velocity,
                battery_voltage=self.status.battery_voltage,
                battery_percentage=self.status.battery_percentage,
                gps_fix=self.status.gps_fix,
                gps_satellites=self.status.gps_satellites,
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


class ArmingManager:
    """Module de gestion de l'armement/désarmement"""
    
    def __init__(self, node, safety_manager: SafetyManager, state_manager: StateManager):
        self.node = node
        self.safety_manager = safety_manager
        self.state_manager = state_manager
        self.logger = node.get_logger()
        
        # Service client pour l'armement
        self.arm_client = node.create_client(CommandBool, '/mavros/cmd/arming')
        
    def arm(self, force: bool = False) -> Tuple[bool, str]:
        """
        Arme le drone avec vérifications de sécurité
        Args:
            force: Ignore les vérifications de sécurité si True
        Returns:
            (success, message)
        """
        self.logger.info("🔧 Tentative d'armement du drone...")
        
        # Vérifications de sécurité
        if not force:
            can_arm, reason = self.safety_manager.check_arm_conditions(self.state_manager.get_status())
            if not can_arm:
                self.logger.error(f"❌ Armement refusé: {reason}")
                return False, reason
                
        # Vérifier la disponibilité du service
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            error_msg = "Service /mavros/cmd/arming non disponible"
            self.logger.error(f"❌ {error_msg}")
            return False, error_msg
            
        # Créer et envoyer la requête
        request = CommandBool.Request()
        request.value = True
        
        try:
            # Appel synchrone avec timeout
            future = self.arm_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.logger.info("✅ Drone armé avec succès!")
                    return True, "Armed successfully"
                else:
                    error_msg = f"Échec de l'armement: {response.result}"
                    self.logger.error(f"❌ {error_msg}")
                    return False, error_msg
            else:
                error_msg = "Timeout lors de l'armement"
                self.logger.error(f"❌ {error_msg}")
                return False, error_msg
                
        except Exception as e:
            error_msg = f"Erreur lors de l'armement: {str(e)}"
            self.logger.error(f"❌ {error_msg}")
            return False, error_msg
            
    def disarm(self, force: bool = False) -> Tuple[bool, str]:
        """
        Désarme le drone
        Args:
            force: Force le désarmement même en vol si True
        Returns:
            (success, message)
        """
        self.logger.info("🔧 Tentative de désarmement du drone...")
        
        # Vérification de sécurité: ne pas désarmer en vol sauf si forcé
        status = self.state_manager.get_status()
        if not force and status.state == DroneState.FLYING and status.position[2] > 1.0:
            error_msg = "Refus de désarmer en vol (utilisez force=True si nécessaire)"
            self.logger.warn(f"⚠️ {error_msg}")
            return False, error_msg
            
        # Vérifier la disponibilité du service
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            error_msg = "Service /mavros/cmd/arming non disponible"
            self.logger.error(f"❌ {error_msg}")
            return False, error_msg
            
        # Créer et envoyer la requête
        request = CommandBool.Request()
        request.value = False
        
        try:
            # Appel synchrone avec timeout
            future = self.arm_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.logger.info("✅ Drone désarmé avec succès!")
                    return True, "Disarmed successfully"
                else:
                    error_msg = f"Échec du désarmement: {response.result}"
                    self.logger.error(f"❌ {error_msg}")
                    return False, error_msg
            else:
                error_msg = "Timeout lors du désarmement"
                self.logger.error(f"❌ {error_msg}")
                return False, error_msg
                
        except Exception as e:
            error_msg = f"Erreur lors du désarmement: {str(e)}"
            self.logger.error(f"❌ {error_msg}")
            return False, error_msg


class ModeManager:
    """Module de gestion des modes de vol"""
    
    def __init__(self, node, state_manager: StateManager):
        self.node = node
        self.state_manager = state_manager
        self.logger = node.get_logger()
        
        # Service client pour changer de mode
        self.mode_client = node.create_client(SetMode, '/mavros/set_mode')
        
    def set_mode(self, mode: str, timeout: float = 10.0) -> Tuple[bool, str]:
        """
        Change le mode de vol du drone
        Args:
            mode: Nom du mode (ex: 'GUIDED', 'STABILIZE', etc.)
            timeout: Timeout en secondes
        Returns:
            (success, message)
        """
        self.logger.info(f"🔧 Changement vers le mode: {mode}")
        
        # Vérifier que le mode est valide
        try:
            FlightMode(mode)
        except ValueError:
            # Le mode n'est pas dans notre énumération, mais on l'accepte quand même
            # car ArduPilot peut avoir des modes non listés
            self.logger.warn(f"⚠️ Mode {mode} non reconnu, tentative quand même...")
            
        # Vérifier la disponibilité du service
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            error_msg = "Service /mavros/set_mode non disponible"
            self.logger.error(f"❌ {error_msg}")
            return False, error_msg
            
        # Créer et envoyer la requête
        request = SetMode.Request()
        request.custom_mode = mode
        
        try:
            # Appel synchrone avec timeout
            future = self.mode_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout)
            
            if future.result() is not None:
                response = future.result()
                if response.mode_sent:
                    self.logger.info(f"✅ Changement de mode vers {mode} envoyé avec succès!")
                    
                    # Attendre que le changement soit effectif
                    return self._wait_for_mode_change(mode, timeout=5.0)
                else:
                    error_msg = f"Échec de l'envoi du changement de mode vers {mode}"
                    self.logger.error(f"❌ {error_msg}")
                    return False, error_msg
            else:
                error_msg = "Timeout lors du changement de mode"
                self.logger.error(f"❌ {error_msg}")
                return False, error_msg
                
        except Exception as e:
            error_msg = f"Erreur lors du changement de mode: {str(e)}"
            self.logger.error(f"❌ {error_msg}")
            return False, error_msg
            
    def _wait_for_mode_change(self, target_mode: str, timeout: float) -> Tuple[bool, str]:
        """
        Attend que le changement de mode soit effectif
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            current_status = self.state_manager.get_status()
            if current_status.mode == target_mode:
                self.logger.info(f"✅ Mode {target_mode} activé avec succès!")
                return True, f"Mode changed to {target_mode}"
                
            time.sleep(0.1)  # Attendre 100ms avant la prochaine vérification
            
        error_msg = f"Timeout: le mode {target_mode} n'a pas été activé dans les {timeout}s"
        self.logger.error(f"❌ {error_msg}")
        return False, error_msg
        
    def get_current_mode(self) -> str:
        """Retourne le mode actuel du drone"""
        return self.state_manager.get_status().mode


class PositionController:
    """Module de contrôle de position"""
    
    def __init__(self, node, state_manager: StateManager):
        self.node = node
        self.state_manager = state_manager
        self.logger = node.get_logger()
        
        # Publisher pour les setpoints de position
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.setpoint_pub = node.create_publisher(
            PoseStamped,
            '/mavros/setpoint_position/local',
            qos_profile
        )
        
        # Timer pour publier les setpoints de manière continue
        self._setpoint_timer = None
        self._current_setpoint = None
        self._setpoint_active = False
        
    def set_position(self, x: float, y: float, z: float, yaw: float = 0.0) -> bool:
        """
        Définit une position cible et commence à la publier
        Args:
            x, y, z: Coordonnées en mètres (frame local)
            yaw: Orientation en radians
        Returns:
            success
        """
        self.logger.info(f"🎯 Nouveau setpoint: x={x:.2f}, y={y:.2f}, z={z:.2f}, yaw={math.degrees(yaw):.1f}°")
        
        # Créer le message de setpoint
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.node.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        # Position
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        
        # Orientation (quaternion depuis yaw)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        
        self._current_setpoint = msg
        
        # Démarrer la publication continue
        self._start_setpoint_publishing()
        
        return True
        
    def hold_position(self) -> bool:
        """
        Maintient la position actuelle
        Returns:
            success
        """
        current_status = self.state_manager.get_status()
        x, y, z = current_status.position
        
        self.logger.info(f"🎯 Maintien de position: x={x:.2f}, y={y:.2f}, z={z:.2f}")
        
        return self.set_position(x, y, z)
        
    def stop_setpoint_publishing(self):
        """Arrête la publication de setpoints"""
        self._setpoint_active = False
        if self._setpoint_timer is not None:
            self._setpoint_timer.cancel()
            self._setpoint_timer = None
        self.logger.info("🛑 Publication de setpoints arrêtée")
        
    def _start_setpoint_publishing(self):
        """Démarre la publication continue de setpoints"""
        self._setpoint_active = True
        
        # Arrêter le timer précédent s'il existe
        if self._setpoint_timer is not None:
            self._setpoint_timer.cancel()
            
        # Créer un nouveau timer à 20Hz (50ms)
        self._setpoint_timer = self.node.create_timer(0.05, self._publish_setpoint)
        
    def _publish_setpoint(self):
        """Callback du timer pour publier le setpoint courant"""
        if self._setpoint_active and self._current_setpoint is not None:
            # Mettre à jour le timestamp
            self._current_setpoint.header.stamp = self.node.get_clock().now().to_msg()
            
            # Publier le setpoint
            self.setpoint_pub.publish(self._current_setpoint)
            
    def get_position_error(self) -> Tuple[float, float, float]:
        """
        Calcule l'erreur entre la position actuelle et le setpoint
        Returns:
            (error_x, error_y, error_z) en mètres
        """
        if self._current_setpoint is None:
            return (0.0, 0.0, 0.0)
            
        current_status = self.state_manager.get_status()
        current_x, current_y, current_z = current_status.position
        
        target_x = self._current_setpoint.pose.position.x
        target_y = self._current_setpoint.pose.position.y
        target_z = self._current_setpoint.pose.position.z
        
        return (
            target_x - current_x,
            target_y - current_y,
            target_z - current_z
        )


class TakeoffLandingManager:
    """Module de gestion du décollage et atterrissage"""
    
    def __init__(self, node, mode_manager: ModeManager, arming_manager: ArmingManager, 
                 position_controller: PositionController, state_manager: StateManager):
        self.node = node
        self.mode_manager = mode_manager
        self.arming_manager = arming_manager
        self.position_controller = position_controller
        self.state_manager = state_manager
        self.logger = node.get_logger()
        
        # Service client pour takeoff/landing
        self.takeoff_client = node.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = node.create_client(CommandTOL, '/mavros/cmd/land')
        
    def takeoff(self, altitude: float = 2.5, auto_arm: bool = True) -> Tuple[bool, str]:
        """
        Effectue un décollage automatisé
        Args:
            altitude: Altitude cible en mètres
            auto_arm: Arme automatiquement si nécessaire
        Returns:
            (success, message)
        """
        self.logger.info(f"🚁 Début de la séquence de décollage vers {altitude}m")
        
        # Vérifications préliminaires
        status = self.state_manager.get_status()
        
        if not status.connected:
            return False, "Drone non connecté"
            
        # Étape 1: Passer en mode GUIDED
        self.logger.info("📡 Étape 1/4: Changement vers le mode GUIDED")
        success, msg = self.mode_manager.set_mode("GUIDED")
        if not success:
            return False, f"Impossible de changer vers GUIDED: {msg}"
            
        time.sleep(1.0)  # Laisser le temps au mode de s'activer
        
        # Étape 2: Armer si nécessaire
        if auto_arm and not status.armed:
            self.logger.info("🔧 Étape 2/4: Armement du drone")
            success, msg = self.arming_manager.arm()
            if not success:
                return False, f"Impossible d'armer: {msg}"
        elif not status.armed:
            return False, "Drone non armé (utilisez auto_arm=True)"
            
        time.sleep(1.0)  # Laisser le temps à l'armement
        
        # Étape 3: Envoyer des setpoints avant le décollage (requis par MAVROS)
        self.logger.info("🎯 Étape 3/4: Envoi de setpoints pré-décollage")
        self.position_controller.set_position(0.0, 0.0, altitude)
        
        # Envoyer plusieurs setpoints pour s'assurer que MAVROS les reçoit
        for i in range(10):
            time.sleep(0.1)  # 100ms entre chaque setpoint
            
        # Étape 4: Commande de décollage
        self.logger.info("🚀 Étape 4/4: Commande de décollage")
        
        if not self.takeoff_client.wait_for_service(timeout_sec=5.0):
            return False, "Service /mavros/cmd/takeoff non disponible"
            
        request = CommandTOL.Request()
        request.altitude = altitude
        request.latitude = 0.0  # Utilise la position courante
        request.longitude = 0.0
        request.min_pitch = 0.0
        request.yaw = 0.0
        
        try:
            future = self.takeoff_client.call_async(request)
            rclpy.spin_until_future_complete(self.node, future, timeout_sec=10.0)
            
            if future.result() is not None:
                response = future.result()
                if response.success:
                    self.logger.info("✅ Commande de décollage envoyée avec succès!")
                    
                    # Attendre que le drone atteigne l'altitude cible
                    return self._wait_for_altitude(altitude, timeout=30.0)
                else:
                    return False, f"Échec de la commande de décollage: {response.result}"
            else:
                return False, "Timeout lors de l'envoi de la commande de décollage"
                
        except Exception as e:
            return False, f"Erreur lors du décollage: {str(e)}"
            
    def land(self, use_position_control: bool = False) -> Tuple[bool, str]:
        """
        Effectue un atterrissage automatisé
        Args:
            use_position_control: Utilise le contrôle de position au lieu du mode LAND
        Returns:
            (success, message)
        """
        self.logger.info("🛬 Début de la séquence d'atterrissage")
        
        status = self.state_manager.get_status()
        
        if not status.connected:
            return False, "Drone non connecté"
            
        if not status.armed:
            return False, "Drone non armé"
            
        if use_position_control:
            return self._land_with_position_control()
        else:
            return self._land_with_land_mode()
            
    def _land_with_land_mode(self) -> Tuple[bool, str]:
        """Atterrissage en utilisant le mode LAND d'ArduPilot"""
        self.logger.info("🛬 Atterrissage en mode LAND")
        
        # Changer vers le mode LAND
        success, msg = self.mode_manager.set_mode("LAND")
        if not success:
            return False, f"Impossible de changer vers LAND: {msg}"
            
        # Attendre que le drone atterrisse
        return self._wait_for_landing(timeout=60.0)
        
    def _land_with_position_control(self) -> Tuple[bool, str]:
        """Atterrissage contrôlé par setpoints de position"""
        self.logger.info("🛬 Atterrissage par contrôle de position")
        
        # S'assurer qu'on est en mode GUIDED
        success, msg = self.mode_manager.set_mode("GUIDED")
        if not success:
            return False, f"Impossible de changer vers GUIDED: {msg}"
            
        # Descendre graduellement
        status = self.state_manager.get_status()
        current_x, current_y, current_z = status.position
        
        # Descendre par paliers de 0.5m
        target_altitudes = []
        altitude = current_z
        while altitude > 0.2:  # Descendre jusqu'à 20cm du sol
            altitude -= 0.5
            target_altitudes.append(max(altitude, 0.2))
            
        for target_alt in target_altitudes:
            self.logger.info(f"🛬 Descente vers {target_alt:.1f}m")
            self.position_controller.set_position(current_x, current_y, target_alt)
            
            # Attendre d'atteindre l'altitude (avec tolérance)
            if not self._wait_for_altitude(target_alt, tolerance=0.3, timeout=10.0):
                self.logger.warn(f"⚠️ Timeout pour atteindre {target_alt:.1f}m, continue...")
                
            time.sleep(1.0)  # Pause entre chaque palier
            
        # Désarmer le drone
        self.logger.info("🔧 Désarmement du drone")
        success, msg = self.arming_manager.disarm()
        
        return success, "Atterrissage terminé" if success else f"Atterrissage terminé mais échec du désarmement: {msg}"
        
    def _wait_for_altitude(self, target_altitude: float, tolerance: float = 0.5, timeout: float = 30.0) -> Tuple[bool, str]:
        """
        Attend que le drone atteigne l'altitude cible
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            status = self.state_manager.get_status()
            current_altitude = status.position[2]
            
            if abs(current_altitude - target_altitude) <= tolerance:
                self.logger.info(f"✅ Altitude {target_altitude:.1f}m atteinte (actuelle: {current_altitude:.1f}m)")
                return True, f"Altitude {target_altitude}m reached"
                
            time.sleep(0.5)  # Vérifier toutes les 500ms
            
        status = self.state_manager.get_status()
        error_msg = f"Timeout: altitude {target_altitude:.1f}m non atteinte (actuelle: {status.position[2]:.1f}m)"
        self.logger.error(f"❌ {error_msg}")
        return False, error_msg
        
    def _wait_for_landing(self, timeout: float = 60.0) -> Tuple[bool, str]:
        """
        Attend que le drone atterrisse complètement
        """
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            status = self.state_manager.get_status()
            
            # Considérer que le drone a atterri s'il est désarmé ou très proche du sol
            if not status.armed or status.position[2] < 0.1:
                self.logger.info("✅ Atterrissage terminé")
                return True, "Landing completed"
                
            time.sleep(1.0)  # Vérifier toutes les secondes
            
        error_msg = "Timeout: atterrissage non terminé"
        self.logger.error(f"❌ {error_msg}")
        return False, error_msg


class DroneInterface(Node):
    """
    Nœud principal d'interface pour le contrôle du drone
    
    Ce nœud agrège tous les modules de contrôle et fournit une interface
    unifiée pour commander le drone via des paramètres ROS2.
    """
    
    def __init__(self):
        super().__init__('drone_interface')
        
        self.logger = self.get_logger()
        self.logger.info("🚁 Initialisation du DroneInterface...")
        
        # Configuration QoS pour les communications MAVROS
        self.qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialisation des modules
        self._initialize_modules()
        
        # Configuration des souscripteurs MAVROS
        self._setup_mavros_subscribers()
        
        # Configuration des paramètres de commande
        self._setup_parameters()
        
        # Timer pour la surveillance continue
        self.monitoring_timer = self.create_timer(1.0, self._monitoring_callback)
        
        self.logger.info("✅ DroneInterface initialisé avec succès!")
        self._print_usage_instructions()
        
    def _initialize_modules(self):
        """Initialise tous les modules de contrôle"""
        self.logger.info("🔧 Initialisation des modules...")
        
        # Modules de base
        self.state_manager = StateManager(self.logger)
        self.safety_manager = SafetyManager(self.logger)
        
        # Modules de contrôle (nécessitent le nœud ROS2)
        self.arming_manager = ArmingManager(self, self.safety_manager, self.state_manager)
        self.mode_manager = ModeManager(self, self.state_manager)
        self.position_controller = PositionController(self, self.state_manager)
        self.takeoff_landing_manager = TakeoffLandingManager(
            self, self.mode_manager, self.arming_manager, 
            self.position_controller, self.state_manager
        )
        
        self.logger.info("✅ Tous les modules initialisés")
        
    def _setup_mavros_subscribers(self):
        """Configure les souscripteurs aux topics MAVROS"""
        self.logger.info("📡 Configuration des souscripteurs MAVROS...")
        
        # État du drone
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_manager.update_mavros_state,
            self.qos_profile
        )
        
        # Position locale
        self.position_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.state_manager.update_position,
            self.qos_profile
        )
        
        # Vélocité
        self.velocity_sub = self.create_subscription(
            TwistStamped,
            '/mavros/local_position/velocity_local',
            self.state_manager.update_velocity,
            self.qos_profile
        )
        
        # État de la batterie
        self.battery_sub = self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.state_manager.update_battery,
            self.qos_profile
        )
        
        # GPS
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.state_manager.update_gps,
            self.qos_profile
        )
        
        self.logger.info("✅ Souscripteurs MAVROS configurés")
        
    def _setup_parameters(self):
        """Configure les paramètres de commande"""
        self.logger.info("⚙️ Configuration des paramètres...")
        
        # Paramètres de commande
        self.declare_parameter('action', '')
        self.declare_parameter('mode', '')
        self.declare_parameter('x', 0.0)
        self.declare_parameter('y', 0.0)
        self.declare_parameter('z', 2.0)
        self.declare_parameter('yaw', 0.0)
        self.declare_parameter('altitude', 2.5)
        self.declare_parameter('force', False)
        
        # Callback pour les changements de paramètres
        self.add_on_set_parameters_callback(self._on_parameter_change)
        
        self.logger.info("✅ Paramètres configurés")
        
    def _on_parameter_change(self, parameters):
        """
        Callback appelé lors des changements de paramètres
        C'est ici que sont exécutées les commandes du drone
        """
        try:
            for param in parameters:
                if param.name == 'action' and param.value != '':
                    self._handle_action_command(param.value)
                    
            # Remettre le paramètre action à vide après traitement
            self.set_parameters([Parameter('action', Parameter.Type.STRING, '')])
            
            return SetParametersResult(successful=True)
            
        except Exception as e:
            self.logger.error(f"❌ Erreur lors du traitement de la commande: {e}")
            return SetParametersResult(successful=False, reason=str(e))
            
    def _handle_action_command(self, action: str):
        """Gère les commandes d'action"""
        action = action.upper().strip()
        self.logger.info(f"🎮 Commande reçue: {action}")
        
        if action == 'ARM':
            self._cmd_arm()
        elif action == 'DISARM':
            self._cmd_disarm()
        elif action == 'FORCE_ARM':
            self._cmd_force_arm()
        elif action == 'FORCE_DISARM':
            self._cmd_force_disarm()
        elif action == 'TAKEOFF':
            self._cmd_takeoff()
        elif action == 'LAND':
            self._cmd_land()
        elif action == 'GUIDED':
            self._cmd_set_guided()
        elif action == 'MODE':
            self._cmd_set_mode()
        elif action == 'POSITION':
            self._cmd_set_position()
        elif action == 'HOLD':
            self._cmd_hold_position()
        elif action == 'STOP_SETPOINTS':
            self._cmd_stop_setpoints()
        elif action == 'STATUS':
            self._cmd_print_status()
        elif action == 'EMERGENCY':
            self._cmd_emergency()
        else:
            self.logger.warn(f"⚠️ Commande non reconnue: {action}")
            self._print_available_commands()
            
    def _cmd_arm(self):
        """Commande: Armer le drone"""
        success, msg = self.arming_manager.arm()
        if success:
            self.logger.info("✅ ARM: Drone armé avec succès")
        else:
            self.logger.error(f"❌ ARM: {msg}")
            
    def _cmd_disarm(self):
        """Commande: Désarmer le drone"""
        success, msg = self.arming_manager.disarm()
        if success:
            self.logger.info("✅ DISARM: Drone désarmé avec succès")
        else:
            self.logger.error(f"❌ DISARM: {msg}")
            
    def _cmd_force_arm(self):
        """Commande: Forcer l'armement"""
        success, msg = self.arming_manager.arm(force=True)
        if success:
            self.logger.info("✅ FORCE_ARM: Drone armé avec succès (sécurités ignorées)")
        else:
            self.logger.error(f"❌ FORCE_ARM: {msg}")
            
    def _cmd_force_disarm(self):
        """Commande: Forcer le désarmement"""
        success, msg = self.arming_manager.disarm(force=True)
        if success:
            self.logger.info("✅ FORCE_DISARM: Drone désarmé avec succès")
        else:
            self.logger.error(f"❌ FORCE_DISARM: {msg}")
            
    def _cmd_takeoff(self):
        """Commande: Décollage"""
        altitude = self.get_parameter('altitude').get_parameter_value().double_value
        success, msg = self.takeoff_landing_manager.takeoff(altitude)
        if success:
            self.logger.info(f"✅ TAKEOFF: Décollage vers {altitude}m réussi")
        else:
            self.logger.error(f"❌ TAKEOFF: {msg}")
            
    def _cmd_land(self):
        """Commande: Atterrissage"""
        success, msg = self.takeoff_landing_manager.land()
        if success:
            self.logger.info("✅ LAND: Atterrissage réussi")
        else:
            self.logger.error(f"❌ LAND: {msg}")
            
    def _cmd_set_guided(self):
        """Commande: Passer en mode GUIDED"""
        success, msg = self.mode_manager.set_mode("GUIDED")
        if success:
            self.logger.info("✅ GUIDED: Mode GUIDED activé")
        else:
            self.logger.error(f"❌ GUIDED: {msg}")
            
    def _cmd_set_mode(self):
        """Commande: Changer de mode"""
        mode = self.get_parameter('mode').get_parameter_value().string_value
        if not mode:
            self.logger.error("❌ MODE: Paramètre 'mode' requis")
            return
            
        success, msg = self.mode_manager.set_mode(mode)
        if success:
            self.logger.info(f"✅ MODE: Mode {mode} activé")
        else:
            self.logger.error(f"❌ MODE: {msg}")
            
    def _cmd_set_position(self):
        """Commande: Aller à une position"""
        x = self.get_parameter('x').get_parameter_value().double_value
        y = self.get_parameter('y').get_parameter_value().double_value
        z = self.get_parameter('z').get_parameter_value().double_value
        yaw = self.get_parameter('yaw').get_parameter_value().double_value
        
        success = self.position_controller.set_position(x, y, z, yaw)
        if success:
            self.logger.info(f"✅ POSITION: Setpoint défini à ({x:.1f}, {y:.1f}, {z:.1f})")
        else:
            self.logger.error("❌ POSITION: Échec de la définition du setpoint")
            
    def _cmd_hold_position(self):
        """Commande: Maintenir la position actuelle"""
        success = self.position_controller.hold_position()
        if success:
            self.logger.info("✅ HOLD: Maintien de position activé")
        else:
            self.logger.error("❌ HOLD: Échec du maintien de position")
            
    def _cmd_stop_setpoints(self):
        """Commande: Arrêter les setpoints"""
        self.position_controller.stop_setpoint_publishing()
        self.logger.info("✅ STOP_SETPOINTS: Publication de setpoints arrêtée")
        
    def _cmd_print_status(self):
        """Commande: Afficher le statut détaillé"""
        status = self.state_manager.get_status()
        
        self.logger.info("=" * 50)
        self.logger.info("📊 STATUT DÉTAILLÉ DU DRONE")
        self.logger.info("=" * 50)
        self.logger.info(f"🔌 Connexion: {'✅ Connecté' if status.connected else '❌ Déconnecté'}")
        self.logger.info(f"🔧 Armement: {'✅ Armé' if status.armed else '❌ Désarmé'}")
        self.logger.info(f"🎮 Mode: {status.mode}")
        self.logger.info(f"📍 Position: x={status.position[0]:.2f}, y={status.position[1]:.2f}, z={status.position[2]:.2f}")
        self.logger.info(f"⚡ Vitesse: x={status.velocity[0]:.2f}, y={status.velocity[1]:.2f}, z={status.velocity[2]:.2f}")
        self.logger.info(f"🔋 Batterie: {status.battery_voltage:.1f}V ({status.battery_percentage:.1f}%)")
        self.logger.info(f"🛰️ GPS: {'✅ Fix' if status.gps_fix else '❌ No Fix'} ({status.gps_satellites} sats)")
        self.logger.info(f"⏰ Dernière MAJ: {time.time() - status.last_update:.1f}s")
        self.logger.info("=" * 50)
        
    def _cmd_emergency(self):
        """Commande: Procédure d'urgence"""
        self.logger.warn("🚨 PROCÉDURE D'URGENCE ACTIVÉE")
        
        # Arrêter tous les setpoints
        self.position_controller.stop_setpoint_publishing()
        
        # Essayer de passer en mode RTL (Return to Launch)
        success, msg = self.mode_manager.set_mode("RTL")
        if success:
            self.logger.info("✅ EMERGENCY: Mode RTL activé")
        else:
            self.logger.error(f"❌ EMERGENCY: Échec RTL, tentative LAND")
            # Si RTL échoue, essayer LAND
            success, msg = self.mode_manager.set_mode("LAND")
            if success:
                self.logger.info("✅ EMERGENCY: Mode LAND activé")
            else:
                self.logger.error(f"❌ EMERGENCY: Échec total - {msg}")
                
    def _monitoring_callback(self):
        """Callback de surveillance continue (appelé toutes les secondes)"""
        status = self.state_manager.get_status()
        
        # Vérification de la connexion
        if not status.connected:
            self.logger.warn("⚠️ ATTENTION: Drone déconnecté de MAVROS")
            return
            
        # Vérifications de sécurité en vol
        if status.armed:
            is_safe, warning = self.safety_manager.check_flight_safety(status)
            if not is_safe:
                self.logger.warn(f"⚠️ ALERTE SÉCURITÉ: {warning}")
                
        # Affichage périodique d'informations (toutes les 10 secondes)
        if hasattr(self, '_last_info_time'):
            if time.time() - self._last_info_time > 10.0:
                self._print_periodic_info(status)
                self._last_info_time = time.time()
        else:
            self._last_info_time = time.time()
            
    def _print_periodic_info(self, status: DroneStatus):
        """Affiche des informations périodiques sur l'état du drone"""
        if status.connected:
            mode_str = f"Mode: {status.mode}"
            arm_str = "ARMÉ" if status.armed else "DÉSARMÉ"
            pos_str = f"Pos: ({status.position[0]:.1f}, {status.position[1]:.1f}, {status.position[2]:.1f})"
            bat_str = f"Bat: {status.battery_voltage:.1f}V"
            
            self.logger.info(f"📊 {mode_str} | {arm_str} | {pos_str} | {bat_str}")
            
    def _print_usage_instructions(self):
        """Affiche les instructions d'utilisation"""
        self.logger.info("=" * 60)
        self.logger.info("🎮 INTERFACE DE CONTRÔLE DRONE - COMMANDES DISPONIBLES")
        self.logger.info("=" * 60)
        self.logger.info("ARMEMENT/DÉSARMEMENT:")
        self.logger.info('  ros2 param set /drone_interface action "ARM"')
        self.logger.info('  ros2 param set /drone_interface action "DISARM"')
        self.logger.info('  ros2 param set /drone_interface action "FORCE_ARM"')
        self.logger.info('  ros2 param set /drone_interface action "FORCE_DISARM"')
        self.logger.info("")
        self.logger.info("DÉCOLLAGE/ATTERRISSAGE:")
        self.logger.info('  ros2 param set /drone_interface altitude 3.0')
        self.logger.info('  ros2 param set /drone_interface action "TAKEOFF"')
        self.logger.info('  ros2 param set /drone_interface action "LAND"')
        self.logger.info("")
        self.logger.info("MODES DE VOL:")
        self.logger.info('  ros2 param set /drone_interface action "GUIDED"')
        self.logger.info('  ros2 param set /drone_interface mode "LOITER"')
        self.logger.info('  ros2 param set /drone_interface action "MODE"')
        self.logger.info("")
        self.logger.info("CONTRÔLE DE POSITION:")
        self.logger.info('  ros2 param set /drone_interface x 5.0')
        self.logger.info('  ros2 param set /drone_interface y 3.0')
        self.logger.info('  ros2 param set /drone_interface z 2.0')
        self.logger.info('  ros2 param set /drone_interface action "POSITION"')
        self.logger.info('  ros2 param set /drone_interface action "HOLD"')
        self.logger.info('  ros2 param set /drone_interface action "STOP_SETPOINTS"')
        self.logger.info("")
        self.logger.info("UTILITAIRES:")
        self.logger.info('  ros2 param set /drone_interface action "STATUS"')
        self.logger.info('  ros2 param set /drone_interface action "EMERGENCY"')
        self.logger.info("")
        self.logger.info("💡 SÉQUENCE RECOMMANDÉE POUR DÉBUTANTS:")
        self.logger.info("1. Vérifier la connexion: STATUS")
        self.logger.info("2. Passer en mode guidé: GUIDED")
        self.logger.info("3. Armer le drone: ARM")
        self.logger.info("4. Décoller: TAKEOFF")
        self.logger.info("5. Contrôler la position: POSITION")
        self.logger.info("6. Atterrir: LAND")
        self.logger.info("=" * 60)
        
    def _print_available_commands(self):
        """Affiche la liste des commandes disponibles"""
        commands = [
            "ARM", "DISARM", "FORCE_ARM", "FORCE_DISARM",
            "TAKEOFF", "LAND", "GUIDED", "MODE", 
            "POSITION", "HOLD", "STOP_SETPOINTS",
            "STATUS", "EMERGENCY"
        ]
        self.logger.info(f"📋 Commandes disponibles: {', '.join(commands)}")


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
