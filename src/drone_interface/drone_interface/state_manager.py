#!/usr/bin/env python3
"""
=============================================================================
STATE MANAGER - Gestionnaire d'état pour drone autonome
=============================================================================
Auteur: Adama Komi
Date: 2025-09-03
Version: 3.0.0

Description:
    Gestionnaire d'état robuste avec validation pour le drone.
    Maintient l'état complet du système et fournit des validations.
=============================================================================
"""

import math
import time
import threading
from enum import IntEnum
from typing import Optional, Tuple, List
from dataclasses import dataclass, field
from builtin_interfaces.msg import Time

# ROS2 message types
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix, BatteryState


class DroneState(IntEnum):
    """États principaux du drone avec priorité"""
    UNKNOWN = 0
    DISCONNECTED = 10
    CONNECTED = 20
    INITIALIZING = 25
    READY = 30
    ARMED = 40
    TAKING_OFF = 45
    FLYING = 50
    LANDING = 55
    EMERGENCY = 100


class ConnectionQuality(IntEnum):
    """Qualité de connexion"""
    EXCELLENT = 4
    GOOD = 3
    FAIR = 2
    POOR = 1
    DISCONNECTED = 0


@dataclass
class DroneStatusData:
    """Structure complète de l'état du drone"""
    timestamp: Time = field(default_factory=lambda: Time())
    state: DroneState = DroneState.UNKNOWN
    mode: str = "UNKNOWN"
    armed: bool = False
    connected: bool = False
    guided: bool = False
    
    # Position et attitude
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    velocity: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    attitude: Tuple[float, float, float, float] = (0.0, 0.0, 0.0, 1.0)  # quaternion
    
    # Capteurs
    battery_voltage: float = 0.0
    battery_percentage: float = 0.0
    battery_current: float = 0.0
    gps_fix: bool = False
    gps_satellites: int = 0
    gps_hdop: float = 99.99
    
    # Navigation
    home_distance: float = 0.0
    ground_speed: float = 0.0
    air_speed: float = 0.0
    climb_rate: float = 0.0
    
    # Connexion
    last_heartbeat: float = 0.0
    connection_quality: ConnectionQuality = ConnectionQuality.DISCONNECTED
    mavlink_version: int = 0
    
    # Sécurité
    safety_level: int = 0  # Évite import circulaire
    safety_messages: List[str] = field(default_factory=list)
    
    # Performance
    cpu_usage: float = 0.0
    memory_usage: float = 0.0


class StateManager:
    """Gestionnaire d'état robuste avec validation"""
    
    def __init__(self, logger, safety_manager=None):
        self.logger = logger
        self.safety_manager = safety_manager
        self.status = DroneStatusData()
        self._lock = threading.RLock()
        self._home_position: Optional[Tuple[float, float, float]] = None
        self._state_history = []
        self._last_mode_change = 0.0
        
    def update_mavros_state(self, msg: State):
        """Met à jour l'état MAVROS avec validation"""
        with self._lock:
            try:
                self.status.connected = msg.connected
                self.status.armed = msg.armed
                self.status.guided = msg.guided
                self.status.mode = msg.mode
                self.status.last_heartbeat = time.time()
                
                # Mise à jour de l'état principal
                if not msg.connected:
                    self.status.state = DroneState.DISCONNECTED
                elif msg.armed:
                    self.status.state = DroneState.ARMED
                else:
                    self.status.state = DroneState.CONNECTED
                    
                self.logger.debug(f"State updated: {msg.mode}, armed={msg.armed}, connected={msg.connected}")
                    
            except Exception as e:
                self.logger.error(f"Error updating MAVROS state: {e}")
                
    def update_position(self, msg: PoseStamped):
        """Met à jour la position avec validation"""
        with self._lock:
            try:
                if self._validate_pose_message(msg):
                    self.status.position = (
                        msg.pose.position.x,
                        msg.pose.position.y,
                        msg.pose.position.z
                    )
                    self.status.attitude = (
                        msg.pose.orientation.x,
                        msg.pose.orientation.y,
                        msg.pose.orientation.z,
                        msg.pose.orientation.w
                    )
                    
                    # Calcul distance home si définie
                    if self._home_position:
                        dx = self.status.position[0] - self._home_position[0]
                        dy = self.status.position[1] - self._home_position[1]
                        self.status.home_distance = math.sqrt(dx*dx + dy*dy)
                    
            except Exception as e:
                self.logger.error(f"Error updating position: {e}")
                
    def update_velocity(self, msg: TwistStamped):
        """Met à jour la vitesse"""
        with self._lock:
            try:
                self.status.velocity = (
                    msg.twist.linear.x,
                    msg.twist.linear.y,
                    msg.twist.linear.z
                )
                
                # Calcul vitesses dérivées
                self.status.ground_speed = math.sqrt(
                    msg.twist.linear.x**2 + msg.twist.linear.y**2
                )
                self.status.climb_rate = msg.twist.linear.z
                
            except Exception as e:
                self.logger.error(f"Error updating velocity: {e}")
                
    def update_battery(self, msg: BatteryState):
        """Met à jour l'état batterie avec validation"""
        with self._lock:
            try:
                self.status.battery_voltage = msg.voltage
                self.status.battery_current = msg.current
                
                # Calcul pourcentage si pas fourni
                if msg.percentage >= 0:
                    self.status.battery_percentage = msg.percentage * 100
                else:
                    # Estimation basée sur tension (pour LiPo 3S: 9.0V-12.6V)
                    if msg.voltage > 0:
                        self.status.battery_percentage = max(0, min(100, 
                            (msg.voltage - 9.0) / (12.6 - 9.0) * 100))
                
                # Mise à jour historique pour le safety manager
                if self.safety_manager:
                    self.safety_manager.update_battery_trend(
                        self.status.battery_voltage,
                        self.status.battery_percentage
                    )
                    
            except Exception as e:
                self.logger.error(f"Error updating battery: {e}")
                
    def update_gps(self, msg: NavSatFix):
        """Met à jour les données GPS avec validation"""
        with self._lock:
            try:
                # Validation du fix GPS
                self.status.gps_fix = (msg.status.status >= 0)  # STATUS_FIX ou mieux
                
                # Extraction des données de qualité
                if hasattr(msg, 'status'):
                    # Extraction satellites depuis service ou message dédié requis
                    pass
                    
                # HDOP depuis position_covariance si disponible
                if len(msg.position_covariance) >= 1 and msg.position_covariance[0] > 0:
                    self.status.gps_hdop = math.sqrt(msg.position_covariance[0])
                    
            except Exception as e:
                self.logger.error(f"Error updating GPS: {e}")
                
    def set_home_position(self, x: float, y: float, z: float):
        """Définit la position home"""
        with self._lock:
            self._home_position = (x, y, z)
            self.logger.info(f"Home position set: {x:.2f}, {y:.2f}, {z:.2f}")
            
    def get_status(self) -> DroneStatusData:
        """Retourne une copie de l'état actuel"""
        with self._lock:
            # Mise à jour du timestamp
            current_time = Time()  # Sera mis à jour par le node appelant
            self.status.timestamp = current_time
            
            # Évaluation sécurité si disponible
            if self.safety_manager:
                safety_level, errors, warnings = self.safety_manager.check_flight_safety(self.status)
                self.status.safety_level = int(safety_level)
                self.status.safety_messages = errors + warnings
            
            # Retour d'une copie complète
            return DroneStatusData(
                timestamp=self.status.timestamp,
                state=self.status.state,
                mode=self.status.mode,
                armed=self.status.armed,
                connected=self.status.connected,
                guided=self.status.guided,
                position=self.status.position,
                velocity=self.status.velocity,
                attitude=self.status.attitude,
                battery_voltage=self.status.battery_voltage,
                battery_percentage=self.status.battery_percentage,
                battery_current=self.status.battery_current,
                gps_fix=self.status.gps_fix,
                gps_satellites=self.status.gps_satellites,
                gps_hdop=self.status.gps_hdop,
                home_distance=self.status.home_distance,
                ground_speed=self.status.ground_speed,
                air_speed=self.status.air_speed,
                climb_rate=self.status.climb_rate,
                last_heartbeat=self.status.last_heartbeat,
                connection_quality=self.status.connection_quality,
                mavlink_version=self.status.mavlink_version,
                safety_level=self.status.safety_level,
                safety_messages=self.status.safety_messages.copy(),
                cpu_usage=self.status.cpu_usage,
                memory_usage=self.status.memory_usage
            )
            
    def _validate_pose_message(self, msg: PoseStamped) -> bool:
        """Valide un message PoseStamped"""
        try:
            # Vérifier les valeurs numériques
            pos = msg.pose.position
            if not all(isinstance(x, (int, float)) and not math.isnan(x) and not math.isinf(x) 
                      for x in [pos.x, pos.y, pos.z]):
                self.logger.warning("Invalid position values in pose message")
                return False
                
            # Vérifier quaternion normalisé
            q = msg.pose.orientation
            quat_norm = math.sqrt(q.x**2 + q.y**2 + q.z**2 + q.w**2)
            if abs(quat_norm - 1.0) > 0.1:
                self.logger.warning(f"Quaternion not normalized: {quat_norm}")
                return False
                
            return True
        except Exception:
            return False
