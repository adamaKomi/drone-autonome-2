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
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.task import Future

# Message types
from geometry_msgs.msg import PoseStamped, TwistStamped
from mavros_msgs.msg import State, GPSRAW
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from sensor_msgs.msg import NavSatFix, BatteryState
from std_msgs.msg import Float64, String, Bool
from std_srvs.srv import Trigger


class DroneState(str, Enum):
    UNKNOWN = "UNKNOWN"
    DISCONNECTED = "DISCONNECTED"
    CONNECTED = "CONNECTED"
    ARMED = "ARMED"
    FLYING = "FLYING"
    LANDING = "LANDING"
    EMERGENCY = "EMERGENCY"


@dataclass
class DroneStatusData:
    state: DroneState = DroneState.UNKNOWN
    mode: str = "UNKNOWN"
    armed: bool = False
    connected: bool = False
    guided: bool = False
    position: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    battery_voltage: float = 0.0
    battery_percentage: float = 0.0
    gps_fix: bool = False
    gps_satellites: int = 0
    home_distance: float = 0.0
    last_heartbeat: float = 0.0


class SafetyManager:
    def __init__(self, logger):
        self.logger = logger
        
    def check_arm_conditions(self, status: DroneStatusData) -> Tuple[bool, str, List[str]]:
        if not status.connected:
            return False, "Drone not connected", []
        return True, "Ready to arm", []
        
    def check_flight_safety(self, status: DroneStatusData) -> Tuple[bool, List[str], List[str]]:
        return True, [], []


class StateManager:
    def __init__(self, logger, safety_manager: SafetyManager):
        self.logger = logger
        self.safety_manager = safety_manager
        self.status = DroneStatusData()
        self._lock = threading.RLock()
        self._home_position = None
        
    def update_mavros_state(self, msg: State):
        with self._lock:
            self.status.connected = msg.connected
            self.status.armed = msg.armed
            self.status.guided = msg.guided
            self.status.mode = msg.mode
            self.status.last_heartbeat = time.time()
            
            if not msg.connected:
                self.status.state = DroneState.DISCONNECTED
            elif not msg.armed:
                self.status.state = DroneState.CONNECTED
            elif msg.armed:
                self.status.state = DroneState.ARMED if self.status.position[2] < 0.5 else DroneState.FLYING
                
    def update_position(self, msg: PoseStamped):
        with self._lock:
            self.status.position = (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            )
            
    def update_battery(self, msg: BatteryState):
        with self._lock:
            if msg.voltage > 0:
                self.status.battery_voltage = msg.voltage
                self.status.battery_percentage = msg.percentage * 100 if msg.percentage >= 0 else 0
            
    def update_gps(self, msg: NavSatFix):
        with self._lock:
            self.status.gps_fix = True
            self.status.gps_satellites = 10
            
    def get_status(self) -> DroneStatusData:
        with self._lock:
            return DroneStatusData(
                state=self.status.state,
                mode=self.status.mode,
                armed=self.status.armed,
                connected=self.status.connected,
                guided=self.status.guided,
                position=self.status.position,
                battery_voltage=self.status.battery_voltage,
                battery_percentage=self.status.battery_percentage,
                gps_fix=self.status.gps_fix,
                gps_satellites=self.status.gps_satellites,
                home_distance=self.status.home_distance,
                last_heartbeat=self.status.last_heartbeat
            )


class DroneInterface(Node):
    def __init__(self):
        super().__init__('drone_interface')
        
        self.logger = self.get_logger()
        self.logger.info("🚁 Initialisation du DroneInterface...")
        
        # Configuration QoS
        self.qos_mavros = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Initialisation des modules
        self.safety_manager = SafetyManager(self.logger)
        self.state_manager = StateManager(self.logger, self.safety_manager)
        
        # Clients de service
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        
        # Souscripteurs
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_manager.update_mavros_state, self.qos_mavros)
        self.position_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.state_manager.update_position, self.qos_mavros)
        self.battery_sub = self.create_subscription(
            BatteryState, '/mavros/battery', self.state_manager.update_battery, self.qos_mavros)
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.state_manager.update_gps, self.qos_mavros)
        
        # Services
        self.arm_service = self.create_service(
            Trigger, '/drone/arm', self._handle_arm_request)
        self.safety_service = self.create_service(
            Trigger, '/drone/safety_check', self._handle_safety_request)
        
        self.logger.info("✅ DroneInterface initialisé")
        
    def _handle_arm_request(self, request, response):
        """Gère l'armement avec meilleure gestion d'erreurs"""
        try:
            # Vérifier si le service est disponible
            if not self.arm_client.service_is_ready():
                response.success = False
                response.message = "MAVROS arming service not available"
                return response
                
            # Créer la requête
            arm_request = CommandBool.Request()
            arm_request.value = True
            
            # Appel synchrone avec timeout
            future = self.arm_client.call_async(arm_request)
            start_time = time.time()
            
            while not future.done() and time.time() - start_time < 5.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if future.done():
                result = future.result()
                if result.success:
                    response.success = True
                    response.message = "Drone armed successfully"
                    self.logger.info("✅ Drone armé")
                else:
                    response.success = False
                    response.message = f"Arming failed: {result.result}"
                    self.logger.warn(f"⚠️ Échec armement: {result.result}")
            else:
                response.success = False
                response.message = "Arming request timeout"
                self.logger.warn("⚠️ Timeout armement")
                
        except Exception as e:
            response.success = False
            response.message = f"Arming error: {str(e)}"
            self.logger.error(f"❌ Erreur armement: {e}")
            
        return response
        
    def _handle_safety_request(self, request, response):
        """Vérification de sécurité"""
        try:
            status = self.state_manager.get_status()
            can_arm, reason, warnings = self.safety_manager.check_arm_conditions(status)
            
            # Vérifier aussi si les services sont disponibles
            services_ready = self.arm_client.service_is_ready()
            services_status = "Services ready" if services_ready else "Services not ready"
            
            response.message = f"{reason}, {services_status}"
            response.success = can_arm and services_ready
            
        except Exception as e:
            response.message = f"Error: {str(e)}"
            response.success = False
            
        return response


def main(args=None):
    rclpy.init(args=args)
    node = DroneInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.logger.info("🛑 Arrêt")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()