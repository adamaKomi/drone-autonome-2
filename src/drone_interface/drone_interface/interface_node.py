#!/usr/bin/env python3
"""
=============================================================================
DRONE INTERFACE NODE - Interface Robuste pour ArduPilot SITL Control
=============================================================================
Auteur: Adama Komi
Date: 2025-09-03
Version: 3.0.0

Description:
    Nœud ROS2 robuste pour l'interface avec MAVROS et ArduPilot.
    Implémentation basée sur les meilleures pratiques ROS2 et lifecycle management.
    
Améliorations v3.0:
    - Lifecycle management complet (états configuré/activé/désactivé)
    - Gestion d'erreurs robuste avec retry et recovery
    - Threading sécurisé et timeout appropriés
    - Validation exhaustive des données
    - Diagnostics intégrés
    - Configuration paramétrable
    - Logging structuré avec niveaux appropriés
    - Health monitoring continu
    - Graceful shutdown
    
Architecture:
    - État managé avec machine d'états lifecycle
    - Séparation des responsabilités (Safety, State, Health)
    - Gestion asynchrone avec futures et callbacks
    - Recovery automatique en cas de déconnexion
    
Services fournis:
    - /drone/arm : Armement sécurisé
    - /drone/disarm : Désarmement
    - /drone/set_mode : Changement de mode
    - /drone/safety_check : Vérification sécurité
    - /drone/health_check : État de santé système

Topics publiés:
    - /drone/status : État détaillé du drone
    - /drone/diagnostics : Diagnostics système

Utilisation:
    ros2 launch drone_interface drone_interface_launch.py
    ros2 lifecycle set /drone_interface configure
    ros2 lifecycle set /drone_interface activate
=============================================================================
"""

import time
import threading
import json
import rclpy
from typing import Optional
from concurrent.futures import ThreadPoolExecutor

# Import des modules internes
from .safety_manager import SafetyManager, SafetyLevel
from .state_manager import StateManager, DroneState, DroneStatusData
from .health_monitor import HealthMonitor

# ROS2 imports
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn
from rclpy.qos import (QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, 
                       QoSDurabilityPolicy, QoSLivelinessPolicy)
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.task import Future
from rclpy.timer import Timer

# Message types
from geometry_msgs.msg import PoseStamped, TwistStamped, Vector3Stamped
from mavros_msgs.msg import State, GPSRAW, OverrideRCIn, ActuatorControl
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL, CommandHome, CommandInt
from sensor_msgs.msg import NavSatFix, BatteryState, Imu
from std_msgs.msg import Float64, String, Bool, Header
from std_srvs.srv import Trigger, Empty
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue
from builtin_interfaces.msg import Time


class DroneInterface(LifecycleNode):
    """Nœud d'interface drone avec lifecycle management"""
    
    def __init__(self):
        super().__init__('drone_interface')
        
        self.logger = self.get_logger()
        self.logger.info("🚀 Initialisation du DroneInterface v3.0...")
        
        # Configuration des paramètres
        self._declare_parameters()
        
        # Managers
        self.safety_manager = SafetyManager(self.logger)
        self.state_manager = StateManager(self.logger, self.safety_manager)
        self.health_monitor = HealthMonitor(self.logger)
        
        # Threading et exécution
        self.thread_executor = ThreadPoolExecutor(max_workers=4, thread_name_prefix="drone_interface")
        self.callback_group = ReentrantCallbackGroup()
        self.service_group = MutuallyExclusiveCallbackGroup()
        
        # Variables d'état
        self._clients_ready = False
        self._publishers_active = False
        self._subscribers_active = False
        self._shutdown_requested = False
        
        # Timers
        self.status_timer: Optional[Timer] = None
        self.health_timer: Optional[Timer] = None
        self.diagnostics_timer: Optional[Timer] = None
        
        self.logger.info("✅ DroneInterface base initialisé")
        
    def _declare_parameters(self):
        """Déclaration des paramètres configurables"""
        # Paramètres de sécurité
        self.declare_parameter('safety.min_battery_voltage', 10.5)
        self.declare_parameter('safety.min_battery_percentage', 20.0)
        self.declare_parameter('safety.max_altitude', 120.0)
        self.declare_parameter('safety.min_gps_satellites', 6)
        
        # Paramètres de communication
        self.declare_parameter('comm.heartbeat_timeout', 5.0)
        self.declare_parameter('comm.service_timeout', 30.0)
        self.declare_parameter('comm.retry_attempts', 3)
        
        # Paramètres de performance
        self.declare_parameter('perf.status_rate', 10.0)  # Hz
        self.declare_parameter('perf.health_check_rate', 1.0)  # Hz
        self.declare_parameter('perf.diagnostics_rate', 0.5)  # Hz
        
    # ========== LIFECYCLE CALLBACKS ==========
    
    def on_configure(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Configuration du nœud - initialisation des ressources"""
        try:
            self.logger.info("🔧 Configuration du nœud...")
            
            # Configuration QoS
            self._setup_qos_profiles()
            
            # Création des clients de service
            self._create_service_clients()
            
            # Création des souscripteurs
            self._create_subscribers()
            
            # Création des services exposés
            self._create_services()
            
            # Création des publishers
            self._create_publishers()
            
            # Attendre les services MAVROS
            if not self._wait_for_mavros_services():
                self.logger.error("❌ Services MAVROS non disponibles")
                return TransitionCallbackReturn.FAILURE
                
            self._clients_ready = True
            self.logger.info("✅ Configuration réussie")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.logger.error(f"❌ Erreur de configuration: {e}")
            return TransitionCallbackReturn.FAILURE
            
    def on_activate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Activation du nœud - démarrage des publishers et timers"""
        try:
            self.logger.info("🚀 Activation du nœud...")
            
            # Activation des publishers
            self._publishers_active = True
            
            # Démarrage des timers
            self._start_timers()
            
            self.logger.info("✅ Nœud activé et opérationnel")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.logger.error(f"❌ Erreur d'activation: {e}")
            return TransitionCallbackReturn.FAILURE
            
    def on_deactivate(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Désactivation du nœud"""
        try:
            self.logger.info("⏸️ Désactivation du nœud...")
            
            # Arrêt des timers
            self._stop_timers()
            
            # Désactivation des publishers
            self._publishers_active = False
            
            self.logger.info("✅ Nœud désactivé")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.logger.error(f"❌ Erreur de désactivation: {e}")
            return TransitionCallbackReturn.FAILURE
            
    def on_cleanup(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Nettoyage des ressources"""
        try:
            self.logger.info("🧹 Nettoyage des ressources...")
            
            # Fermeture de l'executor
            if hasattr(self, 'thread_executor'):
                self.thread_executor.shutdown(wait=True)
                
            # Nettoyage des variables d'état
            self._clients_ready = False
            self._subscribers_active = False
            
            self.logger.info("✅ Nettoyage terminé")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.logger.error(f"❌ Erreur de nettoyage: {e}")
            return TransitionCallbackReturn.FAILURE
            
    def on_shutdown(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Arrêt complet"""
        self.logger.info("🛑 Arrêt du DroneInterface")
        self._shutdown_requested = True
        return TransitionCallbackReturn.SUCCESS
        
    def on_error(self, state: LifecycleState) -> TransitionCallbackReturn:
        """Gestion des erreurs"""
        self.logger.error(f"❌ Erreur dans l'état: {state}")
        # Tentative de recovery automatique
        return TransitionCallbackReturn.SUCCESS
        
    # ========== SETUP METHODS ==========
    
    def _setup_qos_profiles(self):
        """Configuration des profils QoS"""
        # QoS pour données temps réel (télémétrie)
        self.qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.VOLATILE,
            liveliness=QoSLivelinessPolicy.AUTOMATIC
        )
        
        # QoS pour commandes critiques
        self.qos_command = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
        # QoS pour état et diagnostics
        self.qos_status = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL
        )
        
    def _create_service_clients(self):
        """Création des clients de service MAVROS"""
        self.logger.debug("Création des clients de service...")
        
        # Services de base
        self.arm_client = self.create_client(
            CommandBool, '/mavros/cmd/arming',
            callback_group=self.service_group
        )
        self.mode_client = self.create_client(
            SetMode, '/mavros/set_mode',
            callback_group=self.service_group
        )
        self.takeoff_client = self.create_client(
            CommandTOL, '/mavros/cmd/takeoff',
            callback_group=self.service_group
        )
        self.land_client = self.create_client(
            CommandTOL, '/mavros/cmd/land',
            callback_group=self.service_group
        )
        
        # Services avancés
        self.home_client = self.create_client(
            CommandHome, '/mavros/cmd/set_home',
            callback_group=self.service_group
        )
        self.command_client = self.create_client(
            CommandInt, '/mavros/cmd/command_int',
            callback_group=self.service_group
        )
        
    def _create_subscribers(self):
        """Création des souscripteurs"""
        self.logger.debug("Création des souscripteurs...")
        
        # État principal
        self.state_sub = self.create_subscription(
            State, '/mavros/state', 
            self.state_manager.update_mavros_state, 
            self.qos_sensor,
            callback_group=self.callback_group
        )
        
        # Position et navigation
        self.position_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', 
            self.state_manager.update_position, 
            self.qos_sensor,
            callback_group=self.callback_group
        )
        
        self.velocity_sub = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local',
            self.state_manager.update_velocity,
            self.qos_sensor,
            callback_group=self.callback_group
        )
        
        # Capteurs
        self.battery_sub = self.create_subscription(
            BatteryState, '/mavros/battery', 
            self.state_manager.update_battery, 
            self.qos_sensor,
            callback_group=self.callback_group
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix, '/mavros/global_position/global', 
            self.state_manager.update_gps, 
            self.qos_sensor,
            callback_group=self.callback_group
        )
        
        self.imu_sub = self.create_subscription(
            Imu, '/mavros/imu/data',
            self._handle_imu_data,
            self.qos_sensor,
            callback_group=self.callback_group
        )
        
        self._subscribers_active = True
        
    def _create_services(self):
        """Création des services exposés"""
        self.logger.debug("Création des services...")
        
        # Services de contrôle
        self.arm_service = self.create_service(
            Trigger, '/drone/arm', 
            self._handle_arm_request,
            callback_group=self.service_group
        )
        
        self.disarm_service = self.create_service(
            Trigger, '/drone/disarm',
            self._handle_disarm_request,
            callback_group=self.service_group
        )
        
        self.mode_service = self.create_service(
            SetMode, '/drone/set_mode',
            self._handle_mode_request,
            callback_group=self.service_group
        )
        
        # Services de vérification
        self.safety_service = self.create_service(
            Trigger, '/drone/safety_check', 
            self._handle_safety_request,
            callback_group=self.service_group
        )
        
        self.health_service = self.create_service(
            Trigger, '/drone/health_check',
            self._handle_health_request,
            callback_group=self.service_group
        )
        
        # Services d'urgence
        self.emergency_service = self.create_service(
            Trigger, '/drone/emergency_stop',
            self._handle_emergency_request,
            callback_group=self.service_group
        )
        
    def _create_publishers(self):
        """Création des publishers"""
        self.logger.debug("Création des publishers...")
        
        # État du drone
        self.status_publisher = self.create_publisher(
            String, '/drone/status',
            self.qos_status
        )
        
        # Diagnostics
        self.diagnostics_publisher = self.create_publisher(
            DiagnosticArray, '/diagnostics',
            self.qos_status
        )
        
        # Sécurité
        self.safety_publisher = self.create_publisher(
            String, '/drone/safety_status',
            self.qos_command
        )
        
    def _wait_for_mavros_services(self, timeout: float = 30.0) -> bool:
        """Attend la disponibilité des services MAVROS"""
        self.logger.info("⏳ Attente des services MAVROS...")
        
        start_time = time.time()
        services = [
            (self.arm_client, "arming"),
            (self.mode_client, "set_mode"),
            (self.takeoff_client, "takeoff")
        ]
        
        while time.time() - start_time < timeout:
            all_ready = True
            for client, name in services:
                if not client.wait_for_service(timeout_sec=1.0):
                    all_ready = False
                    break
                    
            if all_ready:
                self.logger.info("✅ Tous les services MAVROS sont disponibles")
                return True
                
        self.logger.error("❌ Timeout waiting for MAVROS services")
        return False
        
    def _start_timers(self):
        """Démarre les timers périodiques"""
        # Timer pour publication de l'état
        status_rate = self.get_parameter('perf.status_rate').value
        self.status_timer = self.create_timer(
            1.0 / status_rate, 
            self._publish_status_callback,
            callback_group=self.callback_group
        )
        
        # Timer pour vérifications de santé
        health_rate = self.get_parameter('perf.health_check_rate').value
        self.health_timer = self.create_timer(
            1.0 / health_rate,
            self._health_check_callback,
            callback_group=self.callback_group
        )
        
        # Timer pour diagnostics
        diag_rate = self.get_parameter('perf.diagnostics_rate').value
        self.diagnostics_timer = self.create_timer(
            1.0 / diag_rate,
            self._publish_diagnostics_callback,
            callback_group=self.callback_group
        )
        
    def _stop_timers(self):
        """Arrête les timers"""
        for timer in [self.status_timer, self.health_timer, self.diagnostics_timer]:
            if timer:
                timer.cancel()
                
    # ========== CALLBACK METHODS ==========
    
    def _handle_imu_data(self, msg: Imu):
        """Traite les données IMU"""
        try:
            # Pour l'instant, on log juste la réception
            self.logger.debug("IMU data received")
        except Exception as e:
            self.logger.error(f"Error handling IMU data: {e}")
            
    def _publish_status_callback(self):
        """Callback pour publication périodique de l'état"""
        if not self._publishers_active:
            return
            
        try:
            # Récupération du statut actuel
            status = self.state_manager.get_status()
            
            # Mise à jour du timestamp
            status.timestamp = self.get_clock().now().to_msg()
            
            # Sérialisation JSON
            status_dict = {
                'timestamp': status.timestamp.sec + status.timestamp.nanosec * 1e-9,
                'state': int(status.state),
                'mode': status.mode,
                'armed': status.armed,
                'connected': status.connected,
                'guided': status.guided,
                'position': list(status.position),
                'velocity': list(status.velocity),
                'attitude': list(status.attitude),
                'battery_voltage': status.battery_voltage,
                'battery_percentage': status.battery_percentage,
                'battery_current': status.battery_current,
                'gps_fix': status.gps_fix,
                'gps_satellites': status.gps_satellites,
                'gps_hdop': status.gps_hdop,
                'home_distance': status.home_distance,
                'ground_speed': status.ground_speed,
                'air_speed': status.air_speed,
                'climb_rate': status.climb_rate,
                'last_heartbeat': status.last_heartbeat,
                'connection_quality': int(status.connection_quality),
                'mavlink_version': status.mavlink_version,
                'safety_level': status.safety_level,
                'safety_messages': status.safety_messages,
                'cpu_usage': status.cpu_usage,
                'memory_usage': status.memory_usage
            }
            
            # Publication
            msg = String()
            msg.data = json.dumps(status_dict)
            self.status_publisher.publish(msg)
            
        except Exception as e:
            self.logger.error(f"Error publishing status: {e}")
            
    def _health_check_callback(self):
        """Callback pour vérifications de santé périodiques"""
        try:
            # Vérification des services MAVROS
            self.health_monitor.check_mavros_services(self)
            
            # Mise à jour de la santé avec le statut actuel
            status = self.state_manager.get_status()
            self.health_monitor.update_health(status)
                
        except Exception as e:
            self.logger.error(f"Error in health check: {e}")
            
    def _publish_diagnostics_callback(self):
        """Callback pour publication des diagnostics"""
        if not self._publishers_active:
            return
            
        try:
            # Création du message de diagnostics
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()
            
            # Statut principal
            status = self.state_manager.get_status()
            health = self.health_monitor.get_health()
            
            # Diagnostic de connexion
            conn_diag = DiagnosticStatus()
            conn_diag.name = "drone_interface/connection"
            conn_diag.level = DiagnosticStatus.OK if status.connected else DiagnosticStatus.ERROR
            conn_diag.message = "Connected" if status.connected else "Disconnected"
            conn_diag.values = [
                KeyValue(key="mavros_connection", value=str(health.mavros_connection)),
                KeyValue(key="services_ready", value=str(health.services_ready)),
                KeyValue(key="last_heartbeat", value=f"{status.last_heartbeat:.2f}")
            ]
            diag_array.status.append(conn_diag)
            
            # Diagnostic de sécurité
            safety_diag = DiagnosticStatus()
            safety_diag.name = "drone_interface/safety"
            
            if status.safety_level == SafetyLevel.SAFE:
                safety_diag.level = DiagnosticStatus.OK
                safety_diag.message = "Safety status: SAFE"
            elif status.safety_level == SafetyLevel.WARNING:
                safety_diag.level = DiagnosticStatus.WARN
                safety_diag.message = "Safety status: WARNING"
            else:
                safety_diag.level = DiagnosticStatus.ERROR
                safety_diag.message = "Safety status: CRITICAL/EMERGENCY"
                
            safety_diag.values = [
                KeyValue(key="safety_level", value=str(status.safety_level)),
                KeyValue(key="battery_percentage", value=f"{status.battery_percentage:.1f}%"),
                KeyValue(key="gps_fix", value=str(status.gps_fix)),
                KeyValue(key="message_count", value=str(len(status.safety_messages)))
            ]
            
            # Ajouter les messages de sécurité
            for i, msg in enumerate(status.safety_messages[:5]):  # Limite à 5
                safety_diag.values.append(KeyValue(key=f"safety_msg_{i}", value=msg))
                
            diag_array.status.append(safety_diag)
            
            # Publication
            self.diagnostics_publisher.publish(diag_array)
            
        except Exception as e:
            self.logger.error(f"Error publishing diagnostics: {e}")
            
    # ========== SERVICE HANDLERS ==========
    
    def _handle_arm_request(self, request, response):
        """Gestionnaire robuste pour l'armement"""
        try:
            self.logger.info("🔫 Demande d'armement reçue")
            
            # Vérification de sécurité
            status = self.state_manager.get_status()
            can_arm, errors, warnings = self.safety_manager.check_pre_arm_conditions(status)
            
            if not can_arm:
                response.success = False
                response.message = f"Armement refusé: {'; '.join(errors)}"
                self.logger.warning(f"Armement refusé: {response.message}")
                return response
                
            # Avertissements non bloquants
            if warnings:
                self.logger.warning(f"Avertissements: {'; '.join(warnings)}")
                
            # Appel du service MAVROS
            if not self.arm_client.wait_for_service(timeout_sec=5.0):
                response.success = False
                response.message = "Service d'armement MAVROS non disponible"
                return response
                
            arm_request = CommandBool.Request()
            arm_request.value = True
            
            future = self.arm_client.call_async(arm_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                mavros_response = future.result()
                response.success = mavros_response.success
                response.message = f"Armement: {'Réussi' if mavros_response.success else 'Échoué'}"
                
                if mavros_response.success:
                    self.logger.info("✅ Drone armé avec succès")
                else:
                    self.logger.error(f"❌ Échec armement: {mavros_response.result}")
            else:
                response.success = False
                response.message = "Timeout du service d'armement"
                        
        except Exception as e:
            response.success = False
            response.message = f"Erreur lors de l'armement: {str(e)}"
            self.logger.error(f"❌ Erreur armement: {e}")
            
        return response
        
    def _handle_disarm_request(self, request, response):
        """Gestionnaire pour le désarmement"""
        try:
            self.logger.info("🔓 Demande de désarmement reçue")
            
            if not self.arm_client.wait_for_service(timeout_sec=5.0):
                response.success = False
                response.message = "Service de désarmement MAVROS non disponible"
                return response
                
            disarm_request = CommandBool.Request()
            disarm_request.value = False
            
            future = self.arm_client.call_async(disarm_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                mavros_response = future.result()
                response.success = mavros_response.success
                response.message = f"Désarmement: {'Réussi' if mavros_response.success else 'Échoué'}"
                
                if mavros_response.success:
                    self.logger.info("✅ Drone désarmé avec succès")
                else:
                    self.logger.error(f"❌ Échec désarmement: {mavros_response.result}")
            else:
                response.success = False
                response.message = "Timeout du service de désarmement"
                
        except Exception as e:
            response.success = False
            response.message = f"Erreur lors du désarmement: {str(e)}"
            self.logger.error(f"❌ Erreur désarmement: {e}")
            
        return response
        
    def _handle_mode_request(self, request, response):
        """Gestionnaire pour changement de mode"""
        try:
            self.logger.info(f"🎯 Changement de mode vers: {request.custom_mode}")
            
            if not self.mode_client.wait_for_service(timeout_sec=5.0):
                response.success = False
                response.message = "Service de changement de mode MAVROS non disponible"
                return response
                
            mode_request = SetMode.Request()
            mode_request.custom_mode = request.custom_mode
            
            future = self.mode_client.call_async(mode_request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                mavros_response = future.result()
                response.success = mavros_response.mode_sent
                response.message = f"Mode {request.custom_mode}: {'Accepté' if mavros_response.mode_sent else 'Refusé'}"
                
                if mavros_response.mode_sent:
                    self.logger.info(f"✅ Mode changé vers {request.custom_mode}")
                else:
                    self.logger.error(f"❌ Échec changement mode: {request.custom_mode}")
            else:
                response.success = False
                response.message = "Timeout du service de changement de mode"
                
        except Exception as e:
            response.success = False
            response.message = f"Erreur lors du changement de mode: {str(e)}"
            self.logger.error(f"❌ Erreur changement mode: {e}")
            
        return response
        
    def _handle_safety_request(self, request, response):
        """Gestionnaire pour vérification de sécurité"""
        try:
            self.logger.info("🔒 Vérification de sécurité demandée")
            
            status = self.state_manager.get_status()
            can_arm, errors, warnings = self.safety_manager.check_pre_arm_conditions(status)
            
            response.success = can_arm
            
            if can_arm:
                response.message = "Sécurité validée - Drone prêt"
                if warnings:
                    response.message += f" (Avertissements: {'; '.join(warnings)})"
            else:
                response.message = f"Problèmes de sécurité: {'; '.join(errors)}"
                if warnings:
                    response.message += f" | Avertissements: {'; '.join(warnings)}"
                    
            self.logger.info(f"Résultat sécurité: {response.message}")
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur lors de la vérification: {str(e)}"
            self.logger.error(f"❌ Erreur vérification sécurité: {e}")
            
        return response
        
    def _handle_health_request(self, request, response):
        """Gestionnaire pour vérification de santé système"""
        try:
            self.logger.info("❤️ Vérification de santé système demandée")
            
            health = self.health_monitor.get_health()
            status = self.state_manager.get_status()
            
            # Vérification globale
            is_healthy = self.health_monitor.is_healthy()
            
            response.success = is_healthy
            
            health_details = []
            health_details.append(f"MAVROS: {'✅' if health.mavros_connection else '❌'}")
            health_details.append(f"Services: {'✅' if health.services_ready else '❌'}")
            health_details.append(f"Erreurs: {health.error_count}")
            health_details.append(f"Connexion: {'✅' if status.connected else '❌'}")
            
            if health.warnings:
                health_details.append(f"Avertissements: {len(health.warnings)}")
                
            response.message = " | ".join(health_details)
            
            if health.warnings:
                response.message += f" | Détails: {'; '.join(health.warnings[:3])}"
                
            self.logger.info(f"État de santé: {response.message}")
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur lors de la vérification: {str(e)}"
            self.logger.error(f"❌ Erreur vérification santé: {e}")
            
        return response
        
    def _handle_emergency_request(self, request, response):
        """Gestionnaire pour arrêt d'urgence"""
        try:
            self.logger.warning("🚨 ARRÊT D'URGENCE DEMANDÉ")
            
            # Forcer le désarmement immédiat
            if self.arm_client.wait_for_service(timeout_sec=2.0):
                disarm_request = CommandBool.Request()
                disarm_request.value = False
                
                future = self.arm_client.call_async(disarm_request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.result() and future.result().success:
                    response.success = True
                    response.message = "Arrêt d'urgence exécuté - Drone désarmé"
                    self.logger.warning("⚠️ Arrêt d'urgence réussi")
                else:
                    response.success = False
                    response.message = "Échec du désarmement d'urgence"
                    self.logger.error("❌ Échec arrêt d'urgence")
            else:
                response.success = False
                response.message = "Service de désarmement non disponible"
                
            # Publier un événement de sécurité
            self._publish_safety_event("EMERGENCY_STOP", response.message)
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur lors de l'arrêt d'urgence: {str(e)}"
            self.logger.error(f"❌ Erreur arrêt d'urgence: {e}")
            
        return response
        
    def _publish_safety_event(self, event_type: str, message: str):
        """Publie un événement de sécurité"""
        try:
            if self._publishers_active:
                safety_msg = String()
                safety_msg.data = json.dumps({
                    "timestamp": time.time(),
                    "event_type": event_type,
                    "message": message,
                    "node": self.get_name()
                })
                self.safety_publisher.publish(safety_msg)
        except Exception as e:
            self.logger.error(f"Error publishing safety event: {e}")


def main(args=None):
    """Point d'entrée principal"""
    rclpy.init(args=args)
    
    # Configuration de l'executor multi-thread ROS2
    executor = MultiThreadedExecutor(num_threads=4)
    
    try:
        # Création du nœud
        node = DroneInterface()
        
        # Enregistrement du timestamp de démarrage
        node._start_time = time.time()
        
        # Ajout à l'executor ROS2
        executor.add_node(node)
        
        node.logger.info("🚀 DroneInterface démarré - En attente de configuration...")
        node.logger.info("📋 Commandes de lifecycle:")
        node.logger.info("   ros2 lifecycle set /drone_interface configure")
        node.logger.info("   ros2 lifecycle set /drone_interface activate")
        
        # Lancement
        executor.spin()
        
    except KeyboardInterrupt:
        print("\n🛑 Arrêt demandé par utilisateur")
    except Exception as e:
        print(f"❌ Erreur critique: {e}")
        import traceback
        traceback.print_exc()
    finally:
        try:
            if 'node' in locals():
                node.destroy_node()
        except:
            pass
        try:
            executor.shutdown()
        except:
            pass
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()
