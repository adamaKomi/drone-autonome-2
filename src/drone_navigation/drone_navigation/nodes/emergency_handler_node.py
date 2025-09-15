#!/usr/bin/env python3
"""
emergency_handler_node.py - Nœud de gestion des situations d'urgence
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.executors import MultiThreadedExecutor
from enum import Enum
import time
import threading

from std_msgs.msg import Bool, String
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix, BatteryState
from geometry_msgs.msg import PoseStamped

class EmergencyState(Enum):
    NORMAL = "NORMAL"
    LOW_BATTERY = "LOW_BATTERY"
    GPS_LOSS = "GPS_LOSS"
    COMMUNICATION_LOSS = "COMMUNICATION_LOSS"
    CRITICAL = "CRITICAL"

class EmergencyHandlerNode(Node):
    def __init__(self):
        # Initialisation du nœud de gestion d'urgence
        super().__init__('emergency_handler_node')
        
    # Variables d'état d'urgence (thread-safe)
        self._state_lock = threading.RLock()
        self._emergency_state = EmergencyState.NORMAL
        self._last_gps_time = time.time()
        self._last_communication_time = time.time()
        self._last_battery_level = 100.0
        self._emergency_handled = set()  # Éviter doublons d'actions
        
    # Chargement des paramètres de configuration (seuils, timeouts, etc.)
        self.declare_parameter('low_battery_threshold', 20.0)
        self.declare_parameter('gps_timeout', 10.0)
        self.declare_parameter('comms_timeout', 10.0)
        self.declare_parameter('critical_battery_threshold', 5.0)
        self.declare_parameter('return_home_altitude', 40.0)
        self.declare_parameter('emergency_action_cooldown', 30.0)  # Éviter spam d'actions
        
        self.low_battery_threshold = self.get_parameter('low_battery_threshold').value
        self.gps_timeout = self.get_parameter('gps_timeout').value
        self.comms_timeout = self.get_parameter('comms_timeout').value
        self.critical_battery_threshold = self.get_parameter('critical_battery_threshold').value
        self.return_home_altitude = self.get_parameter('return_home_altitude').value
        self.emergency_cooldown = self.get_parameter('emergency_action_cooldown').value
        
    # États système (connexion MAVROS, armement, mode courant)
        self._mavros_connected = False
        self._drone_armed = False
        self._current_mode = "UNKNOWN"
        
    # Clients pour envoyer des commandes à MAVROS (armement, changement de mode)
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
    # Publishers pour diffuser le statut d'urgence et la sécurité aux autres nœuds
        self._pub_lock = threading.Lock()
        self.emergency_status_pub = self.create_publisher(String, '/drone_nav/emergency_status', 10)
        self.emergency_trigger_pub = self.create_publisher(Bool, '/drone_nav/emergency_trigger', 10)
        self.safe_to_navigate_pub = self.create_publisher(Bool, '/drone_nav/safe_to_navigate', 10)
        
    # Souscriptions aux topics MAVROS pour surveiller l'état du drone
        self.create_subscription(
            State, '/mavros/state', self.state_callback,
            qos_profile_sensor_data
        )
        
        self.create_subscription(
            BatteryState, '/mavros/battery', self.battery_callback,
            qos_profile_sensor_data
        )
        
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback,
            qos_profile_sensor_data
        )
        
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_callback,
            qos_profile_sensor_data
        )
        
    # Timers pour vérifier régulièrement les conditions d'urgence et publier le statut
        self.safety_check_timer = self.create_timer(1.0, self.check_emergency_conditions)
        self.comms_check_timer = self.create_timer(2.0, self.check_communications)
        self.status_publish_timer = self.create_timer(0.5, self.publish_safety_status)
        
        self.get_logger().info("Emergency Handler Node initialized with MultiThreadedExecutor")

    # Propriétés thread-safe
    # Propriétés thread-safe pour accéder/modifier l'état d'urgence et les timestamps
    @property
    def emergency_state(self):
        with self._state_lock:
            return self._emergency_state
    
    @emergency_state.setter
    def emergency_state(self, value):
        with self._state_lock:
            self._emergency_state = value

    @property
    def last_gps_time(self):
        with self._state_lock:
            return self._last_gps_time
    
    @last_gps_time.setter
    def last_gps_time(self, value):
        with self._state_lock:
            self._last_gps_time = value

    @property
    def last_communication_time(self):
        with self._state_lock:
            return self._last_communication_time
    
    @last_communication_time.setter
    def last_communication_time(self, value):
        with self._state_lock:
            self._last_communication_time = value

    def state_callback(self, msg):
        # Callback pour l'état MAVROS (connexion, armement, mode)
        self.last_communication_time = time.time()
        
        with self._state_lock:
            self._mavros_connected = msg.connected
            self._drone_armed = msg.armed
            self._current_mode = msg.mode

    def battery_callback(self, msg):
        # Callback pour le niveau de batterie
        self.last_communication_time = time.time()
        
        # Correction: msg.percentage est déjà en pourcentage (0.0-1.0)
        battery_level = msg.percentage * 100.0
        
        with self._state_lock:
            self._last_battery_level = battery_level
        
        # Vérifications de seuils avec hystérésis
        if battery_level < self.critical_battery_threshold:
            self.trigger_emergency(EmergencyState.CRITICAL, f"Batterie critique: {battery_level:.1f}%")
        elif battery_level < self.low_battery_threshold:
            self.trigger_emergency(EmergencyState.LOW_BATTERY, f"Batterie faible: {battery_level:.1f}%")

    def gps_callback(self, msg):
        # Callback pour le signal GPS
        self.last_communication_time = time.time()
        
        # Vérifier la qualité du signal GPS
        if msg.status.status >= 0:  # STATUS_FIX ou mieux
            self.last_gps_time = time.time()
            
            # Effacer l'urgence GPS si elle était active
            if self.emergency_state == EmergencyState.GPS_LOSS:
                self.clear_emergency("Signal GPS rétabli")

    def pose_callback(self, msg):
        # Callback pour la position locale (non utilisée pour l'urgence)
        self.last_communication_time = time.time()

    def check_emergency_conditions(self):
        # Vérifie périodiquement les conditions d'urgence (perte GPS, comms)
        current_time = time.time()
        
        # Vérifier perte GPS uniquement si le drone est armé
        if self._drone_armed and current_time - self.last_gps_time > self.gps_timeout:
            self.trigger_emergency(EmergencyState.GPS_LOSS, "Perte du signal GPS")
        
        # Vérifier communications MAVROS
        if current_time - self.last_communication_time > self.comms_timeout:
            self.trigger_emergency(EmergencyState.COMMUNICATION_LOSS, "Perte de communication MAVROS")

    def check_communications(self):
        # Vérifie la reconnexion et nettoie le cache des actions d'urgence
        current_time = time.time()
        
        # Réinitialiser l'état d'urgence si les communications reviennent
        if (self.emergency_state == EmergencyState.COMMUNICATION_LOSS and 
            current_time - self.last_communication_time < self.comms_timeout):
            self.clear_emergency("Communications MAVROS rétablies")
        
        # Nettoyer le cache des actions d'urgence (après cooldown)
        with self._state_lock:
            # Garder seulement les actions qui ne sont pas expirées
            # action_key format: "STATE_timestamp_slot"
            self._emergency_handled = {
                action for action in self._emergency_handled 
                # Extraire le timestamp du nom de l'action et vérifier l'expiration
                if len(action.split('_')) == 2 and action.split('_')[1].isdigit()
                    and current_time < (int(action.split('_')[1]) + 1) * self.emergency_cooldown
            }

    def trigger_emergency(self, state, reason):
        """
        Déclenche une procédure d'urgence selon le type (batterie, GPS, comms, etc.)
        Priorise les urgences et évite le spam d'actions.
        """
        current_emergency = self.emergency_state
        
        # Éviter de re-déclencher la même urgence
        if current_emergency == state:
            return
        
        # Priorité des urgences (CRITICAL > GPS_LOSS > LOW_BATTERY > COMMUNICATION_LOSS)
        priority_order = [
            EmergencyState.CRITICAL,
            EmergencyState.GPS_LOSS, 
            EmergencyState.LOW_BATTERY,
            EmergencyState.COMMUNICATION_LOSS,
            EmergencyState.NORMAL
        ]
        
        current_priority = priority_order.index(current_emergency) if current_emergency in priority_order else 99
        new_priority = priority_order.index(state) if state in priority_order else 99
        
        # Ne déclencher que si la nouvelle urgence est plus prioritaire
        if new_priority > current_priority:
            return
        
        self.emergency_state = state
        self.get_logger().error(f"🚨 URGENCE: {state.value} - {reason}")
        
        # Publier le statut d'urgence de manière thread-safe
        self._safe_publish_emergency_status(f"{state.value}: {reason}")
        self._safe_publish_emergency_trigger(True)
        
        # Actions d'urgence selon le type (avec protection contre le spam)
        action_key = f"{state.value}_{int(time.time() / self.emergency_cooldown)}"
        
        with self._state_lock:
            if action_key in self._emergency_handled:
                return
            self._emergency_handled.add(action_key)
        
        # Exécuter l'action d'urgence appropriée
        if state == EmergencyState.CRITICAL:
            self.execute_land_procedure()
        elif state == EmergencyState.GPS_LOSS:
            self.execute_gps_loss_procedure()
        elif state == EmergencyState.LOW_BATTERY:
            self.execute_low_battery_procedure()
        elif state == EmergencyState.COMMUNICATION_LOSS:
            self.execute_communication_loss_procedure()

    def clear_emergency(self, reason):
        # Réinitialise l'état d'urgence si la situation est rétablie
        if self.emergency_state != EmergencyState.NORMAL:
            previous_state = self.emergency_state
            self.emergency_state = EmergencyState.NORMAL
            self.get_logger().info(f"✅ Urgence terminée ({previous_state.value}): {reason}")
            
            self._safe_publish_emergency_status(f"NORMAL: {reason}")
            self._safe_publish_emergency_trigger(False)

    def execute_land_procedure(self):
        # Procédure d'atterrissage d'urgence (mode LAND)
        """Atterrissage d'urgence immédiat"""
        self.get_logger().error("🛬 Exécution de la procédure d'atterrissage d'urgence")
        try:
            if self._mavros_connected:
                # Mode LAND pour atterrissage immédiat
                request = SetMode.Request()
                request.custom_mode = "LAND"
                self.set_mode_client.call_async(request)
            else:
                self.get_logger().error("MAVROS non connecté - impossible d'atterrir")
        except Exception as e:
            self.get_logger().error(f"Erreur lors de l'atterrissage d'urgence: {e}")

    def execute_gps_loss_procedure(self):
        # Procédure en cas de perte GPS (mode LOITER)
        """Procédure de perte GPS - maintien de position"""
        self.get_logger().warn("📡 Exécution de la procédure de perte GPS")
        try:
            if self._mavros_connected:
                # Mode LOITER pour maintenir la position
                request = SetMode.Request()
                request.custom_mode = "LOITER"
                self.set_mode_client.call_async(request)
            else:
                self.get_logger().error("MAVROS non connecté - impossible de changer de mode")
        except Exception as e:
            self.get_logger().error(f"Erreur lors du changement de mode (perte GPS): {e}")

    def execute_low_battery_procedure(self):
        # Procédure batterie faible (mode RTL)
        """Procédure batterie faible - retour à la maison"""
        self.get_logger().warn("🔋 Exécution de la procédure batterie faible")
        try:
            if self._mavros_connected:
                # Mode RTL (Return To Launch)
                request = SetMode.Request()
                request.custom_mode = "RTL"
                self.set_mode_client.call_async(request)
            else:
                self.get_logger().error("MAVROS non connecté - impossible de retourner à la maison")
        except Exception as e:
            self.get_logger().error(f"Erreur lors du retour à la maison: {e}")

    def execute_communication_loss_procedure(self):
        # Procédure en cas de perte de communication MAVROS
        """Procédure de perte de communication"""
        self.get_logger().error("📶 Perte de communication MAVROS - attente de reconnexion")
        # Ne pas changer de mode car on ne peut pas communiquer avec MAVROS

    def publish_safety_status(self):
        # Publie le statut de sécurité (Bool) pour les autres nœuds
        """Publier le statut de sécurité pour les autres nœuds"""
        safe = self.is_safe_to_navigate()
        
        with self._pub_lock:
            msg = Bool()
            msg.data = safe
            self.safe_to_navigate_pub.publish(msg)

    def _safe_publish_emergency_status(self, status):
        # Publie le statut d'urgence (String) de façon thread-safe
        with self._pub_lock:
            status_msg = String()
            status_msg.data = status
            self.emergency_status_pub.publish(status_msg)

    def _safe_publish_emergency_trigger(self, triggered):
        # Publie le trigger d'urgence (Bool) de façon thread-safe
        with self._pub_lock:
            trigger_msg = Bool()
            trigger_msg.data = triggered
            self.emergency_trigger_pub.publish(trigger_msg)

    def is_in_emergency(self):
        # Indique si le drone est en situation d'urgence
        return self.emergency_state != EmergencyState.NORMAL

    def is_safe_to_navigate(self):
        # Détermine si le drone peut naviguer en toute sécurité
        """Détermine si il est sûr de naviguer"""
        return (self.emergency_state == EmergencyState.NORMAL and 
                self._mavros_connected and
                time.time() - self.last_gps_time < self.gps_timeout)

    def get_emergency_state(self):
        # Retourne l'état d'urgence courant
        return self.emergency_state

    def get_system_status(self):
        # Retourne un résumé du statut système (dict)
        """Obtenir un résumé du statut système"""
        with self._state_lock:
            return {
                'emergency_state': self._emergency_state.value,
                'mavros_connected': self._mavros_connected,
                'drone_armed': self._drone_armed,
                'current_mode': self._current_mode,
                'battery_level': self._last_battery_level,
                'gps_age': time.time() - self._last_gps_time,
                'comm_age': time.time() - self._last_communication_time
            }

def main(args=None):
    # Point d'entrée principal du nœud
    rclpy.init(args=args)
    node = EmergencyHandlerNode()
    # Exécuteur multi-thread pour gérer les callbacks et timers
    executor = MultiThreadedExecutor(num_threads=3)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()