#!/usr/bin/env python3
"""
navigation_supervisor_node.py - Nœud superviseur de navigation
"""

import rclpy
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from enum import Enum

from std_msgs.msg import Bool, String
from drone_msgs.msg import (NavigationStatus, MissionStatus, 
                           PathProgress)
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState

class SupervisorState(Enum):
    IDLE = "IDLE"
    NAVIGATING = "NAVIGATING"
    MISSION_RUNNING = "MISSION_RUNNING"
    EMERGENCY = "EMERGENCY"
    PAUSED = "PAUSED"

class NavigationSupervisorNode(Node):
    def __init__(self):
        super().__init__('navigation_supervisor_node')
        
    # Thread safety
    # _state_lock : protège l'accès concurrent à l'état du superviseur
    # _publish_lock : protège la publication sur les topics
    # _executor : exécute les tâches longues (supervision, publication)
        self._state_lock = threading.RLock()
        self._publish_lock = threading.RLock()
        self._executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="supervisor")
        
    # État du superviseur protégé
    # _supervisor_state : état courant du superviseur (IDLE, NAVIGATING, etc.)
    # _last_state_change : timestamp du dernier changement d'état
    # _emergency_active : indique si une urgence est active
    # _safe_to_navigate : indique si la navigation est autorisée
    # _mission_active : indique si une mission est en cours
    # _current_nav_status : statut courant de la navigation
    # _current_mission_status : statut courant de la mission
    # _mavros_connected : indique si MAVROS est connecté
        self._supervisor_state = SupervisorState.IDLE
        self._last_state_change = time.time()
        self._emergency_active = False
        self._safe_to_navigate = False
        self._mission_active = False
        self._current_nav_status = "IDLE"
        self._current_mission_status = "IDLE"
        self._mavros_connected = False
        
    # Paramètres configurables (timeout d'état, temps max de navigation, fréquence de supervision)
        self.declare_parameter('state_timeout', 30.0)
        self.declare_parameter('max_navigation_time', 3600.0)
        self.declare_parameter('supervision_rate', 1.0)
        
        self._state_timeout = self.get_parameter('state_timeout').value
        self._max_navigation_time = self.get_parameter('max_navigation_time').value
        supervision_rate = self.get_parameter('supervision_rate').value
        
    # Publishers thread-safe
    # _supervisor_status_pub : publie le statut du superviseur
    # _navigation_command_pub : publie les commandes du superviseur
        self._supervisor_status_pub = self.create_publisher(String, '/drone_nav/supervisor_status', 10)
        self._navigation_command_pub = self.create_publisher(String, '/drone_nav/supervisor_command', 10)
        
    # QoS profile pour MAVROS
        mavros_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
    # Subscribers
    # /drone_nav/safe_to_navigate : reçoit l'état de sécurité
    # /drone_nav/status : reçoit le statut de navigation
    # /drone_nav/mission_status : reçoit le statut de mission
    # /drone_nav/emergency_status : reçoit le statut d'urgence
    # /mavros/state : reçoit l'état MAVROS
        self.create_subscription(
            Bool, '/drone_nav/safe_to_navigate', self._safety_callback, 10
        )
        
        self.create_subscription(
            NavigationStatus, '/drone_nav/status', self._nav_status_callback, 10
        )
        
        self.create_subscription(
            MissionStatus, '/drone_nav/mission_status', self._mission_status_callback, 10
        )
        
        self.create_subscription(
            String, '/drone_nav/emergency_status', self._emergency_status_callback, 10
        )
        
        self.create_subscription(
            State, '/mavros/state', self._mavros_state_callback, mavros_qos
        )
        
    # Timer de supervision
    # Supervise périodiquement l'état du système de navigation
        self._supervision_timer = self.create_timer(
            1.0 / supervision_rate, 
            self._supervise_navigation_timer_callback
        )
        
        # État initial
        with self._state_lock:
            self._last_state_change = time.time()
            
        self.get_logger().info("Navigation Supervisor Node initialized")

    # Propriétés thread-safe
    # Les propriétés suivantes protègent l'accès concurrent aux variables d'état
    @property
    def supervisor_state(self):
        with self._state_lock:
            return self._supervisor_state
    
    @property
    def safe_to_navigate(self):
        with self._state_lock:
            return self._safe_to_navigate
    
    @property
    def mission_active(self):
        with self._state_lock:
            return self._mission_active
    
    @property
    def emergency_active(self):
        with self._state_lock:
            return self._emergency_active

    def _safety_callback(self, msg):
        # Callback appelé à chaque changement d'état de sécurité
        with self._state_lock:
            previous_safe = self._safe_to_navigate
            self._safe_to_navigate = msg.data
            
        # Réaction aux changements de sécurité
        if previous_safe and not msg.data:
            self._executor.submit(self._handle_safety_change, False)
        elif not previous_safe and msg.data:
            self._executor.submit(self._handle_safety_change, True)

    def _nav_status_callback(self, msg):
        # Callback appelé à chaque changement de statut de navigation
        with self._state_lock:
            self._current_nav_status = msg.status
            
        # Gestion asynchrone des changements d'état
        if msg.status == "NAVIGATING":
            self._executor.submit(self._set_supervisor_state, SupervisorState.NAVIGATING)
        elif msg.status in ["SUCCEEDED", "FAILED", "ABORTED"]:
            if self.supervisor_state == SupervisorState.NAVIGATING:
                self._executor.submit(self._set_supervisor_state, SupervisorState.IDLE)

    def _mission_status_callback(self, msg):
        # Callback appelé à chaque changement de statut de mission
        with self._state_lock:
            self._current_mission_status = msg.status
            
        # Gestion asynchrone des changements d'état mission
        if msg.status == "RUNNING":
            with self._state_lock:
                self._mission_active = True
            self._executor.submit(self._set_supervisor_state, SupervisorState.MISSION_RUNNING)
        elif msg.status == "PAUSED":
            self._executor.submit(self._set_supervisor_state, SupervisorState.PAUSED)
        elif msg.status in ["COMPLETED", "FAILED", "ABORTED"]:
            with self._state_lock:
                self._mission_active = False
            if self.supervisor_state == SupervisorState.MISSION_RUNNING:
                self._executor.submit(self._set_supervisor_state, SupervisorState.IDLE)

    def _emergency_status_callback(self, msg):
        # Callback appelé à chaque changement de statut d'urgence
        emergency_active = "NORMAL" not in msg.data
        
        with self._state_lock:
            previous_emergency = self._emergency_active
            self._emergency_active = emergency_active
            
        if emergency_active and not previous_emergency:
            self._executor.submit(self._set_supervisor_state, SupervisorState.EMERGENCY)
        elif not emergency_active and previous_emergency:
            # Sortie d'urgence - retour à l'état approprié
            self._executor.submit(self._handle_emergency_cleared)

    def _mavros_state_callback(self, msg):
        # Callback appelé à chaque changement d'état MAVROS
        with self._state_lock:
            previous_connected = self._mavros_connected
            self._mavros_connected = msg.connected
            
        # Réaction aux changements de connexion MAVROS
        if previous_connected and not msg.connected:
            self.get_logger().warn("Connexion MAVROS perdue")
            self._executor.submit(self._handle_mavros_disconnection)

    def _set_supervisor_state(self, new_state):
        # Change l'état du superviseur et publie le nouveau statut
        """Changement d'état thread-safe"""
        with self._state_lock:
            if self._supervisor_state != new_state:
                old_state = self._supervisor_state
                self._supervisor_state = new_state
                self._last_state_change = time.time()
                
                self.get_logger().info(f"Changement d'état: {old_state.value} -> {new_state.value}")
                
        # Publication asynchrone
        self._executor.submit(self._publish_supervisor_status)

    def _publish_supervisor_status(self):
        # Publie le statut du superviseur sur le topic dédié
        """Publication thread-safe du statut"""
        try:
            current_state = self.supervisor_state
            
            with self._publish_lock:
                msg = String()
                msg.data = current_state.value
                self._supervisor_status_pub.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f"Erreur publication statut: {e}")

    def _supervise_navigation_timer_callback(self):
        # Timer : déclenche la supervision périodique
        """Callback du timer de supervision"""
        try:
            self._executor.submit(self._perform_supervision_checks)
        except Exception as e:
            self.get_logger().error(f"Erreur soumission supervision: {e}")

    def _perform_supervision_checks(self):
        # Effectue les vérifications de supervision (timeouts, sécurité, etc.)
        """Effectue les vérifications de supervision"""
        try:
            current_time = time.time()
            
            with self._state_lock:
                state = self._supervisor_state
                last_change = self._last_state_change
                safe = self._safe_to_navigate
                
            # Vérifier les timeouts d'état
            if (state != SupervisorState.IDLE and 
                current_time - last_change > self._state_timeout):
                self.get_logger().warn(f"Timeout de l'état {state.value}")
                self._handle_state_timeout(state)
            
            # Vérifier le temps maximum de navigation
            if (state in [SupervisorState.NAVIGATING, SupervisorState.MISSION_RUNNING] and
                current_time - last_change > self._max_navigation_time):
                self.get_logger().error("Temps maximum de navigation dépassé")
                self._trigger_emergency_stop("Navigation timeout")
            
            # Vérifier la sécurité de navigation
            if (state in [SupervisorState.NAVIGATING, SupervisorState.MISSION_RUNNING] and
                not safe):
                self.get_logger().warn("Navigation interrompue: conditions non sécuritaires")
                self._pause_navigation("Safety conditions")
                
        except Exception as e:
            self.get_logger().error(f"Erreur supervision: {e}")

    def _handle_state_timeout(self, state):
        # Gestion des timeouts d'état (déclenche une urgence si nécessaire)
        """Gestion des timeouts d'état"""
        if state in [SupervisorState.NAVIGATING, SupervisorState.MISSION_RUNNING]:
            self.get_logger().error(f"Timeout {state.value} - Arrêt d'urgence")
            self._trigger_emergency_stop(f"State timeout: {state.value}")

    def _handle_safety_change(self, is_safe):
        # Gestion des changements de sécurité (pause/reprise de navigation)
        """Gestion des changements de sécurité"""
        current_state = self.supervisor_state
        
        if not is_safe and current_state in [SupervisorState.NAVIGATING, SupervisorState.MISSION_RUNNING]:
            self._pause_navigation("Safety change")
        elif is_safe and current_state == SupervisorState.PAUSED:
            self._resume_navigation()

    def _handle_emergency_cleared(self):
        # Gestion de la fin d'urgence (retour à l'état approprié)
        """Gestion de la fin d'urgence"""
        if self.safe_to_navigate:
            if self.mission_active:
                self._set_supervisor_state(SupervisorState.MISSION_RUNNING)
            else:
                self._set_supervisor_state(SupervisorState.IDLE)

    def _handle_mavros_disconnection(self):
        # Gestion de la déconnexion MAVROS (arrêt de navigation si nécessaire)
        """Gestion de la déconnexion MAVROS"""
        current_state = self.supervisor_state
        if current_state in [SupervisorState.NAVIGATING, SupervisorState.MISSION_RUNNING]:
            self._trigger_emergency_stop("MAVROS disconnection")

    def _trigger_emergency_stop(self, reason="Unknown"):
        # Déclenchement d'un arrêt d'urgence
        """Déclenchement d'arrêt d'urgence"""
        self.get_logger().error(f"ARRÊT D'URGENCE DÉCLENCHÉ: {reason}")
        
        try:
            with self._publish_lock:
                cmd_msg = String()
                cmd_msg.data = "EMERGENCY_STOP"
                self._navigation_command_pub.publish(cmd_msg)
                
            self._set_supervisor_state(SupervisorState.EMERGENCY)
        except Exception as e:
            self.get_logger().error(f"Erreur envoi arrêt d'urgence: {e}")

    def _pause_navigation(self, reason="Unknown"):
        # Mise en pause de la navigation
        """Mise en pause de la navigation"""
        current_state = self.supervisor_state
        if current_state in [SupervisorState.NAVIGATING, SupervisorState.MISSION_RUNNING]:
            self.get_logger().info(f"Mise en pause navigation: {reason}")
            
            try:
                with self._publish_lock:
                    cmd_msg = String()
                    cmd_msg.data = "PAUSE"
                    self._navigation_command_pub.publish(cmd_msg)
                    
                self._set_supervisor_state(SupervisorState.PAUSED)
            except Exception as e:
                self.get_logger().error(f"Erreur pause navigation: {e}")

    def _resume_navigation(self):
        # Reprise de la navigation après une pause
        """Reprise de la navigation"""
        if self.supervisor_state == SupervisorState.PAUSED and self.safe_to_navigate:
            self.get_logger().info("Reprise de la navigation")
            
            try:
                with self._publish_lock:
                    cmd_msg = String()
                    cmd_msg.data = "RESUME"
                    self._navigation_command_pub.publish(cmd_msg)
                    
                if self.mission_active:
                    self._set_supervisor_state(SupervisorState.MISSION_RUNNING)
                else:
                    self._set_supervisor_state(SupervisorState.NAVIGATING)
            except Exception as e:
                self.get_logger().error(f"Erreur reprise navigation: {e}")

    def stop_navigation(self):
        # Arrêt manuel de la navigation
        """Arrêt de la navigation (méthode publique)"""
        self.get_logger().info("Arrêt de la navigation demandé")
        
        try:
            with self._publish_lock:
                cmd_msg = String()
                cmd_msg.data = "STOP"
                self._navigation_command_pub.publish(cmd_msg)
                
            self._set_supervisor_state(SupervisorState.IDLE)
        except Exception as e:
            self.get_logger().error(f"Erreur arrêt navigation: {e}")

    def get_supervisor_status(self):
        # Retourne le statut courant du superviseur
        """Obtenir le statut actuel du superviseur"""
        with self._state_lock:
            return {
                'state': self._supervisor_state.value,
                'safe_to_navigate': self._safe_to_navigate,
                'mission_active': self._mission_active,
                'emergency_active': self._emergency_active,
                'last_change': self._last_state_change
            }

    def destroy_node(self):
        # Nettoyage du nœud : arrêt du ThreadPoolExecutor
        """Nettoyage lors de la destruction du nœud"""
        try:
            if hasattr(self, '_supervision_timer'):
                self._supervision_timer.destroy()
            if hasattr(self, '_executor'):
                self._executor.shutdown(wait=True)
            self.get_logger().info("Navigation Supervisor Node destroyed cleanly")
        except Exception as e:
            self.get_logger().error(f"Erreur lors du nettoyage: {e}")
        finally:
            super().destroy_node()

def main(args=None):
    # Point d'entrée principal du nœud superviseur de navigation
    rclpy.init(args=args)
    
    node = NavigationSupervisorNode()
    executor = MultiThreadedExecutor(num_threads=4)
    
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt demandé par l'utilisateur")
    except Exception as e:
        node.get_logger().error(f"Erreur dans l'exécuteur: {e}")
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()