#!/usr/bin/env python3
"""
gps_waypoint_manager_node.py - Gestionnaire de waypoints GPS
Ce nœud gère la navigation entre plusieurs waypoints GPS en utilisant le gps_navigation_node
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import threading
import time
from enum import Enum

from drone_msgs.action import GotoPositionAction
from drone_msgs.srv import GotoPosition, UpdatePosition, CancelNavigation
from drone_msgs.msg import Waypoint, WaypointList, MissionStatus, PathProgress
from geometry_msgs.msg import Point
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class WaypointState(Enum):
    PENDING = "PENDING"
    ACTIVE = "ACTIVE"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


class MissionState(Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"
    CANCELLED = "CANCELLED"


class GPSWaypointManagerNode(Node):
    def __init__(self):
        super().__init__('gps_waypoint_manager_node')
        
        # État interne thread-safe
        self._state_lock = threading.RLock()
        self._mission_state = MissionState.IDLE
        self._current_waypoint_index = 0
        self._waypoint_states = []
        self._auto_continue = True
        self._mission_running = False
        
        # Liste statique de waypoints GPS pour test
        self._waypoints = [
            {
                'latitude': 33.70489392,
                'longitude': -7.34817569,
                'altitude': 100.0,
                'yaw_angle': 0.0,
                'tolerance': 2.0,
                'wp_type': 'NORMAL'
            },
            {
                'latitude': 33.70510000,
                'longitude': -7.34820000,
                'altitude': 100.0,
                'yaw_angle': 1.57,  # 90 degrés
                'tolerance': 2.0,
                'wp_type': 'HOLD'
            },
            {
                'latitude': 33.70530000,
                'longitude': -7.34830000,
                'altitude': 98.0,
                'yaw_angle': 3.14,  # 180 degrés
                'tolerance': 2.0,
                'wp_type': 'NORMAL'
            },
            {
                'latitude': 33.70489392,
                'longitude': -7.34817569,
                'altitude': 125.0,
                'yaw_angle': 0.0,
                'tolerance': 2.0,
                'wp_type': 'LAND'
            }
        ]
        
        # Initialiser les états des waypoints
        self._waypoint_states = [WaypointState.PENDING for _ in self._waypoints]
        
        # Paramètres configurables
        self.declare_parameter('auto_continue', True)
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('retry_delay', 5.0)
        self.declare_parameter('waypoint_tolerance', 2.0)  # Tolérance par défaut pour les waypoints
        
        self._auto_continue = self.get_parameter('auto_continue').value
        self._max_retries = self.get_parameter('max_retries').value
        self._retry_delay = self.get_parameter('retry_delay').value
        waypoint_tolerance = self.get_parameter('waypoint_tolerance').value
        
        # Appliquer la tolérance configurée à tous les waypoints
        for wp in self._waypoints:
            if wp['tolerance'] <= 0:
                wp['tolerance'] = waypoint_tolerance
        
        # Action client pour GPS navigation
        self._gps_action_client = ActionClient(
            self, GotoPositionAction, '/drone_nav/goto_position_action'
        )
        
        # Publishers
        self._pub_lock = threading.Lock()
        self.mission_status_pub = self.create_publisher(
            MissionStatus, '/drone_mission/gps_mission_status', 10
        )
        self.waypoint_list_pub = self.create_publisher(
            WaypointList, '/drone_mission/gps_waypoint_list', 10
        )
        
        # Subscribers
        self.create_subscription(
            Bool, '/drone_nav/safe_to_navigate', 
            self.safety_callback, 10
        )
        self.create_subscription(
            PathProgress, '/drone_nav/progress',
            self.progress_callback, 10
        )
        
        # Services offerts par ce nœud
        self.start_mission_service = self.create_service(
            SetBool, '/drone_mission/start_gps_mission', 
            self.handle_start_mission
        )
        self.pause_mission_service = self.create_service(
            SetBool, '/drone_mission/pause_gps_mission',
            self.handle_pause_mission
        )
        self.resume_mission_service = self.create_service(
            SetBool, '/drone_mission/resume_gps_mission',
            self.handle_resume_mission
        )
        self.cancel_mission_service = self.create_service(
            SetBool, '/drone_mission/cancel_gps_mission',
            self.handle_cancel_mission
        )
        
        # Variables d'état
        self._safe_to_navigate = False
        self._current_goal_handle = None
        self._mission_thread = None
        
        # Timer pour publier le statut
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("GPS Waypoint Manager Node initialized")
        self.get_logger().info(f"Loaded {len(self._waypoints)} GPS waypoints")
        
        # Attendre que le service d'action soit disponible
        self.get_logger().info("En attente du service GPS navigation...")
        if self._gps_action_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().info("Service GPS navigation connecté")
        else:
            self.get_logger().warn("Service GPS navigation non disponible - continuant l'initialisation")

    @property
    def mission_state(self):
        with self._state_lock:
            return self._mission_state
    
    @mission_state.setter
    def mission_state(self, value):
        with self._state_lock:
            old_state = self._mission_state
            self._mission_state = value
            if old_state != value:
                self.get_logger().info(f"Mission state changed: {old_state.value} -> {value.value}")

    @property
    def current_waypoint_index(self):
        with self._state_lock:
            return self._current_waypoint_index
    
    @current_waypoint_index.setter
    def current_waypoint_index(self, value):
        with self._state_lock:
            self._current_waypoint_index = value

    def safety_callback(self, msg):
        """Callback pour la sécurité"""
        old_safe = self._safe_to_navigate
        self._safe_to_navigate = msg.data
        
        # Si on perd la sécurité pendant une mission
        if old_safe and not msg.data and self.mission_state == MissionState.RUNNING:
            self.get_logger().warn("Navigation non sécurisée - Pause automatique de la mission")
            self.mission_state = MissionState.PAUSED
            self._cancel_current_navigation()

    def progress_callback(self, msg):
        """Callback pour la progression de navigation"""
        self.get_logger().debug(
            f"Progression: {msg.progress:.2f}, Distance restante: {msg.distance_remaining:.2f}m, Status: {msg.status}"
        )

    def handle_start_mission(self, request, response):
        """Service pour démarrer la mission GPS"""
        if self.mission_state != MissionState.IDLE:
            response.success = False
            response.message = f"Mission déjà en cours (état: {self.mission_state.value})"
            return response
        
        if not self._safe_to_navigate:
            response.success = False
            response.message = "Navigation non sécurisée - Impossible de démarrer"
            return response
        
        # Vérifier la disponibilité du service d'action
        if not self._gps_action_client.server_is_ready():
            self.get_logger().warn("Service d'action GPS non prêt, tentative de reconnexion...")
            if not self._gps_action_client.wait_for_server(timeout_sec=5.0):
                response.success = False
                response.message = "Service GPS navigation non disponible"
                return response
        
        # Réinitialiser les états
        self.current_waypoint_index = 0
        with self._state_lock:
            self._waypoint_states = [WaypointState.PENDING for _ in self._waypoints]
        self.mission_state = MissionState.RUNNING
        self._mission_running = True
        
        # Démarrer la mission dans un thread séparé
        if self._mission_thread and self._mission_thread.is_alive():
            self.get_logger().warn("Thread de mission précédent encore actif, arrêt forcé")
            self._mission_running = False
            self._mission_thread.join(timeout=2.0)
        
        self._mission_thread = threading.Thread(target=self._run_mission, daemon=True)
        self._mission_thread.start()
        
        response.success = True
        response.message = "Mission GPS démarrée"
        self.get_logger().info(response.message)
        return response

    def handle_pause_mission(self, request, response):
        """Service pour mettre en pause la mission"""
        if self.mission_state != MissionState.RUNNING:
            response.success = False
            response.message = f"Mission non en cours (état: {self.mission_state.value})"
            return response
        
        self.mission_state = MissionState.PAUSED
        self._cancel_current_navigation()
        
        response.success = True
        response.message = "Mission GPS mise en pause"
        self.get_logger().info(response.message)
        return response

    def handle_resume_mission(self, request, response):
        """Service pour reprendre la mission"""
        if self.mission_state != MissionState.PAUSED:
            response.success = False
            response.message = f"Mission non en pause (état: {self.mission_state.value})"
            return response
        
        if not self._safe_to_navigate:
            response.success = False
            response.message = "Navigation non sécurisée - Impossible de reprendre"
            return response
        
        self.mission_state = MissionState.RUNNING
        response.success = True
        response.message = "Mission GPS reprise"
        self.get_logger().info(response.message)
        return response

    def handle_cancel_mission(self, request, response):
        """Service pour annuler la mission"""
        if self.mission_state == MissionState.IDLE:
            response.success = False
            response.message = "Aucune mission active à annuler"
            return response
        
        self.mission_state = MissionState.CANCELLED
        self._mission_running = False
        self._cancel_current_navigation()
        
        response.success = True
        response.message = "Mission GPS annulée"
        self.get_logger().info(response.message)
        return response

    def _run_mission(self):
        """Thread principal de la mission"""
        try:
            while (self._mission_running and 
                   self.current_waypoint_index < len(self._waypoints) and
                   self.mission_state not in [MissionState.CANCELLED, MissionState.FAILED]):
                
                # Attendre si en pause
                while (self.mission_state == MissionState.PAUSED and 
                       self.mission_state != MissionState.CANCELLED):
                    time.sleep(0.5)
                
                # Vérifier si annulé pendant la pause
                if self.mission_state == MissionState.CANCELLED or not self._mission_running:
                    break
                
                if not self._safe_to_navigate:
                    self.get_logger().warn("Navigation non sécurisée - Attente...")
                    time.sleep(1.0)
                    continue
                
                # Naviguer vers le waypoint courant
                success = self._navigate_to_current_waypoint()
                
                if success:
                    with self._state_lock:
                        if self.current_waypoint_index < len(self._waypoint_states):
                            self._waypoint_states[self.current_waypoint_index] = WaypointState.COMPLETED
                    self.get_logger().info(f"Waypoint {self.current_waypoint_index} atteint avec succès")
                    self.current_waypoint_index += 1
                else:
                    with self._state_lock:
                        if self.current_waypoint_index < len(self._waypoint_states):
                            self._waypoint_states[self.current_waypoint_index] = WaypointState.FAILED
                    self.get_logger().error(f"Échec du waypoint {self.current_waypoint_index}")
                    
                    if not self._auto_continue:
                        self.mission_state = MissionState.FAILED
                        break
                    else:
                        # Continuer au waypoint suivant
                        self.current_waypoint_index += 1
            
            # Mission terminée
            if self.mission_state == MissionState.CANCELLED:
                self.get_logger().info("Mission GPS annulée")
            elif self.current_waypoint_index >= len(self._waypoints):
                self.mission_state = MissionState.COMPLETED
                self.get_logger().info("Mission GPS terminée avec succès")
            elif self.mission_state != MissionState.FAILED:
                self.mission_state = MissionState.COMPLETED
                
        except Exception as e:
            self.get_logger().error(f"Erreur dans la mission: {e}")
            self.mission_state = MissionState.FAILED
        finally:
            self._mission_running = False
            # Nettoyer après la fin de mission
            if self.mission_state in [MissionState.COMPLETED]:
                # Garder l'état COMPLETED pendant un moment avant de revenir à IDLE
                self.create_timer(5.0, self._reset_to_idle)

    def _reset_to_idle(self):
        """Reset l'état à IDLE après completion"""
        if self.mission_state == MissionState.COMPLETED:
            self.mission_state = MissionState.IDLE

    def _navigate_to_current_waypoint(self):
        """Navigue vers le waypoint courant"""
        if self.current_waypoint_index >= len(self._waypoints):
            return False
        
        waypoint = self._waypoints[self.current_waypoint_index]
        with self._state_lock:
            if self.current_waypoint_index < len(self._waypoint_states):
                self._waypoint_states[self.current_waypoint_index] = WaypointState.ACTIVE
        
        self.get_logger().info(f"=== NAVIGATION VERS WAYPOINT {self.current_waypoint_index} ===")
        self.get_logger().info(f"Latitude: {waypoint['latitude']:.8f}")
        self.get_logger().info(f"Longitude: {waypoint['longitude']:.8f}")
        self.get_logger().info(f"Altitude: {waypoint['altitude']:.2f}m")
        self.get_logger().info(f"Tolérance: {waypoint['tolerance']:.2f}m")
        self.get_logger().info(f"Yaw: {waypoint['yaw_angle']:.3f} rad")
        
        # Vérifier la disponibilité du service
        if not self._gps_action_client.server_is_ready():
            self.get_logger().error("Service d'action GPS non prêt")
            if not self._gps_action_client.wait_for_server(timeout_sec=5.0):
                self.get_logger().error("Service GPS navigation non disponible")
                return False
        
        # Créer la requête d'action
        goal = GotoPositionAction.Goal()
        goal.latitude = waypoint['latitude']
        goal.longitude = waypoint['longitude']
        goal.altitude = waypoint['altitude']
        goal.yaw_angle = waypoint['yaw_angle']
        goal.tolerance = waypoint['tolerance']
        
        try:
            # Envoyer la requête
            self.get_logger().info("Envoi du goal d'action GPS...")
            send_goal_future = self._gps_action_client.send_goal_async(
                goal, feedback_callback=self._action_feedback_callback
            )
            
            # Attendre l'acceptation du goal avec timeout plus court
            timeout_start = time.time()
            while not send_goal_future.done() and (time.time() - timeout_start) < 10.0:
                if self.mission_state in [MissionState.CANCELLED, MissionState.PAUSED] or not self._safe_to_navigate:
                    self.get_logger().info("Navigation interrompue pendant l'envoi du goal")
                    return False
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if not send_goal_future.done():
                self.get_logger().error("Timeout lors de l'envoi du goal")
                return False
                
            goal_handle = send_goal_future.result()
            if goal_handle is None:
                self.get_logger().error("Goal handle nul reçu")
                return False
                
            if not goal_handle.accepted:
                self.get_logger().error("Goal rejeté par le serveur")
                return False
            
            # Sauvegarder le goal_handle pour pouvoir l'annuler
            with self._state_lock:
                self._current_goal_handle = goal_handle
            
            self.get_logger().info(f"Goal accepté pour waypoint {self.current_waypoint_index}")
            
            # Obtenir le résultat
            result_future = goal_handle.get_result_async()
            
            # Attendre le résultat avec vérifications d'état périodiques
            timeout = 300.0  # 5 minutes max par waypoint (augmenté)
            start_time = time.time()
            last_log = start_time
            
            while not result_future.done():
                current_time = time.time()
                
                # Log périodique pour debug
                if current_time - last_log > 10.0:  # Toutes les 10 secondes
                    elapsed = current_time - start_time
                    self.get_logger().info(f"Navigation en cours... Temps écoulé: {elapsed:.1f}s")
                    last_log = current_time
                
                # Vérifier les conditions d'arrêt
                if (self.mission_state in [MissionState.CANCELLED, MissionState.PAUSED] or
                    not self._safe_to_navigate or
                    not self._mission_running or
                    (current_time - start_time) > timeout):
                    
                    reason = "timeout" if (current_time - start_time) > timeout else "interruption"
                    self.get_logger().info(f"Annulation du goal en cours - raison: {reason}")
                    try:
                        cancel_future = goal_handle.cancel_goal_async()
                        # Attendre la confirmation d'annulation
                        timeout_cancel = time.time()
                        while not cancel_future.done() and (time.time() - timeout_cancel) < 5.0:
                            rclpy.spin_once(self, timeout_sec=0.1)
                        self.get_logger().info("Goal annulé avec succès")
                    except Exception as e:
                        self.get_logger().warn(f"Erreur lors de l'annulation: {e}")
                    return False
                
                # Attendre un peu avant de revérifier
                rclpy.spin_once(self, timeout_sec=0.5)
            
            # Analyser le résultat
            if result_future.result() is None:
                self.get_logger().error("Résultat de navigation nul")
                return False
                
            result = result_future.result().result
            if result is None:
                self.get_logger().error("Résultat d'action nul")
                return False
                
            success = result.success
            message = getattr(result, 'message', 'Pas de message')
            
            self.get_logger().info(f"=== RÉSULTAT WAYPOINT {self.current_waypoint_index} ===")
            self.get_logger().info(f"Succès: {success}")
            self.get_logger().info(f"Message: {message}")
            
            return success
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la navigation vers waypoint {self.current_waypoint_index}: {e}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")
            return False
        finally:
            # Nettoyer le goal_handle
            with self._state_lock:
                self._current_goal_handle = None

    def _action_feedback_callback(self, feedback):
        """Callback pour le feedback de l'action"""
        self.get_logger().debug(f"Feedback - Progression: {feedback.feedback.progress:.2f}, "
                               f"Distance restante: {feedback.feedback.distance_remaining:.2f}m")

    def _cancel_current_navigation(self):
        """Annule la navigation courante"""
        with self._state_lock:
            if self._current_goal_handle:
                try:
                    cancel_future = self._current_goal_handle.cancel_goal_async()
                    self.get_logger().info("Demande d'annulation de navigation envoyée")
                except Exception as e:
                    self.get_logger().warn(f"Erreur lors de l'annulation: {e}")
                finally:
                    self._current_goal_handle = None

    def publish_status(self):
        """Publie le statut de la mission"""
        # Mission Status
        mission_msg = MissionStatus()
        mission_msg.stamp = self.get_clock().now().to_msg()
        mission_msg.status = self.mission_state.value
        mission_msg.current_waypoint = self.current_waypoint_index
        mission_msg.total_waypoints = len(self._waypoints)
        
        if len(self._waypoints) > 0:
            mission_msg.progress = float(self.current_waypoint_index) / len(self._waypoints)
        else:
            mission_msg.progress = 0.0
        
        mission_msg.message = f"GPS Mission - {self.mission_state.value}"
        
        with self._pub_lock:
            self.mission_status_pub.publish(mission_msg)
        
        # Waypoint List
        waypoint_list_msg = WaypointList()
        waypoint_list_msg.stamp = self.get_clock().now().to_msg()
        waypoint_list_msg.count = len(self._waypoints)
        waypoint_list_msg.current_file = "static_gps_waypoints"
        
        # Convertir les waypoints
        for i, wp in enumerate(self._waypoints):
            waypoint_msg = Waypoint()
            waypoint_msg.position = Point(
                x=wp['longitude'], 
                y=wp['latitude'], 
                z=wp['altitude']
            )
            waypoint_msg.tolerance = wp['tolerance']
            waypoint_msg.wp_type = wp['wp_type']
            waypoint_msg.yaw = wp['yaw_angle']
            waypoint_msg.speed = 2.0  # vitesse par défaut
            waypoint_list_msg.waypoints.append(waypoint_msg)
        
        with self._pub_lock:
            self.waypoint_list_pub.publish(waypoint_list_msg)

    def destroy_node(self):
        """Nettoyage du nœud"""
        self.get_logger().info("Arrêt du GPS Waypoint Manager...")
        
        self._mission_running = False
        self.mission_state = MissionState.CANCELLED
        
        # Annuler la navigation courante
        self._cancel_current_navigation()
        
        # Attendre que le thread de mission se termine
        if self._mission_thread and self._mission_thread.is_alive():
            self.get_logger().info("Attente de l'arrêt du thread de mission...")
            self._mission_thread.join(timeout=5.0)
            if self._mission_thread.is_alive():
                self.get_logger().warn("Thread de mission n'a pas pu être arrêté proprement")
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = GPSWaypointManagerNode()
    
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Interruption clavier reçue")
    except Exception as e:
        node.get_logger().error(f"Erreur dans l'executor: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()