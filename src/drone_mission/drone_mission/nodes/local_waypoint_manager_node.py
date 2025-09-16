#!/usr/bin/env python3
"""
local_waypoint_manager_node.py - Gestionnaire de waypoints locaux
Ce nœud gère la navigation entre plusieurs waypoints locaux en utilisant le local_navigation_node
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import math
from enum import Enum

from drone_msgs.action import GotoLocalAction
from drone_msgs.srv import GotoLocal, UpdateLocal, CancelNavigation
from drone_msgs.msg import Waypoint, WaypointList, MissionStatus, PathProgress
from geometry_msgs.msg import Point
from std_msgs.msg import Bool


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


class LocalWaypointManagerNode(Node):
    def __init__(self):
        super().__init__('local_waypoint_manager_node')
        
        # État interne thread-safe
        self._state_lock = threading.RLock()
        self._mission_state = MissionState.IDLE
        self._current_waypoint_index = 0
        self._waypoint_states = []
        self._auto_continue = True
        self._mission_running = False
        
        # Liste statique de waypoints locaux pour test
        # Coordonnées en mètres dans le référentiel local NED (North-East-Down)
        self._waypoints = [
            {
                'x': 0.0,
                'y': 0.0,
                'z': 10.0,  # 10m d'altitude
                'yaw_angle': 0.0,
                'tolerance': 1.0,
                'wp_type': 'TAKEOFF'
            },
            {
                'x': 10.0,
                'y': 0.0,
                'z': 10.0,
                'yaw_angle': 0.0,
                'tolerance': 1.0,
                'wp_type': 'NORMAL'
            },
            {
                'x': 10.0,
                'y': 10.0,
                'z': 10.0,
                'yaw_angle': 1.57,  # 90 degrés
                'tolerance': 1.0,
                'wp_type': 'NORMAL'
            },
            {
                'x': 0.0,
                'y': 10.0,
                'z': 10.0,
                'yaw_angle': 3.14,  # 180 degrés
                'tolerance': 1.0,
                'wp_type': 'HOLD'
            },
            {
                'x': 0.0,
                'y': 0.0,
                'z': 10.0,
                'yaw_angle': -1.57,  # -90 degrés
                'tolerance': 1.0,
                'wp_type': 'NORMAL'
            },
            {
                'x': 0.0,
                'y': 0.0,
                'z': 0.5,  # Descendre pour atterrir
                'yaw_angle': 0.0,
                'tolerance': 1.0,
                'wp_type': 'LAND'
            }
        ]
        
        # Initialiser les états des waypoints
        self._waypoint_states = [WaypointState.PENDING for _ in self._waypoints]
        
        # Paramètres configurables
        self.declare_parameter('auto_continue', True)
        self.declare_parameter('max_retries', 3)
        self.declare_parameter('retry_delay', 3.0)
        self.declare_parameter('hold_time', 2.0)  # Temps d'attente aux waypoints HOLD
        
        self._auto_continue = self.get_parameter('auto_continue').value
        self._max_retries = self.get_parameter('max_retries').value
        self._retry_delay = self.get_parameter('retry_delay').value
        self._hold_time = self.get_parameter('hold_time').value
        
        # Action client pour navigation locale
        self._local_action_client = ActionClient(
            self, GotoLocalAction, '/drone_nav/goto_local_action'
        )
        
        # Service clients pour navigation locale
        self._goto_local_client = self.create_client(
            GotoLocal, '/drone_nav/goto_local'
        )
        self._update_local_client = self.create_client(
            UpdateLocal, '/drone_nav/update_local'
        )
        self._cancel_navigation_client = self.create_client(
            CancelNavigation, '/drone_nav/cancel_navigation'
        )
        
        # Publishers
        self._pub_lock = threading.Lock()
        self.mission_status_pub = self.create_publisher(
            MissionStatus, '/drone_mission/local_mission_status', 10
        )
        self.waypoint_list_pub = self.create_publisher(
            WaypointList, '/drone_mission/local_waypoint_list', 10
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
            Bool, '/drone_mission/start_local_mission', 
            self.handle_start_mission
        )
        self.pause_mission_service = self.create_service(
            Bool, '/drone_mission/pause_local_mission',
            self.handle_pause_mission
        )
        self.resume_mission_service = self.create_service(
            Bool, '/drone_mission/resume_local_mission',
            self.handle_resume_mission
        )
        self.cancel_mission_service = self.create_service(
            Bool, '/drone_mission/cancel_local_mission',
            self.handle_cancel_mission
        )
        self.skip_waypoint_service = self.create_service(
            Bool, '/drone_mission/skip_local_waypoint',
            self.handle_skip_waypoint
        )
        self.goto_waypoint_service = self.create_service(
            Bool, '/drone_mission/goto_local_waypoint',
            self.handle_goto_waypoint
        )
        
        # Variables d'état
        self._safe_to_navigate = False
        self._current_action_future = None
        self._mission_thread = None
        self._retry_count = 0
        
        # Timer pour publier le statut
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        self.get_logger().info("Local Waypoint Manager Node initialized")
        self.get_logger().info(f"Loaded {len(self._waypoints)} local waypoints")

    @property
    def mission_state(self):
        with self._state_lock:
            return self._mission_state
    
    @mission_state.setter
    def mission_state(self, value):
        with self._state_lock:
            self._mission_state = value

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
        self._safe_to_navigate = msg.data
        if not self._safe_to_navigate and self.mission_state == MissionState.RUNNING:
            self.get_logger().warn("Navigation non sécurisée - Pause automatique de la mission")
            self.mission_state = MissionState.PAUSED

    def progress_callback(self, msg):
        """Callback pour la progression de navigation"""
        # Log de progression si en debug
        if self.get_logger().get_effective_level() <= 10:  # DEBUG level
            self.get_logger().debug(f"Progression: {msg.progress:.2f}, "
                                   f"Distance: {msg.distance_remaining:.2f}m")

    def handle_start_mission(self, request, response):
        """Service pour démarrer la mission locale"""
        if self.mission_state != MissionState.IDLE:
            response.data = False
            self.get_logger().warn("Mission déjà en cours ou non idle")
            return response
        
        if not self._safe_to_navigate:
            response.data = False
            self.get_logger().warn("Navigation non sécurisée - Impossible de démarrer")
            return response
        
        # Réinitialiser les états
        self.current_waypoint_index = 0
        self._waypoint_states = [WaypointState.PENDING for _ in self._waypoints]
        self._retry_count = 0
        self.mission_state = MissionState.RUNNING
        self._mission_running = True
        
        # Démarrer la mission dans un thread séparé
        self._mission_thread = threading.Thread(target=self._run_mission)
        self._mission_thread.start()
        
        response.data = True
        self.get_logger().info("Mission locale démarrée")
        return response

    def handle_pause_mission(self, request, response):
        """Service pour mettre en pause la mission"""
        if self.mission_state != MissionState.RUNNING:
            response.data = False
            return response
        
        self.mission_state = MissionState.PAUSED
        self._cancel_current_navigation()
        
        response.data = True
        self.get_logger().info("Mission locale mise en pause")
        return response

    def handle_resume_mission(self, request, response):
        """Service pour reprendre la mission"""
        if self.mission_state != MissionState.PAUSED:
            response.data = False
            return response
        
        if not self._safe_to_navigate:
            response.data = False
            self.get_logger().warn("Navigation non sécurisée - Impossible de reprendre")
            return response
        
        self.mission_state = MissionState.RUNNING
        response.data = True
        self.get_logger().info("Mission locale reprise")
        return response

    def handle_cancel_mission(self, request, response):
        """Service pour annuler la mission"""
        if self.mission_state == MissionState.IDLE:
            response.data = False
            return response
        
        self.mission_state = MissionState.CANCELLED
        self._mission_running = False
        self._cancel_current_navigation()
        
        response.data = True
        self.get_logger().info("Mission locale annulée")
        return response

    def handle_skip_waypoint(self, request, response):
        """Service pour passer au waypoint suivant"""
        if self.mission_state != MissionState.RUNNING:
            response.data = False
            return response
        
        if self.current_waypoint_index < len(self._waypoints):
            self._waypoint_states[self.current_waypoint_index] = WaypointState.CANCELLED
            self.current_waypoint_index += 1
            self._cancel_current_navigation()
            
            response.data = True
            self.get_logger().info(f"Waypoint {self.current_waypoint_index - 1} ignoré")
        else:
            response.data = False
        
        return response

    def handle_goto_waypoint(self, request, response):
        """Service pour aller directement à un waypoint spécifique"""
        # Note: Ce service nécessiterait un paramètre index, 
        # mais Bool ne permet qu'un seul paramètre
        # Pour l'instant, on implémente une version simplifiée
        response.data = False
        self.get_logger().warn("Service goto_waypoint non encore implémenté avec Bool")
        return response

    def _run_mission(self):
        """Thread principal de la mission"""
        try:
            while (self._mission_running and 
                   self.current_waypoint_index < len(self._waypoints) and
                   self.mission_state != MissionState.CANCELLED):
                
                # Attendre si en pause
                while self.mission_state == MissionState.PAUSED:
                    time.sleep(0.5)
                    if self.mission_state == MissionState.CANCELLED:
                        return
                
                if not self._safe_to_navigate:
                    self.get_logger().warn("Navigation non sécurisée - Attente...")
                    time.sleep(1.0)
                    continue
                
                # Naviguer vers le waypoint courant
                success = self._navigate_to_current_waypoint()
                
                if success:
                    self._waypoint_states[self.current_waypoint_index] = WaypointState.COMPLETED
                    self.get_logger().info(f"Waypoint {self.current_waypoint_index} atteint avec succès")
                    
                    # Gérer les waypoints spéciaux (HOLD, etc.)
                    self._handle_waypoint_actions()
                    
                    # Passer au waypoint suivant
                    self.current_waypoint_index += 1
                    self._retry_count = 0  # Reset du compteur de retry
                else:
                    self._waypoint_states[self.current_waypoint_index] = WaypointState.FAILED
                    self.get_logger().error(f"Échec du waypoint {self.current_waypoint_index}")
                    
                    # Gestion des retry
                    if self._retry_count < self._max_retries:
                        self._retry_count += 1
                        self.get_logger().info(f"Retry {self._retry_count}/{self._max_retries}")
                        time.sleep(self._retry_delay)
                        continue
                    
                    if not self._auto_continue:
                        self.mission_state = MissionState.FAILED
                        break
                    else:
                        # Continuer au waypoint suivant
                        self.current_waypoint_index += 1
                        self._retry_count = 0
            
            # Mission terminée
            if self.current_waypoint_index >= len(self._waypoints):
                self.mission_state = MissionState.COMPLETED
                self.get_logger().info("Mission locale terminée avec succès")
            
        except Exception as e:
            self.get_logger().error(f"Erreur dans la mission: {e}")
            self.mission_state = MissionState.FAILED
        finally:
            self._mission_running = False

    def _navigate_to_current_waypoint(self):
        """Navigue vers le waypoint courant"""
        if self.current_waypoint_index >= len(self._waypoints):
            return False
        
        waypoint = self._waypoints[self.current_waypoint_index]
        self._waypoint_states[self.current_waypoint_index] = WaypointState.ACTIVE
        
        self.get_logger().info(f"Navigation vers waypoint local {self.current_waypoint_index}: "
                              f"x={waypoint['x']}, y={waypoint['y']}, z={waypoint['z']}")
        
        # Attendre que le service soit disponible
        if not self._local_action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Service navigation locale non disponible")
            return False
        
        # Créer la requête d'action
        goal = GotoLocalAction.Goal()
        goal.x = waypoint['x']
        goal.y = waypoint['y']
        goal.z = waypoint['z']
        goal.yaw_angle = waypoint['yaw_angle']
        goal.tolerance = waypoint['tolerance']
        
        # Envoyer la requête
        self._current_action_future = self._local_action_client.send_goal_async(
            goal, feedback_callback=self._action_feedback_callback
        )
        
        # Attendre le résultat
        try:
            goal_handle = self._current_action_future.result()
            if goal_handle is None:
                self.get_logger().error("Goal rejeté par le serveur")
                return False
            
            result_future = goal_handle.get_result_async()
            
            # Attendre avec timeout
            timeout = 60.0  # 1 minute max par waypoint
            start_time = time.time()
            
            while not result_future.done():
                if time.time() - start_time > timeout:
                    self.get_logger().error("Timeout atteint pour le waypoint")
                    goal_handle.cancel_goal_async()
                    return False
                
                if (self.mission_state == MissionState.CANCELLED or 
                    self.mission_state == MissionState.PAUSED):
                    goal_handle.cancel_goal_async()
                    return False
                
                time.sleep(0.1)
            
            result = result_future.result().result
            return result.success
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la navigation: {e}")
            return False

    def _handle_waypoint_actions(self):
        """Gère les actions spéciales aux waypoints"""
        if self.current_waypoint_index >= len(self._waypoints):
            return
        
        waypoint = self._waypoints[self.current_waypoint_index]
        wp_type = waypoint.get('wp_type', 'NORMAL')
        
        if wp_type == 'HOLD':
            self.get_logger().info(f"Waypoint HOLD - Attente de {self._hold_time}s")
            time.sleep(self._hold_time)
        elif wp_type == 'TAKEOFF':
            self.get_logger().info("Waypoint TAKEOFF atteint")
        elif wp_type == 'LAND':
            self.get_logger().info("Waypoint LAND atteint")

    def _action_feedback_callback(self, feedback):
        """Callback pour le feedback de l'action"""
        if self.get_logger().get_effective_level() <= 10:  # DEBUG level
            self.get_logger().debug(f"Progression: {feedback.feedback.progress:.2f}, "
                                   f"Distance restante: {feedback.feedback.distance_remaining:.2f}m")

    def _cancel_current_navigation(self):
        """Annule la navigation courante"""
        if self._current_action_future and not self._current_action_future.done():
            try:
                goal_handle = self._current_action_future.result()
                if goal_handle:
                    goal_handle.cancel_goal_async()
                    self.get_logger().info("Navigation courante annulée")
            except:
                pass

    def publish_status(self):
        """Publie le statut de la mission"""
        # Mission Status
        mission_msg = MissionStatus()
        mission_msg.stamp = self.get_clock().now().to_msg()
        mission_msg.status = self.mission_state.value
        mission_msg.current_waypoint = self.current_waypoint_index
        mission_msg.total_waypoints = len(self._waypoints)
        
        if self.current_waypoint_index < len(self._waypoints):
            mission_msg.progress = float(self.current_waypoint_index) / len(self._waypoints)
        else:
            mission_msg.progress = 1.0
        
        mission_msg.message = f"Local Mission - {self.mission_state.value}"
        
        with self._pub_lock:
            self.mission_status_pub.publish(mission_msg)
        
        # Waypoint List
        waypoint_list_msg = WaypointList()
        waypoint_list_msg.stamp = self.get_clock().now().to_msg()
        waypoint_list_msg.count = len(self._waypoints)
        waypoint_list_msg.current_file = "static_local_waypoints"
        
        # Convertir les waypoints
        for i, wp in enumerate(self._waypoints):
            waypoint_msg = Waypoint()
            waypoint_msg.position = Point(
                x=wp['x'], 
                y=wp['y'], 
                z=wp['z']
            )
            waypoint_msg.tolerance = wp['tolerance']
            waypoint_msg.wp_type = wp['wp_type']
            waypoint_msg.yaw = wp['yaw_angle']
            waypoint_msg.speed = 2.0  # vitesse par défaut
            waypoint_list_msg.waypoints.append(waypoint_msg)
        
        with self._pub_lock:
            self.waypoint_list_pub.publish(waypoint_list_msg)

    def get_waypoint_summary(self):
        """Retourne un résumé des waypoints pour debug"""
        summary = []
        for i, wp in enumerate(self._waypoints):
            state = self._waypoint_states[i] if i < len(self._waypoint_states) else WaypointState.PENDING
            summary.append({
                'index': i,
                'position': (wp['x'], wp['y'], wp['z']),
                'type': wp['wp_type'],
                'state': state.value
            })
        return summary

    def destroy_node(self):
        """Nettoyage du nœud"""
        self._mission_running = False
        self.mission_state = MissionState.CANCELLED
        
        if self._mission_thread and self._mission_thread.is_alive():
            self._mission_thread.join(timeout=5.0)
        
        self._cancel_current_navigation()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LocalWaypointManagerNode()
    
    executor = MultiThreadedExecutor(num_threads=4)
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
