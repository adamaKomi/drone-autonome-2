#!/usr/bin/env python3
"""
mission_manager_node.py - Nœud de gestion de mission
"""

import rclpy
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from enum import Enum

from drone_msgs.msg import MissionStatus, WaypointReached, Waypoint, NavigationStatus
from drone_msgs.srv import (SetWaypoints, GetWaypoints, AddWaypoint, RemoveWaypoint, 
                           ClearWaypoints, PauseMission, ResumeMission, 
                           GetMissionStatus, SetMissionMode, GotoPosition, GotoLocal)
from std_srvs.srv import Empty

class MissionState(Enum):
    IDLE = "IDLE"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    COMPLETED = "COMPLETED"
    FAILED = "FAILED"

class MissionMode(Enum):
    SEQUENTIAL = "SEQUENTIAL"
    LOOP = "LOOP"
    BACKTRACK = "BACKTRACK"

class MissionManagerNode(Node):
    def __init__(self):
        super().__init__('mission_manager_node')
        
    # Thread safety
    # _state_lock : protège l'accès concurrent à l'état de mission
    # _publish_lock : protège la publication sur les topics
    # _executor : exécute les tâches longues (supervision, publication)
        self._state_lock = threading.RLock()
        self._publish_lock = threading.RLock()
        self._executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="mission_mgr")
        
    # État de la mission protégé
    # _mission_state : état courant de la mission (IDLE, RUNNING, etc.)
    # _mission_mode : mode de la mission (SEQUENTIAL, LOOP, BACKTRACK)
    # _current_waypoint_index : index du waypoint courant
    # _total_waypoints : nombre total de waypoints
    # _loop_count : nombre de boucles effectuées
    # _max_loops : nombre maximal de boucles
    # _waypoints : liste des waypoints de la mission
    # _mission_progress : progrès de la mission (0.0 à 1.0)
    # _mission_start_time : timestamp de début de mission
    # _navigation_active : indique si une navigation est en cours
        self._mission_state = MissionState.IDLE
        self._mission_mode = MissionMode.SEQUENTIAL
        self._current_waypoint_index = 0
        self._total_waypoints = 0
        self._loop_count = 0
        self._max_loops = 1
        self._waypoints = []
        self._mission_progress = 0.0
        self._mission_start_time = None
        self._navigation_active = False
        
    # Paramètres configurables (fréquence de publication, temps max de mission)
        self.declare_parameter('status_publish_rate', 1.0)
        self.declare_parameter('max_mission_time', 3600.0)
        
        status_rate = self.get_parameter('status_publish_rate').value
        self._max_mission_time = self.get_parameter('max_mission_time').value
        
    # Publishers thread-safe
    # _mission_status_pub : publie le statut de la mission
    # _waypoint_reached_pub : publie l'événement d'atteinte de waypoint
        self._mission_status_pub = self.create_publisher(
            MissionStatus, '/drone_nav/mission_status', 10
        )
        self._waypoint_reached_pub = self.create_publisher(
            WaypointReached, '/drone_nav/waypoint_reached', 10
        )
        
    # Subscribers
    # /drone_nav/status : reçoit le statut de navigation pour suivre la progression
        self.create_subscription(
            NavigationStatus, '/drone_nav/status', self._navigation_status_callback, 10
        )
        
    # Services de mission
    # /drone_nav/start_mission : démarre la mission
    # /drone_nav/stop_mission : arrête la mission
    # /drone_nav/pause_mission : met la mission en pause
    # /drone_nav/resume_mission : reprend la mission
    # /drone_nav/get_mission_status : obtient le statut de la mission
    # /drone_nav/set_mission_mode : change le mode de la mission
        self._start_mission_srv = self.create_service(
            Empty, '/drone_nav/start_mission', self._handle_start_mission
        )
        self._stop_mission_srv = self.create_service(
            Empty, '/drone_nav/stop_mission', self._handle_stop_mission
        )
        self._pause_mission_srv = self.create_service(
            PauseMission, '/drone_nav/pause_mission', self._handle_pause_mission
        )
        self._resume_mission_srv = self.create_service(
            ResumeMission, '/drone_nav/resume_mission', self._handle_resume_mission
        )
        self._get_status_srv = self.create_service(
            GetMissionStatus, '/drone_nav/get_mission_status', self._handle_get_status
        )
        self._set_mode_srv = self.create_service(
            SetMissionMode, '/drone_nav/set_mission_mode', self._handle_set_mode
        )
        
    # Services de waypoints (mission level)
    # /drone_nav/mission/set_waypoints : définit la liste des waypoints de la mission
    # /drone_nav/mission/get_waypoints : obtient la liste des waypoints de la mission
    # /drone_nav/mission/add_waypoint : ajoute un waypoint à la mission
    # /drone_nav/mission/remove_waypoint : supprime un waypoint de la mission
    # /drone_nav/mission/clear_waypoints : supprime tous les waypoints de la mission
        self._set_waypoints_srv = self.create_service(
            SetWaypoints, '/drone_nav/mission/set_waypoints', self._handle_set_waypoints
        )
        self._get_waypoints_srv = self.create_service(
            GetWaypoints, '/drone_nav/mission/get_waypoints', self._handle_get_waypoints
        )
        self._add_waypoint_srv = self.create_service(
            AddWaypoint, '/drone_nav/mission/add_waypoint', self._handle_add_waypoint
        )
        self._remove_waypoint_srv = self.create_service(
            RemoveWaypoint, '/drone_nav/mission/remove_waypoint', self._handle_remove_waypoint
        )
        self._clear_waypoints_srv = self.create_service(
            ClearWaypoints, '/drone_nav/mission/clear_waypoints', self._handle_clear_waypoints
        )
        
    # Timer pour la publication de statut
    # Publie périodiquement le statut de la mission
        self._status_timer = self.create_timer(
            1.0 / status_rate, 
            self._status_timer_callback
        )
        
    # Timer pour la supervision de mission
    # Supervise périodiquement l'état de la mission
        self._mission_timer = self.create_timer(0.5, self._mission_supervision_callback)
        
    # Clients pour commander les nœuds de navigation
    # _gps_nav_client : client pour envoyer des commandes GPS
    # _local_nav_client : client pour envoyer des commandes locales
        self._gps_nav_client = self.create_client(GotoPosition, '/drone_nav/goto_position')
        self._local_nav_client = self.create_client(GotoLocal, '/drone_nav/goto_local')
        
    # Clients pour interagir avec waypoint manager
    # _wp_get_client : client pour obtenir les waypoints locaux
    # _wp_set_client : client pour définir les waypoints locaux
        self._wp_get_client = self.create_client(GetWaypoints, '/drone_nav/get_waypoints_local')
        self._wp_set_client = self.create_client(SetWaypoints, '/drone_nav/set_waypoints_local')
        
        self.get_logger().info("Mission Manager Node initialized")

    # Propriétés thread-safe
    # Les propriétés suivantes protègent l'accès concurrent aux variables d'état
    @property
    def mission_state(self):
        with self._state_lock:
            return self._mission_state
    
    @property
    def mission_mode(self):
        with self._state_lock:
            return self._mission_mode
    
    @property
    def current_waypoint_index(self):
        with self._state_lock:
            return self._current_waypoint_index
    
    @property
    def total_waypoints(self):
        with self._state_lock:
            return self._total_waypoints

    def _navigation_status_callback(self, msg):
        # Callback appelé à chaque changement de statut de navigation
        """Callback pour le statut de navigation"""
        with self._state_lock:
            previous_nav_active = self._navigation_active
            self._navigation_active = (msg.status == "NAVIGATING")
            
            # Si navigation terminée avec succès
            if (previous_nav_active and msg.status == "SUCCEEDED" and 
                self._mission_state == MissionState.RUNNING):
                self._executor.submit(self._handle_waypoint_reached)
            
            # Si navigation échouée
            elif (previous_nav_active and msg.status in ["FAILED", "ABORTED"] and
                  self._mission_state == MissionState.RUNNING):
                self._executor.submit(self._handle_navigation_failed)

    def _handle_waypoint_reached(self):
        # Gestion de l'événement d'atteinte de waypoint
        """Gestion de l'atteinte d'un waypoint"""
        try:
            with self._state_lock:
                if self._mission_state != MissionState.RUNNING:
                    return
                    
                current_idx = self._current_waypoint_index
                
                # Publier waypoint atteint
                if current_idx < len(self._waypoints):
                    self._publish_waypoint_reached(current_idx)
                
                # Avancer au waypoint suivant
                next_idx = self._get_next_waypoint_index()
                
                if next_idx is None:
                    # Mission terminée
                    self._mission_state = MissionState.COMPLETED
                    self._mission_progress = 1.0
                    self.get_logger().info("Mission terminée avec succès")
                else:
                    self._current_waypoint_index = next_idx
                    self._update_mission_progress()
                    # Le prochain waypoint sera envoyé par la supervision
                    
        except Exception as e:
            self.get_logger().error(f"Erreur gestion waypoint atteint: {e}")

    def _handle_navigation_failed(self):
        # Gestion de l'événement d'échec de navigation
        """Gestion de l'échec de navigation"""
        try:
            with self._state_lock:
                if self._mission_state == MissionState.RUNNING:
                    self._mission_state = MissionState.FAILED
                    self.get_logger().error("Mission échouée - Navigation impossible")
                    
        except Exception as e:
            self.get_logger().error(f"Erreur gestion échec navigation: {e}")

    def _get_next_waypoint_index(self):
        # Calcule l'index du prochain waypoint selon le mode de mission
        """Calcule l'index du prochain waypoint selon le mode"""
        if not self._waypoints:
            return None
            
        current = self._current_waypoint_index
        total = len(self._waypoints)
        
        if self._mission_mode == MissionMode.SEQUENTIAL:
            if current + 1 >= total:
                return None  # Mission terminée
            return current + 1
            
        elif self._mission_mode == MissionMode.LOOP:
            next_idx = (current + 1) % total
            if next_idx == 0:  # Fin d'une boucle
                self._loop_count += 1
                if self._loop_count >= self._max_loops:
                    return None  # Toutes les boucles terminées
            return next_idx
            
        elif self._mission_mode == MissionMode.BACKTRACK:
            # Alternance aller-retour
            if current + 1 >= total:
                # Retour
                return max(0, current - 1) if current > 0 else None
            return current + 1
            
        return None

    def _update_mission_progress(self):
        # Met à jour le progrès de la mission
        """Met à jour le progrès de la mission"""
        if not self._waypoints:
            self._mission_progress = 0.0
            return
            
        if self._mission_mode == MissionMode.SEQUENTIAL:
            self._mission_progress = self._current_waypoint_index / max(1, len(self._waypoints))
        elif self._mission_mode == MissionMode.LOOP:
            total_waypoints = len(self._waypoints) * self._max_loops
            completed = self._loop_count * len(self._waypoints) + self._current_waypoint_index
            self._mission_progress = completed / max(1, total_waypoints)
        else:
            self._mission_progress = min(1.0, self._current_waypoint_index / max(1, len(self._waypoints)))

    def _mission_supervision_callback(self):
        # Timer : supervise périodiquement la mission
        """Supervision périodique de la mission"""
        try:
            with self._state_lock:
                state = self._mission_state
                
            # Vérifications selon l'état
            if state == MissionState.RUNNING:
                self._check_mission_timeout()
                self._check_next_waypoint_dispatch()
                
        except Exception as e:
            self.get_logger().error(f"Erreur supervision mission: {e}")

    def _check_mission_timeout(self):
        # Vérifie si la mission a dépassé le temps maximal
        """Vérification du timeout de mission"""
        with self._state_lock:
            if (self._mission_start_time and 
                time.time() - self._mission_start_time > self._max_mission_time):
                self._mission_state = MissionState.FAILED
                self.get_logger().error("Mission échouée - Timeout dépassé")

    def _check_next_waypoint_dispatch(self):
        # Vérifie si le prochain waypoint doit être envoyé
        """Vérifie si le prochain waypoint doit être envoyé"""
        with self._state_lock:
            if (self._mission_state == MissionState.RUNNING and 
                not self._navigation_active and 
                self._current_waypoint_index < len(self._waypoints)):
                
                # Obtenir le waypoint actuel
                waypoint = self._waypoints[self._current_waypoint_index]
                self.get_logger().info(f"Envoi waypoint {self._current_waypoint_index}: {waypoint.position}")
                
                # Déterminer si c'est GPS ou local
                if abs(waypoint.position.x) > 90 or abs(waypoint.position.y) > 180:
                    # Coordonnées locales (grandes valeurs)
                    self._send_local_waypoint(waypoint)
                else:
                    # Coordonnées GPS (latitude/longitude)
                    self._send_gps_waypoint(waypoint)

    def _send_gps_waypoint(self, waypoint):
        # Envoie un waypoint GPS au nœud de navigation GPS
        """Envoie un waypoint GPS au nœud de navigation GPS"""
        try:
            if self._gps_nav_client.wait_for_service(timeout_sec=1.0):
                request = GotoPosition.Request()
                request.latitude = waypoint.position.x
                request.longitude = waypoint.position.y
                request.altitude = waypoint.position.z
                request.yaw_angle = waypoint.yaw
                request.tolerance = waypoint.tolerance
                
                future = self._gps_nav_client.call_async(request)
                self.get_logger().info(f"Waypoint GPS envoyé: {waypoint.position.x}, {waypoint.position.y}")
            else:
                self.get_logger().warn("Service GPS navigation non disponible")
        except Exception as e:
            self.get_logger().error(f"Erreur envoi waypoint GPS: {e}")

    def _send_local_waypoint(self, waypoint):
        # Envoie un waypoint local au nœud de navigation locale
        """Envoie un waypoint local au nœud de navigation locale"""
        try:
            if self._local_nav_client.wait_for_service(timeout_sec=1.0):
                request = GotoLocal.Request()
                request.x = waypoint.position.x
                request.y = waypoint.position.y
                request.z = waypoint.position.z
                request.yaw_angle = waypoint.yaw
                
                future = self._local_nav_client.call_async(request)
                self.get_logger().info(f"Waypoint local envoyé: {waypoint.position.x}, {waypoint.position.y}")
            else:
                self.get_logger().warn("Service navigation locale non disponible")
        except Exception as e:
            self.get_logger().error(f"Erreur envoi waypoint local: {e}")

    def _status_timer_callback(self):
        # Timer : publie périodiquement le statut de la mission
        """Callback du timer de publication de statut"""
        try:
            self._executor.submit(self._publish_mission_status)
        except Exception as e:
            self.get_logger().error(f"Erreur soumission publication statut: {e}")

    def _publish_mission_status(self):
        # Publie le statut de la mission sur le topic dédié
        """Publication thread-safe du statut de mission"""
        try:
            with self._state_lock:
                msg = MissionStatus()
                msg.stamp = self.get_clock().now().to_msg()
                msg.status = self._mission_state.value
                msg.mode = self._mission_mode.value
                msg.current_waypoint = self._current_waypoint_index
                msg.total_waypoints = len(self._waypoints)
                msg.progress = self._mission_progress
                msg.message = self._get_status_message()
                
            with self._publish_lock:
                self._mission_status_pub.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f"Erreur publication statut: {e}")

    def _publish_waypoint_reached(self, waypoint_index):
        # Publie l'événement d'atteinte de waypoint
        """Publication thread-safe d'atteinte de waypoint"""
        try:
            msg = WaypointReached()
            msg.stamp = self.get_clock().now().to_msg()
            msg.index = waypoint_index
            
            if waypoint_index < len(self._waypoints):
                msg.waypoint = self._waypoints[waypoint_index]
                
            msg.reached_time = self.get_clock().now().to_msg()
            
            with self._publish_lock:
                self._waypoint_reached_pub.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f"Erreur publication waypoint atteint: {e}")

    def _get_status_message(self):
        # Génère un message descriptif du statut de la mission
        """Génère un message de statut descriptif"""
        if self._mission_state == MissionState.IDLE:
            return "En attente"
        elif self._mission_state == MissionState.RUNNING:
            return f"En cours - Waypoint {self._current_waypoint_index + 1}/{len(self._waypoints)}"
        elif self._mission_state == MissionState.PAUSED:
            return "En pause"
        elif self._mission_state == MissionState.COMPLETED:
            return "Mission terminée"
        elif self._mission_state == MissionState.FAILED:
            return "Mission échouée"
        return "État inconnu"

    # Handlers de services
    # Les méthodes suivantes gèrent les services de mission et de waypoints
    def _handle_start_mission(self, request, response):
        # Service : démarre la mission
        """Service de démarrage de mission"""
        try:
            with self._state_lock:
                if self._mission_state == MissionState.RUNNING:
                    response.success = False
                    response.message = "Mission déjà en cours"
                    return response
                
                if not self._waypoints:
                    response.success = False
                    response.message = "Aucun waypoint défini"
                    return response
                
                self._mission_state = MissionState.RUNNING
                self._current_waypoint_index = 0
                self._loop_count = 0
                self._mission_progress = 0.0
                self._mission_start_time = time.time()
            
            response.success = True
            response.message = "Mission démarrée"
            self.get_logger().info("Mission démarrée")
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur démarrage: {e}"
            
        return response

    def _handle_stop_mission(self, request, response):
        # Service : arrête la mission
        """Service d'arrêt de mission"""
        try:
            with self._state_lock:
                self._mission_state = MissionState.IDLE
                self._current_waypoint_index = 0
                self._mission_progress = 0.0
            
            response.success = True
            response.message = "Mission arrêtée"
            self.get_logger().info("Mission arrêtée")
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur arrêt: {e}"
            
        return response

    def _handle_pause_mission(self, request, response):
        # Service : met la mission en pause
        """Service de pause de mission"""
        try:
            with self._state_lock:
                if self._mission_state == MissionState.RUNNING:
                    self._mission_state = MissionState.PAUSED
                    response.success = True
                    response.message = "Mission mise en pause"
                else:
                    response.success = False
                    response.message = f"Impossible de mettre en pause: état {self._mission_state.value}"
                    
        except Exception as e:
            response.success = False
            response.message = f"Erreur pause: {e}"
            
        return response

    def _handle_resume_mission(self, request, response):
        # Service : reprend la mission
        """Service de reprise de mission"""
        try:
            with self._state_lock:
                if self._mission_state == MissionState.PAUSED:
                    self._mission_state = MissionState.RUNNING
                    response.success = True
                    response.message = "Mission reprise"
                else:
                    response.success = False
                    response.message = f"Impossible de reprendre: état {self._mission_state.value}"
                    
        except Exception as e:
            response.success = False
            response.message = f"Erreur reprise: {e}"
            
        return response

    def _handle_get_status(self, request, response):
        # Service : obtient le statut de la mission
        """Service d'obtention du statut"""
        try:
            with self._state_lock:
                # Créer le message de statut
                status = MissionStatus()
                status.stamp = self.get_clock().now().to_msg()
                status.status = self._mission_state.value
                status.mode = self._mission_mode.value
                status.current_waypoint = self._current_waypoint_index
                status.total_waypoints = len(self._waypoints)
                status.progress = self._mission_progress
                status.message = self._get_status_message()
                
                response.status = status
                response.success = True
                response.message = "Statut obtenu"
                
        except Exception as e:
            response.success = False
            response.message = f"Erreur obtention statut: {e}"
            
        return response

    def _handle_set_mode(self, request, response):
        # Service : change le mode de la mission
        """Service de changement de mode"""
        try:
            new_mode = MissionMode(request.mode)
            with self._state_lock:
                if self._mission_state == MissionState.RUNNING:
                    response.success = False
                    response.message = "Impossible de changer le mode pendant une mission"
                else:
                    self._mission_mode = new_mode
                    response.success = True
                    response.message = f"Mode changé vers {new_mode.value}"
                    
        except ValueError:
            response.success = False
            response.message = f"Mode invalide: {request.mode}"
        except Exception as e:
            response.success = False
            response.message = f"Erreur changement mode: {e}"
            
        return response

    def _handle_set_waypoints(self, request, response):
        # Service : définit la liste des waypoints de la mission
        """Service de définition des waypoints"""
        try:
            with self._state_lock:
                if self._mission_state == MissionState.RUNNING:
                    response.success = False
                    response.message = "Impossible de modifier les waypoints pendant une mission"
                else:
                    self._waypoints = list(request.waypoints)
                    self._total_waypoints = len(self._waypoints)
                    self._current_waypoint_index = 0
                    response.success = True
                    response.message = f"{len(self._waypoints)} waypoints définis"
                    
        except Exception as e:
            response.success = False
            response.message = f"Erreur définition waypoints: {e}"
            
        return response

    def _handle_get_waypoints(self, request, response):
        # Service : obtient la liste des waypoints de la mission
        """Service d'obtention des waypoints"""
        try:
            with self._state_lock:
                response.waypoints = list(self._waypoints)
                response.success = True
                response.message = f"{len(self._waypoints)} waypoints"
                
        except Exception as e:
            response.success = False
            response.message = f"Erreur obtention waypoints: {e}"
            
        return response

    def _handle_add_waypoint(self, request, response):
        # Service : ajoute un waypoint à la mission
        """Service d'ajout de waypoint"""
        try:
            with self._state_lock:
                if self._mission_state == MissionState.RUNNING:
                    response.success = False
                    response.message = "Impossible d'ajouter des waypoints pendant une mission"
                else:
                    index = min(request.index, len(self._waypoints))
                    self._waypoints.insert(index, request.waypoint)
                    self._total_waypoints = len(self._waypoints)
                    response.success = True
                    response.message = f"Waypoint ajouté à l'index {index}"
                    
        except Exception as e:
            response.success = False
            response.message = f"Erreur ajout waypoint: {e}"
            
        return response

    def _handle_remove_waypoint(self, request, response):
        # Service : supprime un waypoint de la mission
        """Service de suppression de waypoint"""
        try:
            with self._state_lock:
                if self._mission_state == MissionState.RUNNING:
                    response.success = False
                    response.message = "Impossible de supprimer des waypoints pendant une mission"
                elif 0 <= request.index < len(self._waypoints):
                    removed = self._waypoints.pop(request.index)
                    self._total_waypoints = len(self._waypoints)
                    response.success = True
                    response.message = f"Waypoint {request.index} supprimé"
                else:
                    response.success = False
                    response.message = f"Index invalide: {request.index}"
                    
        except Exception as e:
            response.success = False
            response.message = f"Erreur suppression waypoint: {e}"
            
        return response

    def _handle_clear_waypoints(self, request, response):
        # Service : supprime tous les waypoints de la mission
        """Service de suppression de tous les waypoints"""
        try:
            with self._state_lock:
                if self._mission_state == MissionState.RUNNING:
                    response.success = False
                    response.message = "Impossible de vider les waypoints pendant une mission"
                else:
                    self._waypoints.clear()
                    self._total_waypoints = 0
                    self._current_waypoint_index = 0
                    response.success = True
                    response.message = "Tous les waypoints supprimés"
                    
        except Exception as e:
            response.success = False
            response.message = f"Erreur suppression waypoints: {e}"
            
        return response

    def destroy_node(self):
        # Nettoyage du nœud : arrêt du ThreadPoolExecutor
        """Nettoyage lors de la destruction du nœud"""
        try:
            if hasattr(self, '_status_timer'):
                self._status_timer.destroy()
            if hasattr(self, '_mission_timer'):
                self._mission_timer.destroy()
            if hasattr(self, '_executor'):
                self._executor.shutdown(wait=True)
            self.get_logger().info("Mission Manager Node destroyed cleanly")
        except Exception as e:
            self.get_logger().error(f"Erreur lors du nettoyage: {e}")
        finally:
            super().destroy_node()

def main(args=None):
    # Point d'entrée principal du nœud de gestion de mission
    rclpy.init(args=args)
    
    node = MissionManagerNode()
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