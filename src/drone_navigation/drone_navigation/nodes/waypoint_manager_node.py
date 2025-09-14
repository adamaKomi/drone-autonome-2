#!/usr/bin/env python3
"""
waypoint_manager_node.py - Nœud de gestion des waypoints
"""

import rclpy
import threading
import json
import os
from concurrent.futures import ThreadPoolExecutor
from datetime import datetime
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point

from drone_msgs.msg import Waypoint, WaypointList, MissionStatus
from drone_msgs.srv import (LoadWaypoints, SaveWaypoints, GetWaypoint, 
                           AddWaypoint, RemoveWaypoint, ClearWaypoints,
                           GetWaypoints, SetWaypoints)

class WaypointManagerNode(Node):
    def __init__(self):
        super().__init__('waypoint_manager_node')
        
        # Thread safety
        self._state_lock = threading.RLock()
        self._publish_lock = threading.RLock()
        self._file_lock = threading.RLock()  # Pour les opérations fichiers
        self._executor = ThreadPoolExecutor(max_workers=2, thread_name_prefix="wp_mgr")
        
        # Configuration protégée
        self._waypoints = []
        self._current_mission_id = None
        self._waypoints_file = None
        self._mission_active = False
        
        # Paramètres
        self.declare_parameter('default_waypoints_dir', './waypoints')
        self.declare_parameter('auto_save', True)
        self.declare_parameter('publish_rate', 2.0)
        
        self._waypoints_dir = self.get_parameter('default_waypoints_dir').value
        self._auto_save = self.get_parameter('auto_save').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Créer le répertoire si nécessaire
        try:
            os.makedirs(self._waypoints_dir, exist_ok=True)
        except Exception as e:
            self.get_logger().error(f"Impossible de créer le répertoire {self._waypoints_dir}: {e}")
            self._waypoints_dir = "./waypoints_fallback"
            os.makedirs(self._waypoints_dir, exist_ok=True)
        
        # Publishers thread-safe
        self._waypoints_list_pub = self.create_publisher(
            WaypointList, '/drone_nav/waypoints_list', 10
        )
        
        # Subscribers
        self.create_subscription(
            MissionStatus, '/drone_nav/mission_status', self._mission_status_callback, 10
        )
        
        # Services de fichiers
        self._load_waypoints_srv = self.create_service(
            LoadWaypoints, '/drone_nav/load_waypoints', self._handle_load_waypoints
        )
        self._save_waypoints_srv = self.create_service(
            SaveWaypoints, '/drone_nav/save_waypoints', self._handle_save_waypoints
        )
        
        # Services de waypoints individuels
        self._get_waypoint_srv = self.create_service(
            GetWaypoint, '/drone_nav/get_waypoint', self._handle_get_waypoint
        )
        self._add_waypoint_srv = self.create_service(
            AddWaypoint, '/drone_nav/add_waypoint_local', self._handle_add_waypoint
        )
        self._remove_waypoint_srv = self.create_service(
            RemoveWaypoint, '/drone_nav/remove_waypoint_local', self._handle_remove_waypoint
        )
        self._clear_waypoints_srv = self.create_service(
            ClearWaypoints, '/drone_nav/clear_waypoints_local', self._handle_clear_waypoints
        )
        
        # Services de gestion de liste
        self._get_waypoints_srv = self.create_service(
            GetWaypoints, '/drone_nav/get_waypoints_local', self._handle_get_waypoints
        )
        self._set_waypoints_srv = self.create_service(
            SetWaypoints, '/drone_nav/set_waypoints_local', self._handle_set_waypoints
        )
        
        # Timer pour publication périodique
        self._publish_timer = self.create_timer(
            1.0 / publish_rate, 
            self._publish_timer_callback
        )
        
        self.get_logger().info("Waypoint Manager Node initialized")

    # Propriétés thread-safe
    @property
    def waypoints_count(self):
        with self._state_lock:
            return len(self._waypoints)
    
    @property
    def mission_active(self):
        with self._state_lock:
            return self._mission_active

    def _mission_status_callback(self, msg):
        """Callback pour le statut de mission"""
        with self._state_lock:
            previous_active = self._mission_active
            self._mission_active = (msg.status in ["RUNNING", "PAUSED"])
            
            # Auto-sauvegarde si mission terminée
            if previous_active and not self._mission_active and self._auto_save:
                if self._waypoints:
                    self._executor.submit(self._auto_save_waypoints)

    def _auto_save_waypoints(self):
        """Sauvegarde automatique des waypoints"""
        try:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            filename = f"waypoints_auto_{timestamp}.json"
            self._save_waypoints_to_file(filename)
            self.get_logger().info(f"Auto-sauvegarde: {filename}")
        except Exception as e:
            self.get_logger().error(f"Erreur auto-sauvegarde: {e}")

    def _publish_timer_callback(self):
        """Callback du timer de publication"""
        try:
            self._executor.submit(self._publish_waypoints_list)
        except Exception as e:
            self.get_logger().error(f"Erreur soumission publication: {e}")

    def _publish_waypoints_list(self):
        """Publication thread-safe de la liste des waypoints"""
        try:
            with self._state_lock:
                msg = WaypointList()
                msg.stamp = self.get_clock().now().to_msg()
                msg.waypoints = list(self._waypoints)
                msg.count = len(self._waypoints)
                msg.current_file = self._waypoints_file or ""
                
            with self._publish_lock:
                self._waypoints_list_pub.publish(msg)
                
        except Exception as e:
            self.get_logger().error(f"Erreur publication liste waypoints: {e}")

    # Handlers de services - Fichiers
    def _handle_load_waypoints(self, request, response):
        """Service de chargement de waypoints depuis fichier"""
        try:
            filename = request.filename.strip()
            if not filename:
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                filename = f"waypoints_default_{timestamp}.json"
            
            success, message, count = self._load_waypoints_from_file(filename)
            
            response.success = success
            response.message = message
            response.count = count
            
            if success:
                self._executor.submit(self._publish_waypoints_list)
                
        except Exception as e:
            response.success = False
            response.message = f"Erreur service load: {e}"
            response.count = 0
            
        return response

    def _handle_save_waypoints(self, request, response):
        """Service de sauvegarde de waypoints vers fichier"""
        try:
            filename = request.filename.strip()
            if not filename:
                timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
                filename = f"waypoints_{timestamp}.json"
            
            success, message, count = self._save_waypoints_to_file(filename)
            
            response.success = success
            response.message = message
            response.count = count
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur service save: {e}"
            response.count = 0
            
        return response

    # Handlers de services - Waypoints individuels
    def _handle_get_waypoint(self, request, response):
        """Service d'obtention d'un waypoint par index"""
        try:
            with self._state_lock:
                index = request.index
                if 0 <= index < len(self._waypoints):
                    response.waypoint = self._waypoints[index]
                    response.success = True
                    response.message = f"Waypoint {index} récupéré"
                else:
                    response.success = False
                    response.message = f"Index invalide: {index} (max: {len(self._waypoints)-1})"
                    
        except Exception as e:
            response.success = False
            response.message = f"Erreur get waypoint: {e}"
            
        return response

    def _handle_add_waypoint(self, request, response):
        """Service d'ajout de waypoint"""
        try:
            with self._state_lock:
                if self._mission_active:
                    response.success = False
                    response.message = "Impossible de modifier pendant une mission active"
                    return response
                
                # Gestion de l'index (-1 = ajouter à la fin)
                if request.index == 4294967295 or request.index >= len(self._waypoints):
                    self._waypoints.append(request.waypoint)
                    index = len(self._waypoints) - 1
                else:
                    index = min(request.index, len(self._waypoints))
                    self._waypoints.insert(index, request.waypoint)
                
                response.success = True
                response.message = f"Waypoint ajouté à l'index {index}"
                
            self._executor.submit(self._publish_waypoints_list)
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur ajout waypoint: {e}"
            
        return response

    def _handle_remove_waypoint(self, request, response):
        """Service de suppression de waypoint"""
        try:
            with self._state_lock:
                if self._mission_active:
                    response.success = False
                    response.message = "Impossible de modifier pendant une mission active"
                    return response
                
                index = request.index
                if 0 <= index < len(self._waypoints):
                    removed = self._waypoints.pop(index)
                    response.success = True
                    response.message = f"Waypoint {index} supprimé"
                else:
                    response.success = False
                    response.message = f"Index invalide: {index}"
                    
            if response.success:
                self._executor.submit(self._publish_waypoints_list)
                
        except Exception as e:
            response.success = False
            response.message = f"Erreur suppression waypoint: {e}"
            
        return response

    def _handle_clear_waypoints(self, request, response):
        """Service de suppression de tous les waypoints"""
        try:
            with self._state_lock:
                if self._mission_active:
                    response.success = False
                    response.message = "Impossible de modifier pendant une mission active"
                    return response
                
                count = len(self._waypoints)
                self._waypoints.clear()
                
                response.success = True
                response.message = f"{count} waypoints supprimés"
                
            self._executor.submit(self._publish_waypoints_list)
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur vidage waypoints: {e}"
            
        return response

    # Handlers de services - Gestion de liste
    def _handle_get_waypoints(self, request, response):
        """Service d'obtention de tous les waypoints"""
        try:
            with self._state_lock:
                response.waypoints = list(self._waypoints)
                response.success = True
                response.message = f"{len(self._waypoints)} waypoints récupérés"
                
        except Exception as e:
            response.success = False
            response.message = f"Erreur get waypoints: {e}"
            
        return response

    def _handle_set_waypoints(self, request, response):
        """Service de définition de la liste complète des waypoints"""
        try:
            with self._state_lock:
                if self._mission_active:
                    response.success = False
                    response.message = "Impossible de modifier pendant une mission active"
                    return response
                
                self._waypoints = list(request.waypoints)
                response.success = True
                response.message = f"{len(self._waypoints)} waypoints définis"
                
            self._executor.submit(self._publish_waypoints_list)
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur set waypoints: {e}"
            
        return response

    # Méthodes de gestion de fichiers
    def _load_waypoints_from_file(self, filename):
        """Charge les waypoints depuis un fichier JSON"""
        try:
            with self._file_lock:
                filepath = os.path.join(self._waypoints_dir, filename)
                
                if not os.path.exists(filepath):
                    return False, f"Fichier non trouvé: {filepath}", 0
                
                with open(filepath, 'r', encoding='utf-8') as f:
                    waypoints_data = json.load(f)
                
                waypoints = []
                for wp_data in waypoints_data:
                    waypoint = Waypoint()
                    
                    # Position (geometry_msgs/Point)
                    waypoint.position = Point()
                    if 'position' in wp_data:
                        waypoint.position.x = float(wp_data['position'].get('x', 0.0))
                        waypoint.position.y = float(wp_data['position'].get('y', 0.0))
                        waypoint.position.z = float(wp_data['position'].get('z', 0.0))
                    elif 'latitude' in wp_data and 'longitude' in wp_data:
                        # Rétrocompatibilité GPS
                        waypoint.position.x = float(wp_data['latitude'])
                        waypoint.position.y = float(wp_data['longitude'])
                        waypoint.position.z = float(wp_data.get('altitude', 0.0))
                    
                    # Autres champs
                    waypoint.tolerance = float(wp_data.get('tolerance', 2.0))
                    waypoint.wp_type = wp_data.get('wp_type', 'NORMAL')
                    waypoint.actions = wp_data.get('actions', [])
                    waypoint.speed = float(wp_data.get('speed', 5.0))
                    waypoint.yaw = float(wp_data.get('yaw', 0.0))
                    waypoint.hold_time = float(wp_data.get('hold_time', 0.0))
                    
                    waypoints.append(waypoint)
                
                with self._state_lock:
                    if self._mission_active:
                        return False, "Impossible de charger pendant une mission active", 0
                    
                    self._waypoints = waypoints
                    self._waypoints_file = filename
                
                return True, f"Waypoints chargés depuis {filename}", len(waypoints)
                
        except json.JSONDecodeError as e:
            return False, f"Fichier JSON invalide: {e}", 0
        except Exception as e:
            return False, f"Erreur chargement: {e}", 0

    def _save_waypoints_to_file(self, filename):
        """Sauvegarde les waypoints vers un fichier JSON"""
        try:
            with self._file_lock:
                filepath = os.path.join(self._waypoints_dir, filename)
                
                with self._state_lock:
                    waypoints_data = []
                    for wp in self._waypoints:
                        wp_data = {
                            'position': {
                                'x': float(wp.position.x),
                                'y': float(wp.position.y),
                                'z': float(wp.position.z)
                            },
                            'tolerance': float(wp.tolerance),
                            'wp_type': wp.wp_type,
                            'actions': list(wp.actions),
                            'speed': float(wp.speed),
                            'yaw': float(wp.yaw),
                            'hold_time': float(wp.hold_time)
                        }
                        waypoints_data.append(wp_data)
                    
                    count = len(waypoints_data)
                
                # Sauvegarde avec backup temporaire
                temp_filepath = filepath + '.tmp'
                with open(temp_filepath, 'w', encoding='utf-8') as f:
                    json.dump(waypoints_data, f, indent=2, ensure_ascii=False)
                
                # Remplacer le fichier original
                if os.path.exists(filepath):
                    os.replace(temp_filepath, filepath)
                else:
                    os.rename(temp_filepath, filepath)
                
                with self._state_lock:
                    self._waypoints_file = filename
                
                return True, f"Waypoints sauvegardés dans {filename}", count
                
        except Exception as e:
            return False, f"Erreur sauvegarde: {e}", 0

    # Méthodes publiques utilitaires
    def get_waypoint(self, index):
        """Obtenir un waypoint par index (méthode publique)"""
        with self._state_lock:
            if 0 <= index < len(self._waypoints):
                return self._waypoints[index]
            return None

    def get_waypoints_count(self):
        """Obtenir le nombre de waypoints (méthode publique)"""
        return self.waypoints_count

    def get_waypoints_list(self):
        """Obtenir une copie de la liste des waypoints (méthode publique)"""
        with self._state_lock:
            return list(self._waypoints)

    def destroy_node(self):
        """Nettoyage lors de la destruction du nœud"""
        try:
            if hasattr(self, '_publish_timer'):
                self._publish_timer.destroy()
            if hasattr(self, '_executor'):
                self._executor.shutdown(wait=True)
            self.get_logger().info("Waypoint Manager Node destroyed cleanly")
        except Exception as e:
            self.get_logger().error(f"Erreur lors du nettoyage: {e}")
        finally:
            super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = WaypointManagerNode()
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