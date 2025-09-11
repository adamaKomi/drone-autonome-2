#!/usr/bin/env python3
"""
Core Navigation Node - Orchestrateur principal du système de navigation
Auteur: Adama Komi
Date: 2025-09-11
Version: 2.0.0

Ce nœud est l'orchestrateur principal du système de navigation autonome.
Il coordonne les micro-nœuds spécialisés et gère les états globaux du système.
Il ne duplique pas les fonctionnalités des autres nœuds mais les coordonne.

Responsabilités:
- Orchestration des nœuds de navigation (trajectory_planner, position_controller, etc.)
- Gestion des états globaux de navigation (IDLE, PLANNING, NAVIGATING, etc.)
- Coordination des missions complexes
- Surveillance de la santé du système
- Interface de haut niveau pour les missions

Références:
- ROS2 Node Design: https://docs.ros.org/en/humble/Concepts/About-Nodes.html
- System Architecture: https://design.ros2.org/articles/composition.html
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Messages ROS2
from std_msgs.msg import String, Bool
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path

# Services ROS2
from std_srvs.srv import Trigger, SetBool

import threading
import time
import json
from typing import Dict, Any, Optional, List
from dataclasses import dataclass
from enum import Enum


class NavigationState(Enum):
    """États de navigation du système"""
    IDLE = "IDLE"                    # Système inactif, prêt à recevoir des missions
    INITIALIZING = "INITIALIZING"    # Initialisation des nœuds et vérifications
    READY = "READY"                  # Prêt à naviguer, tous les nœuds opérationnels
    PLANNING = "PLANNING"            # Planification de trajectoire en cours
    NAVIGATING = "NAVIGATING"        # Navigation active vers un objectif
    PAUSED = "PAUSED"               # Navigation mise en pause
    MISSION_COMPLETE = "COMPLETE"    # Mission terminée avec succès
    EMERGENCY = "EMERGENCY"          # Mode d'urgence activé
    ERROR = "ERROR"                  # Erreur système nécessitant intervention


class SystemHealth(Enum):
    """Niveaux de santé du système de navigation"""
    HEALTHY = "HEALTHY"        # Tous les nœuds opérationnels
    WARNING = "WARNING"        # Problèmes mineurs détectés
    CRITICAL = "CRITICAL"      # Problèmes majeurs, arrêt recommandé
    UNKNOWN = "UNKNOWN"        # État de santé indéterminé


@dataclass
class NavigationStatus:
    """Structure du statut de navigation"""
    state: NavigationState = NavigationState.IDLE
    health: SystemHealth = SystemHealth.UNKNOWN
    
    # Informations de mission
    mission_active: bool = False
    mission_id: str = ""
    mission_progress: float = 0.0
    current_waypoint: int = 0
    total_waypoints: int = 0
    estimated_completion_time: float = 0.0
    
    # Informations de navigation
    current_target: Optional[Point] = None
    navigation_accuracy: float = float('inf')
    
    # Métriques de performance
    nodes_operational: int = 0
    total_nodes: int = 0
    average_response_time: float = 0.0
    
    # Timestamps
    last_update: float = 0.0
    mission_start_time: float = 0.0
    state_change_time: float = 0.0


class CoreNavigationNode(Node):
    """
    Nœud principal d'orchestration de navigation
    
    Ce nœud coordonne les micro-nœuds spécialisés sans dupliquer leurs fonctionnalités:
    - Orchestration des missions de navigation complexes
    - Gestion des états globaux du système de navigation
    - Coordination entre trajectory_planner, position_controller, etc.
    - Surveillance de la santé des nœuds de navigation
    - Interface haut niveau pour les missions utilisateur
    
    Note: Ne gère PAS les commandes drone directes (cf. drone_interface)
          Ne gère PAS les données MAVROS directes (cf. mavros_interface_node)
    """
    
    def __init__(self, node_name: str = 'core_navigation_node'):
        super().__init__(node_name, namespace='drone_nav')
        
        # Configuration QoS
        self.qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # État du système de navigation
        self.nav_status = NavigationStatus()
        self.status_lock = threading.RLock()
        
        # État des micro-nœuds de navigation
        self.navigation_nodes_status = {
            'trajectory_planner_node': {'active': False, 'last_seen': 0.0, 'health': 'UNKNOWN'},
            'position_controller_node': {'active': False, 'last_seen': 0.0, 'health': 'UNKNOWN'},
            'path_optimizer_node': {'active': False, 'last_seen': 0.0, 'health': 'UNKNOWN'},
            'trajectory_follower_node': {'active': False, 'last_seen': 0.0, 'health': 'UNKNOWN'}
        }
        
        # Mission courante
        self.current_mission = None
        self.mission_waypoints = []
        self.mission_queue = []
        
        # Déclarer les paramètres
        self._declare_parameters()
        
        # Setup des composants
        self._setup_publishers()
        self._setup_subscribers()
        self._setup_services()
        self._setup_timers()
        
        # Initialisation
        with self.status_lock:
            self.nav_status.state = NavigationState.INITIALIZING
            self.nav_status.last_update = time.time()
            self.nav_status.state_change_time = time.time()
        
        self.get_logger().info("Core Navigation Node initialisé - Mode orchestrateur")
        self._transition_to_ready()
    
    def _declare_parameters(self):
        """Déclare les paramètres du nœud"""
        # Fréquences de fonctionnement
        self.declare_parameter('status_frequency', 2.0)
        self.declare_parameter('health_check_frequency', 1.0)
        self.declare_parameter('coordination_frequency', 10.0)
        
        # Timeouts et tolérances
        self.declare_parameter('node_timeout', 5.0)
        self.declare_parameter('mission_timeout', 300.0)
        self.declare_parameter('waypoint_tolerance', 1.0)
        
        # Modes de fonctionnement
        self.declare_parameter('auto_recovery_enabled', True)
        self.declare_parameter('mission_queue_enabled', True)
        self.declare_parameter('parallel_planning_enabled', False)
    def _setup_publishers(self):
        """Configure les publishers"""
        # État global du système de navigation
        self.system_state_pub = self.create_publisher(
            String,
            'navigation_system_state',
            self.qos_reliable
        )
        
        # Statut détaillé de navigation
        self.navigation_status_pub = self.create_publisher(
            String,
            'navigation_status',
            self.qos_reliable
        )
        
        # Commandes de coordination pour les micro-nœuds
        self.coordination_cmd_pub = self.create_publisher(
            String,
            'coordination_commands',
            self.qos_reliable
        )
        
        # Missions actives (pour suivi externe)
        self.mission_status_pub = self.create_publisher(
            String,
            'mission_status',
            self.qos_reliable
        )
    
    def _setup_subscribers(self):
        """Configure les subscribers"""
        # Statut des nœuds de navigation
        self.nav_nodes_status_sub = self.create_subscription(
            String,
            'navigation_nodes_status',
            self._nav_nodes_status_callback,
            self.qos_reliable
        )
        
        # Position actuelle (depuis mavros_interface)
        self.position_sub = self.create_subscription(
            PoseStamped,
            'position',
            self._position_callback,
            self.qos_sensor
        )
        
        # Statut de sécurité (depuis mavros_interface)
        self.safety_status_sub = self.create_subscription(
            String,
            'safety_status',
            self._safety_status_callback,
            self.qos_reliable
        )
        
        # Feedback des micro-nœuds de navigation
        self.planner_feedback_sub = self.create_subscription(
            String,
            'trajectory_planner_feedback',
            self._planner_feedback_callback,
            self.qos_reliable
        )
        
        self.controller_feedback_sub = self.create_subscription(
            String,
            'position_controller_feedback',
            self._controller_feedback_callback,
            self.qos_reliable
        )
    
    def _setup_services(self):
        """Configure les services du nœud"""
        # Gestion des missions
        self.start_mission_service = self.create_service(
            Trigger,
            'start_mission',
            self._handle_start_mission
        )
        
        self.stop_mission_service = self.create_service(
            Trigger,
            'stop_mission',
            self._handle_stop_mission
        )
        
        self.pause_mission_service = self.create_service(
            SetBool,
            'pause_mission',
            self._handle_pause_mission
        )
        
        # Navigation vers point
        self.navigate_to_point_service = self.create_service(
            String,  # JSON avec coordinates
            'navigate_to_point',
            self._handle_navigate_to_point
        )
        
        # Statut du système de navigation
        self.get_navigation_status_service = self.create_service(
            Trigger,
            'get_navigation_status',
            self._handle_get_navigation_status
        )
        
        # Gestion des micro-nœuds
        self.restart_navigation_nodes_service = self.create_service(
            Trigger,
            'restart_navigation_nodes',
            self._handle_restart_navigation_nodes
        )
        
        # Arrêt d'urgence de navigation uniquement
        self.emergency_stop_navigation_service = self.create_service(
            Trigger,
            'emergency_stop_navigation',
            self._handle_emergency_stop_navigation
        )
    
    def _setup_timers(self):
        """Configure les timers"""
        # Timer pour publication du statut
        status_freq = self.get_parameter('status_frequency').value
        self.status_timer = self.create_timer(
            1.0 / status_freq,
            self._publish_status
        )
        
        # Timer pour vérification de santé des nœuds de navigation
        health_freq = self.get_parameter('health_check_frequency').value
        self.health_timer = self.create_timer(
            1.0 / health_freq,
            self._check_navigation_nodes_health
        )
        
        # Timer pour coordination des micro-nœuds
        coord_freq = self.get_parameter('coordination_frequency').value
        self.coordination_timer = self.create_timer(
            1.0 / coord_freq,
            self._coordinate_navigation_nodes
        )
    
    def _transition_to_ready(self):
        """Transition vers l'état READY après vérifications"""
        try:
            # Vérifier les nœuds requis
            self._check_navigation_nodes_availability()
            
            # Transition
            with self.status_lock:
                self.nav_status.state = NavigationState.READY
                self.nav_status.health = SystemHealth.HEALTHY
                self.nav_status.state_change_time = time.time()
                self.nav_status.last_update = time.time()
            
            self.get_logger().info("Transition vers état READY - Système de navigation opérationnel")
            
        except Exception as e:
            self.get_logger().error(f"Erreur durant transition vers READY: {e}")
            with self.status_lock:
                self.nav_status.state = NavigationState.ERROR
                self.nav_status.health = SystemHealth.CRITICAL
    
    # Callbacks
    def _nav_nodes_status_callback(self, msg: String):
        """Callback pour le statut des nœuds de navigation"""
        try:
            status_data = json.loads(msg.data)
            node_name = status_data.get('node_name', 'unknown')
            
            if node_name in self.navigation_nodes_status:
                self.navigation_nodes_status[node_name] = {
                    'active': status_data.get('active', False),
                    'last_seen': time.time(),
                    'health': status_data.get('health', 'UNKNOWN'),
                    'performance': status_data.get('performance', {}),
                    'error_count': status_data.get('error_count', 0)
                }
                
                self.get_logger().debug(f"Statut nœud navigation {node_name}: {status_data.get('health', 'UNKNOWN')}")
        
        except Exception as e:
            self.get_logger().error(f"Erreur traitement statut nœud navigation: {e}")
    
    def _position_callback(self, msg: PoseStamped):
        """Callback pour la position actuelle (depuis mavros_interface)"""
        with self.status_lock:
            current_pos = msg.pose.position
            
            # Calculer la précision de navigation si on a une cible
            if self.nav_status.current_target is not None:
                distance = self._calculate_distance_3d(current_pos, self.nav_status.current_target)
                self.nav_status.navigation_accuracy = distance
                
                # Vérifier si le waypoint est atteint
                tolerance = self.get_parameter('waypoint_tolerance').value
                if distance <= tolerance and self.nav_status.state == NavigationState.NAVIGATING:
                    self._handle_waypoint_reached()
            
            self.nav_status.last_update = time.time()
    
    def _safety_status_callback(self, msg: String):
        """Callback pour les alertes de sécurité (depuis mavros_interface)"""
        try:
            safety_data = json.loads(msg.data)
            safety_level = safety_data.get('level', 'NORMAL')
            
            if safety_level == 'CRITICAL':
                self.get_logger().error("Alerte de sécurité critique reçue - Arrêt navigation")
                self._trigger_emergency_navigation_stop()
            elif safety_level == 'WARNING':
                self.get_logger().warning(f"Alerte de sécurité: {safety_data}")
                # Possibilité de ralentir ou modifier la navigation
                
        except Exception as e:
            self.get_logger().error(f"Erreur traitement alerte sécurité: {e}")
    
    def _planner_feedback_callback(self, msg: String):
        """Callback pour le feedback du planificateur de trajectoires"""
        try:
            feedback_data = json.loads(msg.data)
            
            if feedback_data.get('status') == 'PLANNING_COMPLETE':
                with self.status_lock:
                    if self.nav_status.state == NavigationState.PLANNING:
                        self.nav_status.state = NavigationState.NAVIGATING
                        self.nav_status.state_change_time = time.time()
                        self.get_logger().info("Planification terminée - Début navigation")
            
            elif feedback_data.get('status') == 'PLANNING_FAILED':
                with self.status_lock:
                    self.nav_status.state = NavigationState.ERROR
                    self.nav_status.state_change_time = time.time()
                    self.get_logger().error("Échec de planification de trajectoire")
                    
        except Exception as e:
            self.get_logger().error(f"Erreur traitement feedback planificateur: {e}")
    
    def _controller_feedback_callback(self, msg: String):
        """Callback pour le feedback du contrôleur de position"""
        try:
            feedback_data = json.loads(msg.data)
            
            # Mettre à jour les métriques de performance
            with self.status_lock:
                self.nav_status.average_response_time = feedback_data.get('response_time', 0.0)
                
            # Traiter les événements du contrôleur
            if feedback_data.get('event') == 'TARGET_REACHED':
                self._handle_waypoint_reached()
            elif feedback_data.get('event') == 'CONTROL_ERROR':
                self.get_logger().warning(f"Erreur de contrôle: {feedback_data.get('message', '')}")
                
        except Exception as e:
            self.get_logger().error(f"Erreur traitement feedback contrôleur: {e}")
    
    # Gestionnaires de services
    def _handle_start_mission(self, request, response):
        """Gestionnaire pour démarrer une mission de navigation"""
        try:
            with self.status_lock:
                if self.nav_status.state not in [NavigationState.READY, NavigationState.PAUSED]:
                    response.success = False
                    response.message = f"Mission impossible depuis l'état: {self.nav_status.state.value}"
                    return response
                
                # Démarrer la coordination de navigation
                self.nav_status.state = NavigationState.PLANNING
                self.nav_status.mission_active = True
                self.nav_status.mission_start_time = time.time()
                self.nav_status.state_change_time = time.time()
                self.nav_status.last_update = time.time()
            
            # Coordonner les micro-nœuds pour démarrer
            self._coordinate_mission_start()
            
            response.success = True
            response.message = "Mission de navigation démarrée"
            self.get_logger().info("Mission de navigation démarrée - Coordination des micro-nœuds")
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
            self.get_logger().error(f"Erreur lors du démarrage de mission: {e}")
        
        return response
    
    def _handle_stop_mission(self, request, response):
        """Gestionnaire pour arrêter la mission de navigation"""
        try:
            self._stop_current_navigation_mission()
            
            response.success = True
            response.message = "Mission de navigation arrêtée"
            self.get_logger().info("Mission de navigation arrêtée")
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
            self.get_logger().error(f"Erreur lors de l'arrêt de mission: {e}")
        
        return response
    
    def _handle_pause_mission(self, request, response):
        """Gestionnaire pour pause/reprise de mission"""
        try:
            with self.status_lock:
                if request.data:  # Pause
                    if self.nav_status.state == NavigationState.NAVIGATING:
                        self.nav_status.state = NavigationState.PAUSED
                        self.nav_status.state_change_time = time.time()
                        response.success = True
                        response.message = "Mission mise en pause"
                        self._coordinate_mission_pause()
                        self.get_logger().info("Mission de navigation mise en pause")
                    else:
                        response.success = False
                        response.message = f"Pause impossible depuis l'état: {self.nav_status.state.value}"
                else:  # Reprise
                    if self.nav_status.state == NavigationState.PAUSED:
                        self.nav_status.state = NavigationState.NAVIGATING
                        self.nav_status.state_change_time = time.time()
                        response.success = True
                        response.message = "Mission reprise"
                        self._coordinate_mission_resume()
                        self.get_logger().info("Mission de navigation reprise")
                    else:
                        response.success = False
                        response.message = f"Reprise impossible depuis l'état: {self.nav_status.state.value}"
                
                self.nav_status.last_update = time.time()
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
            self.get_logger().error(f"Erreur lors de pause/reprise: {e}")
        
        return response
    
    def _handle_navigate_to_point(self, request, response):
        """Gestionnaire pour navigation vers un point"""
        try:
            # Parser les coordonnées JSON
            coords_data = json.loads(request.data)
            target_point = Point()
            target_point.x = float(coords_data['x'])
            target_point.y = float(coords_data['y'])
            target_point.z = float(coords_data['z'])
            
            # Valider la position cible
            if not self._is_valid_navigation_target(target_point):
                response.success = False
                response.message = "Position cible invalide"
                return response
            
            with self.status_lock:
                # Sauvegarder la cible
                self.nav_status.current_target = target_point
                self.nav_status.state = NavigationState.PLANNING
                self.nav_status.state_change_time = time.time()
                self.nav_status.last_update = time.time()
            
            # Coordonner la planification et navigation
            self._coordinate_navigate_to_point(target_point)
            
            response.success = True
            response.message = f"Navigation vers ({target_point.x:.2f}, {target_point.y:.2f}, {target_point.z:.2f}) initiée"
            self.get_logger().info(f"Navigation vers point: ({target_point.x:.2f}, {target_point.y:.2f}, {target_point.z:.2f})")
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
            self.get_logger().error(f"Erreur lors de navigate_to_point: {e}")
        
        return response
    
    def _handle_get_navigation_status(self, request, response):
        """Gestionnaire pour obtenir le statut du système de navigation"""
        try:
            with self.status_lock:
                status_dict = {
                    'navigation_state': self.nav_status.state.value,
                    'system_health': self.nav_status.health.value,
                    'mission_active': self.nav_status.mission_active,
                    'mission_id': self.nav_status.mission_id,
                    'mission_progress': self.nav_status.mission_progress,
                    'current_waypoint': self.nav_status.current_waypoint,
                    'total_waypoints': self.nav_status.total_waypoints,
                    'current_target': {
                        'x': self.nav_status.current_target.x,
                        'y': self.nav_status.current_target.y,
                        'z': self.nav_status.current_target.z
                    } if self.nav_status.current_target else None,
                    'navigation_accuracy': self.nav_status.navigation_accuracy,
                    'estimated_completion_time': self.nav_status.estimated_completion_time,
                    'nodes_operational': self.nav_status.nodes_operational,
                    'total_nodes': self.nav_status.total_nodes,
                    'average_response_time': self.nav_status.average_response_time,
                    'navigation_nodes_status': self.navigation_nodes_status,
                    'timestamp': time.time()
                }
            
            response.success = True
            response.message = json.dumps(status_dict, indent=2)
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
        
        return response
    
    def _handle_restart_navigation_nodes(self, request, response):
        """Gestionnaire pour redémarrer les nœuds de navigation"""
        try:
            self.get_logger().info("Redémarrage des nœuds de navigation demandé")
            
            # Envoyer commande de redémarrage aux micro-nœuds
            restart_cmd = {
                'command': 'RESTART',
                'timestamp': time.time(),
                'requester': 'core_navigation_node'
            }
            
            cmd_msg = String()
            cmd_msg.data = json.dumps(restart_cmd)
            self.coordination_cmd_pub.publish(cmd_msg)
            
            response.success = True
            response.message = "Commande de redémarrage envoyée aux nœuds de navigation"
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
            self.get_logger().error(f"Erreur redémarrage nœuds navigation: {e}")
        
        return response
    
    def _handle_emergency_stop_navigation(self, request, response):
        """Gestionnaire pour l'arrêt d'urgence de navigation uniquement"""
        try:
            self.get_logger().warning("ARRÊT D'URGENCE NAVIGATION ACTIVÉ")
            
            # Arrêter immédiatement la navigation
            self._trigger_emergency_navigation_stop()
            
            response.success = True
            response.message = "Arrêt d'urgence navigation activé - Navigation arrêtée"
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
            self.get_logger().error(f"Erreur lors de l'arrêt d'urgence navigation: {e}")
        
        return response
    
    # Méthodes de coordination des micro-nœuds
    def _coordinate_mission_start(self):
        """Coordonne le démarrage de mission avec les micro-nœuds"""
        coord_cmd = {
            'command': 'START_MISSION',
            'mission_id': self.nav_status.mission_id,
            'timestamp': time.time()
        }
        
        cmd_msg = String()
        cmd_msg.data = json.dumps(coord_cmd)
        self.coordination_cmd_pub.publish(cmd_msg)
        
        self.get_logger().debug("Commande START_MISSION envoyée aux micro-nœuds")
    
    def _coordinate_mission_pause(self):
        """Coordonne la pause de mission avec les micro-nœuds"""
        coord_cmd = {
            'command': 'PAUSE_MISSION',
            'timestamp': time.time()
        }
        
        cmd_msg = String()
        cmd_msg.data = json.dumps(coord_cmd)
        self.coordination_cmd_pub.publish(cmd_msg)
    
    def _coordinate_mission_resume(self):
        """Coordonne la reprise de mission avec les micro-nœuds"""
        coord_cmd = {
            'command': 'RESUME_MISSION',
            'timestamp': time.time()
        }
        
        cmd_msg = String()
        cmd_msg.data = json.dumps(coord_cmd)
        self.coordination_cmd_pub.publish(cmd_msg)
    
    def _coordinate_navigate_to_point(self, target_point: Point):
        """Coordonne la navigation vers un point avec les micro-nœuds"""
        coord_cmd = {
            'command': 'NAVIGATE_TO_POINT',
            'target': {
                'x': target_point.x,
                'y': target_point.y,
                'z': target_point.z
            },
            'timestamp': time.time()
        }
        
        cmd_msg = String()
        cmd_msg.data = json.dumps(coord_cmd)
        self.coordination_cmd_pub.publish(cmd_msg)
        
        self.get_logger().debug(f"Commande NAVIGATE_TO_POINT envoyée: ({target_point.x:.2f}, {target_point.y:.2f}, {target_point.z:.2f})")
    
    def _coordinate_navigation_nodes(self):
        """Timer de coordination périodique des micro-nœuds"""
        try:
            # Publier le statut de coordination
            coord_status = {
                'coordinator': 'core_navigation_node',
                'navigation_state': self.nav_status.state.value,
                'mission_active': self.nav_status.mission_active,
                'current_target': {
                    'x': self.nav_status.current_target.x,
                    'y': self.nav_status.current_target.y,
                    'z': self.nav_status.current_target.z
                } if self.nav_status.current_target else None,
                'timestamp': time.time()
            }
            
            coord_msg = String()
            coord_msg.data = json.dumps(coord_status)
            self.coordination_cmd_pub.publish(coord_msg)
            
        except Exception as e:
            self.get_logger().error(f"Erreur coordination micro-nœuds: {e}")
    
    # Méthodes utilitaires
    def _stop_current_navigation_mission(self):
        """Arrête la mission de navigation en cours"""
        with self.status_lock:
            self.nav_status.state = NavigationState.READY
            self.nav_status.mission_active = False
            self.nav_status.current_target = None
            self.nav_status.mission_progress = 0.0
            self.nav_status.current_waypoint = 0
            self.nav_status.state_change_time = time.time()
            self.nav_status.last_update = time.time()
        
        # Coordonner l'arrêt avec les micro-nœuds
        coord_cmd = {
            'command': 'STOP_MISSION',
            'timestamp': time.time()
        }
        
        cmd_msg = String()
        cmd_msg.data = json.dumps(coord_cmd)
        self.coordination_cmd_pub.publish(cmd_msg)
    
    def _trigger_emergency_navigation_stop(self):
        """Déclenche l'arrêt d'urgence de navigation uniquement"""
        with self.status_lock:
            self.nav_status.state = NavigationState.EMERGENCY
            self.nav_status.health = SystemHealth.CRITICAL
            self.nav_status.mission_active = False
            self.nav_status.current_target = None
            self.nav_status.state_change_time = time.time()
            self.nav_status.last_update = time.time()
        
        # Arrêt d'urgence des micro-nœuds de navigation uniquement
        emergency_cmd = {
            'command': 'EMERGENCY_STOP_NAVIGATION',
            'level': 'CRITICAL',
            'timestamp': time.time()
        }
        
        cmd_msg = String()
        cmd_msg.data = json.dumps(emergency_cmd)
        self.coordination_cmd_pub.publish(cmd_msg)
        
        self.get_logger().error("ARRÊT D'URGENCE NAVIGATION - Micro-nœuds arrêtés")
    
    def _handle_waypoint_reached(self):
        """Gère l'atteinte d'un waypoint"""
        with self.status_lock:
            self.nav_status.current_waypoint += 1
            
            if self.nav_status.current_waypoint >= self.nav_status.total_waypoints:
                # Mission terminée
                self.nav_status.state = NavigationState.MISSION_COMPLETE
                self.nav_status.mission_active = False
                self.nav_status.mission_progress = 100.0
                self.get_logger().info("🎉 Mission de navigation terminée avec succès")
            else:
                # Waypoint suivant
                self.nav_status.mission_progress = (self.nav_status.current_waypoint / self.nav_status.total_waypoints) * 100.0
                self.get_logger().info(f"✅ Waypoint {self.nav_status.current_waypoint}/{self.nav_status.total_waypoints} atteint")
            
            self.nav_status.state_change_time = time.time()
            self.nav_status.last_update = time.time()
    
    def _is_valid_navigation_target(self, target: Point) -> bool:
        """Vérifie si une cible de navigation est valide"""
        if target is None:
            return False
        
        # Vérifier les limites de navigation (exemple)
        if abs(target.x) > 1000 or abs(target.y) > 1000:
            return False
        
        if target.z < 2.0 or target.z > 120.0:  # Limites d'altitude
            return False
        
        return True
    
    def _calculate_distance_3d(self, pos1: Point, pos2: Point) -> float:
        """Calcule la distance 3D entre deux points"""
        if pos1 is None or pos2 is None:
            return float('inf')
        
        dx = pos2.x - pos1.x
        dy = pos2.y - pos1.y
        dz = pos2.z - pos1.z
        
        return (dx*dx + dy*dy + dz*dz) ** 0.5
    
    def _check_navigation_nodes_availability(self):
        """Vérifie la disponibilité des nœuds de navigation"""
        available_count = 0
        total_count = len(self.navigation_nodes_status)
        
        for node_name, status in self.navigation_nodes_status.items():
            if status.get('active', False):
                available_count += 1
                self.get_logger().debug(f"Nœud navigation {node_name} disponible")
            else:
                self.get_logger().warning(f"Nœud navigation {node_name} non disponible")
        
        with self.status_lock:
            self.nav_status.nodes_operational = available_count
            self.nav_status.total_nodes = total_count
        
        if available_count < total_count:
            self.get_logger().warning(f"Seulement {available_count}/{total_count} nœuds de navigation disponibles")
    
    def _check_navigation_nodes_health(self):
        """Vérifie la santé des nœuds de navigation"""
        try:
            warnings = []
            errors = []
            current_time = time.time()
            timeout = self.get_parameter('node_timeout').value
            
            operational_count = 0
            
            for node_name, node_info in self.navigation_nodes_status.items():
                last_seen = node_info.get('last_seen', 0.0)
                health = node_info.get('health', 'UNKNOWN')
                
                if current_time - last_seen > timeout:
                    errors.append(f"Nœud navigation {node_name} non responsive")
                    node_info['active'] = False
                elif health == 'WARNING':
                    warnings.append(f"Nœud navigation {node_name} en avertissement")
                    operational_count += 1
                elif health == 'CRITICAL':
                    errors.append(f"Nœud navigation {node_name} en état critique")
                else:
                    operational_count += 1
            
            # Mettre à jour la santé globale du système de navigation
            with self.status_lock:
                self.nav_status.nodes_operational = operational_count
                
                if errors:
                    self.nav_status.health = SystemHealth.CRITICAL
                elif warnings:
                    self.nav_status.health = SystemHealth.WARNING
                else:
                    self.nav_status.health = SystemHealth.HEALTHY
            
            # Logger les problèmes
            if errors:
                self.get_logger().error(f"Erreurs nœuds navigation: {', '.join(errors)}")
            if warnings:
                self.get_logger().warning(f"Avertissements nœuds navigation: {', '.join(warnings)}")
                
        except Exception as e:
            self.get_logger().error(f"Erreur vérification santé nœuds navigation: {e}")
    
    def _publish_status(self):
        """Publie le statut périodiquement"""
        try:
            with self.status_lock:
                # État du système
                system_state = {
                    'state': self.nav_status.state.value,
                    'health': self.nav_status.health.value,
                    'timestamp': time.time()
                }
                
                # Statut de navigation détaillé
                nav_status = {
                    'navigation_state': self.nav_status.state.value,
                    'mission_active': self.nav_status.mission_active,
                    'mission_progress': self.nav_status.mission_progress,
                    'current_waypoint': self.nav_status.current_waypoint,
                    'total_waypoints': self.nav_status.total_waypoints,
                    'navigation_accuracy': self.nav_status.navigation_accuracy,
                    'flight_time': self.nav_status.flight_time,
                    'distance_traveled': self.nav_status.distance_traveled,
                    'timestamp': time.time()
                }
            
            # Publier via les lifecycle publishers
            if (hasattr(self, 'system_state_pub') and 
                self.system_state_pub.is_activated):
                state_msg = String()
                state_msg.data = json.dumps(system_state)
                self.system_state_pub.publish(state_msg)
            
            if (hasattr(self, 'navigation_status_pub') and 
                self.navigation_status_pub.is_activated):
                status_msg = String()
                status_msg.data = json.dumps(nav_status)
                self.navigation_status_pub.publish(status_msg)
                
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la publication du statut: {e}")
    
    def _check_system_health(self):
        """Vérifie la santé du système"""
        try:
            warnings = []
            errors = []
            
            # Vérifier les nœuds requis
            current_time = time.time()
            timeout = self.get_parameter('node_timeout').value
            
            for node_name, node_info in self.nodes_status.items():
                if current_time - node_info['last_seen'] > timeout:
                    errors.append(f"Nœud {node_name} non disponible")
                elif node_info['health'] == 'WARNING':
                    warnings.append(f"Nœud {node_name} en avertissement")
                elif node_info['health'] == 'CRITICAL':
                    errors.append(f"Nœud {node_name} en état critique")
            
            # Déterminer la santé globale
            with self.status_lock:
                if errors:
                    self.nav_status.health = SystemHealth.CRITICAL
                elif warnings:
                    self.nav_status.health = SystemHealth.WARNING
                else:
                    self.nav_status.health = SystemHealth.HEALTHY
            
            # Logger les problèmes
            if errors:
                self.get_logger().error(f"Erreurs système: {', '.join(errors)}")
            if warnings:
                self.get_logger().warning(f"Avertissements système: {', '.join(warnings)}")
                
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la vérification de santé: {e}")
    
    def get_navigation_status(self) -> NavigationStatus:
        """Retourne le statut actuel (thread-safe)"""
        with self.status_lock:
            return self.nav_status


def main(args=None):
    """Point d'entrée principal"""
    rclpy.init(args=args)
    
    try:
        node = CoreNavigationNode()
        
        # Utiliser MultiThreadedExecutor pour les services
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        node.get_logger().info("Core Navigation Node créé, en attente de configuration...")
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Erreur: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
