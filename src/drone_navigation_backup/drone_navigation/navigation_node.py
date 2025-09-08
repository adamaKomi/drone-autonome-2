#!/usr/bin/env python3

"""
Nœud principal de navigation pour drone autonome

Implémente un nœud ROS2 avec gestion du cycle de vie pour la navigation
du drone, incluant planification de trajectoire, contrôle de position,
évitement d'obstacles et gestion des géobarrières.

Auteur: Adama Komi
Version: 1.0.0
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from rclpy.parameter import Parameter

from std_msgs.msg import String
from std_srvs.srv import Trigger
from geometry_msgs.msg import PoseStamped, TwistStamped, Point
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue

from mavros_msgs.msg import PositionTarget, State as MavState
from mavros_msgs.srv import SetMode, CommandBool

# Import des messages et services drone_msgs (à adapter selon votre interface)
try:
    from drone_msgs.msg import Trajectory, Waypoint, MissionProgress, ZoneDefinition, Velocity3D
    from drone_msgs.srv import NavigateToWaypoint, SetTrajectory, SetGeofence, SetHome
except ImportError:
    # Messages temporaires si drone_msgs n'existe pas encore
    from geometry_msgs.msg import Point as Waypoint
    from std_msgs.msg import String as Trajectory
    from std_msgs.msg import String as MissionProgress
    from std_msgs.msg import String as ZoneDefinition
    from geometry_msgs.msg import Vector3 as Velocity3D
    from std_srvs.srv import Trigger as NavigateToWaypoint
    from std_srvs.srv import Trigger as SetTrajectory
    from std_srvs.srv import Trigger as SetGeofence
    from std_srvs.srv import Trigger as SetHome

import threading
import time
import json
from typing import Optional, List, Dict, Any

from .trajectory_planner import TrajectoryPlanner
from .position_controller import PositionController
from .path_optimizer import PathOptimizer
from .obstacle_avoidance import ObstacleAvoidance
from .geofence_manager import GeofenceManager
from .coverage_patterns import CoveragePatterns

class NavigationNode(LifecycleNode):
    """
    Nœud principal de navigation avec gestion du cycle de vie ROS2.
    
    Gère la navigation autonome du drone avec:
    - Planification de trajectoire
    - Contrôle de position PID
    - Évitement d'obstacles
    - Gestion des géobarrières
    - Interface MAVROS
    """

    def __init__(self) -> None:
        super().__init__('drone_navigation')
        
        # Déclaration des paramètres par défaut
        self._declare_parameters()
        
        self.callback_group = ReentrantCallbackGroup()
        self.lock = threading.Lock()

        # Modules de navigation (initialisés dans on_configure)
        self.trajectory_planner: Optional[TrajectoryPlanner] = None
        self.position_controller: Optional[PositionController] = None
        self.path_optimizer: Optional[PathOptimizer] = None
        self.obstacle_avoidance: Optional[ObstacleAvoidance] = None
        self.geofence_manager: Optional[GeofenceManager] = None
        self.coverage_patterns: Optional[CoveragePatterns] = None

        # État de la mission
        self.current_trajectory: Optional[Trajectory] = None
        self.current_waypoint_index: int = 0
        self.mission_active: bool = False
        self.mission_paused: bool = False
        self.current_position: Optional[Point] = None
        self.target_position: Optional[Point] = None

        # Publishers (initialisés dans on_configure)
        self.status_publisher: Optional[rclpy.publisher.Publisher] = None
        self.trajectory_publisher: Optional[rclpy.publisher.Publisher] = None
        self.waypoint_reached_publisher: Optional[rclpy.publisher.Publisher] = None
        self.path_progress_publisher: Optional[rclpy.publisher.Publisher] = None
        self.obstacles_detected_publisher: Optional[rclpy.publisher.Publisher] = None
        self.diagnostics_publisher: Optional[rclpy.publisher.Publisher] = None
        self.mavros_velocity_pub: Optional[rclpy.publisher.Publisher] = None

        # Subscribers (initialisés dans on_configure)
        self.mavros_state_sub: Optional[rclpy.subscription.Subscription] = None
        self.mavros_local_pos_sub: Optional[rclpy.subscription.Subscription] = None

        # Timer pour la boucle de contrôle
        self.control_timer: Optional[rclpy.timer.Timer] = None
        
        self.get_logger().info("NavigationNode initialisé")

    def _declare_parameters(self) -> None:
        """Déclaration des paramètres avec valeurs par défaut"""
        
        # Paramètres PID position
        self.declare_parameter('navigation.position_control.pid_gains.position.p_xy', 1.0)
        self.declare_parameter('navigation.position_control.pid_gains.position.i_xy', 0.1)
        self.declare_parameter('navigation.position_control.pid_gains.position.d_xy', 0.05)
        self.declare_parameter('navigation.position_control.pid_gains.position.p_z', 1.5)
        self.declare_parameter('navigation.position_control.pid_gains.position.i_z', 0.2)
        self.declare_parameter('navigation.position_control.pid_gains.position.d_z', 0.1)
        
        # Paramètres PID vitesse  
        self.declare_parameter('navigation.position_control.pid_gains.velocity.p_xy', 0.8)
        self.declare_parameter('navigation.position_control.pid_gains.velocity.i_xy', 0.05)
        self.declare_parameter('navigation.position_control.pid_gains.velocity.d_xy', 0.02)
        
        # Limites
        self.declare_parameter('navigation.position_control.limits.max_velocity_xy', 10.0)
        self.declare_parameter('navigation.position_control.limits.max_velocity_z', 5.0)
        self.declare_parameter('navigation.position_control.limits.max_acceleration', 5.0)
        self.declare_parameter('navigation.position_control.limits.max_jerk', 10.0)
        
        # Trajectoire
        self.declare_parameter('navigation.trajectory.planning.lookahead_distance', 5.0)
        self.declare_parameter('navigation.trajectory.planning.path_resolution', 0.5)
        self.declare_parameter('navigation.trajectory.planning.smoothing_factor', 0.5)
        self.declare_parameter('navigation.trajectory.planning.turn_radius_min', 2.0)
        
        # Optimisation
        self.declare_parameter('navigation.trajectory.optimization.algorithm', 'genetic')
        self.declare_parameter('navigation.trajectory.optimization.population_size', 50)
        self.declare_parameter('navigation.trajectory.optimization.generations', 100)
        self.declare_parameter('navigation.trajectory.optimization.mutation_rate', 0.1)
        
        # Patterns de couverture
        self.declare_parameter('navigation.coverage.patterns.zigzag.overlap_percentage', 20.0)
        self.declare_parameter('navigation.coverage.patterns.zigzag.turn_radius', 2.0)
        self.declare_parameter('navigation.coverage.patterns.spiral.spacing', 1.0)
        self.declare_parameter('navigation.coverage.patterns.spiral.max_radius', 50.0)
        self.declare_parameter('navigation.coverage.patterns.lawn_mower.stripe_width', 2.0)
        self.declare_parameter('navigation.coverage.patterns.lawn_mower.turn_style', 'sharp')
        
        # Obstacles
        self.declare_parameter('navigation.obstacles.detection.enabled', True)
        self.declare_parameter('navigation.obstacles.detection.safety_margin', 2.0)
        self.declare_parameter('navigation.obstacles.detection.max_detection_range', 20.0)
        self.declare_parameter('navigation.obstacles.avoidance.method', 'potential_field')
        self.declare_parameter('navigation.obstacles.avoidance.repulsion_strength', 1.0)
        self.declare_parameter('navigation.obstacles.avoidance.attraction_strength', 0.5)
        
        # Géobarrières
        self.declare_parameter('navigation.geofence.enabled', True)
        self.declare_parameter('navigation.geofence.action', 'rtl')
        self.declare_parameter('navigation.geofence.buffer_distance', 5.0)
        self.declare_parameter('navigation.geofence.altitude_limits.min', 5.0)
        self.declare_parameter('navigation.geofence.altitude_limits.max', 120.0)
        
        # Sécurité
        self.declare_parameter('navigation.safety.emergency.rtl_altitude', 20.0)
        self.declare_parameter('navigation.safety.emergency.emergency_descent_rate', 2.0)
        self.declare_parameter('navigation.safety.emergency.communication_timeout', 30.0)
        self.declare_parameter('navigation.safety.limits.max_tilt_angle', 30.0)
        self.declare_parameter('navigation.safety.limits.min_battery_rtl', 20.0)
        self.declare_parameter('navigation.safety.limits.max_wind_speed', 15.0)
        
        # Performance
        self.declare_parameter('navigation.performance.update_rates.position_control', 50.0)
        self.declare_parameter('navigation.performance.update_rates.trajectory_planning', 10.0)
        self.declare_parameter('navigation.performance.update_rates.obstacle_detection', 20.0)
        self.declare_parameter('navigation.performance.update_rates.status_publishing', 5.0)

    def on_configure(self, state: State) -> TransitionCallbackReturn:
        """Configuration du nœud - création des publishers, subscribers et services"""
        self.get_logger().info("Configuration du nœud de navigation...")
        
        try:
            # Chargement des paramètres
            self._load_parameters()

            # Configuration QoS
            qos_profile = QoSProfile(
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )
            
            qos_best_effort = QoSProfile(
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
                history=HistoryPolicy.KEEP_LAST,
                depth=10
            )

            # Publishers pour les données de navigation
            self.status_publisher = self.create_publisher(
                String, 'navigation/status', qos_profile)
            self.trajectory_publisher = self.create_publisher(
                String, 'navigation/trajectory', qos_profile)  # Utiliser String temporairement
            self.waypoint_reached_publisher = self.create_publisher(
                String, 'navigation/waypoint_reached', qos_profile)
            self.path_progress_publisher = self.create_publisher(
                String, 'navigation/progress', qos_profile)
            self.obstacles_detected_publisher = self.create_publisher(
                String, 'navigation/obstacles', qos_profile)
            self.diagnostics_publisher = self.create_publisher(
                DiagnosticArray, '/diagnostics', qos_profile)

            # Publisher MAVROS pour contrôle de vitesse
            self.mavros_velocity_pub = self.create_publisher(
                TwistStamped, '/mavros/setpoint_velocity/cmd_vel', qos_best_effort)

            # Subscribers MAVROS
            self.mavros_state_sub = self.create_subscription(
                MavState, '/mavros/state', self.mavros_state_callback, 
                qos_best_effort, callback_group=self.callback_group)
            self.mavros_local_pos_sub = self.create_subscription(
                PoseStamped, '/mavros/local_position/pose', 
                self.local_position_callback, qos_best_effort, 
                callback_group=self.callback_group)

            # Services de navigation
            self._create_services()

            # Initialisation des modules de navigation
            self._initialize_navigation_modules()
            
            self.get_logger().info("Configuration terminée avec succès")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la configuration: {e}")
            return TransitionCallbackReturn.FAILURE

    def _create_services(self) -> None:
        """Création des services de navigation"""
        
        # Services de navigation de base
        self.create_service(
            Trigger, 'navigation/goto_position', 
            self.goto_position_callback, callback_group=self.callback_group)
        self.create_service(
            Trigger, 'navigation/plan_path', 
            self.plan_path_callback, callback_group=self.callback_group)
        
        # Services de contrôle de mission
        self.create_service(
            Trigger, 'navigation/start_mission', 
            self.start_mission_callback, callback_group=self.callback_group)
        self.create_service(
            Trigger, 'navigation/pause_mission', 
            self.pause_mission_callback, callback_group=self.callback_group)
        self.create_service(
            Trigger, 'navigation/resume_mission', 
            self.resume_mission_callback, callback_group=self.callback_group)
        self.create_service(
            Trigger, 'navigation/stop_mission', 
            self.stop_mission_callback, callback_group=self.callback_group)
        
        # Services d'urgence
        self.create_service(
            Trigger, 'navigation/emergency_rtl', 
            self.emergency_rtl_callback, callback_group=self.callback_group)
        
        # Services de géobarrières
        self.create_service(
            Trigger, 'navigation/set_geofence', 
            self.set_geofence_callback, callback_group=self.callback_group)
        self.create_service(
            Trigger, 'navigation/clear_geofence', 
            self.clear_geofence_callback, callback_group=self.callback_group)

    def _initialize_navigation_modules(self) -> None:
        """Initialisation des modules de navigation"""
        
        try:
            self.trajectory_planner = TrajectoryPlanner(self)
            self.position_controller = PositionController(self)
            self.path_optimizer = PathOptimizer(self)
            self.obstacle_avoidance = ObstacleAvoidance(self)
            self.geofence_manager = GeofenceManager(self)
            self.coverage_patterns = CoveragePatterns(self)
            
            self.get_logger().info("Modules de navigation initialisés")
            
        except Exception as e:
            self.get_logger().error(f"Erreur initialisation modules: {e}")
            raise

    def on_activate(self, state: State) -> TransitionCallbackReturn:
        """Activation du nœud - démarrage de la boucle de contrôle"""
        self.get_logger().info("Activation du nœud de navigation...")
        
        try:
            # Démarrage du timer de contrôle à 50Hz
            control_rate = self.get_parameter('navigation.performance.update_rates.position_control').get_parameter_value().double_value
            self.control_timer = self.create_timer(1.0 / control_rate, self.control_loop)
            
            self.mission_active = True
            self.get_logger().info("Nœud de navigation activé")
            return super().on_activate(state)
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors de l'activation: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_deactivate(self, state: State) -> TransitionCallbackReturn:
        """Désactivation du nœud - arrêt de la boucle de contrôle"""
        self.get_logger().info("Désactivation du nœud de navigation...")
        
        try:
            # Arrêt de la mission et du timer
            self.mission_active = False
            if self.control_timer:
                self.destroy_timer(self.control_timer)
                self.control_timer = None
                
            # Arrêt d'urgence du drone
            self._emergency_stop()
            
            self.get_logger().info("Nœud de navigation désactivé")
            return super().on_deactivate(state)
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la désactivation: {e}")
            return TransitionCallbackReturn.FAILURE

    def on_cleanup(self, state: State) -> TransitionCallbackReturn:
        """Nettoyage du nœud - destruction des ressources"""
        self.get_logger().info("Nettoyage du nœud de navigation...")
        
        try:
            # Destruction du timer si existant
            if self.control_timer:
                self.destroy_timer(self.control_timer)
                self.control_timer = None
            
            # Nettoyage des modules
            self.trajectory_planner = None
            self.position_controller = None
            self.path_optimizer = None
            self.obstacle_avoidance = None
            self.geofence_manager = None
            self.coverage_patterns = None
            
            self.get_logger().info("Nettoyage terminé")
            return TransitionCallbackReturn.SUCCESS
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors du nettoyage: {e}")
            return TransitionCallbackReturn.FAILURE

    def _load_parameters(self) -> None:
        """Charge tous les paramètres de navigation"""
        
        try:
            # Chargement des gains PID position
            self.pid_position = {
                'p_xy': self.get_parameter('navigation.position_control.pid_gains.position.p_xy').get_parameter_value().double_value,
                'i_xy': self.get_parameter('navigation.position_control.pid_gains.position.i_xy').get_parameter_value().double_value,
                'd_xy': self.get_parameter('navigation.position_control.pid_gains.position.d_xy').get_parameter_value().double_value,
                'p_z': self.get_parameter('navigation.position_control.pid_gains.position.p_z').get_parameter_value().double_value,
                'i_z': self.get_parameter('navigation.position_control.pid_gains.position.i_z').get_parameter_value().double_value,
                'd_z': self.get_parameter('navigation.position_control.pid_gains.position.d_z').get_parameter_value().double_value,
            }

            self.pid_velocity = {
                'p_xy': self.get_parameter('navigation.position_control.pid_gains.velocity.p_xy').get_parameter_value().double_value,
                'i_xy': self.get_parameter('navigation.position_control.pid_gains.velocity.i_xy').get_parameter_value().double_value,
                'd_xy': self.get_parameter('navigation.position_control.pid_gains.velocity.d_xy').get_parameter_value().double_value,
            }

            # Chargement des limites
            self.limits = {
                'max_velocity_xy': self.get_parameter('navigation.position_control.limits.max_velocity_xy').get_parameter_value().double_value,
                'max_velocity_z': self.get_parameter('navigation.position_control.limits.max_velocity_z').get_parameter_value().double_value,
                'max_acceleration': self.get_parameter('navigation.position_control.limits.max_acceleration').get_parameter_value().double_value,
                'max_jerk': self.get_parameter('navigation.position_control.limits.max_jerk').get_parameter_value().double_value,
            }

            self.get_logger().info("Paramètres de navigation chargés avec succès")
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors du chargement des paramètres: {e}")
            raise

    def control_loop(self) -> None:
        """Boucle de contrôle principale (50Hz)"""
        
        with self.lock:
            if not self.mission_active or self.mission_paused:
                return

            try:
                # Vérification des géobarrières
                if self.geofence_manager and self.current_position:
                    if self.geofence_manager.check_violation(self.current_position):
                        self.get_logger().warning("Violation de géobarrière détectée")
                        self.trigger_emergency_rtl()
                        return

                # Détection et évitement d'obstacles
                if self.obstacle_avoidance:
                    obstacles = self.obstacle_avoidance.detect_obstacles()
                    if obstacles and self.current_trajectory:
                        self.current_trajectory = self.obstacle_avoidance.avoid_obstacles(
                            self.current_trajectory, obstacles)

                # Contrôle de position simple (position cible unique)
                if self.current_position and self.target_position and self.position_controller:
                    # Créer un waypoint temporaire à partir de la position cible
                    # Utiliser une classe simple au lieu d'importer Waypoint
                    class TempWaypoint:
                        def __init__(self, x, y, z):
                            self.latitude = x   # Coordonnées locales
                            self.longitude = y
                            self.altitude = z
                    
                    temp_waypoint = TempWaypoint(
                        self.target_position.x, 
                        self.target_position.y, 
                        self.target_position.z
                    )
                    
                    # Calcul de la vitesse nécessaire
                    velocity = self.position_controller.compute_velocity(
                        self.current_position, temp_waypoint)
                    self.send_velocity_command(velocity)
                    
                    # Vérification si la position est atteinte
                    if self.position_controller.is_waypoint_reached(
                        self.current_position, temp_waypoint):
                        self.get_logger().info("Position cible atteinte!")
                        self.mission_active = False
                        self.target_position = None

                # Contrôle de position avec trajectoire (mode avancé)
                elif (self.current_trajectory and self.current_position and 
                    self.current_waypoint_index < len(self.current_trajectory.waypoints)):
                    
                    target = self.current_trajectory.waypoints[self.current_waypoint_index]
                    
                    if self.position_controller:
                        velocity = self.position_controller.compute_velocity(
                            self.current_position, target)
                        self.send_velocity_command(velocity)

                        if self.position_controller.is_waypoint_reached(
                            self.current_position, target):
                            self.publish_waypoint_reached(target)
                            self.current_waypoint_index += 1
                            
                            # Vérification fin de mission
                            if self.current_waypoint_index >= len(self.current_trajectory.waypoints):
                                self.get_logger().info("Mission terminée")
                                self.mission_active = False

                # Publication du statut
                self.publish_status()
                
            except Exception as e:
                self.get_logger().error(f"Erreur dans la boucle de contrôle: {e}")

    # =============================================================================
    # CALLBACKS DES SERVICES
    # =============================================================================

    def goto_position_callback(self, request: Trigger.Request, 
                              response: Trigger.Response) -> Trigger.Response:
        """Service de navigation vers une position"""
        try:
            self.get_logger().info("Service goto_position appelé")
            
            # Vérification que le contrôleur est disponible
            if not self.position_controller:
                response.success = False
                response.message = "Contrôleur de position non initialisé"
                return response
            
            # Pour l'instant, utilisation d'une position de test
            # TODO: Récupérer la destination depuis les paramètres ROS2 ou topic
            target_position = Point()
            target_position.x = 10.0  # mètres
            target_position.y = 10.0  # mètres  
            target_position.z = 15.0  # mètres
            
            self.get_logger().info(f"Navigation vers: x={target_position.x}, y={target_position.y}, z={target_position.z}")
            
            # Démarrage de la navigation
            self.mission_active = True
            self.target_position = target_position
            
            response.success = True
            response.message = "Navigation vers position initiée"
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
            self.get_logger().error(f"Erreur dans goto_position_callback: {e}")
        return response

    def plan_path_callback(self, request: Trigger.Request, 
                          response: Trigger.Response) -> Trigger.Response:
        """Service de planification de chemin"""
        try:
            self.get_logger().info("Service plan_path appelé")
            # TODO: Implémenter la planification de chemin
            response.success = True
            response.message = "Planification de chemin initiée"
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
        return response

    def start_mission_callback(self, request: Trigger.Request, 
                              response: Trigger.Response) -> Trigger.Response:
        """Service de démarrage de mission"""
        try:
            self.mission_active = True
            self.mission_paused = False
            self.current_waypoint_index = 0
            self.get_logger().info("Mission démarrée")
            response.success = True
            response.message = "Mission démarrée avec succès"
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
        return response

    def pause_mission_callback(self, request: Trigger.Request, 
                              response: Trigger.Response) -> Trigger.Response:
        """Service de pause de mission"""
        try:
            self.mission_paused = True
            self.get_logger().info("Mission en pause")
            response.success = True
            response.message = "Mission mise en pause"
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
        return response

    def resume_mission_callback(self, request: Trigger.Request, 
                               response: Trigger.Response) -> Trigger.Response:
        """Service de reprise de mission"""
        try:
            self.mission_paused = False
            self.get_logger().info("Mission reprise")
            response.success = True
            response.message = "Mission reprise"
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
        return response

    def stop_mission_callback(self, request: Trigger.Request, 
                             response: Trigger.Response) -> Trigger.Response:
        """Service d'arrêt de mission"""
        try:
            self.mission_active = False
            self.mission_paused = False
            self.current_waypoint_index = 0
            self._emergency_stop()
            self.get_logger().info("Mission arrêtée")
            response.success = True
            response.message = "Mission arrêtée"
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
        return response

    def emergency_rtl_callback(self, request: Trigger.Request, 
                              response: Trigger.Response) -> Trigger.Response:
        """Service de retour d'urgence"""
        try:
            self.trigger_emergency_rtl()
            response.success = True
            response.message = "RTL d'urgence activé"
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
        return response

    def set_geofence_callback(self, request: Trigger.Request, 
                             response: Trigger.Response) -> Trigger.Response:
        """Service de définition de géobarrière"""
        try:
            self.get_logger().info("Service set_geofence appelé")
            # TODO: Implémenter la définition de géobarrière
            response.success = True
            response.message = "Géobarrière définie"
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
        return response

    def clear_geofence_callback(self, request: Trigger.Request, 
                               response: Trigger.Response) -> Trigger.Response:
        """Service de suppression de géobarrière"""
        try:
            if self.geofence_manager:
                self.geofence_manager.clear_geofence()
            self.get_logger().info("Géobarrière supprimée")
            response.success = True
            response.message = "Géobarrière supprimée"
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
        return response

    # =============================================================================
    # CALLBACKS DES TOPICS
    # =============================================================================

    def mavros_state_callback(self, msg: MavState) -> None:
        """Callback de l'état MAVROS"""
        # TODO: Traiter l'état MAVROS (mode de vol, armé, etc.)
        pass

    def local_position_callback(self, msg: PoseStamped) -> None:
        """Callback de la position locale"""
        self.current_position = msg.pose.position

    # =============================================================================
    # MÉTHODES UTILITAIRES
    # =============================================================================

    def send_velocity_command(self, velocity: Velocity3D) -> None:
        """Envoie une commande de vitesse à MAVROS"""
        if self.mavros_velocity_pub:
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.header.frame_id = "base_link"
            twist.twist.linear.x = velocity.x
            twist.twist.linear.y = velocity.y
            twist.twist.linear.z = velocity.z
            self.mavros_velocity_pub.publish(twist)

    def publish_status(self) -> None:
        """Publie le statut de navigation"""
        if self.status_publisher:
            status = {
                'mission_active': self.mission_active,
                'mission_paused': self.mission_paused,
                'current_waypoint': self.current_waypoint_index,
                'timestamp': time.time()
            }
            msg = String(data=json.dumps(status))
            self.status_publisher.publish(msg)

    def publish_waypoint_reached(self, waypoint) -> None:
        """Publie qu'un waypoint a été atteint"""
        if self.waypoint_reached_publisher:
            msg = String(data=f"Waypoint {self.current_waypoint_index} atteint")
            self.waypoint_reached_publisher.publish(msg)

    def publish_diagnostics(self) -> None:
        """Publie les diagnostics du système"""
        if self.diagnostics_publisher:
            diag_array = DiagnosticArray()
            diag_array.header.stamp = self.get_clock().now().to_msg()
            
            status = DiagnosticStatus()
            status.level = DiagnosticStatus.OK
            status.name = 'drone_navigation'
            status.message = 'Navigation system operational'
            
            # Ajout de valeurs de diagnostic
            status.values.append(KeyValue(key='mission_active', value=str(self.mission_active)))
            status.values.append(KeyValue(key='current_waypoint', value=str(self.current_waypoint_index)))
            
            diag_array.status.append(status)
            self.diagnostics_publisher.publish(diag_array)

    def trigger_emergency_rtl(self) -> None:
        """Déclenche un retour d'urgence"""
        try:
            # Arrêt de la mission en cours
            self.mission_active = False
            self.mission_paused = False
            
            # Commande RTL via MAVROS
            client = self.create_client(SetMode, '/mavros/set_mode')
            if client.wait_for_service(timeout_sec=5.0):
                request = SetMode.Request()
                request.custom_mode = 'RTL'
                
                future = client.call_async(request)
                self.get_logger().warn("RTL d'urgence activé")
            else:
                self.get_logger().error("Service MAVROS set_mode non disponible")
                
        except Exception as e:
            self.get_logger().error(f"Erreur lors du RTL d'urgence: {e}")

    def _emergency_stop(self) -> None:
        """Arrêt d'urgence - envoie vitesse nulle"""
        if self.mavros_velocity_pub:
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            # Toutes les vitesses à zéro
            self.mavros_velocity_pub.publish(twist)


# =============================================================================
# FONCTION MAIN
# =============================================================================

def main(args=None):
    """Point d'entrée principal du nœud de navigation"""
    rclpy.init(args=args)
    
    try:
        node = NavigationNode()
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        node.get_logger().info("Démarrage du nœud de navigation")
        executor.spin()
        
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt demandé par l'utilisateur")
    except Exception as e:
        node.get_logger().error(f"Erreur fatale: {e}")
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()