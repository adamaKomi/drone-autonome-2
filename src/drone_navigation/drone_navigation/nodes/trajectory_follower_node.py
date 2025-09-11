#!/usr/bin/env python3
"""
Trajectory Follower Node - Suiveur de trajectoire avancé pour navigation autonome
Architecture: Micro-nœud spécialisé dans le suivi précis de trajectoires
Algorithmes: Pure Pursuit, Stanley, MPC, Spline Following
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import math
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass, field
from enum import Enum
import threading
import time
from scipy.interpolate import splprep, splev, interp1d
from scipy.optimize import minimize
import bisect

# Messages ROS2
from geometry_msgs.msg import Point, PoseStamped, Twist, Vector3, Quaternion
from nav_msgs.msg import Path, Odometry
from std_msgs.msg import Header, Float64, String, Bool, Int32
from visualization_msgs.msg import Marker, MarkerArray

# Services personnalisés
from drone_msgs.srv import SetTrajectory, GetFollowingStatus, SetFollowingParameters
from drone_msgs.msg import TrajectoryPoint, TrajectoryStatus, FollowingMetrics


class FollowingAlgorithm(Enum):
    """Algorithmes de suivi de trajectoire"""
    PURE_PURSUIT = "pure_pursuit"
    STANLEY = "stanley"
    MODEL_PREDICTIVE = "mpc"
    SPLINE_FOLLOWING = "spline"
    HYBRID = "hybrid"
    ADAPTIVE = "adaptive"


class TrajectoryType(Enum):
    """Types de trajectoires supportées"""
    WAYPOINTS = "waypoints"
    SPLINE = "spline"
    POLYNOMIAL = "polynomial"
    BEZIER = "bezier"
    CONTINUOUS = "continuous"


@dataclass
class TrajectoryFollowerConfig:
    """Configuration du suiveur de trajectoire"""
    # Algorithme de suivi
    following_algorithm: FollowingAlgorithm = FollowingAlgorithm.PURE_PURSUIT
    
    # Paramètres Pure Pursuit
    lookahead_distance: float = 2.0
    min_lookahead: float = 0.5
    max_lookahead: float = 5.0
    lookahead_gain: float = 0.5
    
    # Paramètres Stanley
    stanley_gain: float = 1.0
    cross_track_gain: float = 2.0
    heading_gain: float = 1.5
    
    # Paramètres MPC
    prediction_horizon: int = 10
    control_horizon: int = 5
    mpc_frequency: float = 20.0
    
    # Paramètres de mouvement
    max_velocity: float = 5.0
    max_acceleration: float = 3.0
    max_angular_velocity: float = math.pi
    max_curvature: float = 1.0
    
    # Tolérances
    path_tolerance: float = 0.2  # m
    velocity_tolerance: float = 0.1  # m/s
    angular_tolerance: float = 0.1  # rad
    
    # Interpolation
    interpolation_resolution: float = 0.1  # m
    smoothing_factor: float = 0.1
    
    # Anticipation
    enable_velocity_planning: bool = True
    enable_curvature_analysis: bool = True
    enable_obstacle_avoidance: bool = True
    
    # Contrôle
    control_frequency: float = 50.0
    update_frequency: float = 10.0


@dataclass
class TrajectoryState:
    """État de suivi de trajectoire"""
    current_trajectory: Optional[List[TrajectoryPoint]] = None
    current_segment: int = 0
    target_point_index: int = 0
    lookahead_point: Optional[TrajectoryPoint] = None
    cross_track_error: float = 0.0
    heading_error: float = 0.0
    progress_ratio: float = 0.0
    remaining_distance: float = 0.0
    estimated_time_remaining: float = 0.0


@dataclass
class ControlCommand:
    """Commande de contrôle générée"""
    linear_velocity: Vector3
    angular_velocity: Vector3
    target_pose: PoseStamped
    timestamp: float
    algorithm_used: str


class TrajectoryFollowerNode(Node):
    """
    Nœud de suivi de trajectoire avancé
    
    Responsabilités:
    - Suivi précis de trajectoires complexes
    - Interpolation et lissage de chemins
    - Adaptation dynamique de la vitesse
    - Gestion des contraintes cinématiques
    """
    
    def __init__(self):
        super().__init__('trajectory_follower_node')
        
        # Configuration du nœud
        self.declare_parameters()
        self.load_configuration()
        
        # État interne
        self.trajectory_state = TrajectoryState()
        self.current_pose: Optional[PoseStamped] = None
        self.current_velocity: Optional[Twist] = None
        self.is_following = False
        self.following_metrics = FollowingMetrics()
        
        # Interpolateurs pour trajectoires lisses
        self.spline_interpolator = None
        self.velocity_profile = None
        
        # Threading
        self.state_lock = threading.Lock()
        
        # Callback groups
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.publisher_cb_group = ReentrantCallbackGroup()
        self.subscriber_cb_group = ReentrantCallbackGroup()
        
        # Publishers
        self.velocity_command_publisher = self.create_publisher(
            Twist,
            '/drone_nav/cmd_vel_trajectory',
            10,
            callback_group=self.publisher_cb_group
        )
        
        self.target_pose_publisher = self.create_publisher(
            PoseStamped,
            '/drone_nav/trajectory_target_pose',
            10,
            callback_group=self.publisher_cb_group
        )
        
        self.trajectory_status_publisher = self.create_publisher(
            TrajectoryStatus,
            '/drone_nav/trajectory_status',
            10,
            callback_group=self.publisher_cb_group
        )
        
        self.metrics_publisher = self.create_publisher(
            FollowingMetrics,
            '/drone_nav/following_metrics',
            10,
            callback_group=self.publisher_cb_group
        )
        
        self.visualization_publisher = self.create_publisher(
            MarkerArray,
            '/drone_nav/trajectory_visualization',
            10,
            callback_group=self.publisher_cb_group
        )
        
        # Subscribers
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/drone_nav/current_pose',
            self.pose_callback,
            10,
            callback_group=self.subscriber_cb_group
        )
        
        self.velocity_subscriber = self.create_subscription(
            Twist,
            '/drone_nav/current_velocity',
            self.velocity_callback,
            10,
            callback_group=self.subscriber_cb_group
        )
        
        self.trajectory_subscriber = self.create_subscription(
            Path,
            '/drone_nav/planned_trajectory',
            self.trajectory_callback,
            10,
            callback_group=self.subscriber_cb_group
        )
        
        # Services
        self.set_trajectory_service = self.create_service(
            SetTrajectory,
            '/drone_nav/trajectory_follower_node/set_trajectory',
            self.set_trajectory_callback,
            callback_group=self.service_cb_group
        )
        
        self.get_status_service = self.create_service(
            GetFollowingStatus,
            '/drone_nav/trajectory_follower_node/get_status',
            self.get_status_callback,
            callback_group=self.service_cb_group
        )
        
        self.set_params_service = self.create_service(
            SetFollowingParameters,
            '/drone_nav/trajectory_follower_node/set_parameters',
            self.set_parameters_callback,
            callback_group=self.service_cb_group
        )
        
        # Timers
        self.following_timer = self.create_timer(
            1.0 / self.config.control_frequency,
            self.following_loop,
            callback_group=self.publisher_cb_group
        )
        
        self.status_timer = self.create_timer(
            1.0 / self.config.update_frequency,
            self.publish_status,
            callback_group=self.publisher_cb_group
        )
        
        self.visualization_timer = self.create_timer(
            0.5,  # 2 Hz
            self.publish_visualization,
            callback_group=self.publisher_cb_group
        )
        
        self.get_logger().info("✅ Trajectory Follower Node initialisé")
    
    def declare_parameters(self):
        """Déclaration des paramètres du nœud"""
        self.declare_parameter('following_algorithm', 'pure_pursuit')
        self.declare_parameter('lookahead_distance', 2.0)
        self.declare_parameter('min_lookahead', 0.5)
        self.declare_parameter('max_lookahead', 5.0)
        self.declare_parameter('lookahead_gain', 0.5)
        self.declare_parameter('stanley_gain', 1.0)
        self.declare_parameter('cross_track_gain', 2.0)
        self.declare_parameter('max_velocity', 5.0)
        self.declare_parameter('max_acceleration', 3.0)
        self.declare_parameter('path_tolerance', 0.2)
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('enable_velocity_planning', True)
        self.declare_parameter('interpolation_resolution', 0.1)
    
    def load_configuration(self):
        """Charge la configuration depuis les paramètres"""
        self.config = TrajectoryFollowerConfig(
            following_algorithm=FollowingAlgorithm(self.get_parameter('following_algorithm').value),
            lookahead_distance=self.get_parameter('lookahead_distance').value,
            min_lookahead=self.get_parameter('min_lookahead').value,
            max_lookahead=self.get_parameter('max_lookahead').value,
            lookahead_gain=self.get_parameter('lookahead_gain').value,
            stanley_gain=self.get_parameter('stanley_gain').value,
            cross_track_gain=self.get_parameter('cross_track_gain').value,
            max_velocity=self.get_parameter('max_velocity').value,
            max_acceleration=self.get_parameter('max_acceleration').value,
            path_tolerance=self.get_parameter('path_tolerance').value,
            control_frequency=self.get_parameter('control_frequency').value,
            enable_velocity_planning=self.get_parameter('enable_velocity_planning').value,
            interpolation_resolution=self.get_parameter('interpolation_resolution').value
        )
    
    def pose_callback(self, msg: PoseStamped):
        """Callback pour mise à jour de la pose"""
        with self.state_lock:
            self.current_pose = msg
    
    def velocity_callback(self, msg: Twist):
        """Callback pour mise à jour de la vitesse"""
        with self.state_lock:
            self.current_velocity = msg
    
    def trajectory_callback(self, msg: Path):
        """Callback pour nouvelle trajectoire planifiée"""
        if not self.is_following:  # Éviter les interruptions non désirées
            trajectory_points = self.convert_path_to_trajectory(msg)
            self.set_new_trajectory(trajectory_points)
    
    async def set_trajectory_callback(self, request, response):
        """Service de définition d'une nouvelle trajectoire"""
        try:
            self.get_logger().info(f"🎯 Nouvelle trajectoire: {len(request.trajectory_points)} points")
            
            # Conversion et validation
            trajectory_points = request.trajectory_points
            
            if len(trajectory_points) < 2:
                raise ValueError("Trajectoire insuffisante (< 2 points)")
            
            # Configuration personnalisée
            if hasattr(request, 'algorithm'):
                self.config.following_algorithm = FollowingAlgorithm(request.algorithm)
            
            # Définition de la trajectoire
            await self.set_new_trajectory(trajectory_points)
            
            response.success = True
            response.trajectory_length = len(trajectory_points)
            response.estimated_duration = self.estimate_trajectory_duration(trajectory_points)
            response.total_distance = self.calculate_trajectory_distance(trajectory_points)
            
            self.get_logger().info(
                f"✅ Trajectoire définie: {response.total_distance:.1f}m, "
                f"{response.estimated_duration:.1f}s"
            )
            
        except Exception as e:
            self.get_logger().error(f"❌ Erreur définition trajectoire: {str(e)}")
            response.success = False
            response.error_message = str(e)
        
        return response
    
    async def get_status_callback(self, request, response):
        """Service de récupération du statut de suivi"""
        try:
            with self.state_lock:
                response.success = True
                response.is_following = self.is_following
                response.progress_ratio = self.trajectory_state.progress_ratio
                response.cross_track_error = self.trajectory_state.cross_track_error
                response.heading_error = self.trajectory_state.heading_error
                response.remaining_distance = self.trajectory_state.remaining_distance
                response.estimated_time_remaining = self.trajectory_state.estimated_time_remaining
                response.current_segment = self.trajectory_state.current_segment
                
                if self.trajectory_state.current_trajectory:
                    response.total_segments = len(self.trajectory_state.current_trajectory) - 1
                
                # Métriques de performance
                response.average_cross_track_error = self.following_metrics.average_cross_track_error
                response.max_cross_track_error = self.following_metrics.max_cross_track_error
                response.tracking_accuracy = self.following_metrics.tracking_accuracy
                
        except Exception as e:
            response.success = False
            response.error_message = str(e)
        
        return response
    
    async def set_parameters_callback(self, request, response):
        """Service de mise à jour des paramètres"""
        try:
            # Mise à jour des paramètres
            if hasattr(request, 'lookahead_distance'):
                self.config.lookahead_distance = request.lookahead_distance
            if hasattr(request, 'max_velocity'):
                self.config.max_velocity = request.max_velocity
            if hasattr(request, 'path_tolerance'):
                self.config.path_tolerance = request.path_tolerance
            
            response.success = True
            response.message = "Paramètres mis à jour"
            
            self.get_logger().info("✅ Paramètres de suivi mis à jour")
            
        except Exception as e:
            response.success = False
            response.message = str(e)
        
        return response
    
    def convert_path_to_trajectory(self, path_msg: Path) -> List[TrajectoryPoint]:
        """Convertit un Path en liste de TrajectoryPoint"""
        trajectory_points = []
        
        for i, pose in enumerate(path_msg.poses):
            point = TrajectoryPoint()
            point.position.x = pose.pose.position.x
            point.position.y = pose.pose.position.y
            point.position.z = pose.pose.position.z
            point.orientation = pose.pose.orientation
            
            # Vitesse par défaut (pourrait être calculée dynamiquement)
            point.velocity = self.config.max_velocity
            
            # Index dans la séquence
            point.time_from_start.sec = i
            
            trajectory_points.append(point)
        
        return trajectory_points
    
    async def set_new_trajectory(self, trajectory_points: List[TrajectoryPoint]):
        """Définit une nouvelle trajectoire à suivre"""
        with self.state_lock:
            # Validation
            if len(trajectory_points) < 2:
                raise ValueError("Trajectoire trop courte")
            
            # Interpolation et lissage si nécessaire
            if self.config.interpolation_resolution > 0:
                trajectory_points = await self.interpolate_trajectory(trajectory_points)
            
            # Planification de vitesse si activée
            if self.config.enable_velocity_planning:
                trajectory_points = await self.plan_velocity_profile(trajectory_points)
            
            # Mise à jour de l'état
            self.trajectory_state.current_trajectory = trajectory_points
            self.trajectory_state.current_segment = 0
            self.trajectory_state.target_point_index = 0
            self.trajectory_state.progress_ratio = 0.0
            
            # Préparation des interpolateurs
            await self.prepare_interpolators(trajectory_points)
            
            # Démarrage du suivi
            self.is_following = True
            
            # Reset des métriques
            self.reset_metrics()
    
    async def interpolate_trajectory(self, trajectory_points: List[TrajectoryPoint]) -> List[TrajectoryPoint]:
        """Interpole une trajectoire pour la rendre plus lisse"""
        if len(trajectory_points) < 3:
            return trajectory_points
        
        # Extraction des coordonnées
        x_coords = [p.position.x for p in trajectory_points]
        y_coords = [p.position.y for p in trajectory_points]
        z_coords = [p.position.z for p in trajectory_points]
        
        # Paramètres pour l'interpolation spline
        coords = np.array([x_coords, y_coords, z_coords]).T
        
        try:
            # Calcul des distances cumulées pour paramétrage
            distances = [0.0]
            for i in range(1, len(coords)):
                dist = np.linalg.norm(coords[i] - coords[i-1])
                distances.append(distances[-1] + dist)
            
            # Normalisation
            total_distance = distances[-1]
            if total_distance == 0:
                return trajectory_points
            
            normalized_distances = [d / total_distance for d in distances]
            
            # Interpolation spline
            tck, u = splprep([x_coords, y_coords, z_coords], u=normalized_distances, s=self.config.smoothing_factor)
            
            # Génération de points interpolés
            num_points = max(len(trajectory_points), int(total_distance / self.config.interpolation_resolution))
            u_new = np.linspace(0, 1, num_points)
            interpolated_coords = splev(u_new, tck)
            
            # Reconstruction des TrajectoryPoint
            interpolated_points = []
            for i in range(num_points):
                point = TrajectoryPoint()
                point.position.x = float(interpolated_coords[0][i])
                point.position.y = float(interpolated_coords[1][i])
                point.position.z = float(interpolated_coords[2][i])
                
                # Interpolation de l'orientation
                if i < len(trajectory_points):
                    point.orientation = trajectory_points[i].orientation
                else:
                    # Calcul de l'orientation basée sur la direction
                    if i > 0:
                        dx = interpolated_coords[0][i] - interpolated_coords[0][i-1]
                        dy = interpolated_coords[1][i] - interpolated_coords[1][i-1]
                        yaw = math.atan2(dy, dx)
                        point.orientation = self.yaw_to_quaternion(yaw)
                    else:
                        point.orientation = trajectory_points[0].orientation
                
                # Vitesse par défaut
                point.velocity = self.config.max_velocity
                
                interpolated_points.append(point)
            
            return interpolated_points
            
        except Exception as e:
            self.get_logger().warn(f"⚠️ Échec interpolation: {str(e)}, utilisation trajectoire originale")
            return trajectory_points
    
    async def plan_velocity_profile(self, trajectory_points: List[TrajectoryPoint]) -> List[TrajectoryPoint]:
        """Planifie le profil de vitesse le long de la trajectoire"""
        
        for i, point in enumerate(trajectory_points):
            # Calcul de la courbure locale
            curvature = self.calculate_local_curvature(trajectory_points, i)
            
            # Vitesse maximale basée sur la courbature
            if curvature > 0:
                max_curve_velocity = math.sqrt(self.config.max_acceleration / curvature)
                max_curve_velocity = min(max_curve_velocity, self.config.max_velocity)
            else:
                max_curve_velocity = self.config.max_velocity
            
            # Limitation supplémentaire près des virages serrés
            if i > 0 and i < len(trajectory_points) - 1:
                angle_change = self.calculate_direction_change(trajectory_points, i)
                if angle_change > math.pi / 4:  # Virage > 45°
                    max_curve_velocity *= 0.7  # Réduction de 30%
            
            # Profil de vitesse pour arrêt en fin de trajectoire
            remaining_distance = self.calculate_remaining_distance(trajectory_points, i)
            if remaining_distance < 5.0:  # 5m avant la fin
                decel_velocity = math.sqrt(2 * self.config.max_acceleration * remaining_distance)
                max_curve_velocity = min(max_curve_velocity, decel_velocity)
            
            # Application de la vitesse calculée
            point.velocity = max(0.1, min(max_curve_velocity, self.config.max_velocity))
        
        return trajectory_points
    
    def calculate_local_curvature(self, trajectory_points: List[TrajectoryPoint], index: int) -> float:
        """Calcule la courbure locale en un point"""
        if index <= 0 or index >= len(trajectory_points) - 1:
            return 0.0
        
        # Points avant, actuel, après
        p1 = trajectory_points[index - 1].position
        p2 = trajectory_points[index].position
        p3 = trajectory_points[index + 1].position
        
        # Vecteurs
        v1 = np.array([p2.x - p1.x, p2.y - p1.y])
        v2 = np.array([p3.x - p2.x, p3.y - p2.y])
        
        # Longueurs
        len1 = np.linalg.norm(v1)
        len2 = np.linalg.norm(v2)
        
        if len1 == 0 or len2 == 0:
            return 0.0
        
        # Normalisation
        v1_norm = v1 / len1
        v2_norm = v2 / len2
        
        # Produit vectoriel pour l'angle
        cross_product = v1_norm[0] * v2_norm[1] - v1_norm[1] * v2_norm[0]
        
        # Courbure approximée
        ds = (len1 + len2) / 2
        if ds > 0:
            curvature = abs(cross_product) / ds
        else:
            curvature = 0.0
        
        return curvature
    
    def calculate_direction_change(self, trajectory_points: List[TrajectoryPoint], index: int) -> float:
        """Calcule le changement de direction en un point"""
        if index <= 0 or index >= len(trajectory_points) - 1:
            return 0.0
        
        p1 = trajectory_points[index - 1].position
        p2 = trajectory_points[index].position
        p3 = trajectory_points[index + 1].position
        
        # Directions
        angle1 = math.atan2(p2.y - p1.y, p2.x - p1.x)
        angle2 = math.atan2(p3.y - p2.y, p3.x - p2.x)
        
        # Différence d'angle
        angle_diff = abs(angle2 - angle1)
        if angle_diff > math.pi:
            angle_diff = 2 * math.pi - angle_diff
        
        return angle_diff
    
    def calculate_remaining_distance(self, trajectory_points: List[TrajectoryPoint], start_index: int) -> float:
        """Calcule la distance restante à partir d'un index"""
        total_distance = 0.0
        
        for i in range(start_index, len(trajectory_points) - 1):
            p1 = trajectory_points[i].position
            p2 = trajectory_points[i + 1].position
            
            distance = math.sqrt(
                (p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2
            )
            total_distance += distance
        
        return total_distance
    
    def following_loop(self):
        """Boucle principale de suivi de trajectoire"""
        if not self.is_following or not self.is_ready_for_following():
            return
        
        with self.state_lock:
            try:
                # Mise à jour de l'état de suivi
                self.update_trajectory_state()
                
                # Génération de la commande selon l'algorithme choisi
                if self.config.following_algorithm == FollowingAlgorithm.PURE_PURSUIT:
                    command = self.pure_pursuit_control()
                elif self.config.following_algorithm == FollowingAlgorithm.STANLEY:
                    command = self.stanley_control()
                elif self.config.following_algorithm == FollowingAlgorithm.SPLINE_FOLLOWING:
                    command = self.spline_following_control()
                elif self.config.following_algorithm == FollowingAlgorithm.ADAPTIVE:
                    command = self.adaptive_control()
                else:
                    command = self.pure_pursuit_control()  # Par défaut
                
                # Publication des commandes
                if command:
                    self.publish_control_command(command)
                
                # Mise à jour des métriques
                self.update_metrics()
                
                # Vérification de fin de trajectoire
                if self.is_trajectory_completed():
                    self.complete_trajectory()
                
            except Exception as e:
                self.get_logger().error(f"❌ Erreur boucle de suivi: {str(e)}")
    
    def is_ready_for_following(self) -> bool:
        """Vérifie si le suivi peut commencer"""
        return (
            self.current_pose is not None and
            self.trajectory_state.current_trajectory is not None and
            len(self.trajectory_state.current_trajectory) > 1
        )
    
    def update_trajectory_state(self):
        """Met à jour l'état de suivi de trajectoire"""
        if not self.current_pose or not self.trajectory_state.current_trajectory:
            return
        
        current_pos = self.current_pose.pose.position
        trajectory = self.trajectory_state.current_trajectory
        
        # Recherche du point de trajectoire le plus proche
        min_distance = float('inf')
        closest_index = self.trajectory_state.target_point_index
        
        # Recherche dans une fenêtre autour de l'index actuel
        search_start = max(0, self.trajectory_state.target_point_index - 5)
        search_end = min(len(trajectory), self.trajectory_state.target_point_index + 10)
        
        for i in range(search_start, search_end):
            traj_pos = trajectory[i].position
            distance = math.sqrt(
                (current_pos.x - traj_pos.x)**2 +
                (current_pos.y - traj_pos.y)**2 +
                (current_pos.z - traj_pos.z)**2
            )
            
            if distance < min_distance:
                min_distance = distance
                closest_index = i
        
        # Mise à jour de l'index cible
        self.trajectory_state.target_point_index = closest_index
        self.trajectory_state.cross_track_error = min_distance
        
        # Calcul du progrès
        self.trajectory_state.progress_ratio = closest_index / (len(trajectory) - 1)
        
        # Distance restante
        self.trajectory_state.remaining_distance = self.calculate_remaining_distance(
            trajectory, closest_index
        )
        
        # Temps restant estimé
        if self.current_velocity:
            current_speed = math.sqrt(
                self.current_velocity.linear.x**2 +
                self.current_velocity.linear.y**2 +
                self.current_velocity.linear.z**2
            )
            if current_speed > 0.1:
                self.trajectory_state.estimated_time_remaining = (
                    self.trajectory_state.remaining_distance / current_speed
                )
    
    def pure_pursuit_control(self) -> Optional[ControlCommand]:
        """Implémentation de l'algorithme Pure Pursuit"""
        if not self.current_pose or not self.trajectory_state.current_trajectory:
            return None
        
        current_pos = self.current_pose.pose.position
        trajectory = self.trajectory_state.current_trajectory
        
        # Calcul de la distance de lookahead adaptative
        current_speed = 0.0
        if self.current_velocity:
            current_speed = math.sqrt(
                self.current_velocity.linear.x**2 +
                self.current_velocity.linear.y**2
            )
        
        lookahead = self.config.lookahead_distance + self.config.lookahead_gain * current_speed
        lookahead = np.clip(lookahead, self.config.min_lookahead, self.config.max_lookahead)
        
        # Recherche du point de lookahead
        lookahead_point = None
        start_index = self.trajectory_state.target_point_index
        
        for i in range(start_index, len(trajectory)):
            traj_pos = trajectory[i].position
            distance = math.sqrt(
                (current_pos.x - traj_pos.x)**2 +
                (current_pos.y - traj_pos.y)**2
            )
            
            if distance >= lookahead:
                lookahead_point = trajectory[i]
                break
        
        # Si pas de point trouvé, prendre le dernier point
        if lookahead_point is None:
            lookahead_point = trajectory[-1]
        
        self.trajectory_state.lookahead_point = lookahead_point
        
        # Calcul de la commande de vitesse
        target_pos = lookahead_point.position
        
        # Direction vers le point cible
        dx = target_pos.x - current_pos.x
        dy = target_pos.y - current_pos.y
        dz = target_pos.z - current_pos.z
        
        distance_to_target = math.sqrt(dx*dx + dy*dy + dz*dz)
        
        if distance_to_target > 0.01:
            # Vitesse linéaire
            linear_velocity = Vector3()
            linear_velocity.x = (dx / distance_to_target) * lookahead_point.velocity
            linear_velocity.y = (dy / distance_to_target) * lookahead_point.velocity
            linear_velocity.z = (dz / distance_to_target) * lookahead_point.velocity
            
            # Limitation de vitesse
            speed = math.sqrt(linear_velocity.x**2 + linear_velocity.y**2 + linear_velocity.z**2)
            if speed > self.config.max_velocity:
                scale = self.config.max_velocity / speed
                linear_velocity.x *= scale
                linear_velocity.y *= scale
                linear_velocity.z *= scale
            
            # Vitesse angulaire (orientation vers le point cible)
            target_yaw = math.atan2(dy, dx)
            current_yaw = self.quaternion_to_yaw(self.current_pose.pose.orientation)
            yaw_error = self.normalize_angle(target_yaw - current_yaw)
            
            angular_velocity = Vector3()
            angular_velocity.z = np.clip(yaw_error * 2.0, -self.config.max_angular_velocity, self.config.max_angular_velocity)
            
            # Construction de la commande
            command = ControlCommand(
                linear_velocity=linear_velocity,
                angular_velocity=angular_velocity,
                target_pose=self.build_target_pose(lookahead_point),
                timestamp=time.time(),
                algorithm_used="pure_pursuit"
            )
            
            return command
        
        return None
    
    def stanley_control(self) -> Optional[ControlCommand]:
        """Implémentation de l'algorithme Stanley"""
        # Implémentation simplifiée - à développer pour une version complète
        return self.pure_pursuit_control()
    
    def spline_following_control(self) -> Optional[ControlCommand]:
        """Suivi de trajectoire par spline"""
        # Implémentation simplifiée - à développer pour une version complète
        return self.pure_pursuit_control()
    
    def adaptive_control(self) -> Optional[ControlCommand]:
        """Contrôle adaptatif qui choisit l'algorithme optimal"""
        # Analyse de la situation
        cross_track_error = self.trajectory_state.cross_track_error
        
        # Choix de l'algorithme selon l'erreur
        if cross_track_error > self.config.path_tolerance * 2:
            # Erreur importante -> Pure Pursuit plus agressif
            old_lookahead = self.config.lookahead_distance
            self.config.lookahead_distance *= 0.7  # Réduction temporaire
            command = self.pure_pursuit_control()
            self.config.lookahead_distance = old_lookahead
            return command
        else:
            # Erreur faible -> Pure Pursuit standard
            return self.pure_pursuit_control()
    
    def publish_control_command(self, command: ControlCommand):
        """Publie la commande de contrôle"""
        # Commande de vitesse
        vel_msg = Twist()
        vel_msg.linear = command.linear_velocity
        vel_msg.angular = command.angular_velocity
        self.velocity_command_publisher.publish(vel_msg)
        
        # Pose cible
        self.target_pose_publisher.publish(command.target_pose)
    
    def build_target_pose(self, trajectory_point: TrajectoryPoint) -> PoseStamped:
        """Construit un message PoseStamped depuis un TrajectoryPoint"""
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position = trajectory_point.position
        pose_msg.pose.orientation = trajectory_point.orientation
        return pose_msg
    
    def is_trajectory_completed(self) -> bool:
        """Vérifie si la trajectoire est terminée"""
        if not self.trajectory_state.current_trajectory:
            return False
        
        # Vérification de proximité avec le dernier point
        last_point = self.trajectory_state.current_trajectory[-1]
        if not self.current_pose:
            return False
        
        current_pos = self.current_pose.pose.position
        distance_to_end = math.sqrt(
            (current_pos.x - last_point.position.x)**2 +
            (current_pos.y - last_point.position.y)**2 +
            (current_pos.z - last_point.position.z)**2
        )
        
        return distance_to_end < self.config.path_tolerance
    
    def complete_trajectory(self):
        """Finalise le suivi de trajectoire"""
        self.is_following = False
        
        # Arrêt du drone
        stop_cmd = Twist()
        self.velocity_command_publisher.publish(stop_cmd)
        
        self.get_logger().info("✅ Trajectoire terminée avec succès")
    
    def update_metrics(self):
        """Met à jour les métriques de suivi"""
        if not self.trajectory_state.current_trajectory:
            return
        
        # Erreur transversale
        self.following_metrics.current_cross_track_error = self.trajectory_state.cross_track_error
        
        # Mise à jour des moyennes
        if not hasattr(self.following_metrics, 'error_history'):
            self.following_metrics.error_history = []
        
        self.following_metrics.error_history.append(self.trajectory_state.cross_track_error)
        
        # Limitation de l'historique
        if len(self.following_metrics.error_history) > 100:
            self.following_metrics.error_history = self.following_metrics.error_history[-50:]
        
        # Calcul des statistiques
        if self.following_metrics.error_history:
            self.following_metrics.average_cross_track_error = (
                sum(self.following_metrics.error_history) / len(self.following_metrics.error_history)
            )
            self.following_metrics.max_cross_track_error = max(self.following_metrics.error_history)
            
            # Précision de suivi (pourcentage de points dans la tolérance)
            in_tolerance = sum(1 for e in self.following_metrics.error_history if e < self.config.path_tolerance)
            self.following_metrics.tracking_accuracy = (in_tolerance / len(self.following_metrics.error_history)) * 100
    
    def reset_metrics(self):
        """Reset des métriques de suivi"""
        self.following_metrics = FollowingMetrics()
        self.following_metrics.error_history = []
    
    def publish_status(self):
        """Publication du statut de suivi"""
        status_msg = TrajectoryStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        
        status_msg.is_following = self.is_following
        status_msg.progress_ratio = self.trajectory_state.progress_ratio
        status_msg.cross_track_error = self.trajectory_state.cross_track_error
        status_msg.remaining_distance = self.trajectory_state.remaining_distance
        status_msg.estimated_time_remaining = self.trajectory_state.estimated_time_remaining
        
        if self.trajectory_state.current_trajectory:
            status_msg.total_waypoints = len(self.trajectory_state.current_trajectory)
            status_msg.current_waypoint = self.trajectory_state.target_point_index
        
        self.trajectory_status_publisher.publish(status_msg)
        
        # Métriques
        self.following_metrics.header.stamp = self.get_clock().now().to_msg()
        self.metrics_publisher.publish(self.following_metrics)
    
    def publish_visualization(self):
        """Publication des marqueurs de visualisation"""
        if not self.is_following or not self.trajectory_state.current_trajectory:
            return
        
        marker_array = MarkerArray()
        
        # Marqueur de la trajectoire
        trajectory_marker = Marker()
        trajectory_marker.header.frame_id = "map"
        trajectory_marker.header.stamp = self.get_clock().now().to_msg()
        trajectory_marker.ns = "trajectory"
        trajectory_marker.id = 0
        trajectory_marker.type = Marker.LINE_STRIP
        trajectory_marker.action = Marker.ADD
        trajectory_marker.scale.x = 0.1
        trajectory_marker.color.r = 0.0
        trajectory_marker.color.g = 0.0
        trajectory_marker.color.b = 1.0
        trajectory_marker.color.a = 0.8
        
        for point in self.trajectory_state.current_trajectory:
            p = Point()
            p.x = point.position.x
            p.y = point.position.y
            p.z = point.position.z
            trajectory_marker.points.append(p)
        
        marker_array.markers.append(trajectory_marker)
        
        # Marqueur du point de lookahead
        if self.trajectory_state.lookahead_point:
            lookahead_marker = Marker()
            lookahead_marker.header.frame_id = "map"
            lookahead_marker.header.stamp = self.get_clock().now().to_msg()
            lookahead_marker.ns = "lookahead"
            lookahead_marker.id = 1
            lookahead_marker.type = Marker.SPHERE
            lookahead_marker.action = Marker.ADD
            lookahead_marker.scale.x = 0.5
            lookahead_marker.scale.y = 0.5
            lookahead_marker.scale.z = 0.5
            lookahead_marker.color.r = 1.0
            lookahead_marker.color.g = 0.0
            lookahead_marker.color.b = 0.0
            lookahead_marker.color.a = 1.0
            lookahead_marker.pose.position = self.trajectory_state.lookahead_point.position
            lookahead_marker.pose.orientation.w = 1.0
            
            marker_array.markers.append(lookahead_marker)
        
        self.visualization_publisher.publish(marker_array)
    
    def quaternion_to_yaw(self, quaternion: Quaternion) -> float:
        """Convertit un quaternion en angle yaw"""
        w = quaternion.w
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw
    
    def yaw_to_quaternion(self, yaw: float) -> Quaternion:
        """Convertit un angle yaw en quaternion"""
        quat = Quaternion()
        quat.w = math.cos(yaw / 2)
        quat.x = 0.0
        quat.y = 0.0
        quat.z = math.sin(yaw / 2)
        return quat
    
    def normalize_angle(self, angle: float) -> float:
        """Normalise un angle entre -π et π"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def estimate_trajectory_duration(self, trajectory_points: List[TrajectoryPoint]) -> float:
        """Estime la durée de parcours d'une trajectoire"""
        total_time = 0.0
        
        for i in range(len(trajectory_points) - 1):
            p1 = trajectory_points[i].position
            p2 = trajectory_points[i + 1].position
            
            distance = math.sqrt(
                (p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2
            )
            
            avg_velocity = (trajectory_points[i].velocity + trajectory_points[i + 1].velocity) / 2
            if avg_velocity > 0:
                total_time += distance / avg_velocity
        
        return total_time
    
    def calculate_trajectory_distance(self, trajectory_points: List[TrajectoryPoint]) -> float:
        """Calcule la distance totale d'une trajectoire"""
        total_distance = 0.0
        
        for i in range(len(trajectory_points) - 1):
            p1 = trajectory_points[i].position
            p2 = trajectory_points[i + 1].position
            
            distance = math.sqrt(
                (p2.x - p1.x)**2 + (p2.y - p1.y)**2 + (p2.z - p1.z)**2
            )
            total_distance += distance
        
        return total_distance


def main(args=None):
    """Point d'entrée principal"""
    rclpy.init(args=args)
    
    try:
        node = TrajectoryFollowerNode()
        
        # Utilisation d'un exécuteur multi-threadé
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        node.get_logger().info("🚀 Trajectory Follower Node démarré")
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Erreur fatale: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
