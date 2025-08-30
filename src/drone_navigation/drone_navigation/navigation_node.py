#!/usr/bin/env python3
"""
=============================================================================
DRONE NAVIGATION NODE - Nœud de navigation et contrôle de position
=============================================================================
Auteur: Adama Komi
Date: 2025-08-29
Version: 1.0.0

Description:
    Nœud ROS2 spécialisé pour la navigation et le contrôle de position du drone.
    Fournit des services de navigation avancés et gère les waypoints.
    
Responsabilités:
    - Navigation autonome vers des waypoints
    - Contrôle de position et d'altitude
    - Gestion des trajectoires
    - Évitement d'obstacles basique
    - Interface avec le nœud drone_interface central

Utilisation:
    ros2 run drone_navigation navigation_node
=============================================================================
"""

import math
import time
import threading
import numpy as np
from enum import Enum
from typing import List, Optional, Tuple, Dict, Any
from dataclasses import dataclass

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.task import Future

# Message types
from geometry_msgs.msg import PoseStamped, Point, Quaternion, TwistStamped, Vector3
from nav_msgs.msg import Path
from std_msgs.msg import Header, Float32, Bool, String
from std_srvs.srv import SetBool, Trigger
from sensor_msgs.msg import NavSatFix, Imu

# Action types - IMPORT CORRIGÉ
from drone_navigation.action import NavigateToGoal, FollowPath
from drone_navigation.srv import SetWaypoints, GetWaypoints, SetPosition, SetVelocity
from drone_navigation.msg import Waypoint, NavigationStatus, ObstacleAlert


class NavigationState(str, Enum):
    IDLE = "IDLE"
    TAKING_OFF = "TAKING_OFF"
    NAVIGATING = "NAVIGATING"
    HOVERING = "HOVERING"
    LANDING = "LANDING"
    EMERGENCY = "EMERGENCY"
    RETURNING_HOME = "RETURNING_HOME"


@dataclass
class NavigationParams:
    # Paramètres de navigation
    max_velocity: float = 2.0  # m/s
    max_acceleration: float = 1.0  # m/s²
    position_tolerance: float = 0.3  # m
    yaw_tolerance: float = 0.1  # rad
    update_rate: float = 20.0  # Hz
    safety_distance: float = 2.0  # m
    max_altitude: float = 50.0  # m
    min_altitude: float = 2.0  # m


class PIDController:
    """Contrôleur PID pour le contrôle de position et d'altitude"""
    
    def __init__(self, kp: float, ki: float, kd: float, max_output: float, min_output: float):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.max_output = max_output
        self.min_output = min_output
        
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None
        
    def compute(self, error: float, dt: float) -> float:
        # Terme proportionnel
        p_term = self.kp * error
        
        # Terme intégral avec anti-windup
        self.integral += error * dt
        i_term = self.ki * self.integral
        
        # Terme dérivé
        d_term = self.kd * (error - self.previous_error) / dt if dt > 0 else 0.0
        self.previous_error = error
        
        # Calcul de la sortie
        output = p_term + i_term + d_term
        
        # Saturation de la sortie
        output = np.clip(output, self.min_output, self.max_output)
        
        return output
    
    def reset(self):
        self.integral = 0.0
        self.previous_error = 0.0
        self.previous_time = None


class TrajectoryPlanner:
    """Planificateur de trajectoire pour waypoints"""
    
    def __init__(self, params: NavigationParams):
        self.params = params
        self.current_waypoint_index = 0
        self.waypoints: List[Waypoint] = []
        
    def set_waypoints(self, waypoints: List[Waypoint]):
        self.waypoints = waypoints
        self.current_waypoint_index = 0
        
    def get_next_waypoint(self) -> Optional[Waypoint]:
        if self.current_waypoint_index < len(self.waypoints):
            return self.waypoints[self.current_waypoint_index]
        return None
    
    def advance_to_next_waypoint(self):
        if self.current_waypoint_index < len(self.waypoints):
            self.current_waypoint_index += 1
            
    def is_finished(self) -> bool:
        return self.current_waypoint_index >= len(self.waypoints)
    
    def compute_trajectory(self, current_pose: PoseStamped, target_pose: PoseStamped) -> TwistStamped:
        """Calcule la commande de vitesse pour suivre une trajectoire"""
        twist = TwistStamped()
        twist.header.stamp = current_pose.header.stamp
        twist.header.frame_id = "map"
        
        # Calcul des erreurs de position
        dx = target_pose.pose.position.x - current_pose.pose.position.x
        dy = target_pose.pose.position.y - current_pose.pose.position.y
        dz = target_pose.pose.position.z - current_pose.pose.position.z
        
        # Calcul de la distance
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        if distance > self.params.position_tolerance:
            # Normalisation et limitation de vitesse
            vx = dx / distance * min(self.params.max_velocity, distance * 2.0)
            vy = dy / distance * min(self.params.max_velocity, distance * 2.0)
            vz = dz / distance * min(self.params.max_velocity, distance * 2.0)
            
            twist.twist.linear.x = vx
            twist.twist.linear.y = vy
            twist.twist.linear.z = vz
        
        return twist


class ObstacleAvoidance:
    """Module d'évitement d'obstacles basique"""
    
    def __init__(self, safety_distance: float):
        self.safety_distance = safety_distance
        self.obstacles_detected = False
        self.obstacle_direction = np.array([0.0, 0.0, 0.0])
        
    def update_obstacles(self, obstacles: List[ObstacleAlert]):
        """Met à jour la détection d'obstacles"""
        self.obstacles_detected = len(obstacles) > 0
        
        if self.obstacles_detected:
            # Calcul de la direction globale d'obstacle (moyenne des directions)
            directions = np.array([[
                obs.direction.x, 
                obs.direction.y, 
                obs.direction.z
            ] for obs in obstacles])
            
            self.obstacle_direction = np.mean(directions, axis=0)
            
    def adjust_trajectory(self, trajectory: TwistStamped) -> TwistStamped:
        """Ajuste la trajectoire pour éviter les obstacles"""
        if not self.obstacles_detected:
            return trajectory
            
        # Calcul de la composante d'évitement
        avoidance_vector = self.obstacle_direction * self.safety_distance
        
        # Ajustement de la trajectoire
        adjusted_trajectory = trajectory
        adjusted_trajectory.twist.linear.x += avoidance_vector[0]
        adjusted_trajectory.twist.linear.y += avoidance_vector[1]
        adjusted_trajectory.twist.linear.z += avoidance_vector[2]
        
        return adjusted_trajectory


class NavigationNode(Node):
    def __init__(self):
        super().__init__('navigation_node')
        
        self.logger = self.get_logger()
        self.logger.info("🧭 Initialisation du NavigationNode...")
        
        # Paramètres de navigation
        self.params = NavigationParams()
        
        # État de navigation
        self.navigation_state = NavigationState.IDLE
        self.current_pose = None
        self.current_velocity = None
        self.home_position = None
        
        # Modules de navigation
        self.trajectory_planner = TrajectoryPlanner(self.params)
        self.obstacle_avoidance = ObstacleAvoidance(self.params.safety_distance)
        
        # Contrôleurs PID
        self.x_controller = PIDController(1.0, 0.01, 0.1, 2.0, -2.0)
        self.y_controller = PIDController(1.0, 0.01, 0.1, 2.0, -2.0)
        self.z_controller = PIDController(1.5, 0.01, 0.2, 2.0, -2.0)
        self.yaw_controller = PIDController(1.0, 0.0, 0.1, 1.0, -1.0)
        
        # Configuration QoS
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # Groupes de callback pour les actions
        self.callback_group = ReentrantCallbackGroup()
        
        # Abonnements
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', 
            self.pose_callback, qos_profile)
        
        self.velocity_sub = self.create_subscription(
            TwistStamped, '/mavros/local_position/velocity_local', 
            self.velocity_callback, qos_profile)
        
        self.obstacle_sub = self.create_subscription(
            ObstacleAlert, '/perception/obstacles', 
            self.obstacle_callback, 10)
        
        # Publications
        self.setpoint_pub = self.create_publisher(
            TwistStamped, '/mavros/setpoint_velocity/cmd_vel', 10)
        
        self.status_pub = self.create_publisher(
            NavigationStatus, '/navigation/status', 10)
        
        self.path_pub = self.create_publisher(
            Path, '/navigation/current_path', 10)
        
        # Services
        self.set_position_service = self.create_service(
            SetPosition, '/navigation/set_position', 
            self.set_position_callback)
        
        self.set_velocity_service = self.create_service(
            SetVelocity, '/navigation/set_velocity', 
            self.set_velocity_callback)
        
        self.set_waypoints_service = self.create_service(
            SetWaypoints, '/navigation/set_waypoints', 
            self.set_waypoints_callback)
        
        self.get_waypoints_service = self.create_service(
            GetWaypoints, '/navigation/get_waypoints', 
            self.get_waypoints_callback)
        
        self.return_home_service = self.create_service(
            Trigger, '/navigation/return_home', 
            self.return_home_callback)
        
        self.hold_position_service = self.create_service(
            Trigger, '/navigation/hold_position', 
            self.hold_position_callback)
        
        # Actions
        self.navigate_action_server = ActionServer(
            self, NavigateToGoal, '/navigation/navigate_to_goal',
            execute_callback=self.navigate_to_goal_callback,
            goal_callback=self.navigate_goal_callback,
            cancel_callback=self.navigate_cancel_callback,
            callback_group=self.callback_group)
        
        self.follow_path_action_server = ActionServer(
            self, FollowPath, '/navigation/follow_path',
            execute_callback=self.follow_path_callback,
            goal_callback=self.follow_path_goal_callback,
            cancel_callback=self.follow_path_cancel_callback,
            callback_group=self.callback_group)
        
        # Timer pour la boucle de contrôle
        self.control_timer = self.create_timer(
            1.0 / self.params.update_rate, 
            self.control_loop_callback)
        
        self.logger.info("✅ NavigationNode initialisé")
    
    def pose_callback(self, msg: PoseStamped):
        """Callback de mise à jour de la position"""
        self.current_pose = msg
        
        # Définition de la position home si non définie
        if self.home_position is None and self.navigation_state == NavigationState.IDLE:
            self.home_position = msg
            self.logger.info(f"🏠 Position home définie: {msg.pose.position}")
    
    def velocity_callback(self, msg: TwistStamped):
        """Callback de mise à jour de la vitesse"""
        self.current_velocity = msg
    
    def obstacle_callback(self, msg: ObstacleAlert):
        """Callback de détection d'obstacles"""
        # Cette implémentation simplifiée suppose un message par obstacle
        # Dans une implémentation réelle, on aurait une liste d'obstacles
        self.obstacle_avoidance.update_obstacles([msg])
    
    def set_position_callback(self, request, response):
        """Service pour définir une position cible"""
        try:
            if self.navigation_state != NavigationState.IDLE:
                response.success = False
                response.message = "Navigation déjà en cours"
                return response
                
            # Création d'un waypoint à partir de la requête
            waypoint = Waypoint()
            waypoint.position = request.position
            waypoint.yaw = request.yaw
            waypoint.tolerance = self.params.position_tolerance
            waypoint.yaw_tolerance = self.params.yaw_tolerance
            
            # Navigation vers la position
            self.trajectory_planner.set_waypoints([waypoint])
            self.navigation_state = NavigationState.NAVIGATING
            
            response.success = True
            response.message = "Navigation vers la position démarrée"
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {str(e)}"
            
        return response
    
    def set_velocity_callback(self, request, response):
        """Service pour définir une vitesse"""
        try:
            if self.navigation_state != NavigationState.IDLE:
                response.success = False
                response.message = "Navigation déjà en cours"
                return response
                
            # Publication directe de la vitesse
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            twist.twist.linear = request.velocity
            twist.twist.angular.z = request.yaw_rate
            
            self.setpoint_pub.publish(twist)
            self.navigation_state = NavigationState.HOVERING
            
            response.success = True
            response.message = "Contrôle de vitesse activé"
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {str(e)}"
            
        return response
    
    def set_waypoints_callback(self, request, response):
        """Service pour définir une liste de waypoints"""
        try:
            if self.navigation_state != NavigationState.IDLE:
                response.success = False
                response.message = "Navigation déjà en cours"
                return response
                
            # Validation des waypoints
            for wp in request.waypoints:
                if wp.position.z > self.params.max_altitude:
                    response.success = False
                    response.message = f"Altitude {wp.position.z} supérieure au maximum {self.params.max_altitude}"
                    return response
                    
                if wp.position.z < self.params.min_altitude:
                    response.success = False
                    response.message = f"Altitude {wp.position.z} inférieure au minimum {self.params.min_altitude}"
                    return response
            
            # Définition des waypoints
            self.trajectory_planner.set_waypoints(request.waypoints)
            self.navigation_state = NavigationState.NAVIGATING
            
            response.success = True
            response.message = f"{len(request.waypoints)} waypoints définis"
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {str(e)}"
            
        return response
    
    def get_waypoints_callback(self, request, response):
        """Service pour obtenir la liste des waypoints actuels"""
        try:
            response.waypoints = self.trajectory_planner.waypoints
            response.success = True
            response.message = "Waypoints récupérés avec succès"
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {str(e)}"
            
        return response
    
    def return_home_callback(self, request, response):
        """Service pour retourner à la position home"""
        try:
            if self.home_position is None:
                response.success = False
                response.message = "Position home non définie"
                return response
                
            # Création d'un waypoint pour la position home
            waypoint = Waypoint()
            waypoint.position = self.home_position.pose.position
            waypoint.yaw = 0.0
            waypoint.tolerance = self.params.position_tolerance
            
            # Navigation vers home
            self.trajectory_planner.set_waypoints([waypoint])
            self.navigation_state = NavigationState.RETURNING_HOME
            
            response.success = True
            response.message = "Retour à la maison initié"
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {str(e)}"
            
        return response
    
    def hold_position_callback(self, request, response):
        """Service pour maintenir la position actuelle"""
        try:
            if self.current_pose is None:
                response.success = False
                response.message = "Position actuelle inconnue"
                return response
                
            # Arrêt de tout mouvement
            twist = TwistStamped()
            twist.header.stamp = self.get_clock().now().to_msg()
            self.setpoint_pub.publish(twist)
            
            # Remettre le système en état IDLE pour permettre de nouveaux commands
            self.navigation_state = NavigationState.IDLE
            
            response.success = True
            response.message = "Position maintenue"
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {str(e)}"
            
        return response
    
    def navigate_goal_callback(self, goal_request):
        """Callback pour l'acceptation des goals de navigation"""
        if self.navigation_state != NavigationState.IDLE:
            self.get_logger().warn("Navigation déjà en cours - rejet du goal")
            return GoalResponse.REJECT
            
        return GoalResponse.ACCEPT
    
    def navigate_cancel_callback(self, goal_handle):
        """Callback pour l'annulation de navigation"""
        self.get_logger().info("Annulation de la navigation demandée")
        return CancelResponse.ACCEPT
    
    def navigate_to_goal_callback(self, goal_handle):
        """Callback d'exécution pour la navigation vers un goal"""
        try:
            goal = goal_handle.request
            
            # Création du waypoint à partir du goal
            waypoint = Waypoint()
            waypoint.position = goal.target_position
            waypoint.yaw = goal.target_yaw
            waypoint.tolerance = goal.position_tolerance if goal.position_tolerance > 0 else self.params.position_tolerance
            waypoint.yaw_tolerance = goal.yaw_tolerance if goal.yaw_tolerance > 0 else self.params.yaw_tolerance
            
            # Définition du waypoint
            self.trajectory_planner.set_waypoints([waypoint])
            self.navigation_state = NavigationState.NAVIGATING
            
            # Boucle de navigation
            result = NavigateToGoal.Result()
            feedback = NavigateToGoal.Feedback()
            
            while self.navigation_state == NavigationState.NAVIGATING:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.navigation_state = NavigationState.IDLE
                    result.success = False
                    result.message = "Navigation annulée"
                    return result
                
                # Publication du feedback
                if self.current_pose:
                    dx = waypoint.position.x - self.current_pose.pose.position.x
                    dy = waypoint.position.y - self.current_pose.pose.position.y
                    dz = waypoint.position.z - self.current_pose.pose.position.z
                    distance = math.sqrt(dx**2 + dy**2 + dz**2)
                    
                    feedback.distance_to_goal = distance
                    goal_handle.publish_feedback(feedback)
                
                # Vérification de l'atteinte du goal
                if self.check_waypoint_reached(waypoint):
                    self.navigation_state = NavigationState.IDLE
                    result.success = True
                    result.message = "Goal atteint avec succès"
                    goal_handle.succeed()
                    return result
                
                # Attente avant la prochaine itération
                time.sleep(0.1)
            
            # Si on sort de la boucle sans avoir atteint le goal
            result.success = False
            result.message = "Navigation interrompue"
            goal_handle.abort()
            return result
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la navigation: {str(e)}")
            result.success = False
            result.message = f"Erreur: {str(e)}"
            goal_handle.abort()
            return result
    
    def follow_path_goal_callback(self, goal_request):
        """Callback pour l'acceptation des goals de suivi de chemin"""
        if self.navigation_state != NavigationState.IDLE:
            self.get_logger().warn("Navigation déjà en cours - rejet du goal")
            return GoalResponse.REJECT
            
        return GoalResponse.ACCEPT
    
    def follow_path_cancel_callback(self, goal_handle):
        """Callback pour l'annulation de suivi de chemin"""
        self.get_logger().info("Annulation du suivi de chemin demandée")
        return CancelResponse.ACCEPT
    
    def follow_path_callback(self, goal_handle):
        """Callback d'exécution pour le suivi de chemin"""
        try:
            goal = goal_handle.request
            
            # Validation des waypoints
            for wp in goal.waypoints:
                if wp.position.z > self.params.max_altitude:
                    result = FollowPath.Result()
                    result.success = False
                    result.message = f"Altitude {wp.position.z} supérieure au maximum {self.params.max_altitude}"
                    goal_handle.abort()
                    return result
                    
                if wp.position.z < self.params.min_altitude:
                    result = FollowPath.Result()
                    result.success = False
                    result.message = f"Altitude {wp.position.z} inférieure au minimum {self.params.min_altitude}"
                    goal_handle.abort()
                    return result
            
            # Définition des waypoints
            self.trajectory_planner.set_waypoints(goal.waypoints)
            self.navigation_state = NavigationState.NAVIGATING
            
            # Boucle de navigation
            result = FollowPath.Result()
            feedback = FollowPath.Feedback()
            
            while self.navigation_state == NavigationState.NAVIGATING:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.navigation_state = NavigationState.IDLE
                    result.success = False
                    result.message = "Suivi de chemin annulé"
                    return result
                
                # Publication du feedback
                feedback.current_waypoint_index = self.trajectory_planner.current_waypoint_index
                feedback.waypoints_completed = feedback.current_waypoint_index
                feedback.waypoints_total = len(goal.waypoints)
                
                if self.current_pose and self.trajectory_planner.get_next_waypoint():
                    next_wp = self.trajectory_planner.get_next_waypoint()
                    dx = next_wp.position.x - self.current_pose.pose.position.x
                    dy = next_wp.position.y - self.current_pose.pose.position.y
                    dz = next_wp.position.z - self.current_pose.pose.position.z
                    feedback.distance_to_next_waypoint = math.sqrt(dx**2 + dy**2 + dz**2)
                
                goal_handle.publish_feedback(feedback)
                
                # Vérification de l'atteinte du waypoint courant
                current_waypoint = self.trajectory_planner.get_next_waypoint()
                if current_waypoint and self.check_waypoint_reached(current_waypoint):
                    self.trajectory_planner.advance_to_next_waypoint()
                
                # Vérification de la fin du chemin
                if self.trajectory_planner.is_finished():
                    self.navigation_state = NavigationState.IDLE
                    result.success = True
                    result.message = "Chefin complété avec succès"
                    goal_handle.succeed()
                    return result
                
                # Attente avant la prochaine itération
                time.sleep(0.1)
            
            # Si on sort de la boucle sans avoir complété le chemin
            result.success = False
            result.message = "Suivi de chemin interrompu"
            goal_handle.abort()
            return result
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors du suivi de chemin: {str(e)}")
            result.success = False
            result.message = f"Erreur: {str(e)}"
            goal_handle.abort()
            return result
    
    def control_loop_callback(self):
        """Boucle de contrôle principale"""
        if self.current_pose is None:
            return
        
        # Publication de l'état
        self.publish_status()
        
        # Gestion selon l'état de navigation
        if self.navigation_state == NavigationState.NAVIGATING:
            self.execute_navigation()
        elif self.navigation_state == NavigationState.RETURNING_HOME:
            self.execute_navigation()
        elif self.navigation_state == NavigationState.HOVERING:
            # Rien à faire, on maintient la position
            pass
    
    def execute_navigation(self):
        """Exécute la navigation vers le waypoint courant"""
        next_waypoint = self.trajectory_planner.get_next_waypoint()
        if next_waypoint is None:
            self.navigation_state = NavigationState.IDLE
            return
        
        # Vérification si le waypoint est atteint
        if self.check_waypoint_reached(next_waypoint):
            self.trajectory_planner.advance_to_next_waypoint()
            
            # Vérification si c'était le dernier waypoint
            if self.trajectory_planner.is_finished():
                self.navigation_state = NavigationState.IDLE
                self.logger.info("✅ Navigation terminée - Tous les waypoints atteints")
                return
        
        # Calcul de la trajectoire
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position = next_waypoint.position
        
        # Calcul de la commande de vitesse
        trajectory = self.trajectory_planner.compute_trajectory(self.current_pose, target_pose)
        
        # Ajustement pour l'évitement d'obstacles
        trajectory = self.obstacle_avoidance.adjust_trajectory(trajectory)
        
        # Publication de la commande
        self.setpoint_pub.publish(trajectory)
    
    def check_waypoint_reached(self, waypoint: Waypoint) -> bool:
        """Vérifie si le waypoint a été atteint"""
        if self.current_pose is None:
            return False
        
        # Calcul de la distance
        dx = waypoint.position.x - self.current_pose.pose.position.x
        dy = waypoint.position.y - self.current_pose.pose.position.y
        dz = waypoint.position.z - self.current_pose.pose.position.z
        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        
        # Vérification de la tolérance
        tolerance = waypoint.tolerance if waypoint.tolerance > 0 else self.params.position_tolerance
        return distance <= tolerance
    
    def publish_status(self):
        """Publie l'état de navigation"""
        status = NavigationStatus()
        status.header.stamp = self.get_clock().now().to_msg()
        status.state = self.navigation_state.value
        
        if self.current_pose:
            status.current_position = self.current_pose.pose.position
        
        if self.trajectory_planner.get_next_waypoint():
            status.next_waypoint = self.trajectory_planner.get_next_waypoint().position
        
        status.waypoints_completed = self.trajectory_planner.current_waypoint_index
        status.waypoints_total = len(self.trajectory_planner.waypoints)
        
        self.status_pub.publish(status)


def main(args=None):
    rclpy.init(args=args)
    node = NavigationNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.logger.info("🛑 Arrêt du NavigationNode")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()