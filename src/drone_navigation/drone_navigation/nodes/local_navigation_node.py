#!/usr/bin/env python3
"""
local_navigation_node.py - Nœud de navigation locale avec MultiThreadedExecutor
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point, PoseStamped
from std_msgs.msg import Bool
import math
import time
import threading
from concurrent.futures import ThreadPoolExecutor
from transforms3d.euler import euler2quat

from drone_msgs.action import GotoLocalAction
from drone_msgs.srv import GotoLocal, UpdateLocal
from drone_msgs.msg import PathProgress, NavigationStatus

class LocalNavigationNode(Node):
    def __init__(self):
        super().__init__('local_navigation_node')
        
    # État interne avec protection mutex
    # _state_lock : protège l'accès aux variables d'état du nœud (navigation_active, target_position, etc.)
    # _navigation_active : indique si une navigation locale est en cours
    # _target_position : dictionnaire contenant la cible locale (x, y, z, yaw, tolerance)
    # _current_pose : dernière position locale reçue du drone
    # _safe_to_navigate : indique si la navigation est autorisée (donnée par le nœud de sécurité)
    # _navigation_future : future de la tâche de navigation asynchrone
        self._state_lock = threading.RLock()
        self._navigation_active = False
        self._target_position = None
        self._current_pose = None
        self._safe_to_navigate = False
        self._navigation_future = None
        
    # ThreadPoolExecutor pour les tâches longues (navigation asynchrone, feedback)
        self._executor = ThreadPoolExecutor(max_workers=2)
        
    # Paramètres configurables (tolérance, vitesse, vitesse max)
        self.declare_parameter('default_tolerance', 1.0)
        self.declare_parameter('default_speed', 2.0)
        self.declare_parameter('max_velocity', 5.0)
        self.declare_parameter('max_altitude_local', 100.0)  # Altitude locale maximale (sécurité)
        self.declare_parameter('min_altitude_local', 0.5)    # Altitude locale minimale (sécurité)
        
        self.default_tolerance = self.get_parameter('default_tolerance').value
        self.default_speed = self.get_parameter('default_speed').value
        self.max_velocity = self.get_parameter('max_velocity').value
        self.max_altitude_local = self.get_parameter('max_altitude_local').value
        self.min_altitude_local = self.get_parameter('min_altitude_local').value
        
    # Publishers avec protection
    # setpoint_pub : publie la cible locale (PoseStamped) vers MAVROS
    # progress_pub : publie la progression de la navigation (PathProgress)
    # status_pub : publie le statut de la navigation (NavigationStatus)
        self._pub_lock = threading.Lock()
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.progress_pub = self.create_publisher(PathProgress, '/drone_nav/progress', 10)
        self.status_pub = self.create_publisher(NavigationStatus, '/drone_nav/status', 10)
        
    # Subscribers
    # /drone_nav/safe_to_navigate : reçoit l'état de sécurité du nœud safety
    # /mavros/local_position/pose : reçoit la position locale du drone
        self.create_subscription(Bool, '/drone_nav/safe_to_navigate', self.safety_callback, 10)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
    # Services
    # /drone_nav/goto_local : démarre une navigation locale vers une cible
    # /drone_nav/update_local : met à jour la cible locale en cours
        self.goto_service = self.create_service(GotoLocal, '/drone_nav/goto_local', self.handle_goto_local)
        self.update_service = self.create_service(UpdateLocal, '/drone_nav/update_local', self.handle_update_local)
        
    # Action Server avec la bonne action GotoLocalAction
    # Permet de recevoir des requêtes d'action pour navigation locale avec feedback
        self.action_server = ActionServer(self, GotoLocalAction, '/drone_nav/goto_local_action', self.execute_action)
        
    # Timer pour setpoints
    # Publie périodiquement la cible locale vers MAVROS
        self.setpoint_timer = self.create_timer(0.1, self.publish_setpoint)
        
        self.get_logger().info("Local Navigation Node initialized with MultiThreadedExecutor")

    # Propriétés thread-safe
    # Les propriétés suivantes protègent l'accès concurrent aux variables d'état
    @property
    def navigation_active(self):
        with self._state_lock:
            return self._navigation_active
    
    @navigation_active.setter
    def navigation_active(self, value):
        with self._state_lock:
            self._navigation_active = value

    @property
    def target_position(self):
        with self._state_lock:
            return self._target_position.copy() if self._target_position else None
    
    @target_position.setter
    def target_position(self, value):
        with self._state_lock:
            self._target_position = value

    @property
    def current_pose(self):
        with self._state_lock:
            return self._current_pose
    
    @current_pose.setter
    def current_pose(self, value):
        with self._state_lock:
            self._current_pose = value

    @property
    def safe_to_navigate(self):
        with self._state_lock:
            return self._safe_to_navigate
    
    @safe_to_navigate.setter
    def safe_to_navigate(self, value):
        with self._state_lock:
            self._safe_to_navigate = value

    def safety_callback(self, msg):
        # Callback appelé à chaque message de sécurité reçu
        self.safe_to_navigate = msg.data

    def pose_callback(self, msg):
        # Callback appelé à chaque message de position locale reçu
        self.current_pose = msg

    def handle_goto_local(self, request, response):
        # Service : démarre une navigation locale si la sécurité est OK et aucune navigation en cours
        if not self.safe_to_navigate:
            response.success = False
            response.message = "Non sécurisé pour naviguer"
            return response
        
        if self.navigation_active:
            response.success = False
            response.message = "Navigation déjà en cours"
            return response
        
        # Validation des paramètres
        tolerance = self.default_tolerance
        yaw_angle = getattr(request, 'yaw_angle', 0.0)
        
        # Valider et ajuster la position locale
        x_safe, y_safe, z_safe, validation_msg = self._validate_local_position(request.x, request.y, request.z)
        self.get_logger().info(f"🎯 Navigation locale - {validation_msg}")
        
        self.target_position = {
            'x': x_safe,
            'y': y_safe,
            'z': z_safe,
            'original_x': request.x,
            'original_y': request.y,
            'original_z': request.z,
            'yaw': yaw_angle,
            'tolerance': tolerance
        }
        
        self.navigation_active = True
        self._safe_publish_status("NAVIGATING")
        
        # Navigation asynchrone avec ThreadPoolExecutor
        self._navigation_future = self._executor.submit(self._navigate_to_position)
        
        response.success = True
        response.message = "Navigation locale démarrée"
        return response

    def handle_update_local(self, request, response):
        # Service : met à jour la cible locale pendant une navigation active
        if not self.navigation_active:
            response.success = False
            response.message = "Aucune navigation active"
            return response
        
        # Valider et ajuster la nouvelle position locale
        x_safe, y_safe, z_safe, validation_msg = self._validate_local_position(request.x, request.y, request.z)
        self.get_logger().info(f"🔄 Mise à jour locale - {validation_msg}")
        
        # Mise à jour thread-safe avec validation
        with self._state_lock:
            if self._target_position:
                self._target_position.update({
                    'x': x_safe,
                    'y': y_safe,
                    'z': z_safe,
                    'original_x': request.x,
                    'original_y': request.y,
                    'original_z': request.z,
                    'yaw': request.yaw_angle
                })
        
        response.success = True
        response.message = "Position locale mise à jour"
        return response

    def execute_action(self, goal_handle):
        # Action Server : exécute une navigation locale avec feedback et résultat
        # Utilise correctement les champs de GotoLocalAction
        tolerance = goal_handle.request.tolerance if goal_handle.request.tolerance > 0 else self.default_tolerance
        
        # Valider et ajuster la position locale pour l'action
        x_safe, y_safe, z_safe, validation_msg = self._validate_local_position(
            goal_handle.request.x, goal_handle.request.y, goal_handle.request.z)
        self.get_logger().info(f"🎯 Action locale - {validation_msg}")
        
        self.target_position = {
            'x': x_safe,
            'y': y_safe,
            'z': z_safe,
            'original_x': goal_handle.request.x,
            'original_y': goal_handle.request.y,
            'original_z': goal_handle.request.z,
            'yaw': goal_handle.request.yaw_angle,
            'tolerance': tolerance
        }
        
        self.navigation_active = True
        self._safe_publish_status("NAVIGATING")
        
        # Navigation avec feedback dans ThreadPoolExecutor
        future = self._executor.submit(self._navigate_with_feedback, goal_handle)
        success = future.result()  # Attend la fin
        
        result = GotoLocalAction.Result()
        result.success = success
        result.message = "Navigation locale terminée" if success else "Navigation locale échouée"
        
        # Ajouter final_position
        current = self.current_pose
        if current:
            current_pos = current.pose.position
            result.final_position = Point(x=current_pos.x, y=current_pos.y, z=current_pos.z)
        else:
            result.final_position = Point(x=0.0, y=0.0, z=0.0)
        
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        
        return result

    def _navigate_with_feedback(self, goal_handle):
        # Fonction interne : navigation locale avec feedback pour l'action
        try:
            for _ in range(600):  # 5 min timeout
                if not self.safe_to_navigate or goal_handle.is_cancel_requested:
                    self.navigation_active = False
                    return False
                
                distance = self.calculate_distance()
                self.get_logger().info(f"===============[_navigate_with_feedback] Distance to target: {distance:.2f} meters ===============")
                target = self.target_position
                if not target:
                    break
                    
                tolerance = target.get('tolerance', self.default_tolerance)
                self.get_logger().info(f"===============[_navigate_with_feedback] Current tolerance: {tolerance:.2f} meters ===============")
                
                # Feedback thread-safe selon GotoLocalAction.Feedback
                feedback = GotoLocalAction.Feedback()
                feedback.distance_remaining = distance
                feedback.progress = max(0.0, min(1.0, 1.0 - (distance / 50.0)))
                feedback.current_status = "NAVIGATING"
                
                current = self.current_pose
                if current:
                    current_pos = current.pose.position
                    feedback.current_position = Point(x=current_pos.x, y=current_pos.y, z=current_pos.z)
                else:
                    feedback.current_position = Point(x=0.0, y=0.0, z=0.0)
                
                goal_handle.publish_feedback(feedback)
                
                if distance <= tolerance:
                    self.navigation_active = False
                    self._safe_publish_status("SUCCEEDED")
                    self._safe_publish_progress(distance, status="SUCCEEDED")
                    self.get_logger().info("[_navigate_with_feedback] Navigation locale réussie")
                    return True
                
                self._safe_publish_progress(distance, status="NAVIGATING")
                time.sleep(0.5)
            
            self.navigation_active = False
            self._safe_publish_status("FAILED")
            self._safe_publish_progress(0.0, status="FAILED")
            return False
            
        except Exception as e:
            self.get_logger().error(f"Erreur navigation locale: {e}")
            self.navigation_active = False
            self._safe_publish_status("FAILED")
            self._safe_publish_progress(0.0, status="FAILED")
            return False

    def _navigate_to_position(self):
        # Fonction interne : navigation locale sans feedback (service)
        try:
            for _ in range(600):  # 5 min timeout
                if not self.safe_to_navigate:
                    break
                
                distance = self.calculate_distance()
                self.get_logger().info(f"=============== Distance to target: {distance:.2f} meters ===============")
                target = self.target_position
                if not target:
                    break
                    
                tolerance = target.get('tolerance', self.default_tolerance)
                self.get_logger().info(f"=============== Current tolerance: {tolerance:.2f} meters ===============")
                
                if distance <= tolerance:
                    self.navigation_active = False
                    self._safe_publish_status("SUCCEEDED")
                    self._safe_publish_progress(distance, status="SUCCEEDED")
                    self.get_logger().info("Navigation locale réussie")
                    return True
                
                self._safe_publish_progress(distance, status="NAVIGATING")
                time.sleep(0.5)
            
            self.navigation_active = False
            self._safe_publish_status("FAILED")
            self._safe_publish_progress(0.0, status="FAILED")
            return False
            
        except Exception as e:
            self.get_logger().error(f"Erreur navigation locale: {e}")
            self.navigation_active = False
            self._safe_publish_status("FAILED")
            self._safe_publish_progress(0.0, status="FAILED")
            return False

    def _validate_local_position(self, x, y, z):
        """
        Valide et ajuste une position locale pour la sécurité
        
        Args:
            x, y, z: Coordonnées locales demandées
            
        Returns:
            tuple: (x_safe, y_safe, z_safe, validation_message)
        """
        x_safe, y_safe, z_safe = x, y, z
        warnings = []
        
        # Validation de l'altitude locale (Z)
        if z < self.min_altitude_local:
            self.get_logger().warn(f"⚠️ Altitude locale {z:.2f}m trop basse ! Minimum: {self.min_altitude_local:.2f}m")
            z_safe = self.min_altitude_local
            warnings.append(f"Z ajustée à {z_safe:.2f}m (sécurité)")
        elif z > self.max_altitude_local:
            self.get_logger().warn(f"⚠️ Altitude locale {z:.2f}m trop haute ! Maximum: {self.max_altitude_local:.2f}m")
            z_safe = self.max_altitude_local
            warnings.append(f"Z ajustée à {z_safe:.2f}m (sécurité)")
        
        # Validation de distance horizontale (optionnel - limite la zone de navigation)
        horizontal_dist = math.sqrt(x**2 + y**2)
        max_horizontal = 200.0  # 200m max du point d'origine
        if horizontal_dist > max_horizontal:
            scale = max_horizontal / horizontal_dist
            x_safe = x * scale
            y_safe = y * scale
            warnings.append(f"Position horizontale limitée à {max_horizontal}m de l'origine")
        
        message = "Position locale validée"
        if warnings:
            message += f" avec ajustements: {'; '.join(warnings)}"
        
        return x_safe, y_safe, z_safe, message

    def calculate_distance(self):
        # Calcule la distance 3D entre la position actuelle et la cible
        current = self.current_pose
        target = self.target_position
        
        if not current or not target:
            return float('inf')
        
        # Distance euclidienne 3D thread-safe
        current_pos = current.pose.position
        x_dist = target['x'] - current_pos.x
        y_dist = target['y'] - current_pos.y
        z_dist = target['z'] - current_pos.z
        
        return math.sqrt(x_dist**2 + y_dist**2 + z_dist**2)

    def publish_setpoint(self):
        # Publie la cible locale (PoseStamped) vers MAVROS si navigation active et sécurité OK
        if not self.navigation_active or not self.safe_to_navigate:
            return
            
        target = self.target_position
        if not target:
            return
        
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = target['x']
        msg.pose.position.y = target['y']
        msg.pose.position.z = target['z']
        
        # Orientation correcte avec quaternion du yaw
        yaw = target.get('yaw', 0.0)
        # euler2quat retourne (w, x, y, z)
        q = euler2quat(0, 0, yaw)
        msg.pose.orientation.x = q[1]
        msg.pose.orientation.y = q[2]
        msg.pose.orientation.z = q[3]
        msg.pose.orientation.w = q[0]
        
        with self._pub_lock:
            self.setpoint_pub.publish(msg)

    def _safe_publish_progress(self, distance, status="NAVIGATING"):
        # Publie la progression de la navigation (PathProgress) avec distance restante et statut
        msg = PathProgress()
        msg.stamp = self.get_clock().now().to_msg()
        msg.progress = max(0.0, min(1.0, 1.0 - (distance / 50.0)))
        msg.distance_remaining = distance
        
        current = self.current_pose
        target = self.target_position
        
        if current:
            current_pos = current.pose.position
            msg.current_position = Point(x=current_pos.x, y=current_pos.y, z=current_pos.z)
        else:
            msg.current_position = Point(x=0.0, y=0.0, z=0.0)
        
        if target:
            msg.target_position = Point(x=target['x'], y=target['y'], z=target['z'])
        else:
            msg.target_position = Point(x=0.0, y=0.0, z=0.0)
        
        msg.status = status
        
        with self._pub_lock:
            self.progress_pub.publish(msg)

    def _safe_publish_status(self, status):
        # Publie le statut de la navigation (NavigationStatus) avec mode et progrès
        msg = NavigationStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.status = status
        msg.mode = "LOCAL"
        
        target = self.target_position
        if target:
            msg.target_position = Point(x=target['x'], y=target['y'], z=target['z'])
        else:
            msg.target_position = Point(x=0.0, y=0.0, z=0.0)
        
        # Calcul du progrès basé sur la distance
        if status == "NAVIGATING":
            distance = self.calculate_distance()
            if distance != float('inf'):
                msg.progress = max(0.0, min(1.0, 1.0 - (distance / 50.0)))
            else:
                msg.progress = 0.0
        else:
            msg.progress = 1.0 if status == "SUCCEEDED" else 0.0
        
        msg.message = ""
        
        with self._pub_lock:
            self.status_pub.publish(msg)

    def maintain_current_position(self):
        # Maintient la position actuelle du drone (utilisé en cas d'arrêt ou de pause)
        """Maintenir la position actuelle - thread-safe"""
        current = self.current_pose
        if current:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose = current.pose
            
            with self._pub_lock:
                self.setpoint_pub.publish(msg)

    def destroy_node(self):
        # Nettoyage du nœud : arrêt de la navigation et du ThreadPoolExecutor
        # Nettoyage thread-safe
        self.navigation_active = False
        if self._navigation_future:
            self._navigation_future.cancel()
        self._executor.shutdown(wait=True)
        super().destroy_node()

def main(args=None):
    # Point d'entrée principal du nœud de navigation locale
    rclpy.init(args=args)
    node = LocalNavigationNode()
    
    # MultiThreadedExecutor avec 4 threads
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