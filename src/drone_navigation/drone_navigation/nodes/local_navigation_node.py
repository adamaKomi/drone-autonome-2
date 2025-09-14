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
from tf_transformations import quaternion_from_euler

from drone_msgs.action import GotoLocalAction
from drone_msgs.srv import GotoLocal, UpdateLocal
from drone_msgs.msg import PathProgress, NavigationStatus

class LocalNavigationNode(Node):
    def __init__(self):
        super().__init__('local_navigation_node')
        
        # État interne avec protection mutex
        self._state_lock = threading.RLock()
        self._navigation_active = False
        self._target_position = None
        self._current_pose = None
        self._safe_to_navigate = False
        self._navigation_future = None
        
        # ThreadPoolExecutor pour les tâches longues
        self._executor = ThreadPoolExecutor(max_workers=2)
        
        # Configuration
        self.declare_parameter('default_tolerance', 1.0)
        self.declare_parameter('default_speed', 2.0)
        self.declare_parameter('max_velocity', 5.0)
        
        self.default_tolerance = self.get_parameter('default_tolerance').value
        self.default_speed = self.get_parameter('default_speed').value
        self.max_velocity = self.get_parameter('max_velocity').value
        
        # Publishers avec protection
        self._pub_lock = threading.Lock()
        self.setpoint_pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.progress_pub = self.create_publisher(PathProgress, '/drone_nav/progress', 10)
        self.status_pub = self.create_publisher(NavigationStatus, '/drone_nav/status', 10)
        
        # Subscribers
        self.create_subscription(Bool, '/drone_nav/safe_to_navigate', self.safety_callback, 10)
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Services
        self.goto_service = self.create_service(GotoLocal, '/drone_nav/goto_local', self.handle_goto_local)
        self.update_service = self.create_service(UpdateLocal, '/drone_nav/update_local', self.handle_update_local)
        
        # Action Server avec la bonne action GotoLocalAction
        self.action_server = ActionServer(self, GotoLocalAction, '/drone_nav/goto_local_action', self.execute_action)
        
        # Timer pour setpoints
        self.setpoint_timer = self.create_timer(0.1, self.publish_setpoint)
        
        self.get_logger().info("Local Navigation Node initialized with MultiThreadedExecutor")

    # Propriétés thread-safe
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
        self.safe_to_navigate = msg.data

    def pose_callback(self, msg):
        self.current_pose = msg

    def handle_goto_local(self, request, response):
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
        
        self.target_position = {
            'x': request.x,
            'y': request.y,
            'z': request.z,
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
        if not self.navigation_active:
            response.success = False
            response.message = "Aucune navigation active"
            return response
        
        # Mise à jour thread-safe avec validation
        with self._state_lock:
            if self._target_position:
                self._target_position.update({
                    'x': request.x,
                    'y': request.y,
                    'z': request.z,
                    'yaw': request.yaw_angle
                })
        
        response.success = True
        response.message = "Position locale mise à jour"
        return response

    def execute_action(self, goal_handle):
        # Utilise correctement les champs de GotoLocalAction
        tolerance = goal_handle.request.tolerance if goal_handle.request.tolerance > 0 else self.default_tolerance
        
        self.target_position = {
            'x': goal_handle.request.x,
            'y': goal_handle.request.y,
            'z': goal_handle.request.z,
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
        try:
            for _ in range(600):  # 5 min timeout
                if not self.safe_to_navigate or goal_handle.is_cancel_requested:
                    self.navigation_active = False
                    return False
                
                distance = self.calculate_distance()
                target = self.target_position
                if not target:
                    break
                    
                tolerance = target.get('tolerance', self.default_tolerance)
                
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
                    return True
                
                self._safe_publish_progress(distance)
                time.sleep(0.5)
            
            self.navigation_active = False
            self._safe_publish_status("FAILED")
            return False
            
        except Exception as e:
            self.get_logger().error(f"Erreur navigation locale: {e}")
            self.navigation_active = False
            self._safe_publish_status("FAILED")
            return False

    def _navigate_to_position(self):
        try:
            for _ in range(600):  # 5 min timeout
                if not self.safe_to_navigate:
                    break
                
                distance = self.calculate_distance()
                target = self.target_position
                if not target:
                    break
                    
                tolerance = target.get('tolerance', self.default_tolerance)
                
                if distance <= tolerance:
                    self.navigation_active = False
                    self._safe_publish_status("SUCCEEDED")
                    return True
                
                self._safe_publish_progress(distance)
                time.sleep(0.5)
            
            self.navigation_active = False
            self._safe_publish_status("FAILED")
            return False
            
        except Exception as e:
            self.get_logger().error(f"Erreur navigation locale: {e}")
            self.navigation_active = False
            self._safe_publish_status("FAILED")
            return False

    def calculate_distance(self):
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
        q = quaternion_from_euler(0, 0, yaw)  # roll=0, pitch=0, yaw=yaw
        msg.pose.orientation.x = q[0]
        msg.pose.orientation.y = q[1]
        msg.pose.orientation.z = q[2]
        msg.pose.orientation.w = q[3]
        
        with self._pub_lock:
            self.setpoint_pub.publish(msg)

    def _safe_publish_progress(self, distance):
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
        
        msg.status = "NAVIGATING"
        
        with self._pub_lock:
            self.progress_pub.publish(msg)

    def _safe_publish_status(self, status):
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
        # Nettoyage thread-safe
        self.navigation_active = False
        if self._navigation_future:
            self._navigation_future.cancel()
        self._executor.shutdown(wait=True)
        super().destroy_node()

def main(args=None):
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