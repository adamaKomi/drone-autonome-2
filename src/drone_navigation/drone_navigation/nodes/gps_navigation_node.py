#!/usr/bin/env python3
"""
gps_navigation_node.py - Nœud de navigation GPS avec MultiThreadedExecutor
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Point
from geographic_msgs.msg import GeoPoseStamped
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import Bool
import math
import time
import threading
from concurrent.futures import ThreadPoolExecutor

from drone_msgs.action import GotoPositionAction
from drone_msgs.srv import GotoPosition, UpdatePosition
from drone_msgs.msg import PathProgress, NavigationStatus

class GPSNavigationNode(Node):
    def __init__(self):
        super().__init__('gps_navigation_node')
        
        # État interne avec protection mutex
        self._state_lock = threading.RLock()
        self._navigation_active = False
        self._target_position = None
        self._current_gps = None
        self._safe_to_navigate = False
        self._navigation_future = None
        
        # ThreadPoolExecutor pour les tâches longues
        self._executor = ThreadPoolExecutor(max_workers=2)
        
        # Configuration
        self.declare_parameter('default_tolerance', 2.0)
        self.declare_parameter('navigation_timeout', 300.0)  # Timeout en secondes (par défaut 5 min)
        self.default_tolerance = self.get_parameter('default_tolerance').value
        self.navigation_timeout = self.get_parameter('navigation_timeout').value
        
        # Publishers avec protection
        self._pub_lock = threading.Lock()
        self.setpoint_pub = self.create_publisher(GeoPoseStamped, '/mavros/setpoint_position/global', 10)
        self.progress_pub = self.create_publisher(PathProgress, '/drone_nav/progress', 10)
        self.status_pub = self.create_publisher(NavigationStatus, '/drone_nav/status', 10)
        
        # Subscribers
        self.create_subscription(Bool, '/drone_nav/safe_to_navigate', self.safety_callback, 10)
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Services
        self.goto_service = self.create_service(GotoPosition, '/drone_nav/goto_position', self.handle_goto_position)
        self.update_service = self.create_service(UpdatePosition, '/drone_nav/update_position', self.handle_update_position)
        
        # Action Server
        self.action_server = ActionServer(self, GotoPositionAction, '/drone_nav/goto_position_action', self.execute_action)
        
        # Timer pour setpoints
        self.setpoint_timer = self.create_timer(0.1, self.publish_setpoint)
        
        self.get_logger().info("GPS Navigation Node initialized with MultiThreadedExecutor")

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
    def current_gps(self):
        with self._state_lock:
            return self._current_gps
    
    @current_gps.setter
    def current_gps(self, value):
        with self._state_lock:
            self._current_gps = value

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

    def gps_callback(self, msg):
        self.current_gps = msg

    def handle_goto_position(self, request, response):
        if not self.safe_to_navigate:
            response.success = False
            response.message = "Non sécurisé pour naviguer"
            return response
        
        if self.navigation_active:
            response.success = False
            response.message = "Navigation déjà en cours"
            return response
        
        tolerance = request.tolerance if request.tolerance > 0 else self.default_tolerance
        
        self.target_position = {
            'latitude': request.latitude,
            'longitude': request.longitude,
            'altitude': request.altitude,
            'tolerance': tolerance
        }
        
        self.navigation_active = True
        self._safe_publish_status("NAVIGATING")
        
        # Navigation asynchrone avec ThreadPoolExecutor
        self._navigation_future = self._executor.submit(self._navigate_to_position)
        
        response.success = True
        response.message = "Navigation démarrée"
        return response

    def handle_update_position(self, request, response):
        if not self.navigation_active:
            response.success = False
            response.message = "Aucune navigation active"
            return response
        
        tolerance = request.tolerance if request.tolerance > 0 else self.default_tolerance
        
        # Mise à jour thread-safe
        with self._state_lock:
            if self._target_position:
                self._target_position.update({
                    'latitude': request.latitude,
                    'longitude': request.longitude,
                    'altitude': request.altitude,
                    'tolerance': tolerance
                })
        
        response.success = True
        response.message = "Position mise à jour"
        return response

    def execute_action(self, goal_handle):
        self.target_position = {
            'latitude': goal_handle.request.latitude,
            'longitude': goal_handle.request.longitude,
            'altitude': goal_handle.request.altitude,
            'tolerance': goal_handle.request.tolerance if goal_handle.request.tolerance > 0 else self.default_tolerance
        }
        
        self.navigation_active = True
        self._safe_publish_status("NAVIGATING")
        
        # Navigation avec feedback dans ThreadPoolExecutor
        future = self._executor.submit(self._navigate_with_feedback, goal_handle)
        success = future.result()  # Attend la fin
        
        result = GotoPositionAction.Result()
        result.success = success
        result.message = "Navigation terminée" if success else "Navigation échouée"
        
        if success:
            goal_handle.succeed()
        else:
            goal_handle.abort()
        
        return result

    def _navigate_with_feedback(self, goal_handle):
        try:
            step_duration = 0.5  # secondes
            max_steps = int(self.navigation_timeout / step_duration)
            for _ in range(max_steps):
                if not self.safe_to_navigate or goal_handle.is_cancel_requested:
                    self.navigation_active = False
                    return False
                distance = self.calculate_distance()
                self.get_logger().info(f"=============== Distance to target: {distance:.2f} meters ===============")
                target = self.target_position
                if not target:
                    break
                tolerance = target.get('tolerance', self.default_tolerance)
                # Feedback thread-safe
                feedback = GotoPositionAction.Feedback()
                feedback.distance_remaining = distance
                feedback.progress = max(0.0, min(1.0, 1.0 - (distance / 100.0)))
                if self.current_gps and target:
                    feedback.current_position = Point(
                        x=self.current_gps.longitude,
                        y=self.current_gps.latitude, 
                        z=self.current_gps.altitude
                    )
                goal_handle.publish_feedback(feedback)
                if distance <= tolerance:
                    self.navigation_active = False
                    self._safe_publish_status("SUCCEEDED")
                    self.get_logger().info("########################## Arrivé à la position cible. ##############################")
                    return True
                self._safe_publish_progress(distance)
                self._safe_publish_status("NAVIGATING")
                time.sleep(step_duration)
            self.navigation_active = False
            self._safe_publish_status("FAILED")
            return False
        except Exception as e:
            self.get_logger().error(f"Erreur navigation: {e}")
            self.navigation_active = False
            self._safe_publish_status("FAILED")
            return False

    def _navigate_to_position(self):
        try:
            step_duration = 0.5  # secondes
            max_steps = int(self.navigation_timeout / step_duration)
            for _ in range(max_steps):
                if not self.safe_to_navigate:
                    break
                distance = self.calculate_distance()
                self.get_logger().info(f"=============== Distance to target: {distance:.2f} meters ===============")
                target = self.target_position
                if not target:
                    break
                tolerance = target.get('tolerance', self.default_tolerance)
                if distance <= tolerance:
                    self.navigation_active = False
                    self._safe_publish_status("SUCCEEDED")
                    self.get_logger().info("########################## Arrivé à la position cible. ##############################")
                    return True
                self._safe_publish_progress(distance)
                self._safe_publish_status("NAVIGATING")
                time.sleep(step_duration)
            self.navigation_active = False
            self._safe_publish_status("FAILED")
            return False
        except Exception as e:
            self.get_logger().error(f"Erreur navigation: {e}")
            self.navigation_active = False
            self._safe_publish_status("FAILED")
            return False

    def calculate_distance(self):
        current = self.current_gps
        target = self.target_position
        
        if not current or not target:
            return float('inf')
        
        # Calcul géodésique thread-safe
        lat1, lon1 = math.radians(current.latitude), math.radians(current.longitude)
        lat2, lon2 = math.radians(target['latitude']), math.radians(target['longitude'])
        
        dlat = lat2 - lat1
        dlon = lon2 - lon1
        a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
        c = 2 * math.asin(math.sqrt(a))
        horizontal_dist = 6371000 * c
        
        alt_dist = abs(target['altitude'] - current.altitude)
        
        return math.sqrt(horizontal_dist**2 + alt_dist**2)

    def publish_setpoint(self):
        if not self.navigation_active or not self.safe_to_navigate:
            return
            
        target = self.target_position
        if not target:
            return
        
        msg = GeoPoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        
        msg.pose.position.latitude = target['latitude']
        msg.pose.position.longitude = target['longitude']
        msg.pose.position.altitude = target['altitude']
        
        msg.pose.orientation.w = 1.0
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        
        with self._pub_lock:
            self.setpoint_pub.publish(msg)

    def _safe_publish_progress(self, distance):
        msg = PathProgress()
        msg.stamp = self.get_clock().now().to_msg()
        msg.progress = max(0.0, min(1.0, 1.0 - (distance / 100.0)))
        msg.distance_remaining = distance
        
        current = self.current_gps
        target = self.target_position
        
        if current:
            msg.current_position = Point(x=current.longitude, y=current.latitude, z=current.altitude)
        
        if target:
            msg.target_position = Point(x=target['longitude'], y=target['latitude'], z=target['altitude'])
        
        msg.status = "NAVIGATING"
        
        with self._pub_lock:
            self.progress_pub.publish(msg)

    def _safe_publish_status(self, status):
        msg = NavigationStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.status = status
        msg.mode = "GPS"
        
        target = self.target_position
        if target:
            msg.target_position = Point(x=target['longitude'], y=target['latitude'], z=target['altitude'])
        
        msg.progress = 0.0
        msg.message = ""
        
        with self._pub_lock:
            self.status_pub.publish(msg)

    def destroy_node(self):
        # Nettoyage thread-safe
        self.navigation_active = False
        if self._navigation_future:
            self._navigation_future.cancel()
        self._executor.shutdown(wait=True)
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GPSNavigationNode()
    
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