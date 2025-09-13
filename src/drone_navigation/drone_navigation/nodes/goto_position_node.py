#!/usr/bin/env python3
"""
goto_position_node.py - Nœud ROS2 pour navigation vers position GPS ou locale
Version corrigée avec vérifications de sécurité conformes aux standards MAVROS/ArduPilot
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import math

# Messages et services personnalisés
from drone_msgs.srv import GotoPosition, GotoLocal
from drone_msgs.msg import PathProgress, NavigationStatus
from drone_msgs.action import GotoPositionAction

# Messages standard ROS2 et MAVROS
from geometry_msgs.msg import PoseStamped, Point
from geographic_msgs.msg import GeoPoseStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import NavSatFix, BatteryState
from std_msgs.msg import Float64


class GotoPositionNode(Node):
    def __init__(self):
        super().__init__('goto_position_node')

        # Callback groups pour la concurrence
        self.cb_group = ReentrantCallbackGroup()

        # État interne
        self.current_pose = None
        self.current_gps = None
        self.mavros_state = None
        self.navigation_active = False
        self.navigation_lock = threading.Lock()
        self.target_position = None
        self.navigation_mode = "IDLE"
        self._cancel_requested = threading.Event()
        self.current_rel_alt = None
        self._new_target_requested = threading.Event()
        self._new_target_data = None
        self.previous_distance = float('inf')
        self.estimated_speed = 5.0
        self.emergency_stop = False

        # Batterie
        self.battery_level = 100.0
        self.low_battery_threshold = 20.0

        # Tolérances
        self.position_tolerance = 1.0
        self.gps_tolerance = 2.0

        # Vérification décollage
        self.min_takeoff_altitude = 1.5
        self.takeoff_verification_enabled = True

        self.sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )

        # Initialisation
        self.create_services()
        self.create_actions()
        self.create_publishers()
        self.create_subscriptions()

        # Timer pour setpoints
        self.setpoint_timer = self.create_timer(0.1, self.publish_setpoints)
        
        self.get_logger().info("Goto Position Node initialized - Ready for navigation")
        self.publish_status("IDLE")

    def create_services(self):
        self.srv_gps = self.create_service(
            GotoPosition,
            '/drone_nav/goto_position',
            self.handle_goto_position,
            callback_group=self.cb_group
        )
        
        self.srv_local = self.create_service(
            GotoLocal,
            '/drone_nav/goto_local', 
            self.handle_goto_local,
            callback_group=self.cb_group
        )

        self.update_gps_srv = self.create_service(
            GotoPosition,
            '/drone_nav/update_position',
            self.handle_update_position,
            callback_group=self.cb_group
        )
        
        self.update_local_srv = self.create_service(
            GotoLocal,
            '/drone_nav/update_local', 
            self.handle_update_local,
            callback_group=self.cb_group
        )

    def create_actions(self):
        self.action_server = ActionServer(
            self,
            GotoPositionAction,
            '/drone_nav/goto_position_action',
            self.execute_goto_action,
            callback_group=self.cb_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
    def create_publishers(self):
        self.local_setpoint_pub = self.create_publisher(
            PoseStamped, 
            '/mavros/setpoint_position/local', 
            10
        )
        
        self.global_setpoint_pub = self.create_publisher(
            GeoPoseStamped,
            '/mavros/setpoint_position/global',
            10
        )
    
        self.progress_pub = self.create_publisher(PathProgress, '/drone_nav/path_progress', 10)
        self.status_pub = self.create_publisher(NavigationStatus, '/drone_nav/status', 10)
        
    def create_subscriptions(self):
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            self.sensor_qos,
            callback_group=self.cb_group
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            self.sensor_qos,
            callback_group=self.cb_group
        )
        
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            self.sensor_qos,
            callback_group=self.cb_group
        )
        
        self.rel_alt_sub = self.create_subscription(
            Float64,
            '/mavros/global_position/rel_alt',
            self.rel_alt_callback,
            self.sensor_qos,
            callback_group=self.cb_group
        )

        self.battery_sub = self.create_subscription(
            BatteryState,
            '/mavros/battery',
            self.battery_callback,
            10,
            callback_group=self.cb_group
        )

    # ========== CALLBACKS ==========
    
    def pose_callback(self, msg):
        self.current_pose = msg

    def gps_callback(self, msg):
        self.current_gps = msg

    def state_callback(self, msg):
        self.mavros_state = msg

    def rel_alt_callback(self, msg):
        self.current_rel_alt = msg.data

    def battery_callback(self, msg):
        self.battery_level = msg.percentage * 100.0

    # ========== VÉRIFICATIONS DE SÉCURITÉ ==========
    
    def is_drone_ready_for_navigation(self):
        if not self.mavros_state:
            return False, "État MAVROS non disponible"
        
        if not self.mavros_state.connected:
            return False, "MAVROS non connecté"
        
        if not self.mavros_state.armed:
            return False, "Drone non armé"
        
        valid_modes = ['GUIDED', 'AUTO', 'LOITER', 'POSCTL']
        if self.mavros_state.mode not in valid_modes:
            return False, f"Mode {self.mavros_state.mode} non supporté"
        
        if self.current_rel_alt is not None and self.current_rel_alt < self.min_takeoff_altitude:
            return False, f"Altitude insuffisante ({self.current_rel_alt:.1f}m)"
        
        if not self.current_pose:
            return False, "Position locale non disponible"
        
        if self.current_gps:
            if (math.isnan(self.current_gps.latitude) or 
                math.isnan(self.current_gps.longitude) or
                abs(self.current_gps.latitude) > 90.0 or
                abs(self.current_gps.longitude) > 180.0):
                return False, "Coordonnées GPS invalides"
            
            if self.current_gps.altitude < 0 or self.current_gps.altitude > 10000:
                return False, "Altitude GPS invalide"
        else:
            return False, "Données GPS non disponibles"
        
        return True, "Prêt pour navigation"
    
    # ========== ACTION SERVER ==========

    def goal_callback(self, goal_request):
        if self.navigation_active:
            return GoalResponse.REJECT
        
        success, error_msg = self.is_drone_ready_for_navigation()
        return GoalResponse.ACCEPT if success else GoalResponse.REJECT

    def cancel_callback(self, goal_handle):
        self._cancel_requested.set()
        return CancelResponse.ACCEPT

    def execute_goto_action(self, goal_handle):
        self._cancel_requested.clear()
        
        with self.navigation_lock:
            self.navigation_active = True
            self.navigation_mode = "GPS"
            self.target_position = {
                'latitude': goal_handle.request.latitude,
                'longitude': goal_handle.request.longitude,
                'altitude': goal_handle.request.altitude,
                'yaw': getattr(goal_handle.request, 'yaw_angle', 0.0),
                'tolerance': goal_handle.request.tolerance
            }

        feedback_msg = GotoPositionAction.Feedback()
        result = GotoPositionAction.Result()
        
        try:
            self.publish_status("NAVIGATING")
            
            tolerance = goal_handle.request.tolerance if goal_handle.request.tolerance > 0 else self.gps_tolerance
            start_time = time.time()
            last_progress_time = start_time
            stuck_counter = 0
            
            while True:
                # Vérifier les nouvelles cibles
                if self._new_target_requested.is_set():
                    with self.navigation_lock:
                        self.target_position.update(self._new_target_data)
                        self._new_target_requested.clear()
                        self._new_target_data = None
                        self.get_logger().info("Nouvelle cible appliquée")

                # Vérifier l'annulation
                if self._cancel_requested.is_set() or goal_handle.is_cancel_requested:
                    self.handle_cancellation()
                    result.success = False
                    result.message = "Navigation annulée"
                    goal_handle.canceled()
                    return result
                
                # Vérifier la batterie
                if self.is_battery_low():
                    result.success = False
                    result.message = "Batterie faible"
                    goal_handle.abort()
                    self.publish_status("FAILED")
                    return result

                # Calculer la distance
                distance = self.calculate_gps_distance(
                    self.target_position['latitude'],
                    self.target_position['longitude'],
                    self.target_position['altitude']
                )
                
                # Vérifier si la cible est atteinte
                if distance <= tolerance:
                    result.success = True
                    result.message = "Position cible atteinte"
                    result.final_position = self.get_current_position()
                    goal_handle.succeed()
                    self.publish_status("SUCCEEDED")
                    return result
                
                # Vérifier si le drone est bloqué
                current_time = time.time()
                if current_time - last_progress_time > 5.0:
                    if distance > self.previous_distance * 0.99:
                        stuck_counter += 1
                        if stuck_counter >= 3:
                            result.success = False
                            result.message = "Drone bloqué"
                            goal_handle.abort()
                            self.publish_status("FAILED")
                            return result
                    else:
                        stuck_counter = 0
                        self.previous_distance = distance
                    last_progress_time = current_time
                
                # Vérifier timeout
                if current_time - start_time > 3600.0:
                    result.success = False
                    result.message = "Timeout de sécurité"
                    goal_handle.abort()
                    self.publish_status("FAILED")
                    return result
                
                # Publier le feedback
                progress = max(0.0, min(1.0, 1.0 - (distance / 100.0)))
                feedback_msg.progress = progress
                feedback_msg.distance_remaining = distance
                feedback_msg.current_position = self.get_current_position()
                feedback_msg.current_status = "NAVIGATING"
                goal_handle.publish_feedback(feedback_msg)
                
                self.publish_progress(
                    progress, 
                    distance, 
                    self.get_current_position(),
                    self.gps_to_point(self.target_position),
                    "NAVIGATING"
                )
                
                time.sleep(0.5)
            
        except Exception as e:
            self.get_logger().error(f"Erreur navigation: {str(e)}")
            result.success = False
            result.message = f"Erreur: {str(e)}"
            goal_handle.abort()
            self.publish_status("FAILED")
        
        finally:
            with self.navigation_lock:
                self.navigation_active = False
                self.navigation_mode = "IDLE"
                self.target_position = None
                self._cancel_requested.clear()
        
        return result

    # ========== SERVICE HANDLERS ==========

    def handle_goto_position(self, request, response):
        if self.navigation_active:
            response.success = False
            response.message = "Navigation déjà en cours"
            return response
        
        success, error_msg = self.is_drone_ready_for_navigation()
        if not success:
            response.success = False
            response.message = error_msg
            return response
        
        try:
            with self.navigation_lock:
                self.navigation_active = True
                self.navigation_mode = "GPS"
                self.target_position = {
                    'latitude': request.latitude,
                    'longitude': request.longitude,
                    'altitude': request.altitude,
                    'yaw': getattr(request, 'yaw_angle', 0.0),
                    'tolerance': request.tolerance if request.tolerance > 0 else self.gps_tolerance
                }
            
            self.publish_status("NAVIGATING")
            success = self.navigate_to_gps_position(timeout=300.0)
            
            response.success = success
            response.message = "Navigation terminée" if success else "Navigation échouée"
            
        except Exception as e:
            response.success = False
            response.message = str(e)
        
        finally:
            with self.navigation_lock:
                self.navigation_active = False
                self.navigation_mode = "IDLE"
                self.target_position = None
            
        return response

    def handle_goto_local(self, request, response):
        if self.navigation_active:
            response.success = False
            response.message = "Navigation déjà en cours"
            return response
        
        success, error_msg = self.is_drone_ready_for_navigation()
        if not success:
            response.success = False
            response.message = error_msg
            return response
        
        try:
            with self.navigation_lock:
                self.navigation_active = True
                self.navigation_mode = "LOCAL"
                self.target_position = {
                    'x': request.x,
                    'y': request.y,
                    'z': request.z,
                    'yaw': getattr(request, 'yaw_angle', 0.0),
                    'tolerance': self.position_tolerance
                }
            
            self.publish_status("NAVIGATING")
            success = self.navigate_to_local_position(timeout=300.0)
            
            response.success = success
            response.message = "Navigation terminée" if success else "Navigation échouée"
            
        except Exception as e:
            response.success = False
            response.message = str(e)
        
        finally:
            with self.navigation_lock:
                self.navigation_active = False
                self.navigation_mode = "IDLE"
                self.target_position = None
            
        return response

    def handle_update_position(self, request, response):
        if not self.navigation_active or self.navigation_mode != "GPS":
            response.success = False
            response.message = "Aucune navigation GPS active"
            return response
        
        new_target = {
            'latitude': request.latitude,
            'longitude': request.longitude,
            'altitude': request.altitude,
            'yaw': getattr(request, 'yaw_angle', 0.0),
            'tolerance': request.tolerance if request.tolerance > 0 else self.gps_tolerance
        }
        
        success = self.update_target_position(new_target)
        response.success = success
        response.message = "Cible GPS mise à jour" if success else "Échec mise à jour"
        return response

    def handle_update_local(self, request, response):
        if not self.navigation_active or self.navigation_mode != "LOCAL":
            response.success = False
            response.message = "Aucune navigation locale active"
            return response
        
        new_target = {
            'x': request.x,
            'y': request.y,
            'z': request.z,
            'yaw': getattr(request, 'yaw_angle', 0.0),
            'tolerance': self.position_tolerance
        }
        
        success = self.update_target_position(new_target)
        response.success = success
        response.message = "Cible locale mise à jour" if success else "Échec mise à jour"
        return response

    def handle_cancellation(self):
        self.navigation_active = False
        if self.current_pose:
            self.maintain_current_position()
        self.get_logger().warn("Navigation annulée")
        self.publish_status("ABORTED")

    # ========== NAVIGATION METHODS ==========

    def navigate_to_gps_position(self, timeout=300.0):
        start_time = time.time()
        tolerance = self.target_position.get('tolerance', self.gps_tolerance)
        
        while time.time() - start_time < timeout:
            distance = self.calculate_gps_distance(
                self.target_position['latitude'],
                self.target_position['longitude'],
                self.target_position['altitude']
            )
            
            if distance <= tolerance:
                return True
            
            self.publish_progress(
                max(0.0, min(1.0, 1.0 - (distance / 100.0))),
                distance,
                self.get_current_position(),
                self.gps_to_point(self.target_position),
                "NAVIGATING"
            )
            
            time.sleep(0.5)
        
        return False

    def navigate_to_local_position(self, timeout=300.0):
        start_time = time.time()
        tolerance = self.target_position.get('tolerance', self.position_tolerance)
        
        while time.time() - start_time < timeout:
            if not self.current_pose:
                time.sleep(0.1)
                continue
                
            current_pos = self.current_pose.pose.position
            distance = math.sqrt(
                (current_pos.x - self.target_position['x'])**2 +
                (current_pos.y - self.target_position['y'])**2 +
                (current_pos.z - self.target_position['z'])**2
            )
            
            if distance <= tolerance:
                return True
            
            self.publish_progress(
                max(0.0, min(1.0, 1.0 - (distance / 50.0))),
                distance,
                self.get_current_position(),
                Point(
                    x=self.target_position['x'],
                    y=self.target_position['y'], 
                    z=self.target_position['z']
                ),
                "NAVIGATING"
            )
            
            time.sleep(0.5)
        
        return False

    def publish_setpoints(self):
        if not self.navigation_active or not self.target_position:
            return
        
        if self.navigation_mode == "GPS" and self.current_gps:
            msg = GeoPoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.latitude = self.target_position['latitude']
            msg.pose.position.longitude = self.target_position['longitude']
            msg.pose.position.altitude = self.target_position['altitude']
            msg.pose.orientation.w = 1.0
            self.global_setpoint_pub.publish(msg)
            
        elif self.navigation_mode == "LOCAL" and self.current_pose:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.x = self.target_position['x']
            msg.pose.position.y = self.target_position['y']
            msg.pose.position.z = self.target_position['z']
            msg.pose.orientation.w = 1.0
            self.local_setpoint_pub.publish(msg)

    def update_target_position(self, new_target):
        with self.navigation_lock:
            if self.navigation_active:
                self.target_position.update(new_target)
                self.get_logger().info(f"Cible mise à jour: {new_target}")
                return True
        return False

    # ========== UTILITY METHODS ==========

    def calculate_gps_distance(self, target_lat, target_lon, target_alt):
        if not self.current_gps:
            return float('inf')
        
        lat_dist = (target_lat - self.current_gps.latitude) * 111320.0
        lon_dist = (target_lon - self.current_gps.longitude) * 111320.0 * math.cos(math.radians(self.current_gps.latitude))
        alt_dist = abs(target_alt - self.current_gps.altitude)
        
        return math.sqrt(lat_dist**2 + lon_dist**2 + alt_dist**2)

    def gps_to_point(self, gps_pos):
        return Point(
            x=gps_pos['longitude'],
            y=gps_pos['latitude'],
            z=gps_pos['altitude']
        )

    def get_current_position(self):
        if self.current_pose:
            pos = self.current_pose.pose.position
            return Point(x=pos.x, y=pos.y, z=pos.z)
        return Point()

    def publish_progress(self, progress, distance, current_pos, target_pos, status):
        msg = PathProgress()
        msg.stamp = self.get_clock().now().to_msg()
        msg.progress = progress
        msg.distance_remaining = distance
        msg.current_position = current_pos
        # Use correct target conversion based on navigation mode
        if self.navigation_mode == "GPS":
            msg.target_position = self.gps_to_point(self.target_position)
        elif self.navigation_mode == "LOCAL":
            msg.target_position = Point(
                x=self.target_position['x'],
                y=self.target_position['y'],
                z=self.target_position['z']
            )
        else:
            msg.target_position = Point()
        msg.status = status
        self.progress_pub.publish(msg)

    def publish_status(self, status):
        msg = NavigationStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.status = status
        msg.mode = self.navigation_mode
        # Use correct target conversion based on navigation mode
        if self.target_position:
            if self.navigation_mode == "GPS":
                msg.target_position = self.gps_to_point(self.target_position)
            elif self.navigation_mode == "LOCAL":
                msg.target_position = Point(
                    x=self.target_position['x'],
                    y=self.target_position['y'],
                    z=self.target_position['z']
                )
            else:
                msg.target_position = Point()
        else:
            msg.target_position = Point()
        msg.progress = 0.0
        msg.message = ""
        self.status_pub.publish(msg)

    def maintain_current_position(self):
        if self.navigation_mode == "GPS" and self.current_gps:
            msg = GeoPoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.latitude = self.current_gps.latitude
            msg.pose.position.longitude = self.current_gps.longitude
            msg.pose.position.altitude = self.current_gps.altitude
            self.global_setpoint_pub.publish(msg)
        
        elif self.navigation_mode == "LOCAL" and self.current_pose:
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position = self.current_pose.pose.position
            self.local_setpoint_pub.publish(msg)

    def is_battery_low(self):
        return self.battery_level < self.low_battery_threshold

    def smooth_target_transition(self, old_target, new_target, transition_time=5.0):
        start_time = time.time()
        start_pos = {
            'latitude': old_target['latitude'],
            'longitude': old_target['longitude'],
            'altitude': old_target['altitude']
        }
        
        while time.time() - start_time < transition_time:
            progress = (time.time() - start_time) / transition_time
            progress = min(1.0, progress)
            
            current_target = {
                'latitude': start_pos['latitude'] + (new_target['latitude'] - start_pos['latitude']) * progress,
                'longitude': start_pos['longitude'] + (new_target['longitude'] - start_pos['longitude']) * progress,
                'altitude': start_pos['altitude'] + (new_target['altitude'] - start_pos['altitude']) * progress
            }
            
            self.update_target_position(current_target)
            time.sleep(0.1)
        
        self.update_target_position(new_target)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = GotoPositionNode()
        executor = MultiThreadedExecutor()
        rclpy.spin(node, executor)
        
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()