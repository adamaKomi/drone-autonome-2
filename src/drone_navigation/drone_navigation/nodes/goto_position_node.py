#!/usr/bin/env python3
"""
goto_position_node.py - Version complète
Nœud ROS2 pour la navigation vers une position GPS ou locale.
Envoie réellement les commandes au drone via MAVROS.
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
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
from sensor_msgs.msg import NavSatFix


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
        
        # Tolérance pour considérer qu'on a atteint la cible
        self.position_tolerance = 1.0  # mètres
        self.gps_tolerance = 0.00001  # degrés (~1m)
        
        # Services pour navigation
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
        
        # Action server pour navigation avec feedback
        self.action_server = ActionServer(
            self,
            GotoPositionAction,
            '/drone_nav/goto_position_action',
            self.execute_goto_action,
            callback_group=self.cb_group,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # Publishers vers MAVROS
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
        
        # Publishers de monitoring
        self.progress_pub = self.create_publisher(PathProgress, '/drone_nav/path_progress', 10)
        self.status_pub = self.create_publisher(NavigationStatus, '/drone_nav/status', 10)
        
        # Subscriptions MAVROS
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            10,
            callback_group=self.cb_group
        )
        
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self.gps_callback,
            10,
            callback_group=self.cb_group
        )
        
        self.state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self.state_callback,
            10,
            callback_group=self.cb_group
        )
        
        # Timer pour publier les setpoints en continu (10Hz)
        self.setpoint_timer = self.create_timer(0.1, self.publish_setpoints)
        
        self.get_logger().info("Goto Position Node initialized - Ready for real navigation!")
        self.publish_status("IDLE")

    def goal_callback(self, goal_request):
        """Callback pour accepter/rejeter les buts d'action"""
        with self.navigation_lock:
            if self.navigation_active:
                self.get_logger().info("Navigation déjà active, rejet du nouveau but")
                return GoalResponse.REJECT
        
        # Vérifier que MAVROS est connecté
        if not self.mavros_state or not self.mavros_state.connected:
            self.get_logger().warning("MAVROS non connecté, rejet du but")
            return GoalResponse.REJECT
            
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Callback pour annuler une action"""
        self.get_logger().info("Demande d'annulation reçue")
        self._cancel_requested.set()
        return CancelResponse.ACCEPT

    def execute_goto_action(self, goal_handle):
        """Exécution de l'action de navigation GPS avec commandes réelles"""
        self.get_logger().info(
            f"Action GPS: lat={goal_handle.request.latitude}, "
            f"lon={goal_handle.request.longitude}, alt={goal_handle.request.altitude}"
        )
        
        # Reset du flag d'annulation
        self._cancel_requested.clear()
        
        with self.navigation_lock:
            self.navigation_active = True
            self.navigation_mode = "GPS"
            self.target_position = {
                'latitude': goal_handle.request.latitude,
                'longitude': goal_handle.request.longitude,
                'altitude': goal_handle.request.altitude,
                'yaw': getattr(goal_handle.request, 'yaw_angle', 0.0)
            }

        feedback_msg = GotoPositionAction.Feedback()
        result = GotoPositionAction.Result()
        
        try:
            self.publish_status("NAVIGATING")
            
            # Navigation en boucle fermée
            max_time = 120.0  # Timeout de 2 minutes
            start_time = time.time()
            
            while time.time() - start_time < max_time:
                # Vérifier l'annulation
                if self._cancel_requested.is_set() or goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    result.success = False
                    result.message = "Navigation annulée"
                    self.publish_status("ABORTED")
                    return result
                
                # Calculer la progression
                distance_remaining = self.calculate_gps_distance(
                    self.target_position['latitude'],
                    self.target_position['longitude'],
                    self.target_position['altitude']
                )
                
                # Vérifier si on a atteint la cible
                if distance_remaining < goal_handle.request.tolerance:
                    result.success = True
                    result.message = "Position cible atteinte"
                    result.final_position = self.get_current_position()
                    goal_handle.succeed()
                    self.publish_status("SUCCEEDED")
                    return result
                
                # Publier le feedback
                progress = max(0.0, min(1.0, 1.0 - (distance_remaining / 100.0)))
                feedback_msg.progress = progress
                feedback_msg.distance_remaining = distance_remaining
                feedback_msg.current_position = self.get_current_position()
                feedback_msg.current_status = "NAVIGATING"
                goal_handle.publish_feedback(feedback_msg)
                
                # Publier la progression
                self.publish_progress(
                    progress, 
                    distance_remaining, 
                    self.get_current_position(),
                    self.gps_to_point(self.target_position),
                    "NAVIGATING"
                )
                
                time.sleep(0.5)  # Vérifier toutes les 500ms
            
            # Timeout atteint
            result.success = False
            result.message = f"Timeout atteint ({max_time}s) - Position non atteinte"
            goal_handle.abort()
            self.publish_status("FAILED")
            
        except Exception as e:
            self.get_logger().error(f"Erreur pendant la navigation: {str(e)}")
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

    def handle_goto_position(self, request, response):
        """Service handler pour navigation GPS"""
        with self.navigation_lock:
            if self.navigation_active:
                response.success = False
                response.message = "Navigation déjà en cours"
                return response
        
        # Vérifier MAVROS
        if not self.mavros_state or not self.mavros_state.connected:
            response.success = False
            response.message = "MAVROS non connecté"
            return response
        
        self.get_logger().info(
            f"Service GPS: lat={request.latitude}, "
            f"lon={request.longitude}, alt={request.altitude}"
        )
        
        try:
            with self.navigation_lock:
                self.navigation_active = True
                self.navigation_mode = "GPS"
                self.target_position = {
                    'latitude': request.latitude,
                    'longitude': request.longitude,
                    'altitude': request.altitude,
                    'yaw': getattr(request, 'yaw_angle', 0.0)
                }
            
            self.publish_status("NAVIGATING")
            
            # Navigation avec timeout
            success = self.navigate_to_gps_position(timeout=60.0)
            
            if success:
                response.success = True
                response.message = "Navigation GPS terminée"
                self.publish_status("SUCCEEDED")
            else:
                response.success = False
                response.message = "Navigation GPS échouée (timeout ou erreur)"
                self.publish_status("FAILED")
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur navigation GPS: {str(e)}"
            self.publish_status("FAILED")
        
        finally:
            with self.navigation_lock:
                self.navigation_active = False
                self.navigation_mode = "IDLE"
                self.target_position = None
            
        return response

    def handle_goto_local(self, request, response):
        """Service handler pour navigation locale"""
        with self.navigation_lock:
            if self.navigation_active:
                response.success = False
                response.message = "Navigation déjà en cours"
                return response
        
        # Vérifier MAVROS
        if not self.mavros_state or not self.mavros_state.connected:
            response.success = False
            response.message = "MAVROS non connecté"
            return response
        
        self.get_logger().info(f"Service Local: x={request.x}, y={request.y}, z={request.z}")
        
        try:
            with self.navigation_lock:
                self.navigation_active = True
                self.navigation_mode = "LOCAL"
                self.target_position = {
                    'x': request.x,
                    'y': request.y,
                    'z': request.z,
                    'yaw': getattr(request, 'yaw_angle', 0.0)
                }
            
            self.publish_status("NAVIGATING")
            
            # Navigation avec timeout
            success = self.navigate_to_local_position(timeout=60.0)
            
            if success:
                response.success = True
                response.message = "Navigation locale terminée"
                self.publish_status("SUCCEEDED")
            else:
                response.success = False
                response.message = "Navigation locale échouée (timeout ou erreur)"
                self.publish_status("FAILED")
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur navigation locale: {str(e)}"
            self.publish_status("FAILED")
        
        finally:
            with self.navigation_lock:
                self.navigation_active = False
                self.navigation_mode = "IDLE"
                self.target_position = None
            
        return response

    def navigate_to_gps_position(self, timeout=60.0):
        """Navigation GPS en boucle fermée"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            distance = self.calculate_gps_distance(
                self.target_position['latitude'],
                self.target_position['longitude'],
                self.target_position['altitude']
            )
            
            if distance < self.gps_tolerance * 111320:  # Conversion degrés -> mètres
                self.get_logger().info(f"Position GPS atteinte (distance: {distance:.2f}m)")
                return True
            
            # Publier la progression
            progress = max(0.0, min(1.0, 1.0 - (distance / 100.0)))
            self.publish_progress(
                progress, 
                distance,
                self.get_current_position(),
                self.gps_to_point(self.target_position),
                "NAVIGATING" if distance > 1.0 else "REACHED"
            )
            
            time.sleep(0.5)
        
        return False

    def navigate_to_local_position(self, timeout=60.0):
        """Navigation locale en boucle fermée"""
        start_time = time.time()
        
        while time.time() - start_time < timeout:
            if not self.current_pose:
                time.sleep(0.1)
                continue
                
            distance = math.sqrt(
                (self.current_pose.pose.position.x - self.target_position['x'])**2 +
                (self.current_pose.pose.position.y - self.target_position['y'])**2 +
                (self.current_pose.pose.position.z - self.target_position['z'])**2
            )
            
            if distance < self.position_tolerance:
                self.get_logger().info(f"Position locale atteinte (distance: {distance:.2f}m)")
                return True
            
            # Publier la progression
            progress = max(0.0, min(1.0, 1.0 - (distance / 50.0)))
            target_point = Point(
                x=self.target_position['x'],
                y=self.target_position['y'], 
                z=self.target_position['z']
            )
            
            self.publish_progress(
                progress,
                distance,
                self.get_current_position(),
                target_point,
                "NAVIGATING" if distance > self.position_tolerance else "REACHED"
            )
            
            time.sleep(0.5)
        
        return False

    def publish_setpoints(self):
        """Timer callback pour publier les setpoints en continu"""
        if not self.navigation_active or not self.target_position:
            return
        
        if self.navigation_mode == "GPS":
            # Publier setpoint GPS
            msg = GeoPoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.latitude = self.target_position['latitude']
            msg.pose.position.longitude = self.target_position['longitude']
            msg.pose.position.altitude = self.target_position['altitude']
            msg.pose.orientation.w = 1.0  # Orientation par défaut
            
            self.global_setpoint_pub.publish(msg)
            
        elif self.navigation_mode == "LOCAL":
            # Publier setpoint local
            msg = PoseStamped()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = "map"
            msg.pose.position.x = self.target_position['x']
            msg.pose.position.y = self.target_position['y']
            msg.pose.position.z = self.target_position['z']
            msg.pose.orientation.w = 1.0  # Orientation par défaut
            
            self.local_setpoint_pub.publish(msg)

    def calculate_gps_distance(self, target_lat, target_lon, target_alt):
        """Calculer la distance vers une position GPS"""
        if not self.current_gps:
            return float('inf')
        
        # Distance horizontale (approximation)
        dlat = target_lat - self.current_gps.latitude
        dlon = target_lon - self.current_gps.longitude
        horizontal_distance = math.sqrt(dlat*dlat + dlon*dlon) * 111320  # Conversion en mètres
        
        # Distance verticale
        vertical_distance = abs(target_alt - self.current_gps.altitude)
        
        # Distance totale
        return math.sqrt(horizontal_distance*horizontal_distance + vertical_distance*vertical_distance)

    def gps_to_point(self, gps_pos):
        """Convertir une position GPS en Point pour l'affichage"""
        return Point(
            x=gps_pos['longitude'],
            y=gps_pos['latitude'],
            z=gps_pos['altitude']
        )

    def pose_callback(self, msg):
        """Callback pour la position courante du drone"""
        self.current_pose = msg

    def gps_callback(self, msg):
        """Callback pour la position GPS courante"""
        self.current_gps = msg

    def state_callback(self, msg):
        """Callback pour l'état MAVROS"""
        self.mavros_state = msg

    def get_current_position(self):
        """Retourne la position courante du drone"""
        if self.current_pose:
            return Point(
                x=self.current_pose.pose.position.x,
                y=self.current_pose.pose.position.y,
                z=self.current_pose.pose.position.z
            )
        return Point(x=0.0, y=0.0, z=0.0)

    def publish_progress(self, progress, distance_remaining, current_pos, target_pos, status):
        """Publier la progression de navigation"""
        msg = PathProgress()
        msg.stamp = self.get_clock().now().to_msg()
        msg.progress = progress
        msg.distance_remaining = distance_remaining
        msg.current_position = current_pos
        msg.target_position = target_pos
        msg.status = status
        self.progress_pub.publish(msg)

    def publish_status(self, status):
        """Publier l'état de navigation"""
        msg = NavigationStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.status = status
        msg.mode = self.navigation_mode
        msg.progress = 0.0 if status in ["IDLE", "NAVIGATING"] else 1.0
        msg.message = f"Navigation {status.lower()}"
        
        if self.target_position and self.navigation_mode == "LOCAL":
            msg.target_position = Point(
                x=self.target_position['x'],
                y=self.target_position['y'],
                z=self.target_position['z']
            )
        elif self.target_position and self.navigation_mode == "GPS":
            msg.target_position = self.gps_to_point(self.target_position)
        else:
            msg.target_position = Point()
            
        self.status_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    
    node = GotoPositionNode()
    executor = MultiThreadedExecutor(num_threads=4)
    
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()