#!/usr/bin/env python3
"""
gps_navigation_node.py - Nœud de navigation GPS avec MultiThreadedExecutor
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
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
        
                # Variables d'état thread-safe protégées par des verrous
        self._state_lock = threading.Lock()
        self._pub_lock = threading.Lock()
        self._target_position = None
        self._current_gps = None
        self._safe_to_navigate = False
        self._navigation_active = False
        self._initial_distance = None
        self._navigation_future = None
        
        # Variables pour la gestion de l'altitude
        self._home_altitude = None  # Altitude AMSL au démarrage
        self._terrain_altitude = None  # Altitude estimée du terrain
        self._last_valid_altitude = None  # Dernière altitude valide connue
        
        # Log d'initialisation avec emoji
        self.get_logger().info("🛰️🟢 Safe to navigate initialisé à TRUE par défaut")
        
        # ThreadPoolExecutor pour exécuter la navigation en tâche de fond
        self._executor = ThreadPoolExecutor(max_workers=2)
        
        # Chargement des paramètres de configuration
        self.declare_parameter('default_tolerance', 2.0)
        self.declare_parameter('navigation_timeout', 300.0)
        self.declare_parameter('setpoint_rate', 10.0)  # Hz
        self.declare_parameter('altitude_mode', 'relative')  # 'relative' ou 'absolute'
        self.declare_parameter('relative_altitude_default', 50.0)  # Altitude relative par défaut
        self.declare_parameter('min_safe_altitude', 100.0)  # Altitude AMSL minimale de sécurité
        
        self.default_tolerance = self.get_parameter('default_tolerance').value
        self.navigation_timeout = self.get_parameter('navigation_timeout').value
        setpoint_rate = self.get_parameter('setpoint_rate').value
        self.altitude_mode = self.get_parameter('altitude_mode').value
        self.relative_altitude_default = self.get_parameter('relative_altitude_default').value
        self.min_safe_altitude = self.get_parameter('min_safe_altitude').value
        
        # Publishers pour diffuser les consignes, la progression et le statut
        self._pub_lock = threading.Lock()
        self.setpoint_pub = self.create_publisher(GeoPoseStamped, '/mavros/setpoint_position/global', 10)
        self.progress_pub = self.create_publisher(PathProgress, '/drone_nav/progress', 10)
        self.status_pub = self.create_publisher(NavigationStatus, '/drone_nav/status', 10)
        self.safety_pub = self.create_publisher(Bool, '/drone_nav/safe_to_navigate', 10)
        
        # Souscriptions aux topics de GPS
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self.gps_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        )
        
        # Services pour recevoir les commandes de navigation et mise à jour de cible
        self.goto_service = self.create_service(GotoPosition, '/drone_nav/goto_position', self.handle_goto_position)
        self.update_service = self.create_service(UpdatePosition, '/drone_nav/update_position', self.handle_update_position)
        
        # Action Server pour navigation avec feedback
        self.action_server = ActionServer(
            self, 
            GotoPositionAction, 
            '/drone_nav/goto_position_action', 
            execute_callback=self.execute_action,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )
        
        # Timers
        setpoint_period = 1.0 / setpoint_rate
        self.setpoint_timer = self.create_timer(setpoint_period, self.publish_setpoint)
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.safety_timer = self.create_timer(2.0, self.publish_safety_status)
        
        self.get_logger().info("GPS Navigation Node initialized with MultiThreadedExecutor")
        self.get_logger().info(f"Default tolerance: {self.default_tolerance}m, Timeout: {self.navigation_timeout}s")

    # Propriétés thread-safe pour accéder/modifier l'état interne
    @property
    def navigation_active(self):
        with self._state_lock:
            return self._navigation_active
    
    @navigation_active.setter
    def navigation_active(self, value):
        with self._state_lock:
            old_value = self._navigation_active
            self._navigation_active = value
            if old_value != value:
                self.get_logger().info(f"Navigation active changed: {old_value} -> {value}")

    @property
    def target_position(self):
        with self._state_lock:
            return self._target_position.copy() if self._target_position else None
    
    @target_position.setter
    def target_position(self, value):
        with self._state_lock:
            self._target_position = value
            if value:
                self.get_logger().info(f"New target set: lat={value['latitude']}, lon={value['longitude']}, alt={value['altitude']}")

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
            old_value = self._safe_to_navigate
            self._safe_to_navigate = value
            if old_value != value:
                change_emoji = "🔄" if value else "⚠️"
                self.get_logger().info(f"{change_emoji} Safety status changed: {old_value} -> {value}")
                # Log supplémentaire pour le débogage
                self.get_logger().info(f"🔍 Debug: Publication imminente de safe_to_navigate = {value}")

    def gps_callback(self, msg):
        """Callback pour les données GPS"""
        # Vérifier la validité du GPS
        if msg.status.status >= 0:  # GPS fix valide
            self.current_gps = msg
            
            # Initialiser les références d'altitude au premier GPS valide
            self._initialize_altitude_references()
            
            # Considérer comme sûr si on a un GPS valide
            if not self.safe_to_navigate:
                self.get_logger().info("🛰️✅ GPS fix récupéré - Navigation redevient sécurisée")
                self.safe_to_navigate = True
        else:
            # GPS invalide
            if self.safe_to_navigate:
                self.get_logger().warn("🛰️❌ GPS fix perdu - Navigation non sécurisée")
                self.safe_to_navigate = False
            # Arrêter la navigation si plus sécurisé
            if self.navigation_active:
                self._abort_current_navigation("GPS fix perdu")

    def publish_safety_status(self):
        """Publie le statut de sécurité"""
        msg = Bool()
        msg.data = self.safe_to_navigate
        
        # Log de débogage avec emoji
        status_emoji = "🟢" if self.safe_to_navigate else "🔴"
        self.get_logger().debug(f"📡 Publication safe_to_navigate: {status_emoji} {self.safe_to_navigate}")
        
        with self._pub_lock:
            self.safety_pub.publish(msg)

    def goal_callback(self, goal_request):
        """Callback pour accepter ou rejeter les goals"""
        if not self.safe_to_navigate:
            self.get_logger().warn("Goal rejeté - Navigation non sécurisée")
            return GoalResponse.REJECT
        
        if not self.current_gps:
            self.get_logger().warn("Goal rejeté - Pas de données GPS")
            return GoalResponse.REJECT
        
        self.get_logger().info("Goal accepté")
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """Callback pour l'annulation des goals"""
        self.get_logger().info("Demande d'annulation reçue")
        return CancelResponse.ACCEPT

    def handle_goto_position(self, request, response):
        """Service pour aller à une position"""
        if not self.safe_to_navigate:
            response.success = False
            response.message = "Non sécurisé pour naviguer"
            return response
        
        if not self.current_gps:
            response.success = False
            response.message = "Pas de données GPS disponibles"
            return response
        
        # Arrêter la navigation en cours si active
        if self.navigation_active:
            self.navigation_active = False
            if self._navigation_future:
                self._navigation_future.cancel()
        
        tolerance = request.tolerance if request.tolerance > 0 else self.default_tolerance
        
        # Valider et ajuster l'altitude
        validated_altitude, altitude_info = self._validate_altitude_request(request.altitude)
        self.get_logger().info(f"📍 Nouvelle navigation - {altitude_info}")
        
        self.target_position = {
            'latitude': request.latitude,
            'longitude': request.longitude,
            'altitude': validated_altitude,  # Utiliser l'altitude validée
            'original_altitude': request.altitude,  # Garder l'altitude originale pour référence
            'tolerance': tolerance
        }
        
        # Mémoriser la distance initiale
        self._initial_distance = self.calculate_distance()
        
        self.navigation_active = True
        self._safe_publish_status("NAVIGATING")
        
        # Navigation asynchrone avec ThreadPoolExecutor
        self._navigation_future = self._executor.submit(self._navigate_to_position)
        
        response.success = True
        response.message = "Navigation démarrée"
        return response

    def handle_update_position(self, request, response):
        """Service pour mettre à jour la position cible"""
        if not self.navigation_active:
            response.success = False
            response.message = "Aucune navigation active"
            return response
        
        tolerance = request.tolerance if request.tolerance > 0 else self.default_tolerance
        
        # Valider et ajuster l'altitude
        validated_altitude, altitude_info = self._validate_altitude_request(request.altitude)
        self.get_logger().info(f"🔄 Mise à jour position - {altitude_info}")
        
        # Mise à jour thread-safe
        with self._state_lock:
            if self._target_position:
                self._target_position.update({
                    'latitude': request.latitude,
                    'longitude': request.longitude,
                    'altitude': validated_altitude,  # Utiliser l'altitude validée
                    'original_altitude': request.altitude,  # Garder l'altitude originale
                    'tolerance': tolerance
                })
                # Recalculer la distance initiale pour la nouvelle cible
                self._initial_distance = self.calculate_distance()
        
        response.success = True
        response.message = "Position mise à jour"
        return response

    def execute_action(self, goal_handle):
        """Execute l'action de navigation avec feedback"""
        # Sauvegarder le goal_handle pour pouvoir l'annuler
        with self._state_lock:
            self._current_goal_handle = goal_handle
        
        try:
            # Vérifier la sécurité avant de commencer
            if not self.safe_to_navigate:
                result = GotoPositionAction.Result()
                result.success = False
                result.message = "Non sécurisé pour naviguer"
                goal_handle.abort()
                return result
            
            if not self.current_gps:
                result = GotoPositionAction.Result()
                result.success = False
                result.message = "Pas de données GPS disponibles"
                goal_handle.abort()
                return result
            
            # Annuler toute navigation en cours
            if self.navigation_active:
                self.get_logger().warn("Navigation en cours annulée pour nouveau goal")
                self.navigation_active = False
                if self._navigation_future:
                    self._navigation_future.cancel()
                    
            tolerance = goal_handle.request.tolerance if goal_handle.request.tolerance > 0 else self.default_tolerance
            
            # Valider et ajuster l'altitude
            validated_altitude, altitude_info = self._validate_altitude_request(goal_handle.request.altitude)
            self.get_logger().info(f"🎯 Action navigation - {altitude_info}")
            
            self.target_position = {
                'latitude': goal_handle.request.latitude,
                'longitude': goal_handle.request.longitude,
                'altitude': validated_altitude,  # Utiliser l'altitude validée
                'original_altitude': goal_handle.request.altitude,  # Garder l'altitude originale
                'tolerance': tolerance,
                'yaw_angle': goal_handle.request.yaw_angle
            }
            
            # Mémoriser la distance initiale
            self._initial_distance = self.calculate_distance()
            
            self.navigation_active = True
            self._safe_publish_status("NAVIGATING")

            self.get_logger().info(f"Target Position: lat={self.target_position['latitude']}, "
                                 f"lon={self.target_position['longitude']}, alt={self.target_position['altitude']}")
            self.get_logger().info(f"Initial Distance: {self._initial_distance:.2f} meters")
            
            # Navigation avec feedback dans ThreadPoolExecutor
            future = self._executor.submit(self._navigate_with_feedback, goal_handle)
            success = future.result()  # Attend la fin
            
            result = GotoPositionAction.Result()
            result.success = success
            result.message = "Navigation terminée" if success else "Navigation échouée"
            
            if success:
                goal_handle.succeed()
                self._safe_publish_status("SUCCEEDED")
            else:
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self._safe_publish_status("CANCELLED")
                else:
                    goal_handle.abort()
                    self._safe_publish_status("FAILED")
            
            return result
            
        except Exception as e:
            self.get_logger().error(f"Erreur dans execute_action: {e}")
            result = GotoPositionAction.Result()
            result.success = False
            result.message = f"Erreur: {e}"
            goal_handle.abort()
            self._safe_publish_status("FAILED")
            return result
        finally:
            # Nettoyer le goal_handle
            with self._state_lock:
                self._current_goal_handle = None
                self.navigation_active = False

    def _navigate_with_feedback(self, goal_handle):
        """Navigation avec feedback pour l'action"""
        try:
            step_duration = 0.5
            max_steps = int(self.navigation_timeout / step_duration)
            
            self.get_logger().info(f"Démarrage navigation avec feedback - timeout: {self.navigation_timeout}s")
            
            for step in range(max_steps):
                # Vérifier les conditions d'arrêt
                if goal_handle.is_cancel_requested:
                    self.get_logger().info("Navigation annulée par demande")
                    self.navigation_active = False
                    return False
                    
                if not self.safe_to_navigate:
                    self.get_logger().warn("Navigation arrêtée - non sécurisée")
                    self.navigation_active = False
                    return False
                    
                if not self.navigation_active:
                    self.get_logger().warn("Navigation arrêtée - flag inactif")
                    return False
                
                distance = self.calculate_distance()
                target = self.target_position
                
                if not target or distance == float('inf'):
                    self.get_logger().error("Cible ou GPS invalide")
                    break
                
                tolerance = target.get('tolerance', self.default_tolerance)
                
                # Log détaillé pour debug
                if step % 4 == 0:  # Log toutes les 2 secondes
                    current = self.current_gps
                    if current:
                        self.get_logger().info(f"Step {step}: Distance={distance:.2f}m, "
                                             f"Tolérance={tolerance:.2f}m, "
                                             f"Position actuelle: lat={current.latitude:.8f}, "
                                             f"lon={current.longitude:.8f}, alt={current.altitude:.2f}")
                        self.get_logger().info(f"Position cible: lat={target['latitude']:.8f}, "
                                             f"lon={target['longitude']:.8f}, alt={target['altitude']:.2f}")
                
                # Publier le feedback
                feedback = GotoPositionAction.Feedback()
                feedback.distance_remaining = distance
                
                # Calculer la progression
                if self._initial_distance and self._initial_distance > 0:
                    progress = max(0.0, min(1.0, 1.0 - (distance / self._initial_distance)))
                else:
                    progress = 0.0
                feedback.progress = progress
                
                if self.current_gps:
                    feedback.current_position = Point(
                        x=self.current_gps.longitude,
                        y=self.current_gps.latitude, 
                        z=self.current_gps.altitude
                    )
                
                try:
                    goal_handle.publish_feedback(feedback)
                except Exception as e:
                    self.get_logger().debug(f"Erreur publication feedback: {e}")
                
                # Vérifier si arrivé (condition principale)
                if distance <= tolerance:
                    self.navigation_active = False
                    self.get_logger().info(f"SUCCÈS: Waypoint atteint! Distance finale: {distance:.2f}m "
                                         f"(tolérance: {tolerance:.2f}m)")
                    return True
                
                # Publier la progression
                self._safe_publish_progress(distance, progress)
                
                time.sleep(step_duration)
            
            # Timeout atteint
            self.navigation_active = False
            final_distance = self.calculate_distance()
            self.get_logger().error(f"TIMEOUT: Navigation échouée après {self.navigation_timeout}s. "
                                  f"Distance finale: {final_distance:.2f}m, Tolérance: {tolerance:.2f}m")
            return False
            
        except Exception as e:
            self.get_logger().error(f"Erreur navigation avec feedback: {e}")
            self.navigation_active = False
            return False

    def _navigate_to_position(self):
        """Navigation simple sans feedback"""
        try:
            step_duration = 0.5
            max_steps = int(self.navigation_timeout / step_duration)
            
            for step in range(max_steps):
                if not self.safe_to_navigate or not self.navigation_active:
                    break
                
                distance = self.calculate_distance()
                target = self.target_position
                
                if not target or distance == float('inf'):
                    break
                
                tolerance = target.get('tolerance', self.default_tolerance)
                
                if distance <= tolerance:
                    self.navigation_active = False
                    self._safe_publish_status("SUCCEEDED")
                    self.get_logger().info(f"Arrivé à la position cible (distance: {distance:.2f}m)")
                    return True
                
                # Calculer la progression
                progress = 0.0
                if self._initial_distance and self._initial_distance > 0:
                    progress = max(0.0, min(1.0, 1.0 - (distance / self._initial_distance)))
                
                self._safe_publish_progress(distance, progress)
                
                time.sleep(step_duration)
            
            self.navigation_active = False
            self._safe_publish_status("FAILED")
            return False
            
        except Exception as e:
            self.get_logger().error(f"Erreur navigation: {e}")
            self.navigation_active = False
            self._safe_publish_status("FAILED")
            return False

    def _abort_current_navigation(self, reason="Navigation interrompue"):
        """Interrompt la navigation courante"""
        with self._state_lock:
            if self._current_goal_handle and not self._current_goal_handle.is_cancel_requested:
                try:
                    self._current_goal_handle.abort()
                    self.get_logger().info(f"Navigation interrompue: {reason}")
                except Exception as e:
                    self.get_logger().warn(f"Erreur lors de l'interruption: {e}")
            self.navigation_active = False

    def calculate_distance(self):
        """Calcule la distance à la cible"""
        current = self.current_gps
        target = self.target_position
        
        if not current or not target:
            return float('inf')
        
        try:
            # Calcul géodésique thread-safe avec protection contre les valeurs invalides
            lat1, lon1 = math.radians(current.latitude), math.radians(current.longitude)
            lat2, lon2 = math.radians(target['latitude']), math.radians(target['longitude'])
            
            dlat = lat2 - lat1
            dlon = lon2 - lon1
            
            # Formule haversine améliorée
            a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
            # Protection contre les erreurs d'arrondi
            a = min(1.0, max(0.0, a))
            c = 2 * math.asin(math.sqrt(a))
            horizontal_dist = 6371000 * c  # Rayon de la Terre en mètres
            
            # Distance verticale
            alt_dist = abs(target['altitude'] - current.altitude)
            
            # Distance totale 3D
            total_distance = math.sqrt(horizontal_dist**2 + alt_dist**2)
            
            # Debug logging
            self.get_logger().debug(f"Distance calculée: {total_distance:.2f}m "
                                  f"(horizontal: {horizontal_dist:.2f}m, vertical: {alt_dist:.2f}m)")
            
            return total_distance
            
        except Exception as e:
            self.get_logger().error(f"Erreur calcul distance: {e}")
            return float('inf')

    def _initialize_altitude_references(self):
        """Initialise les références d'altitude au démarrage"""
        if self.current_gps and self._home_altitude is None:
            self._home_altitude = self.current_gps.altitude
            # Estimation simple du terrain (peut être améliorée avec des données DEM)
            self._terrain_altitude = self._home_altitude - 10.0  # Assume 10m au-dessus du terrain
            self.get_logger().info(f"🏠 Altitude références initialisées - Home: {self._home_altitude:.1f}m AMSL, "
                                 f"Terrain estimé: {self._terrain_altitude:.1f}m AMSL")

    def _convert_altitude(self, requested_altitude, is_relative=None):
        """
        Convertit l'altitude selon le mode configuré
        
        Args:
            requested_altitude: Altitude demandée par l'utilisateur
            is_relative: Force le mode relatif (True) ou absolu (False), sinon utilise la config
            
        Returns:
            tuple: (altitude_amsl, mode_info_string)
        """
        if is_relative is None:
            is_relative = (self.altitude_mode == 'relative')
        
        if is_relative:
            # Mode relatif : ajouter à l'altitude du terrain estimée
            terrain_alt = self._terrain_altitude or (self._home_altitude - 10.0 if self._home_altitude else 100.0)
            altitude_amsl = terrain_alt + requested_altitude
            mode_info = f"relative +{requested_altitude:.1f}m (terrain ~{terrain_alt:.1f}m)"
        else:
            # Mode absolu : utiliser directement l'altitude AMSL
            altitude_amsl = requested_altitude
            mode_info = f"absolue {requested_altitude:.1f}m AMSL"
        
        # Vérification de sécurité
        if altitude_amsl < self.min_safe_altitude:
            self.get_logger().warn(f"⚠️ Altitude {altitude_amsl:.1f}m AMSL trop basse ! "
                                 f"Minimum sécurisé: {self.min_safe_altitude:.1f}m")
            altitude_amsl = self.min_safe_altitude
            mode_info += f" → ajustée à {altitude_amsl:.1f}m (sécurité)"
        
        return altitude_amsl, mode_info

    def _validate_altitude_request(self, requested_altitude):
        """
        Valide et ajuste une demande d'altitude
        
        Returns:
            tuple: (altitude_amsl_valide, message_info)
        """
        # Auto-détection du mode basé sur la valeur
        if requested_altitude < 200:  # Probablement relatif
            is_relative = True
            self.get_logger().info(f"🔍 Auto-détection: altitude {requested_altitude:.1f}m interprétée comme RELATIVE")
        else:  # Probablement absolu
            is_relative = False
            self.get_logger().info(f"🔍 Auto-détection: altitude {requested_altitude:.1f}m interprétée comme ABSOLUE")
        
        altitude_amsl, mode_info = self._convert_altitude(requested_altitude, is_relative)
        
        return altitude_amsl, f"Altitude: {mode_info}"

    def publish_setpoint(self):
        """Publie les consignes de position"""
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
        
        # Gérer l'orientation si spécifiée
        yaw = target.get('yaw_angle', 0.0)
        msg.pose.orientation.w = math.cos(yaw / 2.0)
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = math.sin(yaw / 2.0)
        
        with self._pub_lock:
            self.setpoint_pub.publish(msg)

    def _safe_publish_progress(self, distance, progress):
        """Publie la progression de manière thread-safe"""
        msg = PathProgress()
        msg.stamp = self.get_clock().now().to_msg()
        msg.progress = progress
        msg.distance_remaining = distance
        
        current = self.current_gps
        target = self.target_position
        
        if current:
            msg.current_position = Point(x=current.longitude, y=current.latitude, z=current.altitude)
        
        if target:
            msg.target_position = Point(x=target['longitude'], y=target['latitude'], z=target['altitude'])
        
        msg.status = "NAVIGATING" if self.navigation_active else "IDLE"
        
        with self._pub_lock:
            self.progress_pub.publish(msg)

    def _safe_publish_status(self, status):
        """Publie le statut de navigation"""
        msg = NavigationStatus()
        msg.stamp = self.get_clock().now().to_msg()
        msg.status = status
        msg.mode = "GPS"
        
        target = self.target_position
        if target:
            msg.target_position = Point(x=target['longitude'], y=target['latitude'], z=target['altitude'])
        
        # Calculer la progression
        progress = 0.0
        if self._initial_distance and self._initial_distance > 0:
            current_distance = self.calculate_distance()
            if current_distance < float('inf'):
                progress = max(0.0, min(1.0, 1.0 - (current_distance / self._initial_distance)))
        
        msg.progress = progress
        msg.message = f"GPS Navigation - {status}"
        
        with self._pub_lock:
            self.status_pub.publish(msg)

    def publish_status(self):
        """Timer pour publier le statut régulièrement"""
        if self.navigation_active:
            self._safe_publish_status("NAVIGATING")
        else:
            self._safe_publish_status("IDLE")

    def destroy_node(self):
        """Nettoyage du nœud"""
        self.get_logger().info("Arrêt du GPS Navigation Node...")
        
        # Nettoyage thread-safe
        self.navigation_active = False
        if self._navigation_future:
            self._navigation_future.cancel()
        
        # Arrêter les actions en cours
        with self._state_lock:
            if self._current_goal_handle:
                try:
                    self._current_goal_handle.abort()
                except Exception as e:
                    self.get_logger().warn(f"Erreur lors de l'arrêt du goal: {e}")
        
        if self._executor:
            self._executor.shutdown(wait=True)
        
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = GPSNavigationNode()
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Interruption clavier reçue")
    except Exception as e:
        node.get_logger().error(f"Erreur dans l'executor: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()