#!/usr/bin/env python3
"""
navigation_safety_node.py - Nœud de vérification de sécurité pour la navigation
"""

import rclpy
import threading
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Float64, String
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState
from sensor_msgs.msg import NavSatFix
from geometry_msgs.msg import PoseStamped

class NavigationSafetyNode(Node):
    def __init__(self):
        super().__init__('navigation_safety_node')
        
    # Thread safety
    # _state_lock : protège l'accès concurrent à l'état du drone (batterie, GPS, etc.)
    # _publish_lock : protège la publication sur les topics
        self._state_lock = threading.RLock()
        self._publish_lock = threading.RLock()
        
    # État interne protégé
    # _mavros_state : dernier état MAVROS reçu
    # _battery_level : niveau de batterie en %
    # _current_gps : dernière position GPS reçue
    # _current_pose : dernière position locale reçue
    # _current_rel_alt : dernière altitude relative reçue
    # _last_safety_check : timestamp du dernier check sécurité
    # _altitude_violations : compteur pour violations temporaires d'altitude
        self._mavros_state = None
        self._battery_level = 100.0
        self._current_gps = None
        self._current_pose = None
        self._current_rel_alt = None
        self._last_safety_check = None
        self._altitude_violations = 0
        self._max_altitude_violations = 3  # Tolérer 3 violations consécutives
        
    # Paramètres de sécurité
    # low_battery_threshold : seuil de batterie faible
    # min_takeoff_altitude : altitude minimale pour navigation (réduite pour tolérer les variations)
    # max_speed : vitesse maximale autorisée
    # safety_check_rate : fréquence des vérifications de sécurité
        self.declare_parameter('low_battery_threshold', 20.0)
        self.declare_parameter('min_takeoff_altitude', 0.8)  # Réduit de 1.5m à 0.8m
        self.declare_parameter('max_speed', 15.0)
        self.declare_parameter('safety_check_rate', 1.0)
        
        self._low_battery_threshold = self.get_parameter('low_battery_threshold').value
        self._min_takeoff_altitude = self.get_parameter('min_takeoff_altitude').value
        self._safety_check_rate = self.get_parameter('safety_check_rate').value
        
    # Publishers thread-safe
    # _safe_to_navigate_pub : publie l'état de sécurité global
    # _safety_status_pub : publie le message de statut de sécurité
        self._safe_to_navigate_pub = self.create_publisher(Bool, '/drone_nav/safe_to_navigate', 10)
        self._safety_status_pub = self.create_publisher(String, '/drone_nav/safety_status', 10)
        
    # QoS profile pour topics MAVROS
        mavros_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        battery_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        
    # Subscribers
    # /mavros/state : reçoit l'état MAVROS
    # /mavros/battery : reçoit le niveau de batterie
    # /mavros/global_position/global : reçoit la position GPS
    # /mavros/local_position/pose : reçoit la position locale
    # /mavros/global_position/rel_alt : reçoit l'altitude relative
        self.create_subscription(
            State, '/mavros/state', self._state_callback, mavros_qos
        )
        
        self.create_subscription(
            BatteryState, '/mavros/battery', self._battery_callback, battery_qos
        )
        
        self.create_subscription(
            NavSatFix, '/mavros/global_position/global', self._gps_callback, mavros_qos
        )
        
        self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self._pose_callback, mavros_qos
        )
        
        self.create_subscription(
            Float64, '/mavros/global_position/rel_alt', self._rel_alt_callback, mavros_qos
        )
        
    # Timer pour les vérifications de sécurité
    # Vérifie périodiquement les conditions de sécurité
        self._safety_timer = self.create_timer(
            1.0 / self._safety_check_rate, 
            self._check_safety_timer_callback
        )
        
        self.get_logger().info("Navigation Safety Node initialized")

    # Propriétés thread-safe pour l'état
    # Les propriétés suivantes protègent l'accès concurrent aux variables d'état
    @property
    def mavros_state(self):
        with self._state_lock:
            return self._mavros_state
    
    @property
    def battery_level(self):
        with self._state_lock:
            return self._battery_level
    
    @property
    def current_gps(self):
        with self._state_lock:
            return self._current_gps
    
    @property
    def current_pose(self):
        with self._state_lock:
            return self._current_pose
    
    @property
    def current_rel_alt(self):
        with self._state_lock:
            return self._current_rel_alt

    def _state_callback(self, msg):
        # Callback appelé à chaque message d'état MAVROS reçu
        with self._state_lock:
            self._mavros_state = msg

    def _battery_callback(self, msg):
        # Callback appelé à chaque message de batterie reçu
        with self._state_lock:
            self._battery_level = msg.percentage * 100.0

    def _gps_callback(self, msg):
        # Callback appelé à chaque message GPS reçu
        with self._state_lock:
            self._current_gps = msg

    def _pose_callback(self, msg):
        # Callback appelé à chaque message de position locale reçu
        with self._state_lock:
            self._current_pose = msg

    def _rel_alt_callback(self, msg):
        # Callback appelé à chaque message d'altitude relative reçu
        with self._state_lock:
            self._current_rel_alt = msg.data

    def _check_safety_timer_callback(self):
        # Timer : vérifie périodiquement les conditions de sécurité
        """Callback du timer pour vérification de sécurité périodique"""
        try:
            safe, message = self._perform_safety_check()
            self._publish_safety_status(safe, message)
            
            with self._state_lock:
                self._last_safety_check = self.get_clock().now()
                
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la vérification de sécurité: {e}")

    def _perform_safety_check(self):
        # Vérifie si toutes les conditions de sécurité sont remplies pour autoriser la navigation
        """Vérifie si les conditions de sécurité sont remplies pour la navigation"""
        
        # Obtenir une copie thread-safe des états
        mavros_state = self.mavros_state
        battery_level = self.battery_level
        current_gps = self.current_gps
        current_pose = self.current_pose
        current_rel_alt = self.current_rel_alt
        
        # Vérifications de base
        if not mavros_state:
            return False, "État MAVROS non disponible"
        
        if not mavros_state.connected:
            return False, "MAVROS non connecté"
        
        if not mavros_state.armed:
            return False, "Drone non armé"
        
        # Vérification du mode de vol
        valid_modes = ['GUIDED', 'AUTO', 'LOITER', 'POSCTL', 'POSITION', 'STABILIZED']
        if mavros_state.mode not in valid_modes:
            return False, f"Mode {mavros_state.mode} non supporté"
        
        # Vérification de l'altitude minimale avec tolérance pour variations temporaires
        if current_rel_alt is not None:
            if current_rel_alt < self._min_takeoff_altitude:
                # Incrémenter le compteur de violations
                with self._state_lock:
                    self._altitude_violations += 1
                
                # Seulement échouer si on a trop de violations consécutives
                if self._altitude_violations > self._max_altitude_violations:
                    return False, f"Altitude insuffisante persistante ({current_rel_alt:.1f}m < {self._min_takeoff_altitude:.1f}m)"
                else:
                    # Violation temporaire tolérée
                    pass
            else:
                # Altitude OK, réinitialiser le compteur
                with self._state_lock:
                    self._altitude_violations = 0
        
        # Vérification de la position locale
        if not current_pose:
            return False, "Position locale non disponible"
        
        # Vérification GPS
        if current_gps:
            if (current_gps.latitude == 0.0 or 
                current_gps.longitude == 0.0 or
                abs(current_gps.latitude) > 90.0 or
                abs(current_gps.longitude) > 180.0):
                return False, "Coordonnées GPS invalides"
            
            # Vérification altitude GPS raisonnable
            if current_gps.altitude < -500 or current_gps.altitude > 10000:
                return False, "Altitude GPS invalide"
                
            # Vérification de la précision GPS (variance X)
            if hasattr(current_gps, 'position_covariance') and len(current_gps.position_covariance) > 0:
                if current_gps.position_covariance[0] > 100.0:  # Variance X trop élevée
                    return False, "Précision GPS insuffisante"
        else:
            return False, "Données GPS non disponibles"
        
        # Vérification de la batterie
        if battery_level < self._low_battery_threshold:
            return False, f"Batterie faible: {battery_level:.1f}%"
        
        return True, "Prêt pour navigation"

    def _publish_safety_status(self, safe, message):
        # Publie l'état de sécurité (booléen et message) sur les topics dédiés
        """Publication thread-safe du statut de sécurité"""
        try:
            with self._publish_lock:
                # Log avec emoji avant publication
                safety_emoji = "🛡️✅" if safe else "🛡️❌"
                self.get_logger().info(f"{safety_emoji} SAFETY NODE: Publication safe_to_navigate = {safe} - {message}")
                
                # Publier le statut booléen
                safe_msg = Bool()
                safe_msg.data = safe
                self._safe_to_navigate_pub.publish(safe_msg)
                
                # Publier le message de statut
                status_msg = String()
                status_msg.data = message
                self._safety_status_pub.publish(status_msg)
                
            # Log selon le niveau approprié
            if not safe:
                self.get_logger().warn(f"🚨 Safety check failed: {message}")
            else:
                self.get_logger().debug(f"✅ Safety check passed: {message}")
                
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la publication du statut: {e}")

    def get_safety_status(self):
        # Méthode publique pour obtenir le statut de sécurité actuel
        """Méthode publique pour obtenir le statut de sécurité actuel"""
        return self._perform_safety_check()

    def destroy_node(self):
        # Nettoyage du nœud : arrêt du timer et destruction propre
        """Nettoyage lors de la destruction du nœud"""
        try:
            if hasattr(self, '_safety_timer'):
                self._safety_timer.destroy()
            self.get_logger().info("Navigation Safety Node destroyed cleanly")
        except Exception as e:
            self.get_logger().error(f"Erreur lors du nettoyage: {e}")
        finally:
            super().destroy_node()

def main(args=None):
    # Point d'entrée principal du nœud de sécurité navigation
    rclpy.init(args=args)
    
    node = NavigationSafetyNode()
    executor = MultiThreadedExecutor(num_threads=4)
    
    try:
        executor.add_node(node)
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info("Arrêt demandé par l'utilisateur")
    except Exception as e:
        node.get_logger().error(f"Erreur dans l'exécuteur: {e}")
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()

if __name__ == '__main__':
    main()