#!/usr/bin/env python3

"""
Outil CLI pour surveillance de l'état de navigation

Affiche l'état en temps réel du système de navigation avec
métriques de performance et diagnostics détaillés.

Auteur: Adama Komi
Version: 1.1.0
"""

import argparse
import sys
import time
from typing import Optional, Dict, Any
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from std_msgs.msg import String
from geometry_msgs.msg import Point, Twist, PoseStamped
from sensor_msgs.msg import NavSatFix
from mavros_msgs.msg import State as MavrosState


class NavStatusTool(Node):
    """
    Outil CLI pour surveillance navigation
    
    Fournit une surveillance en temps réel de l'état du système
    de navigation du drone avec affichage formaté et options
    d'export.
    """
    
    def __init__(self):
        super().__init__('nav_status_tool')
        
        # Configuration QoS pour data temps réel
        self._qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # État de navigation
        self._navigation_status = None
        self._navigation_metrics = None
        self._position = None
        self._velocity = None
        self._gps_status = None
        self._mavros_state = None
        
        # Threading pour affichage continu
        self._display_thread = None
        self._running = False
        self._lock = threading.Lock()
        
        # Statistiques
        self._stats = {
            'messages_received': 0,
            'last_update': None,
            'update_rate': 0.0,
            'connection_status': 'DISCONNECTED'
        }
        
        # Initialisation des subscribers
        self._setup_subscribers()
        
        self.get_logger().info("Outil nav_status initialisé")

    def _setup_subscribers(self):
        """Configure les subscribers pour surveillance"""
        
        # Position locale
        self._position_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self._position_callback,
            self._qos_profile
        )
        
        # Vélocité
        self._velocity_sub = self.create_subscription(
            Twist,
            '/mavros/local_position/velocity_local',
            self._velocity_callback,
            self._qos_profile
        )
        
        # GPS
        self._gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self._gps_callback,
            self._qos_profile
        )
        
        # État MAVROS
        self._mavros_state_sub = self.create_subscription(
            MavrosState,
            '/mavros/state',
            self._mavros_state_callback,
            self._qos_profile
        )
        
        # Statut de navigation personnalisé
        self._nav_status_sub = self.create_subscription(
            String,
            '/drone_nav/status',
            self._nav_status_callback,
            self._qos_profile
        )
        
        # Métriques de navigation personnalisées
        self._nav_metrics_sub = self.create_subscription(
            String,
            '/drone_nav/metrics',
            self._nav_metrics_callback,
            self._qos_profile
        )

    def _nav_status_callback(self, msg):
        """
        Callback statut navigation
        
        Args:
            msg: Message de statut de navigation
        """
        with self._lock:
            self._navigation_status = msg
            self._update_stats()

    def _nav_metrics_callback(self, msg):
        """
        Callback métriques navigation
        
        Args:
            msg: Message de métriques de navigation
        """
        with self._lock:
            self._navigation_metrics = msg
            self._update_stats()

    def _position_callback(self, msg):
        """
        Callback position
        
        Args:
            msg: Message de position
        """
        with self._lock:
            self._position = msg
            self._update_stats()

    def _velocity_callback(self, msg):
        """
        Callback vélocité
        
        Args:
            msg: Message de vélocité
        """
        with self._lock:
            self._velocity = msg
            self._update_stats()

    def _gps_callback(self, msg):
        """
        Callback GPS
        
        Args:
            msg: Message GPS
        """
        with self._lock:
            self._gps_status = msg
            self._update_stats()

    def _mavros_state_callback(self, msg):
        """
        Callback état MAVROS
        
        Args:
            msg: Message d'état MAVROS
        """
        with self._lock:
            self._mavros_state = msg
            self._stats['connection_status'] = 'CONNECTED' if msg.connected else 'DISCONNECTED'
            self._update_stats()

    def _update_stats(self):
        """Met à jour les statistiques de réception"""
        current_time = time.time()
        
        if self._stats['last_update']:
            dt = current_time - self._stats['last_update']
            if dt > 0:
                # Calcul du taux de mise à jour (moyenne mobile)
                alpha = 0.1
                new_rate = 1.0 / dt
                self._stats['update_rate'] = (
                    alpha * new_rate + (1 - alpha) * self._stats['update_rate']
                )
        
        self._stats['messages_received'] += 1
        self._stats['last_update'] = current_time

    def start_monitoring(self, continuous: bool = False, refresh_rate: float = 1.0):
        """
        Démarre la surveillance
        
        Args:
            continuous: Mode surveillance continue
            refresh_rate: Taux de rafraîchissement en Hz
        """
        if continuous:
            self._running = True
            self._display_thread = threading.Thread(
                target=self._continuous_display,
                args=(refresh_rate,)
            )
            self._display_thread.start()
        else:
            self._single_display()

    def stop_monitoring(self):
        """Arrête la surveillance"""
        self._running = False
        if self._display_thread:
            self._display_thread.join()

    def _continuous_display(self, refresh_rate: float):
        """
        Affichage continu
        
        Args:
            refresh_rate: Taux de rafraîchissement en Hz
        """
        while self._running:
            try:
                # Nettoie l'écran
                print("\033[2J\033[H", end="")
                
                # Affiche les données
                self._display_status()
                
                # Attente
                time.sleep(1.0 / refresh_rate)
                
            except KeyboardInterrupt:
                break

    def _single_display(self):
        """Affichage unique"""
        self._display_status()

    def _display_status(self):
        """Affiche l'état complet du système"""
        
        print("🚁 DRONE NAVIGATION STATUS")
        print("=" * 50)
        
        # En-tête avec horodatage
        current_time = time.strftime("%Y-%m-%d %H:%M:%S")
        print(f"Timestamp: {current_time}")
        print(f"Update Rate: {self._stats['update_rate']:.1f} Hz")
        print(f"Messages: {self._stats['messages_received']}")
        print(f"Connection: {self._stats['connection_status']}")
        print()
        
        with self._lock:
            # État MAVROS
            self._display_mavros_status()
            
            # Position et navigation
            self._display_position_info()
            
            # Statut de navigation
            self._display_navigation_status()
            
            # Métriques de performance
            self._display_performance_metrics()
            
            # État GPS
            self._display_gps_status()

    def _display_mavros_status(self):
        """Affiche l'état MAVROS"""
        print("📡 MAVROS STATUS")
        print("-" * 20)
        
        if self._mavros_state:
            state = self._mavros_state
            
            # État de connexion
            connection_status = "🟢 CONNECTED" if state.connected else "🔴 DISCONNECTED"
            print(f"Connection: {connection_status}")
            
            # Mode de vol
            print(f"Flight Mode: {state.mode}")
            
            # État armé
            armed_status = "🟢 ARMED" if state.armed else "🔴 DISARMED"
            print(f"Armed: {armed_status}")
            
            # Guidé
            guided_status = "🟢 GUIDED" if state.guided else "🔴 MANUAL"
            print(f"Guided: {guided_status}")
            
        else:
            print("❌ Aucune données MAVROS")
        
        print()

    def _display_position_info(self):
        """Affiche les informations de position"""
        print("📍 POSITION & VELOCITY")
        print("-" * 25)
        
        # Position locale
        if self._position:
            pos = self._position.pose.position
            print(f"Local Position:")
            print(f"  X: {pos.x:8.2f} m")
            print(f"  Y: {pos.y:8.2f} m") 
            print(f"  Z: {pos.z:8.2f} m")
            
            # Orientation (quaternion vers angles d'Euler approximatif)
            q = self._position.pose.orientation
            yaw = self._quat_to_yaw(q.x, q.y, q.z, q.w)
            print(f"  Yaw: {yaw:6.1f}°")
        else:
            print("❌ Position locale non disponible")
        
        print()
        
        # Vélocité
        if self._velocity:
            vel = self._velocity
            speed = (vel.linear.x**2 + vel.linear.y**2 + vel.linear.z**2)**0.5
            print(f"Velocity:")
            print(f"  VX: {vel.linear.x:7.2f} m/s")
            print(f"  VY: {vel.linear.y:7.2f} m/s")
            print(f"  VZ: {vel.linear.z:7.2f} m/s")
            print(f"  Speed: {speed:5.2f} m/s")
            print(f"  Yaw Rate: {vel.angular.z:5.2f} rad/s")
        else:
            print("❌ Vélocité non disponible")
        
        print()

    def _display_navigation_status(self):
        """Affiche le statut de navigation"""
        print("🧭 NAVIGATION STATUS")
        print("-" * 20)
        
        if self._navigation_status:
            print(f"Status: {self._navigation_status.data}")
        else:
            print("❌ Statut navigation non disponible")
        
        print()

    def _display_performance_metrics(self):
        """Affiche les métriques de performance"""
        print("📊 PERFORMANCE METRICS")
        print("-" * 22)
        
        if self._navigation_metrics:
            print(f"Metrics: {self._navigation_metrics.data}")
        else:
            print("❌ Métriques non disponibles")
        
        print()

    def _display_gps_status(self):
        """Affiche l'état GPS"""
        print("🛰️  GPS STATUS")
        print("-" * 12)
        
        if self._gps_status:
            gps = self._gps_status
            
            # Position GPS
            print(f"Latitude: {gps.latitude:11.7f}°")
            print(f"Longitude: {gps.longitude:11.7f}°")
            print(f"Altitude: {gps.altitude:8.2f} m")
            
            # Qualité du signal
            status_map = {
                -1: "NO_FIX",
                0: "FIX_NOT_AVAILABLE", 
                1: "GPS_FIX",
                2: "DGPS_FIX"
            }
            status_str = status_map.get(gps.status.status, "UNKNOWN")
            print(f"Fix Type: {status_str}")
            
            # Précision
            if hasattr(gps, 'position_covariance') and gps.position_covariance_type > 0:
                cov = gps.position_covariance
                h_accuracy = (cov[0] + cov[4])**0.5
                v_accuracy = cov[8]**0.5
                print(f"H. Accuracy: {h_accuracy:.2f} m")
                print(f"V. Accuracy: {v_accuracy:.2f} m")
            
        else:
            print("❌ GPS non disponible")
        
        print()

    def _quat_to_yaw(self, x: float, y: float, z: float, w: float) -> float:
        """
        Convertit quaternion vers angle yaw en degrés
        
        Args:
            x, y, z, w: Composantes du quaternion
            
        Returns:
            Angle yaw en degrés
        """
        import math
        
        # Calcul simplifié du yaw
        yaw_rad = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return math.degrees(yaw_rad)

    def get_summary_dict(self) -> Dict[str, Any]:
        """
        Retourne un dictionnaire résumé de l'état
        
        Returns:
            Dictionnaire contenant le résumé de l'état
        """
        with self._lock:
            summary = {
                'timestamp': time.time(),
                'connection_status': self._stats['connection_status'],
                'update_rate': self._stats['update_rate'],
                'messages_received': self._stats['messages_received']
            }
            
            # Position
            if self._position:
                pos = self._position.pose.position
                summary['position'] = {'x': pos.x, 'y': pos.y, 'z': pos.z}
            
            # Vélocité
            if self._velocity:
                vel = self._velocity.linear
                speed = (vel.x**2 + vel.y**2 + vel.z**2)**0.5
                summary['velocity'] = {'vx': vel.x, 'vy': vel.y, 'vz': vel.z, 'speed': speed}
            
            # GPS
            if self._gps_status:
                summary['gps'] = {
                    'lat': self._gps_status.latitude,
                    'lon': self._gps_status.longitude,
                    'alt': self._gps_status.altitude
                }
            
            # État MAVROS
            if self._mavros_state:
                summary['mavros'] = {
                    'connected': self._mavros_state.connected,
                    'armed': self._mavros_state.armed,
                    'mode': self._mavros_state.mode
                }
            
            # Statut navigation
            if self._navigation_status:
                summary['navigation_status'] = self._navigation_status.data
            
            # Métriques
            if self._navigation_metrics:
                summary['navigation_metrics'] = self._navigation_metrics.data
            
            return summary


def parse_arguments():
    """
    Parse les arguments de ligne de commande
    
    Returns:
        Arguments parsés
    """
    parser = argparse.ArgumentParser(
        description="Surveille l'état de navigation du drone",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples d'utilisation:
  # Affichage unique de l'état
  ros2 run drone_navigation nav_status

  # Surveillance continue (mise à jour chaque seconde)
  ros2 run drone_navigation nav_status --continuous

  # Surveillance continue avec taux personnalisé
  ros2 run drone_navigation nav_status --continuous --rate 2.0

  # Surveillance avec timeout
  ros2 run drone_navigation nav_status --continuous --timeout 60

  # Export JSON de l'état
  ros2 run drone_navigation nav_status --export status.json
        """
    )
    
    parser.add_argument('--continuous', '-c', action='store_true',
                       help='Surveillance continue')
    parser.add_argument('--rate', '-r', type=float, default=1.0,
                       help='Taux de rafraîchissement en Hz (défaut: 1.0)')
    parser.add_argument('--timeout', '-t', type=float,
                       help='Timeout en secondes pour surveillance continue')
    parser.add_argument('--export', '-e', type=str,
                       help='Fichier JSON pour export de l\'état')
    parser.add_argument('--quiet', '-q', action='store_true',
                       help='Mode silencieux (minimal output)')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Affichage détaillé')
    
    return parser.parse_args()


def main(args=None):
    """
    Point d'entrée principal de l'outil de surveillance
    
    Args:
        args: Arguments de ligne de commande
        
    Returns:
        Code de sortie (0 = succès, autre = erreur)
    """
    import json
    
    # Parse des arguments
    parsed_args = parse_arguments()
    
    # Initialisation ROS2
    rclpy.init(args=args)
    
    try:
        # Création de l'outil
        tool = NavStatusTool()
        
        # Configuration logging
        if parsed_args.verbose:
            tool.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        elif parsed_args.quiet:
            tool.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
        
        # Attente des données initiales
        if not parsed_args.quiet:
            print("🔄 Connexion aux topics de navigation...")
        
        # Spinning pour recevoir des données
        start_time = time.time()
        timeout = 5.0  # 5 secondes pour établir la connexion
        
        while time.time() - start_time < timeout:
            rclpy.spin_once(tool, timeout_sec=0.1)
            if tool._stats['messages_received'] > 0:
                break
        
        if tool._stats['messages_received'] == 0 and not parsed_args.quiet:
            print("⚠️  Aucune donnée reçue - vérifiez que les nodes de navigation sont actifs")
        
        # Démarrage de la surveillance
        if parsed_args.continuous:
            if not parsed_args.quiet:
                print("🎯 Surveillance continue (Ctrl+C pour arrêter)")
                print()
            
            tool.start_monitoring(continuous=True, refresh_rate=parsed_args.rate)
            
            # Boucle principale avec timeout optionnel
            end_time = None
            if parsed_args.timeout:
                end_time = time.time() + parsed_args.timeout
            
            try:
                while rclpy.ok():
                    rclpy.spin_once(tool, timeout_sec=0.1)
                    
                    if end_time and time.time() > end_time:
                        if not parsed_args.quiet:
                            print("\n⏰ Timeout atteint")
                        break
                        
            except KeyboardInterrupt:
                if not parsed_args.quiet:
                    print("\n🛑 Surveillance interrompue")
            
            tool.stop_monitoring()
            
        else:
            # Affichage unique
            tool.start_monitoring(continuous=False)
        
        # Export JSON si demandé
        if parsed_args.export:
            summary = tool.get_summary_dict()
            try:
                with open(parsed_args.export, 'w') as f:
                    json.dump(summary, f, indent=2)
                if not parsed_args.quiet:
                    print(f"💾 État exporté vers: {parsed_args.export}")
            except Exception as e:
                print(f"❌ Erreur export: {e}", file=sys.stderr)
                return 1
        
        if not parsed_args.quiet:
            print("✅ Surveillance terminée")
            
        return 0
            
    except KeyboardInterrupt:
        if not parsed_args.quiet:
            print("\n🛑 Interruption par l'utilisateur")
        return 0
        
    except Exception as e:
        print(f"❌ Erreur: {e}", file=sys.stderr)
        return 1
        
    finally:
        # Nettoyage
        try:
            if 'tool' in locals():
                tool.destroy_node()
        except Exception as e:
            print(f"⚠️  Erreur lors du nettoyage: {e}", file=sys.stderr)
        finally:
            rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())