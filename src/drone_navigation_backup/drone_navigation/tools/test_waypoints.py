#!/usr/bin/env python3

"""
Outil CLI pour tests de navigation par waypoints

Teste la navigation avec séquences de waypoints prédéfinies
ou générées automatiquement pour validation du système.

Auteur: Adama Komi
Version: 1.1.0
"""

import argparse
import sys
import time
import json
import math
import random
from typing import List, Dict, Optional, Any, Tuple
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.task import Future
from geometry_msgs.msg import Point, PoseStamped, Twist
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger


class TestWaypointsTool(Node):
    """Outil CLI pour tests de waypoints"""
    
    def __init__(self):
        super().__init__('test_waypoints_tool')
        
        # Configuration QoS
        self._qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        # Services de navigation
        self._goto_client = self.create_client(Trigger, '/drone_nav/goto_position')
        self._mission_start_client = self.create_client(Trigger, '/drone_nav/start_mission')
        self._mission_stop_client = self.create_client(Trigger, '/drone_nav/stop_mission')
        
        # Attendre que les services soient disponibles
        self.get_logger().info("En attente des services de navigation...")
        self._goto_client.wait_for_service(timeout_sec=10.0)
        self._mission_start_client.wait_for_service(timeout_sec=10.0)
        self._mission_stop_client.wait_for_service(timeout_sec=10.0)
        
        # Subscribers pour surveillance
        self._position_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose',
            self._position_callback, self._qos_profile
        )
        
        self._velocity_sub = self.create_subscription(
            Twist, '/mavros/local_position/velocity_local',
            self._velocity_callback, self._qos_profile
        )
        
        # État actuel
        self._current_position = None
        self._current_velocity = None
        self._lock = threading.Lock()
        
        # Résultats de test
        self._test_results = {
            'waypoints': [],
            'metrics': {},
            'status': 'INITIALIZED'
        }
        
        self.get_logger().info("Outil test_waypoints initialisé")

    def _position_callback(self, msg: PoseStamped):
        """Callback de position MAVROS"""
        with self._lock:
            self._current_position = msg

    def _velocity_callback(self, msg: Twist):
        """Callback de vélocité MAVROS"""
        with self._lock:
            self._current_velocity = msg

    def get_current_position(self) -> Optional[Point]:
        """Retourne la position actuelle du drone
        
        Returns:
            Point: La position actuelle ou None si non disponible
        """
        with self._lock:
            if self._current_position:
                return self._current_position.pose.position
        return None

    def test_basic_navigation(self, distance: float = 10.0, verbose: bool = False) -> Dict[str, Any]:
        """Test de navigation basique avec pattern carré
        
        Args:
            distance: Taille du côté du carré en mètres
            verbose: Si True, affiche des informations détaillées
            
        Returns:
            Dict: Résultats du test
        """
        
        if verbose:
            print("🔲 Test navigation basique (pattern carré)")
        
        # Génération du pattern carré
        waypoints = self._generate_square_pattern(distance)
        
        return self._execute_waypoint_test(waypoints, "basic_square", verbose)

    def test_complex_pattern(self, pattern_type: str, size: float = 20.0, 
                           verbose: bool = False) -> Dict[str, Any]:
        """Test avec pattern complexe
        
        Args:
            pattern_type: Type de pattern ('spiral', 'zigzag', 'figure8', 'random')
            size: Taille du pattern en mètres
            verbose: Si True, affiche des informations détaillées
            
        Returns:
            Dict: Résultats du test
        """
        
        if verbose:
            print(f"🌀 Test pattern complexe: {pattern_type}")
        
        # Génération du pattern selon le type
        if pattern_type == "spiral":
            waypoints = self._generate_spiral_pattern(size)
        elif pattern_type == "zigzag":
            waypoints = self._generate_zigzag_pattern(size)
        elif pattern_type == "figure8":
            waypoints = self._generate_figure8_pattern(size)
        elif pattern_type == "random":
            waypoints = self._generate_random_pattern(size, 10)
        else:
            self.get_logger().error(f"Pattern non supporté: {pattern_type}")
            return {'status': 'ERROR', 'error': 'Pattern non supporté'}
        
        return self._execute_waypoint_test(waypoints, f"complex_{pattern_type}", verbose)

    def test_precision_landing(self, target_accuracy: float = 0.5, 
                             verbose: bool = False) -> Dict[str, Any]:
        """Test de précision d'atterrissage
        
        Args:
            target_accuracy: Précision cible en mètres
            verbose: Si True, affiche des informations détaillées
            
        Returns:
            Dict: Résultats du test
        """
        
        if verbose:
            print(f"🎯 Test précision atterrissage (±{target_accuracy}m)")
        
        # Position de départ (position actuelle ou par défaut)
        start_pos = self.get_current_position()
        if not start_pos:
            start_pos = Point(x=0.0, y=0.0, z=10.0)
            self.get_logger().warn("Utilisation de la position par défaut (0, 0, 10)")
        
        # Séquence de test: montée, déplacement, descente précise
        waypoints = [
            Point(x=start_pos.x, y=start_pos.y, z=15.0),  # Montée
            Point(x=start_pos.x + 5.0, y=start_pos.y + 5.0, z=15.0),  # Déplacement
            Point(x=start_pos.x + 5.0, y=start_pos.y + 5.0, z=5.0),   # Descente
            Point(x=start_pos.x + 5.0, y=start_pos.y + 5.0, z=1.0),   # Atterrissage précis
        ]
        
        # Test avec tolérance de précision
        result = self._execute_waypoint_test(waypoints, "precision_landing", verbose)
        
        # Validation de la précision
        if result['status'] == 'SUCCESS':
            final_pos_dict = result.get('final_position')
            target_pos = waypoints[-1]
            
            if final_pos_dict:
                final_pos = Point()
                final_pos.x = final_pos_dict['x']
                final_pos.y = final_pos_dict['y']
                final_pos.z = final_pos_dict['z']
                
                error = self._calculate_position_error(final_pos, target_pos)
                result['precision_error'] = error
                result['precision_met'] = error <= target_accuracy
                
                if not result['precision_met']:
                    result['status'] = 'PRECISION_FAIL'
        
        return result

    def test_stress_navigation(self, num_waypoints: int = 50, area_size: float = 30.0,
                             verbose: bool = False) -> Dict[str, Any]:
        """Test de stress avec nombreux waypoints
        
        Args:
            num_waypoints: Nombre de waypoints à générer
            area_size: Taille de la zone de test en mètres
            verbose: Si True, affiche des informations détaillées
            
        Returns:
            Dict: Résultats du test
        """
        
        if verbose:
            print(f"💪 Test de stress ({num_waypoints} waypoints)")
        
        # Génération d'un grand nombre de waypoints aléatoires
        waypoints = self._generate_random_pattern(area_size, num_waypoints)
        
        return self._execute_waypoint_test(waypoints, "stress_test", verbose)

    def test_from_file(self, filename: str, verbose: bool = False) -> Dict[str, Any]:
        """Test avec waypoints depuis fichier JSON
        
        Args:
            filename: Chemin vers le fichier JSON contenant les waypoints
            verbose: Si True, affiche des informations détaillées
            
        Returns:
            Dict: Résultats du test
        """
        
        if verbose:
            print(f"📁 Test depuis fichier: {filename}")
        
        try:
            waypoints = self._load_waypoints_from_file(filename)
            if not waypoints:
                return {'status': 'ERROR', 'error': 'Impossible de charger les waypoints'}
            
            return self._execute_waypoint_test(waypoints, f"file_{filename}", verbose)
            
        except Exception as e:
            return {'status': 'ERROR', 'error': str(e)}

    def _execute_waypoint_test(self, waypoints: List[Point], test_name: str,
                              verbose: bool = False) -> Dict[str, Any]:
        """Exécute un test de waypoints
        
        Args:
            waypoints: Liste des waypoints à suivre
            test_name: Nom du test pour identification
            verbose: Si True, affiche des informations détaillées
            
        Returns:
            Dict: Résultats détaillés du test
        """
        
        test_result = {
            'test_name': test_name,
            'status': 'RUNNING',
            'waypoints_count': len(waypoints),
            'start_time': time.time(),
            'waypoint_results': [],
            'metrics': {}
        }
        
        if verbose:
            print(f"▶️  Démarrage test '{test_name}' avec {len(waypoints)} waypoints")
        
        try:
            # Attente de la position initiale
            start_time = time.time()
            while not self.get_current_position() and (time.time() - start_time) < 10.0:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            initial_position = self.get_current_position()
            if not initial_position:
                test_result['status'] = 'ERROR'
                test_result['error'] = 'Position initiale non disponible après 10s'
                return test_result
            
            test_result['initial_position'] = {
                'x': initial_position.x,
                'y': initial_position.y, 
                'z': initial_position.z
            }
            
            total_distance = 0.0
            successful_waypoints = 0
            
            # Navigation vers chaque waypoint
            for i, waypoint in enumerate(waypoints):
                if verbose:
                    print(f"  Waypoint {i+1}/{len(waypoints)}: ({waypoint.x:.1f}, {waypoint.y:.1f}, {waypoint.z:.1f})")
                
                waypoint_start_time = time.time()
                
                # Navigation vers le waypoint
                waypoint_result = self._navigate_to_waypoint(waypoint, timeout=30.0)
                waypoint_result['waypoint_index'] = i
                waypoint_result['target'] = {'x': waypoint.x, 'y': waypoint.y, 'z': waypoint.z}
                
                test_result['waypoint_results'].append(waypoint_result)
                
                if waypoint_result['status'] == 'SUCCESS':
                    successful_waypoints += 1
                    
                    # Calcul de la distance parcourue
                    if i > 0:
                        prev_wp = waypoints[i-1]
                        distance = self._calculate_distance(prev_wp, waypoint)
                        total_distance += distance
                        waypoint_result['distance_to_previous'] = distance
                
                if verbose:
                    status_icon = "✅" if waypoint_result['status'] == 'SUCCESS' else "❌"
                    print(f"    {status_icon} {waypoint_result['status']} "
                          f"(temps: {waypoint_result['execution_time']:.1f}s)")
            
            # Calcul des métriques finales
            test_result['end_time'] = time.time()
            test_result['total_duration'] = test_result['end_time'] - test_result['start_time']
            test_result['successful_waypoints'] = successful_waypoints
            test_result['success_rate'] = successful_waypoints / len(waypoints) if waypoints else 0
            test_result['total_distance'] = total_distance
            
            # Position finale
            final_position = self.get_current_position()
            if final_position:
                test_result['final_position'] = {
                    'x': final_position.x,
                    'y': final_position.y,
                    'z': final_position.z
                }
            
            # Statut final
            if test_result['success_rate'] >= 0.9:
                test_result['status'] = 'SUCCESS'
            elif test_result['success_rate'] >= 0.7:
                test_result['status'] = 'PARTIAL_SUCCESS'
            else:
                test_result['status'] = 'FAIL'
            
            if verbose:
                print(f"🏁 Test terminé: {test_result['status']} "
                      f"({successful_waypoints}/{len(waypoints)} réussis)")
            
        except Exception as e:
            test_result['status'] = 'ERROR'
            test_result['error'] = str(e)
            test_result['end_time'] = time.time()
            
            if verbose:
                print(f"❌ Erreur durant test: {e}")
        
        return test_result

    def _navigate_to_waypoint(self, waypoint: Point, timeout: float = 30.0) -> Dict[str, Any]:
        """Navigation vers un waypoint spécifique en utilisant le service ROS
        
        Args:
            waypoint: Point cible à atteindre
            timeout: Délai maximum en secondes pour atteindre le waypoint
            
        Returns:
            Dict: Résultats de la navigation vers le waypoint
        """
        
        result = {
            'status': 'RUNNING',
            'start_time': time.time(),
            'target_reached': False,
            'execution_time': 0.0,
            'max_error': 0.0,
            'avg_error': 0.0
        }
        
        try:
            # Préparer la requête pour le service de navigation
            request = Trigger.Request()
            
            # Appeler le service de navigation (implémentation factice pour l'exemple)
            # Dans une implémentation réelle, vous enverriez les coordonnées via le service
            future = self._goto_client.call_async(request)
            
            start_time = time.time()
            position_errors = []
            
            # Surveiller la progression vers le waypoint
            while time.time() - start_time < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
                
                # Vérifier la position actuelle
                current_pos = self.get_current_position()
                if current_pos:
                    error = self._calculate_position_error(current_pos, waypoint)
                    position_errors.append(error)
                    
                    # Vérifier si le waypoint est atteint (tolérance de 1m)
                    if error < 1.0:
                        result['status'] = 'SUCCESS'
                        result['target_reached'] = True
                        break
                
                # Vérifier si le service a terminé
                if future.done():
                    try:
                        response = future.result()
                        if not response.success:
                            result['status'] = 'NAVIGATION_ERROR'
                            break
                    except Exception as e:
                        result['status'] = 'SERVICE_ERROR'
                        result['error'] = str(e)
                        break
            
            # Vérifier le timeout
            if result['status'] == 'RUNNING' and (time.time() - start_time) >= timeout:
                result['status'] = 'TIMEOUT'
            
            # Calculer les métriques d'erreur
            if position_errors:
                result['max_error'] = max(position_errors)
                result['avg_error'] = sum(position_errors) / len(position_errors)
                result['final_error'] = position_errors[-1] if position_errors else 0.0
            
            result['execution_time'] = time.time() - result['start_time']
            
        except Exception as e:
            result['status'] = 'ERROR'
            result['error'] = str(e)
            result['execution_time'] = time.time() - result['start_time']
        
        return result

    def _calculate_position_error(self, current: Point, target: Point) -> float:
        """Calcule l'erreur de position 3D entre la position actuelle et la cible
        
        Args:
            current: Position actuelle
            target: Position cible
            
        Returns:
            float: Distance en mètres entre les deux points
        """
        dx = current.x - target.x
        dy = current.y - target.y
        dz = current.z - target.z
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def _calculate_distance(self, p1: Point, p2: Point) -> float:
        """Calcule la distance entre deux points
        
        Args:
            p1: Premier point
            p2: Deuxième point
            
        Returns:
            float: Distance en mètres entre les deux points
        """
        return self._calculate_position_error(p1, p2)

    def _generate_square_pattern(self, size: float) -> List[Point]:
        """Génère un pattern carré
        
        Args:
            size: Taille du côté du carré en mètres
            
        Returns:
            List[Point]: Liste des points formant un carré
        """
        half_size = size / 2.0
        altitude = 10.0
        
        return [
            Point(x=half_size, y=half_size, z=altitude),
            Point(x=half_size, y=-half_size, z=altitude),
            Point(x=-half_size, y=-half_size, z=altitude),
            Point(x=-half_size, y=half_size, z=altitude),
            Point(x=0.0, y=0.0, z=altitude)  # Retour au centre
        ]

    def _generate_spiral_pattern(self, max_radius: float, num_points: int = 20) -> List[Point]:
        """Génère un pattern spiral
        
        Args:
            max_radius: Rayon maximum de la spirale en mètres
            num_points: Nombre de points à générer
            
        Returns:
            List[Point]: Liste des points formant une spirale
        """
        waypoints = []
        altitude = 10.0
        
        for i in range(num_points):
            t = i / (num_points - 1) if num_points > 1 else 0  # Paramètre de 0 à 1
            radius = max_radius * t
            angle = 6 * math.pi * t  # 3 tours complets
            
            x = radius * math.cos(angle)
            y = radius * math.sin(angle)
            
            waypoints.append(Point(x=x, y=y, z=altitude))
        
        return waypoints

    def _generate_zigzag_pattern(self, size: float, num_lines: int = 5) -> List[Point]:
        """Génère un pattern zigzag
        
        Args:
            size: Taille du pattern en mètres
            num_lines: Nombre de lignes du zigzag
            
        Returns:
            List[Point]: Liste des points formant un zigzag
        """
        waypoints = []
        altitude = 10.0
        
        if num_lines < 2:
            num_lines = 2
        
        y_step = size / (num_lines - 1)
        
        for i in range(num_lines):
            y = -size/2 + i * y_step
            
            if i % 2 == 0:  # Ligne paire: gauche vers droite
                waypoints.append(Point(x=-size/2, y=y, z=altitude))
                waypoints.append(Point(x=size/2, y=y, z=altitude))
            else:  # Ligne impaire: droite vers gauche
                waypoints.append(Point(x=size/2, y=y, z=altitude))
                waypoints.append(Point(x=-size/2, y=y, z=altitude))
        
        return waypoints

    def _generate_figure8_pattern(self, size: float, num_points: int = 24) -> List[Point]:
        """Génère un pattern en forme de 8
        
        Args:
            size: Taille du pattern en mètres
            num_points: Nombre de points à générer
            
        Returns:
            List[Point]: Liste des points formant un 8
        """
        waypoints = []
        altitude = 10.0
        
        for i in range(num_points):
            t = 2 * math.pi * i / num_points if num_points > 0 else 0
            
            # Équations paramétriques d'un 8
            x = size/2 * math.sin(t)
            y = size/4 * math.sin(2*t)
            
            waypoints.append(Point(x=x, y=y, z=altitude))
        
        return waypoints

    def _generate_random_pattern(self, area_size: float, num_points: int) -> List[Point]:
        """Génère un pattern aléatoire
        
        Args:
            area_size: Taille de la zone en mètres
            num_points: Nombre de points à générer
            
        Returns:
            List[Point]: Liste de points aléatoires
        """
        waypoints = []
        min_altitude, max_altitude = 5.0, 15.0
        
        for _ in range(num_points):
            x = random.uniform(-area_size/2, area_size/2)
            y = random.uniform(-area_size/2, area_size/2)
            z = random.uniform(min_altitude, max_altitude)
            
            waypoints.append(Point(x=x, y=y, z=z))
        
        return waypoints

    def _load_waypoints_from_file(self, filename: str) -> Optional[List[Point]]:
        """Charge des waypoints depuis un fichier JSON
        
        Args:
            filename: Chemin vers le fichier JSON
            
        Returns:
            List[Point] or None: Liste des points ou None en cas d'erreur
        """
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            waypoints = []
            for wp_data in data.get('waypoints', []):
                point = Point()
                point.x = float(wp_data.get('x', 0.0))
                point.y = float(wp_data.get('y', 0.0))
                point.z = float(wp_data.get('z', 10.0))
                waypoints.append(point)
            
            return waypoints
            
        except Exception as e:
            self.get_logger().error(f"Erreur lecture waypoints: {e}")
            return None


def parse_arguments():
    """Parse les arguments de ligne de commande"""
    parser = argparse.ArgumentParser(
        description="Teste la navigation par waypoints",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples d'utilisation:
  # Test basique (pattern carré)
  ros2 run drone_navigation test_waypoints --basic

  # Test pattern complexe
  ros2 run drone_navigation test_waypoints --pattern spiral --size 20

  # Test de précision
  ros2 run drone_navigation test_waypoints --precision --accuracy 0.3

  # Test de stress
  ros2 run drone_navigation test_waypoints --stress --waypoints 100

  # Test depuis fichier
  ros2 run drone_navigation test_waypoints --file mission_waypoints.json

  # Test complet avec export
  ros2 run drone_navigation test_waypoints --all --export test_results.json
        """)
    
    # Types de test
    test_group = parser.add_mutually_exclusive_group(required=True)
    test_group.add_argument('--basic', action='store_true',
                           help='Test de navigation basique (carré)')
    test_group.add_argument('--pattern', choices=['spiral', 'zigzag', 'figure8', 'random'],
                           help='Test avec pattern complexe')
    test_group.add_argument('--precision', action='store_true',
                           help='Test de précision d\'atterrissage')
    test_group.add_argument('--stress', action='store_true',
                           help='Test de stress avec nombreux waypoints')
    test_group.add_argument('--file', type=str,
                           help='Test avec waypoints depuis fichier JSON')
    test_group.add_argument('--all', action='store_true',
                           help='Exécute tous les tests')
    
    # Paramètres de test
    parser.add_argument('--size', type=float, default=20.0,
                       help='Taille de la zone de test en mètres (défaut: 20)')
    parser.add_argument('--waypoints', type=int, default=50,
                       help='Nombre de waypoints pour test de stress (défaut: 50)')
    parser.add_argument('--accuracy', type=float, default=0.5,
                       help='Précision requise en mètres (défaut: 0.5)')
    
    # Options générales
    parser.add_argument('--export', '-e', type=str,
                       help='Fichier JSON pour export des résultats')
    parser.add_argument('--timeout', '-t', type=float, default=30.0,
                       help='Timeout par waypoint en secondes (défaut: 30)')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Affichage détaillé')
    parser.add_argument('--quiet', '-q', action='store_true',
                       help='Mode silencieux')
    
    return parser.parse_args()


def display_test_results(results: Dict[str, Any], verbose: bool = False):
    """Affiche les résultats de test
    
    Args:
        results: Résultats du test à afficher
        verbose: Si True, affiche des détails supplémentaires
    """
    
    test_name = results.get('test_name', 'Unknown')
    status = results.get('status', 'UNKNOWN')
    
    # Icône de statut
    status_icons = {
        'SUCCESS': '✅',
        'PARTIAL_SUCCESS': '🟡',
        'FAIL': '❌',
        'ERROR': '💥',
        'TIMEOUT': '⏰',
        'PRECISION_FAIL': '🎯'
    }
    
    icon = status_icons.get(status, '❓')
    
    print(f"\n{icon} TEST '{test_name.upper()}' - {status}")
    print("=" * 50)
    
    # Métriques principales
    waypoints_count = results.get('waypoints_count', 0)
    successful = results.get('successful_waypoints', 0)
    success_rate = results.get('success_rate', 0.0)
    duration = results.get('total_duration', 0.0)
    
    print(f"Waypoints: {successful}/{waypoints_count} ({success_rate:.1%})")
    print(f"Durée totale: {duration:.1f}s")
    
    if 'total_distance' in results:
        print(f"Distance totale: {results['total_distance']:.1f}m")
    
    # Métriques de précision
    if 'precision_error' in results:
        precision_met = results.get('precision_met', False)
        precision_icon = "✅" if precision_met else "❌"
        print(f"Précision: {results['precision_error']:.2f}m {precision_icon}")
    
    # Détails des waypoints si verbose
    if verbose and 'waypoint_results' in results:
        print("\n📍 DÉTAILS PAR WAYPOINT:")
        for wp_result in results['waypoint_results']:
            idx = wp_result.get('waypoint_index', 0)
            wp_status = wp_result.get('status', 'UNKNOWN')
            exec_time = wp_result.get('execution_time', 0.0)
            
            wp_icon = "✅" if wp_status == 'SUCCESS' else "❌"
            print(f"  {idx+1:2d}. {wp_icon} {exec_time:5.1f}s")
            
            if 'final_error' in wp_result:
                print(f"      Erreur: {wp_result['final_error']:.2f}m")
    
    # Erreurs
    if 'error' in results:
        print(f"\n❌ Erreur: {results['error']}")
    
    print()


def main(args=None):
    """Point d'entrée principal"""
    
    # Parse des arguments
    parsed_args = parse_arguments()
    
    # Initialisation ROS2
    rclpy.init(args=args)
    
    try:
        # Création de l'outil
        tool = TestWaypointsTool()
        
        # Configuration logging
        if parsed_args.verbose:
            tool.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        elif parsed_args.quiet:
            tool.get_logger().set_level(rclpy.logging.LoggingSeverity.WARN)
        
        all_results = []
        
        # Exécution des tests selon les arguments
        if parsed_args.all:
            # Tous les tests
            tests = [
                ('basic', lambda: tool.test_basic_navigation(parsed_args.size, parsed_args.verbose)),
                ('spiral', lambda: tool.test_complex_pattern('spiral', parsed_args.size, parsed_args.verbose)),
                ('zigzag', lambda: tool.test_complex_pattern('zigzag', parsed_args.size, parsed_args.verbose)),
                ('precision', lambda: tool.test_precision_landing(parsed_args.accuracy, parsed_args.verbose)),
                ('stress', lambda: tool.test_stress_navigation(parsed_args.waypoints, parsed_args.size, parsed_args.verbose))
            ]
            
            for test_name, test_func in tests:
                if not parsed_args.quiet:
                    print(f"\n🚀 Démarrage test: {test_name}")
                
                result = test_func()
                all_results.append(result)
                
                if not parsed_args.quiet:
                    display_test_results(result, parsed_args.verbose)
        
        elif parsed_args.basic:
            result = tool.test_basic_navigation(parsed_args.size, parsed_args.verbose)
            all_results.append(result)
            
        elif parsed_args.pattern:
            result = tool.test_complex_pattern(parsed_args.pattern, parsed_args.size, parsed_args.verbose)
            all_results.append(result)
            
        elif parsed_args.precision:
            result = tool.test_precision_landing(parsed_args.accuracy, parsed_args.verbose)
            all_results.append(result)
            
        elif parsed_args.stress:
            result = tool.test_stress_navigation(parsed_args.waypoints, parsed_args.size, parsed_args.verbose)
            all_results.append(result)
            
        elif parsed_args.file:
            result = tool.test_from_file(parsed_args.file, parsed_args.verbose)
            all_results.append(result)
        
        # Affichage des résultats
        if not parsed_args.quiet and len(all_results) == 1:
            display_test_results(all_results[0], parsed_args.verbose)
        
        # Résumé pour tests multiples
        if not parsed_args.quiet and len(all_results) > 1:
            print("\n📊 RÉSUMÉ DES TESTS")
            print("=" * 30)
            
            total_tests = len(all_results)
            successful_tests = sum(1 for r in all_results if r.get('status') == 'SUCCESS')
            partial_tests = sum(1 for r in all_results if r.get('status') == 'PARTIAL_SUCCESS')
            
            print(f"Tests réussis: {successful_tests}/{total_tests}")
            print(f"Tests partiellement réussis: {partial_tests}/{total_tests}")
            
            for result in all_results:
                test_name = result.get('test_name', 'Unknown')
                status = result.get('status', 'UNKNOWN')
                icon = '✅' if status == 'SUCCESS' else '🟡' if status == 'PARTIAL_SUCCESS' else '❌'
                print(f"  {icon} {test_name}: {status}")
        
        # Export JSON
        if parsed_args.export:
            export_data = {
                'timestamp': time.time(),
                'test_configuration': {
                    'size': parsed_args.size,
                    'waypoints': parsed_args.waypoints,
                    'accuracy': parsed_args.accuracy,
                    'timeout': parsed_args.timeout
                },
                'results': all_results
            }
            
            try:
                with open(parsed_args.export, 'w') as f:
                    json.dump(export_data, f, indent=2, default=str)
                
                if not parsed_args.quiet:
                    print(f"💾 Résultats exportés: {parsed_args.export}")
                    
            except Exception as e:
                print(f"❌ Erreur export: {e}", file=sys.stderr)
        
        # Code de retour
        if all_results:
            # Échec si aucun test n'a réussi
            success_count = sum(1 for r in all_results if r.get('status') == 'SUCCESS')
            if success_count == 0:
                sys.exit(1)
        
        if not parsed_args.quiet:
            print("✅ Tests terminés")
            
    except KeyboardInterrupt:
        if not parsed_args.quiet:
            print("\n🛑 Tests interrompus par l'utilisateur")
        sys.exit(0)
        
    except Exception as e:
        print(f"❌ Erreur: {e}", file=sys.stderr)
        sys.exit(1)
        
    finally:
        # Nettoyage
        try:
            tool.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()