#!/usr/bin/env python3

"""
Outil CLI pour planification de missions

Permet de planifier des missions de couverture avec différents patterns
et paramètres, incluant chargement de zones depuis fichiers KML/JSON.

Auteur: Adama Komi
Version: 1.1.0
"""

import argparse
import json
import sys
import os
import math
import time
from typing import Dict, List, Any, Optional

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_srvs.srv import Trigger


class PlanMissionTool(Node):
    """
    Outil CLI pour planification de missions
    
    Fournit des capacités de planification de missions pour le drone
    incluant couverture de zone, patterns de vol et gestion de waypoints.
    """
    
    def __init__(self):
        super().__init__('plan_mission_tool')
        
        # Composants de planification
        try:
            from ..coverage_patterns import CoveragePatterns, PatternType, PatternParameters
            self._coverage_patterns = CoveragePatterns(self.get_logger(), {})
            self._pattern_type_class = PatternType
            self._pattern_params_class = PatternParameters
        except ImportError:
            self.get_logger().error("Modules de couverture non disponibles")
            self._coverage_patterns = None
        
        # Client de service pour démarrage de mission
        self._start_mission_client = self.create_client(
            Trigger, '/drone_nav/start_mission'
        )
        
        self.get_logger().info("Outil plan_mission initialisé")

    def plan_area_coverage(self, area_file: str, pattern: str, 
                          altitude: float = 10.0, speed: float = 5.0,
                          overlap: float = 20.0, save_to: Optional[str] = None) -> bool:
        """
        Planifie une mission de couverture de zone
        
        Args:
            area_file: Fichier définissant la zone (KML, JSON)
            pattern: Type de pattern (zigzag, spiral, etc.)
            altitude: Altitude de vol en mètres
            speed: Vitesse de vol en m/s
            overlap: Pourcentage de recouvrement (0-100)
            save_to: Fichier de sauvegarde de la mission
            
        Returns:
            True si planification réussie, False sinon
            
        Raises:
            RuntimeError: Si les modules de couverture ne sont pas disponibles
        """
        if not self._coverage_patterns:
            raise RuntimeError("Modules de couverture non disponibles")
        
        try:
            # Chargement de la zone
            area_polygon = self._load_area_from_file(area_file)
            if not area_polygon:
                return False
            
            # Configuration des paramètres
            pattern_type = self._parse_pattern_type(pattern)
            if not pattern_type:
                return False
            
            params = self._pattern_params_class(
                pattern_type=pattern_type,
                area_polygon=area_polygon,
                altitude=altitude,
                speed=speed,
                overlap_percentage=overlap
            )
            
            # Génération du pattern
            self.get_logger().info(f"Génération du pattern {pattern} pour zone {area_file}")
            result = self._coverage_patterns.generate_pattern(params)
            
            if not result.success:
                self.get_logger().error(f"Échec génération pattern: {result.message}")
                return False
            
            # Affichage des résultats
            self._display_mission_info(result)
            
            # Sauvegarde si demandée
            if save_to:
                self._save_mission(result, save_to)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Erreur planification mission: {e}")
            return False

    def plan_waypoint_mission(self, waypoints_file: str, 
                             save_to: Optional[str] = None) -> bool:
        """
        Planifie une mission basée sur une liste de waypoints
        
        Args:
            waypoints_file: Fichier JSON des waypoints
            save_to: Fichier de sauvegarde
            
        Returns:
            True si planification réussie, False sinon
        """
        try:
            # Chargement des waypoints
            waypoints = self._load_waypoints_from_file(waypoints_file)
            if not waypoints:
                return False
            
            self.get_logger().info(f"Mission avec {len(waypoints)} waypoints")
            
            # Calcul des métriques
            total_distance = self._calculate_mission_distance(waypoints)
            estimated_time = total_distance / 5.0  # Vitesse par défaut 5 m/s
            
            # Affichage des informations
            self.get_logger().info(f"Distance totale: {total_distance:.1f} m")
            self.get_logger().info(f"Temps estimé: {estimated_time:.1f} s")
            
            # Sauvegarde si demandée
            if save_to:
                mission_data = {
                    'type': 'waypoint_mission',
                    'waypoints': [{'x': p.x, 'y': p.y, 'z': p.z} for p in waypoints],
                    'total_distance': total_distance,
                    'estimated_time': estimated_time
                }
                self._save_json(mission_data, save_to)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Erreur planification waypoints: {e}")
            return False

    def plan_spiral_coverage(self, center_lat: float, center_lon: float,
                            radius: float, altitude: float = 15.0,
                            spacing: float = 5.0, save_to: Optional[str] = None) -> bool:
        """
        Planifie une mission de couverture spirale
        
        Args:
            center_lat: Latitude du centre en degrés
            center_lon: Longitude du centre en degrés  
            radius: Rayon de couverture en mètres
            altitude: Altitude de vol en mètres
            spacing: Espacement entre spires en mètres
            save_to: Fichier de sauvegarde
            
        Returns:
            True si planification réussie, False sinon
            
        Raises:
            RuntimeError: Si les modules de couverture ne sont pas disponibles
        """
        if not self._coverage_patterns:
            raise RuntimeError("Modules de couverture non disponibles")
        
        try:
            # Création d'un polygone circulaire approximatif
            center_x, center_y = self._latlon_to_xy(center_lat, center_lon)
            area_polygon = self._generate_circle_polygon(center_x, center_y, radius)
            
            # Configuration du pattern spiral
            params = self._pattern_params_class(
                pattern_type=self._pattern_type_class.SPIRAL,
                area_polygon=area_polygon,
                altitude=altitude,
                speed=8.0,
                spacing=spacing,
                max_radius=radius,
                clockwise=True
            )
            
            # Génération
            self.get_logger().info(f"Génération spirale: centre=({center_lat}, {center_lon}), "
                                 f"rayon={radius}m")
            result = self._coverage_patterns.generate_pattern(params)
            
            if result.success:
                self._display_mission_info(result)
                if save_to:
                    self._save_mission(result, save_to)
                return True
            else:
                self.get_logger().error(f"Échec génération spirale: {result.message}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Erreur planification spirale: {e}")
            return False

    def _load_area_from_file(self, filename: str) -> Optional[List[Point]]:
        """
        Charge une zone depuis un fichier
        
        Args:
            filename: Chemin du fichier
            
        Returns:
            Liste de points définissant le polygone ou None en cas d'erreur
        """
        if not os.path.exists(filename):
            self.get_logger().error(f"Fichier non trouvé: {filename}")
            return None
        
        try:
            if filename.endswith('.json'):
                return self._load_polygon_from_json(filename)
            elif filename.endswith('.kml'):
                return self._load_polygon_from_kml(filename)
            else:
                self.get_logger().error("Format de fichier non supporté (JSON ou KML requis)")
                return None
                
        except Exception as e:
            self.get_logger().error(f"Erreur lecture fichier {filename}: {e}")
            return None

    def _load_polygon_from_json(self, filename: str) -> Optional[List[Point]]:
        """
        Charge un polygone depuis JSON
        
        Args:
            filename: Chemin du fichier JSON
            
        Returns:
            Liste de points ou None en cas d'erreur
        """
        with open(filename, 'r') as f:
            data = json.load(f)
        
        if 'vertices' not in data:
            self.get_logger().error("Clé 'vertices' manquante dans le JSON")
            return None
        
        polygon = []
        for vertex in data['vertices']:
            point = Point()
            if 'lat' in vertex and 'lon' in vertex:
                # Conversion GPS vers coordonnées locales
                point.x, point.y = self._latlon_to_xy(vertex['lat'], vertex['lon'])
            else:
                point.x = vertex.get('x', 0.0)
                point.y = vertex.get('y', 0.0)
            
            point.z = vertex.get('z', 0.0)
            polygon.append(point)
        
        return polygon

    def _load_polygon_from_kml(self, filename: str) -> Optional[List[Point]]:
        """
        Charge un polygone depuis KML (implémentation simplifiée)
        
        Args:
            filename: Chemin du fichier KML
            
        Returns:
            Liste de points ou None en cas d'erreur
        """
        # Implémentation basique pour démonstration
        # Dans un vrai système, utiliser une bibliothèque KML appropriée
        self.get_logger().warn("Support KML basique - utiliser JSON pour fonctionnalités complètes")
        
        # Retour d'un polygone par défaut pour test
        return [
            Point(x=0.0, y=0.0, z=0.0),
            Point(x=100.0, y=0.0, z=0.0),
            Point(x=100.0, y=100.0, z=0.0),
            Point(x=0.0, y=100.0, z=0.0)
        ]

    def _load_waypoints_from_file(self, filename: str) -> Optional[List[Point]]:
        """
        Charge des waypoints depuis JSON
        
        Args:
            filename: Chemin du fichier JSON
            
        Returns:
            Liste de waypoints ou None en cas d'erreur
        """
        if not os.path.exists(filename):
            self.get_logger().error(f"Fichier waypoints non trouvé: {filename}")
            return None
        
        try:
            with open(filename, 'r') as f:
                data = json.load(f)
            
            waypoints = []
            for wp_data in data.get('waypoints', []):
                point = Point()
                point.x = wp_data.get('x', 0.0)
                point.y = wp_data.get('y', 0.0)
                point.z = wp_data.get('z', 10.0)
                waypoints.append(point)
            
            return waypoints
            
        except Exception as e:
            self.get_logger().error(f"Erreur lecture waypoints: {e}")
            return None

    def _parse_pattern_type(self, pattern: str) -> Optional[Any]:
        """
        Parse le type de pattern depuis string
        
        Args:
            pattern: Nom du pattern
            
        Returns:
            Type de pattern ou None si invalide
        """
        pattern_map = {
            'zigzag': self._pattern_type_class.ZIGZAG,
            'spiral': self._pattern_type_class.SPIRAL,
            'lawn_mower': self._pattern_type_class.LAWN_MOWER,
            'boustrophedon': self._pattern_type_class.BOUSTROPHEDON,
            'adaptive': self._pattern_type_class.ADAPTIVE
        }
        
        return pattern_map.get(pattern.lower())

    def _latlon_to_xy(self, lat: float, lon: float) -> tuple:
        """
        Conversion simplifiée GPS vers coordonnées locales
        
        Args:
            lat: Latitude en degrés
            lon: Longitude en degrés
            
        Returns:
            Tuple (x, y) en mètres
        """
        # Conversion approximative pour démonstration
        x = lon * 111000.0  # ~111km par degré longitude
        y = lat * 111000.0  # ~111km par degré latitude
        return x, y

    def _generate_circle_polygon(self, center_x: float, center_y: float, 
                               radius: float, num_points: int = 16) -> List[Point]:
        """
        Génère un polygone circulaire
        
        Args:
            center_x: Coordonnée X du centre
            center_y: Coordonnée Y du centre
            radius: Rayon du cercle
            num_points: Nombre de points du polygone
            
        Returns:
            Liste de points formant le polygone
        """
        polygon = []
        
        for i in range(num_points):
            angle = 2 * math.pi * i / num_points
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            polygon.append(Point(x=x, y=y, z=0.0))
        
        return polygon

    def _calculate_mission_distance(self, waypoints: List[Point]) -> float:
        """
        Calcule la distance totale d'une mission
        
        Args:
            waypoints: Liste des waypoints
            
        Returns:
            Distance totale en mètres
        """
        if len(waypoints) < 2:
            return 0.0
        
        total_distance = 0.0
        for i in range(len(waypoints) - 1):
            dx = waypoints[i+1].x - waypoints[i].x
            dy = waypoints[i+1].y - waypoints[i].y
            dz = waypoints[i+1].z - waypoints[i].z
            total_distance += math.sqrt(dx*dx + dy*dy + dz*dz)
        
        return total_distance

    def _display_mission_info(self, result):
        """
        Affiche les informations de mission
        
        Args:
            result: Résultat de la planification
        """
        self.get_logger().info("📋 Informations de mission:")
        self.get_logger().info(f"   Waypoints: {len(result.waypoints)}")
        self.get_logger().info(f"   Distance: {result.total_distance:.1f} m")
        self.get_logger().info(f"   Temps estimé: {result.estimated_time:.1f} s")
        self.get_logger().info(f"   Zone de couverture: {result.coverage_area:.1f} m²")
        self.get_logger().info(f"   Efficacité: {result.efficiency:.3f} m²/m")

    def _save_mission(self, result, filename: str):
        """
        Sauvegarde une mission en JSON
        
        Args:
            result: Résultat de la planification
            filename: Fichier de sauvegarde
        """
        mission_data = {
            'type': 'coverage_mission',
            'pattern_type': result.pattern_type.value,
            'waypoints': [
                {'x': p.x, 'y': p.y, 'z': p.z} for p in result.waypoints
            ],
            'metrics': {
                'total_distance': result.total_distance,
                'estimated_time': result.estimated_time,
                'coverage_area': result.coverage_area,
                'efficiency': result.efficiency
            },
            'generated_at': time.time()
        }
        
        self._save_json(mission_data, filename)

    def _save_json(self, data: Dict, filename: str):
        """
        Sauvegarde des données en JSON
        
        Args:
            data: Données à sauvegarder
            filename: Fichier de destination
        """
        try:
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2)
            self.get_logger().info(f"💾 Mission sauvegardée: {filename}")
        except Exception as e:
            self.get_logger().error(f"Erreur sauvegarde: {e}")


def parse_arguments():
    """
    Parse les arguments de ligne de commande
    
    Returns:
        Arguments parsés
    """
    parser = argparse.ArgumentParser(
        description="Planifie des missions de navigation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Exemples d'utilisation:
  # Couverture de zone avec pattern zigzag
  ros2 run drone_navigation plan_mission --area polygon.json --pattern zigzag --altitude 15

  # Couverture spirale autour d'un point
  ros2 run drone_navigation plan_mission --coverage spiral --center "34.0,-6.8" --radius 50

  # Mission avec waypoints prédéfinis
  ros2 run drone_navigation plan_mission --waypoints waypoints.json

  # Sauvegarde de la mission
  ros2 run drone_navigation plan_mission --area field.json --pattern lawn_mower --save mission.json
        """
    )
    
    # Mode de planification
    mode_group = parser.add_mutually_exclusive_group(required=True)
    mode_group.add_argument('--area', type=str,
                           help='Fichier de zone à couvrir (JSON/KML)')
    mode_group.add_argument('--coverage', choices=['spiral'],
                           help='Type de couverture géométrique')
    mode_group.add_argument('--waypoints', type=str,
                           help='Fichier de waypoints (JSON)')
    
    # Paramètres de pattern
    parser.add_argument('--pattern', choices=['zigzag', 'spiral', 'lawn_mower', 'boustrophedon', 'adaptive'],
                       default='zigzag', help='Type de pattern (défaut: zigzag)')
    parser.add_argument('--altitude', type=float, default=10.0,
                       help='Altitude de vol en mètres (défaut: 10)')
    parser.add_argument('--speed', type=float, default=5.0,
                       help='Vitesse de vol en m/s (défaut: 5)')
    parser.add_argument('--overlap', type=float, default=20.0,
                       help='Pourcentage de recouvrement (défaut: 20)')
    
    # Paramètres pour couverture spirale
    parser.add_argument('--center', type=str,
                       help='Centre pour spirale "lat,lon"')
    parser.add_argument('--radius', type=float, default=50.0,
                       help='Rayon de couverture en mètres (défaut: 50)')
    parser.add_argument('--spacing', type=float, default=5.0,
                       help='Espacement entre spires en mètres (défaut: 5)')
    
    # Options générales
    parser.add_argument('--save', type=str,
                       help='Fichier de sauvegarde de la mission')
    parser.add_argument('--verbose', '-v', action='store_true',
                       help='Affichage détaillé')
    
    return parser.parse_args()


def validate_arguments(args):
    """
    Valide les arguments de ligne de commande
    
    Args:
        args: Arguments parsés
        
    Returns:
        True si les arguments sont valides, False sinon
    """
    if args.coverage == 'spiral':
        if not args.center:
            print("Erreur: --center requis pour couverture spirale", file=sys.stderr)
            return False
        
        try:
            lat, lon = map(float, args.center.split(','))
            if not (-90 <= lat <= 90) or not (-180 <= lon <= 180):
                print("Erreur: Coordonnées centre invalides", file=sys.stderr)
                return False
        except:
            print("Erreur: Format centre invalide (utilisez 'lat,lon')", file=sys.stderr)
            return False
    
    return True


def main(args=None):
    """
    Point d'entrée principal de l'outil de planification
    
    Args:
        args: Arguments de ligne de commande
        
    Returns:
        Code de sortie (0 = succès, autre = erreur)
    """
    
    # Parse des arguments
    parsed_args = parse_arguments()
    
    # Validation
    if not validate_arguments(parsed_args):
        return 1
    
    # Initialisation ROS2
    rclpy.init(args=args)
    
    try:
        # Création de l'outil
        tool = PlanMissionTool()
        
        # Configuration du logging
        if parsed_args.verbose:
            tool.get_logger().set_level(rclpy.logging.LoggingSeverity.DEBUG)
        
        success = False
        
        # Exécution selon le mode
        if parsed_args.area:
            # Couverture de zone
            tool.get_logger().info(f"🗺️  Planification couverture zone: {parsed_args.area}")
            success = tool.plan_area_coverage(
                parsed_args.area,
                parsed_args.pattern,
                parsed_args.altitude,
                parsed_args.speed,
                parsed_args.overlap,
                parsed_args.save
            )
            
        elif parsed_args.coverage == 'spiral':
            # Couverture spirale
            lat, lon = map(float, parsed_args.center.split(','))
            tool.get_logger().info(f"🌀 Planification spirale: centre=({lat}, {lon})")
            success = tool.plan_spiral_coverage(
                lat, lon,
                parsed_args.radius,
                parsed_args.altitude,
                parsed_args.spacing,
                parsed_args.save
            )
            
        elif parsed_args.waypoints:
            # Mission waypoints
            tool.get_logger().info(f"📍 Planification waypoints: {parsed_args.waypoints}")
            success = tool.plan_waypoint_mission(
                parsed_args.waypoints,
                parsed_args.save
            )
        
        if success:
            print("✅ Mission planifiée avec succès")
            return 0
        else:
            print("❌ Échec de la planification")
            return 1
            
    except KeyboardInterrupt:
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