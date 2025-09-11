#!/usr/bin/env python3
"""
Coverage Pattern Node - Générateur de motifs de couverture pour missions de pollinisation
Architecture: Micro-nœud spécialisé dans la génération de motifs de vol
Patterns: Zigzag, Spiral, Adaptatif, Grid
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import math
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass, field
from enum import Enum
import json

# Messages ROS2
from geometry_msgs.msg import Point, PoseStamped, Polygon, Point32
from nav_msgs.msg import Path
from std_msgs.msg import Header, String
from visualization_msgs.msg import Marker, MarkerArray

# Services personnalisés
from drone_msgs.srv import GenerateCoveragePattern, OptimizePattern
from drone_msgs.msg import CoverageArea, PatternParameters, CoverageStats

# Utilitaires
from shapely.geometry import Polygon as ShapelyPolygon, Point as ShapelyPoint
from shapely.ops import unary_union
import matplotlib.pyplot as plt
from scipy.spatial import ConvexHull


class PatternType(Enum):
    """Types de motifs de couverture disponibles"""
    ZIGZAG = "zigzag"
    SPIRAL = "spiral"
    GRID = "grid"
    ADAPTIVE = "adaptive"
    LAWN_MOWER = "lawn_mower"
    BOUSTROPHEDON = "boustrophedon"


@dataclass
class CoverageConfiguration:
    """Configuration pour la génération de motifs"""
    pattern_type: PatternType = PatternType.ZIGZAG
    line_spacing: float = 2.0  # Espacement entre lignes (m)
    flight_altitude: float = 5.0  # Altitude de vol (m)
    overlap_percentage: float = 20.0  # Pourcentage de recouvrement
    turn_radius: float = 1.0  # Rayon de virage (m)
    max_velocity: float = 3.0  # Vitesse maximale (m/s)
    entry_point: Optional[Tuple[float, float]] = None
    exit_point: Optional[Tuple[float, float]] = None
    wind_direction: float = 0.0  # Direction du vent (rad)
    wind_speed: float = 0.0  # Vitesse du vent (m/s)
    optimization_enabled: bool = True
    pattern_parameters: Dict[str, Any] = field(default_factory=dict)


@dataclass
class GeneratedPattern:
    """Motif de couverture généré"""
    waypoints: List[Tuple[float, float, float]]  # x, y, z
    total_distance: float
    estimated_time: float
    coverage_percentage: float
    pattern_metadata: Dict[str, Any]
    visualization_markers: List[Marker] = field(default_factory=list)


class CoveragePatternNode(Node):
    """
    Nœud de génération de motifs de couverture pour missions autonomes
    
    Responsabilités:
    - Génération de motifs de vol optimisés
    - Adaptation aux conditions météo
    - Optimisation de la couverture
    - Visualisation des patterns
    """
    
    def __init__(self):
        super().__init__('coverage_pattern_node')
        
        # Configuration du nœud
        self.declare_parameters()
        self.load_configuration()
        
        # État interne
        self.current_patterns: Dict[str, GeneratedPattern] = {}
        self.active_area: Optional[ShapelyPolygon] = None
        self.coverage_stats = CoverageStats()
        
        # Callback groups pour threading
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.publisher_cb_group = ReentrantCallbackGroup()
        
        # Publishers
        self.path_publisher = self.create_publisher(
            Path, 
            '/drone_nav/coverage_path', 
            10,
            callback_group=self.publisher_cb_group
        )
        
        self.marker_publisher = self.create_publisher(
            MarkerArray,
            '/drone_nav/coverage_visualization',
            10,
            callback_group=self.publisher_cb_group
        )
        
        self.stats_publisher = self.create_publisher(
            CoverageStats,
            '/drone_nav/coverage_stats',
            10,
            callback_group=self.publisher_cb_group
        )
        
        # Services
        self.generate_service = self.create_service(
            GenerateCoveragePattern,
            '/drone_nav/coverage_pattern_node/generate_pattern',
            self.generate_pattern_callback,
            callback_group=self.service_cb_group
        )
        
        self.optimize_service = self.create_service(
            OptimizePattern,
            '/drone_nav/coverage_pattern_node/optimize_pattern',
            self.optimize_pattern_callback,
            callback_group=self.service_cb_group
        )
        
        # Timer pour publication périodique des statistiques
        self.stats_timer = self.create_timer(
            2.0,  # 0.5 Hz
            self.publish_stats,
            callback_group=self.publisher_cb_group
        )
        
        self.get_logger().info("✅ Coverage Pattern Node initialisé")
    
    def declare_parameters(self):
        """Déclaration des paramètres du nœud"""
        self.declare_parameter('default_pattern_type', 'zigzag')
        self.declare_parameter('default_line_spacing', 2.0)
        self.declare_parameter('default_flight_altitude', 5.0)
        self.declare_parameter('default_overlap_percentage', 20.0)
        self.declare_parameter('default_turn_radius', 1.0)
        self.declare_parameter('default_max_velocity', 3.0)
        self.declare_parameter('enable_wind_compensation', True)
        self.declare_parameter('enable_visualization', True)
        self.declare_parameter('optimization_algorithm', 'genetic')
        self.declare_parameter('max_pattern_complexity', 1000)
    
    def load_configuration(self):
        """Charge la configuration depuis les paramètres"""
        self.config = CoverageConfiguration(
            pattern_type=PatternType(self.get_parameter('default_pattern_type').value),
            line_spacing=self.get_parameter('default_line_spacing').value,
            flight_altitude=self.get_parameter('default_flight_altitude').value,
            overlap_percentage=self.get_parameter('default_overlap_percentage').value,
            turn_radius=self.get_parameter('default_turn_radius').value,
            max_velocity=self.get_parameter('default_max_velocity').value
        )
        
        self.enable_wind_compensation = self.get_parameter('enable_wind_compensation').value
        self.enable_visualization = self.get_parameter('enable_visualization').value
        self.optimization_algorithm = self.get_parameter('optimization_algorithm').value
        self.max_complexity = self.get_parameter('max_pattern_complexity').value
    
    async def generate_pattern_callback(self, request, response):
        """Service de génération de motif de couverture"""
        try:
            self.get_logger().info(f"🎯 Génération motif: {request.pattern_type}")
            
            # Conversion de la zone de couverture
            area_polygon = self.convert_coverage_area(request.coverage_area)
            
            # Configuration du motif
            config = self.build_configuration_from_request(request)
            
            # Génération du motif
            pattern = await self.generate_coverage_pattern(area_polygon, config)
            
            # Stockage du motif
            pattern_id = f"pattern_{len(self.current_patterns)}"
            self.current_patterns[pattern_id] = pattern
            
            # Construction de la réponse
            response.success = True
            response.pattern_id = pattern_id
            response.total_distance = pattern.total_distance
            response.estimated_time = pattern.estimated_time
            response.coverage_percentage = pattern.coverage_percentage
            response.waypoint_count = len(pattern.waypoints)
            
            # Publication du chemin
            self.publish_pattern_path(pattern, pattern_id)
            
            # Visualisation si activée
            if self.enable_visualization:
                self.publish_visualization(pattern, pattern_id)
            
            self.get_logger().info(f"✅ Motif généré: {len(pattern.waypoints)} waypoints")
            
        except Exception as e:
            self.get_logger().error(f"❌ Erreur génération motif: {str(e)}")
            response.success = False
            response.error_message = str(e)
        
        return response
    
    async def optimize_pattern_callback(self, request, response):
        """Service d'optimisation de motif existant"""
        try:
            pattern_id = request.pattern_id
            
            if pattern_id not in self.current_patterns:
                raise ValueError(f"Motif {pattern_id} non trouvé")
            
            original_pattern = self.current_patterns[pattern_id]
            
            # Optimisation du motif
            optimized_pattern = await self.optimize_pattern(
                original_pattern, 
                request.optimization_criteria
            )
            
            # Mise à jour du motif
            self.current_patterns[pattern_id] = optimized_pattern
            
            # Calcul des améliorations
            distance_improvement = (
                (original_pattern.total_distance - optimized_pattern.total_distance) / 
                original_pattern.total_distance * 100
            )
            
            time_improvement = (
                (original_pattern.estimated_time - optimized_pattern.estimated_time) / 
                original_pattern.estimated_time * 100
            )
            
            response.success = True
            response.distance_improvement = distance_improvement
            response.time_improvement = time_improvement
            response.new_total_distance = optimized_pattern.total_distance
            response.new_estimated_time = optimized_pattern.estimated_time
            
            # Publication du motif optimisé
            self.publish_pattern_path(optimized_pattern, pattern_id)
            
            self.get_logger().info(
                f"✅ Motif optimisé: {distance_improvement:.1f}% distance, "
                f"{time_improvement:.1f}% temps"
            )
            
        except Exception as e:
            self.get_logger().error(f"❌ Erreur optimisation: {str(e)}")
            response.success = False
            response.error_message = str(e)
        
        return response
    
    def convert_coverage_area(self, coverage_area_msg) -> ShapelyPolygon:
        """Convertit un message CoverageArea en Shapely Polygon"""
        points = []
        for point in coverage_area_msg.boundary.points:
            points.append((point.x, point.y))
        
        return ShapelyPolygon(points)
    
    def build_configuration_from_request(self, request) -> CoverageConfiguration:
        """Construit la configuration à partir de la requête"""
        config = CoverageConfiguration(
            pattern_type=PatternType(request.pattern_type),
            line_spacing=request.parameters.line_spacing,
            flight_altitude=request.parameters.flight_altitude,
            overlap_percentage=request.parameters.overlap_percentage,
            turn_radius=request.parameters.turn_radius,
            max_velocity=request.parameters.max_velocity
        )
        
        # Paramètres météo
        if hasattr(request.parameters, 'wind_direction'):
            config.wind_direction = request.parameters.wind_direction
            config.wind_speed = request.parameters.wind_speed
        
        return config
    
    async def generate_coverage_pattern(
        self, 
        area: ShapelyPolygon, 
        config: CoverageConfiguration
    ) -> GeneratedPattern:
        """Génère un motif de couverture selon le type demandé"""
        
        if config.pattern_type == PatternType.ZIGZAG:
            return await self.generate_zigzag_pattern(area, config)
        elif config.pattern_type == PatternType.SPIRAL:
            return await self.generate_spiral_pattern(area, config)
        elif config.pattern_type == PatternType.GRID:
            return await self.generate_grid_pattern(area, config)
        elif config.pattern_type == PatternType.ADAPTIVE:
            return await self.generate_adaptive_pattern(area, config)
        elif config.pattern_type == PatternType.LAWN_MOWER:
            return await self.generate_lawn_mower_pattern(area, config)
        elif config.pattern_type == PatternType.BOUSTROPHEDON:
            return await self.generate_boustrophedon_pattern(area, config)
        else:
            raise ValueError(f"Type de motif non supporté: {config.pattern_type}")
    
    async def generate_zigzag_pattern(
        self, 
        area: ShapelyPolygon, 
        config: CoverageConfiguration
    ) -> GeneratedPattern:
        """Génère un motif en zigzag optimisé"""
        
        # Calcul de l'orientation optimale
        bounds = area.bounds
        width = bounds[2] - bounds[0]
        height = bounds[3] - bounds[1]
        
        # Orientation préférée (parallèle au côté le plus long)
        if width > height:
            angle = 0.0  # Horizontal
        else:
            angle = math.pi / 2  # Vertical
        
        # Compensation du vent si activée
        if self.enable_wind_compensation and config.wind_speed > 0.5:
            angle = self.optimize_angle_for_wind(angle, config.wind_direction)
        
        # Génération des lignes de balayage
        waypoints = []
        
        # Calcul du nombre de lignes
        if angle == 0.0:  # Horizontal
            num_lines = int(height / config.line_spacing) + 1
            for i in range(num_lines):
                y = bounds[1] + i * config.line_spacing
                
                if i % 2 == 0:  # Ligne paire - gauche vers droite
                    x_start, x_end = bounds[0], bounds[2]
                else:  # Ligne impaire - droite vers gauche
                    x_start, x_end = bounds[2], bounds[0]
                
                # Points d'intersection avec le polygone
                line_points = self.intersect_line_with_polygon(
                    area, y, x_start, x_end, angle
                )
                
                for point in line_points:
                    waypoints.append((point[0], point[1], config.flight_altitude))
                
                # Ajout des virages si nécessaire
                if i < num_lines - 1:
                    turn_points = self.generate_turn_waypoints(
                        waypoints[-1], config.turn_radius, angle
                    )
                    waypoints.extend(turn_points)
        
        else:  # Vertical
            num_lines = int(width / config.line_spacing) + 1
            for i in range(num_lines):
                x = bounds[0] + i * config.line_spacing
                
                if i % 2 == 0:  # Ligne paire - bas vers haut
                    y_start, y_end = bounds[1], bounds[3]
                else:  # Ligne impaire - haut vers bas
                    y_start, y_end = bounds[3], bounds[1]
                
                # Points d'intersection avec le polygone
                line_points = self.intersect_line_with_polygon(
                    area, x, y_start, y_end, angle
                )
                
                for point in line_points:
                    waypoints.append((point[0], point[1], config.flight_altitude))
                
                # Ajout des virages
                if i < num_lines - 1:
                    turn_points = self.generate_turn_waypoints(
                        waypoints[-1], config.turn_radius, angle
                    )
                    waypoints.extend(turn_points)
        
        # Calcul des métriques
        total_distance = self.calculate_path_distance(waypoints)
        estimated_time = total_distance / config.max_velocity
        coverage_percentage = self.estimate_coverage_percentage(area, waypoints, config)
        
        pattern = GeneratedPattern(
            waypoints=waypoints,
            total_distance=total_distance,
            estimated_time=estimated_time,
            coverage_percentage=coverage_percentage,
            pattern_metadata={
                'type': 'zigzag',
                'angle': angle,
                'num_lines': num_lines,
                'line_spacing': config.line_spacing
            }
        )
        
        return pattern
    
    async def generate_spiral_pattern(
        self, 
        area: ShapelyPolygon, 
        config: CoverageConfiguration
    ) -> GeneratedPattern:
        """Génère un motif en spirale"""
        
        # Centre de la zone
        centroid = area.centroid
        center_x, center_y = centroid.x, centroid.y
        
        # Rayon initial et incrémentation
        radius_increment = config.line_spacing
        max_radius = max(area.bounds[2] - area.bounds[0], area.bounds[3] - area.bounds[1]) / 2
        
        waypoints = []
        radius = radius_increment
        angle = 0.0
        angle_increment = radius_increment / radius if radius > 0 else 0.1
        
        while radius < max_radius:
            # Point sur la spirale
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            point = ShapelyPoint(x, y)
            
            # Vérifier si le point est dans la zone
            if area.contains(point):
                waypoints.append((x, y, config.flight_altitude))
            
            # Mise à jour des paramètres
            angle += angle_increment
            radius += radius_increment * angle_increment / (2 * math.pi)
            
            # Recalcul de l'incrément d'angle
            if radius > 0:
                angle_increment = radius_increment / radius
        
        # Métriques
        total_distance = self.calculate_path_distance(waypoints)
        estimated_time = total_distance / config.max_velocity
        coverage_percentage = self.estimate_coverage_percentage(area, waypoints, config)
        
        return GeneratedPattern(
            waypoints=waypoints,
            total_distance=total_distance,
            estimated_time=estimated_time,
            coverage_percentage=coverage_percentage,
            pattern_metadata={
                'type': 'spiral',
                'center': (center_x, center_y),
                'max_radius': max_radius,
                'radius_increment': radius_increment
            }
        )
    
    async def generate_adaptive_pattern(
        self, 
        area: ShapelyPolygon, 
        config: CoverageConfiguration
    ) -> GeneratedPattern:
        """Génère un motif adaptatif basé sur la forme de la zone"""
        
        # Analyse de la forme
        shape_analysis = self.analyze_polygon_shape(area)
        
        if shape_analysis['aspect_ratio'] > 3.0:
            # Zone très allongée -> zigzag
            return await self.generate_zigzag_pattern(area, config)
        elif shape_analysis['convexity'] < 0.7:
            # Zone très concave -> décomposition en sous-zones
            return await self.generate_decomposed_pattern(area, config)
        elif shape_analysis['area'] < 100:  # Petite zone
            # Petite zone -> spirale
            return await self.generate_spiral_pattern(area, config)
        else:
            # Zone normale -> grille
            return await self.generate_grid_pattern(area, config)
    
    def analyze_polygon_shape(self, polygon: ShapelyPolygon) -> Dict[str, float]:
        """Analyse les caractéristiques géométriques d'un polygone"""
        bounds = polygon.bounds
        width = bounds[2] - bounds[0]
        height = bounds[3] - bounds[1]
        
        # Ratio d'aspect
        aspect_ratio = max(width, height) / min(width, height)
        
        # Convexité (aire du polygone / aire de l'enveloppe convexe)
        convex_hull = polygon.convex_hull
        convexity = polygon.area / convex_hull.area if convex_hull.area > 0 else 0
        
        # Compacité (périmètre² / aire)
        compactness = (polygon.length ** 2) / polygon.area if polygon.area > 0 else float('inf')
        
        return {
            'aspect_ratio': aspect_ratio,
            'convexity': convexity,
            'compactness': compactness,
            'area': polygon.area,
            'perimeter': polygon.length
        }
    
    def intersect_line_with_polygon(
        self, 
        polygon: ShapelyPolygon, 
        coord: float, 
        start: float, 
        end: float, 
        angle: float
    ) -> List[Tuple[float, float]]:
        """Calcule les intersections d'une ligne avec un polygone"""
        from shapely.geometry import LineString
        
        if angle == 0.0:  # Ligne horizontale
            line = LineString([(start, coord), (end, coord)])
        else:  # Ligne verticale
            line = LineString([(coord, start), (coord, end)])
        
        intersection = polygon.intersection(line)
        
        points = []
        if hasattr(intersection, 'geoms'):  # MultiLineString
            for geom in intersection.geoms:
                if hasattr(geom, 'coords'):
                    points.extend(list(geom.coords))
        elif hasattr(intersection, 'coords'):  # LineString
            points.extend(list(intersection.coords))
        
        return points
    
    def generate_turn_waypoints(
        self, 
        last_point: Tuple[float, float, float], 
        turn_radius: float, 
        angle: float
    ) -> List[Tuple[float, float, float]]:
        """Génère des waypoints pour les virages"""
        # Implémentation simplifiée - arc de cercle
        waypoints = []
        
        # Calcul de l'arc de virage
        num_turn_points = max(3, int(turn_radius * 2))
        turn_angle = math.pi  # Demi-tour
        
        for i in range(1, num_turn_points):
            t = i / num_turn_points
            arc_angle = t * turn_angle
            
            # Point sur l'arc
            offset_x = turn_radius * math.cos(arc_angle)
            offset_y = turn_radius * math.sin(arc_angle)
            
            x = last_point[0] + offset_x
            y = last_point[1] + offset_y
            z = last_point[2]
            
            waypoints.append((x, y, z))
        
        return waypoints
    
    def calculate_path_distance(self, waypoints: List[Tuple[float, float, float]]) -> float:
        """Calcule la distance totale d'un chemin"""
        if len(waypoints) < 2:
            return 0.0
        
        total_distance = 0.0
        for i in range(1, len(waypoints)):
            p1 = waypoints[i-1]
            p2 = waypoints[i]
            
            dx = p2[0] - p1[0]
            dy = p2[1] - p1[1]
            dz = p2[2] - p1[2]
            
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            total_distance += distance
        
        return total_distance
    
    def estimate_coverage_percentage(
        self, 
        area: ShapelyPolygon, 
        waypoints: List[Tuple[float, float, float]], 
        config: CoverageConfiguration
    ) -> float:
        """Estime le pourcentage de couverture du motif"""
        
        # Calcul simplifié basé sur l'espacement et la zone
        # TODO: Implémentation plus précise avec simulation de capteur
        
        total_area = area.area
        covered_area = 0.0
        
        # Simulation de couverture avec buffer autour des waypoints
        coverage_radius = config.line_spacing / 2  # Rayon de couverture du capteur
        
        from shapely.geometry import Point as ShapelyPoint
        
        covered_zones = []
        for waypoint in waypoints:
            point = ShapelyPoint(waypoint[0], waypoint[1])
            coverage_circle = point.buffer(coverage_radius)
            covered_zones.append(coverage_circle)
        
        # Union de toutes les zones couvertes
        if covered_zones:
            total_coverage = unary_union(covered_zones)
            intersection_with_area = area.intersection(total_coverage)
            covered_area = intersection_with_area.area
        
        coverage_percentage = (covered_area / total_area * 100.0) if total_area > 0 else 0.0
        return min(coverage_percentage, 100.0)
    
    def optimize_angle_for_wind(self, base_angle: float, wind_direction: float) -> float:
        """Optimise l'angle du motif en fonction du vent"""
        # Stratégie: orienter le motif pour minimiser la dérive due au vent
        # L'angle optimal est perpendiculaire à la direction du vent
        optimal_angle = wind_direction + math.pi / 2
        
        # Pondération entre l'angle de base et l'angle optimal
        weight = 0.3  # 30% d'influence du vent
        optimized_angle = base_angle * (1 - weight) + optimal_angle * weight
        
        return optimized_angle
    
    async def optimize_pattern(
        self, 
        pattern: GeneratedPattern, 
        criteria: str
    ) -> GeneratedPattern:
        """Optimise un motif existant selon des critères donnés"""
        
        if criteria == "distance":
            return await self.optimize_for_distance(pattern)
        elif criteria == "time":
            return await self.optimize_for_time(pattern)
        elif criteria == "coverage":
            return await self.optimize_for_coverage(pattern)
        elif criteria == "energy":
            return await self.optimize_for_energy(pattern)
        else:
            return pattern  # Pas d'optimisation
    
    async def optimize_for_distance(self, pattern: GeneratedPattern) -> GeneratedPattern:
        """Optimisation pour minimiser la distance"""
        # Algorithme TSP (Traveling Salesman Problem) simplifié
        waypoints = pattern.waypoints.copy()
        
        if len(waypoints) <= 3:
            return pattern
        
        # Algorithme 2-opt pour l'optimisation TSP
        improved = True
        iterations = 0
        max_iterations = 100
        
        while improved and iterations < max_iterations:
            improved = False
            iterations += 1
            
            for i in range(1, len(waypoints) - 2):
                for j in range(i + 1, len(waypoints)):
                    if j - i == 1:
                        continue  # Skip adjacent edges
                    
                    # Calcul de la distance actuelle
                    current_distance = (
                        self.distance_between_points(waypoints[i-1], waypoints[i]) +
                        self.distance_between_points(waypoints[j-1], waypoints[j])
                    )
                    
                    # Calcul de la distance après échange
                    new_distance = (
                        self.distance_between_points(waypoints[i-1], waypoints[j-1]) +
                        self.distance_between_points(waypoints[i], waypoints[j])
                    )
                    
                    # Si amélioration, effectuer l'échange
                    if new_distance < current_distance:
                        waypoints[i:j] = reversed(waypoints[i:j])
                        improved = True
        
        # Recalcul des métriques
        total_distance = self.calculate_path_distance(waypoints)
        estimated_time = total_distance / 3.0  # Vitesse par défaut
        
        optimized_pattern = GeneratedPattern(
            waypoints=waypoints,
            total_distance=total_distance,
            estimated_time=estimated_time,
            coverage_percentage=pattern.coverage_percentage,
            pattern_metadata=pattern.pattern_metadata.copy()
        )
        
        optimized_pattern.pattern_metadata['optimized_for'] = 'distance'
        optimized_pattern.pattern_metadata['optimization_iterations'] = iterations
        
        return optimized_pattern
    
    def distance_between_points(
        self, 
        p1: Tuple[float, float, float], 
        p2: Tuple[float, float, float]
    ) -> float:
        """Calcule la distance euclidienne entre deux points"""
        dx = p2[0] - p1[0]
        dy = p2[1] - p1[1]
        dz = p2[2] - p1[2]
        return math.sqrt(dx*dx + dy*dy + dz*dz)
    
    def publish_pattern_path(self, pattern: GeneratedPattern, pattern_id: str):
        """Publie le chemin du motif"""
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        for waypoint in pattern.waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = waypoint[2]
            
            # Orientation par défaut
            pose.pose.orientation.w = 1.0
            
            path_msg.poses.append(pose)
        
        self.path_publisher.publish(path_msg)
        self.get_logger().info(f"📍 Chemin publié: {len(pattern.waypoints)} waypoints")
    
    def publish_visualization(self, pattern: GeneratedPattern, pattern_id: str):
        """Publie les marqueurs de visualisation"""
        marker_array = MarkerArray()
        
        # Marqueur pour les waypoints
        waypoint_marker = Marker()
        waypoint_marker.header.frame_id = "map"
        waypoint_marker.header.stamp = self.get_clock().now().to_msg()
        waypoint_marker.ns = f"pattern_{pattern_id}"
        waypoint_marker.id = 0
        waypoint_marker.type = Marker.SPHERE_LIST
        waypoint_marker.action = Marker.ADD
        waypoint_marker.scale.x = 0.3
        waypoint_marker.scale.y = 0.3
        waypoint_marker.scale.z = 0.3
        waypoint_marker.color.r = 0.0
        waypoint_marker.color.g = 1.0
        waypoint_marker.color.b = 0.0
        waypoint_marker.color.a = 0.8
        
        for waypoint in pattern.waypoints:
            point = Point()
            point.x = waypoint[0]
            point.y = waypoint[1]
            point.z = waypoint[2]
            waypoint_marker.points.append(point)
        
        marker_array.markers.append(waypoint_marker)
        
        # Marqueur pour les lignes de connexion
        line_marker = Marker()
        line_marker.header.frame_id = "map"
        line_marker.header.stamp = self.get_clock().now().to_msg()
        line_marker.ns = f"pattern_{pattern_id}"
        line_marker.id = 1
        line_marker.type = Marker.LINE_STRIP
        line_marker.action = Marker.ADD
        line_marker.scale.x = 0.1
        line_marker.color.r = 1.0
        line_marker.color.g = 0.0
        line_marker.color.b = 0.0
        line_marker.color.a = 0.6
        
        for waypoint in pattern.waypoints:
            point = Point()
            point.x = waypoint[0]
            point.y = waypoint[1]
            point.z = waypoint[2]
            line_marker.points.append(point)
        
        marker_array.markers.append(line_marker)
        
        self.marker_publisher.publish(marker_array)
    
    def publish_stats(self):
        """Publie les statistiques de couverture"""
        self.coverage_stats.header.stamp = self.get_clock().now().to_msg()
        self.coverage_stats.active_patterns = len(self.current_patterns)
        
        if self.current_patterns:
            total_waypoints = sum(len(p.waypoints) for p in self.current_patterns.values())
            total_distance = sum(p.total_distance for p in self.current_patterns.values())
            avg_coverage = sum(p.coverage_percentage for p in self.current_patterns.values()) / len(self.current_patterns)
            
            self.coverage_stats.total_waypoints = total_waypoints
            self.coverage_stats.total_distance = total_distance
            self.coverage_stats.average_coverage_percentage = avg_coverage
        
        self.stats_publisher.publish(self.coverage_stats)


def main(args=None):
    """Point d'entrée principal"""
    rclpy.init(args=args)
    
    try:
        node = CoveragePatternNode()
        
        # Utilisation d'un exécuteur multi-threadé pour les callback groups
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        node.get_logger().info("🚀 Coverage Pattern Node démarré")
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Erreur fatale: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
