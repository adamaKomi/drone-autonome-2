#!/usr/bin/env python3

"""
Générateur de patterns de couverture pour drone autonome

Implémente différents patterns de vol pour missions de pollinisation
et cartographie avec optimisation de la couverture de zone.

Patterns supportés:
- Zigzag (boustrophédon)
- Spirale
- Lawn mower (tondeuse)
- Grille personnalisée

Auteur: Adama Komi
Version: 1.0.0
"""

import math
import numpy as np
from typing import List, Tuple, Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum
from geometry_msgs.msg import Point, Polygon, Point32


class PatternType(Enum):
    """Types de patterns de couverture disponibles"""
    ZIGZAG = "zigzag"
    SPIRAL = "spiral" 
    LAWN_MOWER = "lawn_mower"
    GRID = "grid"
    PARALLEL_LINES = "parallel_lines"
    CONCENTRIC = "concentric"


class TurnDirection(Enum):
    """Direction de virage"""
    LEFT = "left"
    RIGHT = "right"
    SHORTEST = "shortest"


@dataclass
class PatternConfig:
    """Configuration pour génération de pattern"""
    pattern_type: PatternType
    line_spacing: float = 5.0      # Espacement entre lignes en mètres
    overlap_percentage: float = 20.0 # Pourcentage de recouvrement
    altitude: float = 10.0         # Altitude de vol en mètres
    turn_radius: float = 3.0       # Rayon de virage minimum en mètres
    entry_point: str = "bottom_left"  # Point d'entrée dans la zone
    turn_direction: TurnDirection = TurnDirection.SHORTEST
    optimize_path: bool = True     # Optimiser le chemin pour réduire la distance


@dataclass
class CoverageZone:
    """Définition d'une zone à couvrir"""
    boundary: List[Point]          # Périmètre de la zone
    holes: List[List[Point]] = None # Trous à éviter dans la zone
    name: str = ""                 # Nom de la zone
    priority: int = 1              # Priorité de couverture


class CoveragePatterns:
    """
    Générateur de patterns de couverture pour navigation drone
    
    Génère des trajectoires optimisées pour couvrir efficacement
    des zones définies selon différents patterns de vol.
    """
    
    def __init__(self, node):
        """
        Initialise le générateur de patterns
        
        Args:
            node: Nœud ROS2 parent
        """
        self._node = node
        self._logger = node.get_logger()
        
        # Configuration par défaut
        self._default_config = PatternConfig(
            pattern_type=PatternType.ZIGZAG,
            line_spacing=5.0,
            overlap_percentage=20.0,
            altitude=10.0,
            turn_radius=3.0
        )
        
        # Statistiques
        self._stats = {
            'patterns_generated': 0,
            'total_distance': 0.0,
            'zones_covered': 0
        }
        
        self._logger.info("CoveragePatterns initialisé")

    def generate_zigzag_pattern(self, zone: CoverageZone, 
                               config: PatternConfig) -> List[Point]:
        """
        Génère un pattern zigzag (boustrophédon)
        
        Args:
            zone: Zone à couvrir
            config: Configuration du pattern
            
        Returns:
            Liste de points définissant la trajectoire
        """
        if not zone.boundary or len(zone.boundary) < 3:
            self._logger.error("Zone invalide pour pattern zigzag")
            return []
            
        # Calcul de la bounding box
        min_x = min(p.x for p in zone.boundary)
        max_x = max(p.x for p in zone.boundary)
        min_y = min(p.y for p in zone.boundary)
        max_y = max(p.y for p in zone.boundary)
        
        waypoints = []
        y = min_y
        going_right = True
        
        # Génération des lignes parallèles
        while y <= max_y:
            if going_right:
                start_x, end_x = min_x, max_x
            else:
                start_x, end_x = max_x, min_x
                
            # Points de début et fin de ligne
            start_point = Point(x=start_x, y=y, z=config.altitude)
            end_point = Point(x=end_x, y=y, z=config.altitude)
            
            # Intersection avec le polygone de la zone
            line_points = self._clip_line_to_polygon(
                start_point, end_point, zone.boundary
            )
            
            if line_points:
                waypoints.extend(line_points)
                
                # Ajout du virage si pas la dernière ligne
                if y + config.line_spacing <= max_y:
                    turn_points = self._generate_turn(
                        line_points[-1], y + config.line_spacing, 
                        config, not going_right
                    )
                    waypoints.extend(turn_points)
            
            y += config.line_spacing
            going_right = not going_right
        
        self._stats['patterns_generated'] += 1
        self._stats['total_distance'] += self._calculate_path_distance(waypoints)
        
        self._logger.info(f"Pattern zigzag généré: {len(waypoints)} waypoints")
        return waypoints

    def generate_spiral_pattern(self, zone: CoverageZone, 
                               config: PatternConfig) -> List[Point]:
        """
        Génère un pattern spirale
        
        Args:
            zone: Zone à couvrir
            config: Configuration du pattern
            
        Returns:
            Liste de points définissant la trajectoire spirale
        """
        if not zone.boundary or len(zone.boundary) < 3:
            self._logger.error("Zone invalide pour pattern spirale")
            return []
            
        # Calcul du centre de la zone
        center_x = sum(p.x for p in zone.boundary) / len(zone.boundary)
        center_y = sum(p.y for p in zone.boundary) / len(zone.boundary)
        center = Point(x=center_x, y=center_y, z=config.altitude)
        
        # Calcul du rayon maximum
        max_radius = max(
            math.sqrt((p.x - center_x)**2 + (p.y - center_y)**2) 
            for p in zone.boundary
        )
        
        waypoints = []
        radius = config.line_spacing / 2
        angle = 0.0
        angle_increment = math.pi / 16  # 32 points par tour
        
        # Génération de la spirale
        while radius <= max_radius:
            x = center_x + radius * math.cos(angle)
            y = center_y + radius * math.sin(angle)
            point = Point(x=x, y=y, z=config.altitude)
            
            # Vérifier si le point est dans la zone
            if self._point_in_polygon(point, zone.boundary):
                waypoints.append(point)
            
            # Augmentation progressive du rayon
            radius += config.line_spacing / (2 * math.pi / angle_increment)
            angle += angle_increment
            
        self._stats['patterns_generated'] += 1
        self._stats['total_distance'] += self._calculate_path_distance(waypoints)
        
        self._logger.info(f"Pattern spirale généré: {len(waypoints)} waypoints")
        return waypoints

    def generate_lawn_mower_pattern(self, zone: CoverageZone, 
                                   config: PatternConfig) -> List[Point]:
        """
        Génère un pattern lawn mower (tondeuse)
        
        Args:
            zone: Zone à couvrir
            config: Configuration du pattern
            
        Returns:
            Liste de points définissant la trajectoire
        """
        # Pattern lawn mower similaire au zigzag mais avec virages optimisés
        zigzag_points = self.generate_zigzag_pattern(zone, config)
        
        if not zigzag_points:
            return []
            
        # Optimisation des virages pour pattern tondeuse
        optimized_points = []
        i = 0
        
        while i < len(zigzag_points):
            optimized_points.append(zigzag_points[i])
            
            # Recherche du prochain point de virage
            if i + 2 < len(zigzag_points):
                # Ajout de points de virage arrondis
                turn_start = zigzag_points[i]
                turn_end = zigzag_points[i + 2]
                
                rounded_turn = self._generate_rounded_turn(
                    turn_start, turn_end, config.turn_radius
                )
                optimized_points.extend(rounded_turn)
                
                i += 2
            else:
                i += 1
        
        self._logger.info(f"Pattern lawn mower généré: {len(optimized_points)} waypoints")
        return optimized_points

    def generate_grid_pattern(self, zone: CoverageZone, 
                             config: PatternConfig) -> List[Point]:
        """
        Génère un pattern grille régulière
        
        Args:
            zone: Zone à couvrir
            config: Configuration du pattern
            
        Returns:
            Liste de points définissant la grille
        """
        if not zone.boundary or len(zone.boundary) < 3:
            self._logger.error("Zone invalide pour pattern grille")
            return []
            
        # Calcul de la bounding box
        min_x = min(p.x for p in zone.boundary)
        max_x = max(p.x for p in zone.boundary)
        min_y = min(p.y for p in zone.boundary)
        max_y = max(p.y for p in zone.boundary)
        
        waypoints = []
        
        # Génération de la grille
        y = min_y
        while y <= max_y:
            x = min_x
            while x <= max_x:
                point = Point(x=x, y=y, z=config.altitude)
                
                # Vérifier si le point est dans la zone
                if self._point_in_polygon(point, zone.boundary):
                    waypoints.append(point)
                    
                x += config.line_spacing
            y += config.line_spacing
        
        # Optimisation de l'ordre de visite
        if config.optimize_path:
            waypoints = self._optimize_grid_path(waypoints)
        
        self._stats['patterns_generated'] += 1
        self._stats['total_distance'] += self._calculate_path_distance(waypoints)
        
        self._logger.info(f"Pattern grille généré: {len(waypoints)} waypoints")
        return waypoints

    def generate_pattern(self, pattern_type: PatternType, zone: CoverageZone,
                        config: Optional[PatternConfig] = None) -> List[Point]:
        """
        Génère un pattern selon le type spécifié
        
        Args:
            pattern_type: Type de pattern à générer
            zone: Zone à couvrir
            config: Configuration (utilise défaut si None)
            
        Returns:
            Liste de waypoints pour le pattern
        """
        if config is None:
            config = self._default_config
            config.pattern_type = pattern_type
        
        if pattern_type == PatternType.ZIGZAG:
            return self.generate_zigzag_pattern(zone, config)
        elif pattern_type == PatternType.SPIRAL:
            return self.generate_spiral_pattern(zone, config)
        elif pattern_type == PatternType.LAWN_MOWER:
            return self.generate_lawn_mower_pattern(zone, config)
        elif pattern_type == PatternType.GRID:
            return self.generate_grid_pattern(zone, config)
        else:
            self._logger.error(f"Type de pattern non supporté: {pattern_type}")
            return []

    def _clip_line_to_polygon(self, start: Point, end: Point, 
                             polygon: List[Point]) -> List[Point]:
        """
        Découpe une ligne selon un polygone (algorithme de Sutherland-Hodgman)
        
        Args:
            start: Point de début de ligne
            end: Point de fin de ligne
            polygon: Polygone de découpe
            
        Returns:
            Points de la ligne à l'intérieur du polygone
        """
        # Implémentation simplifiée - retourne les points d'intersection
        points = []
        
        # Subdivision de la ligne en segments
        num_segments = 50
        for i in range(num_segments + 1):
            t = i / num_segments
            x = start.x + t * (end.x - start.x)
            y = start.y + t * (end.y - start.y)
            point = Point(x=x, y=y, z=start.z)
            
            if self._point_in_polygon(point, polygon):
                points.append(point)
        
        return points

    def _point_in_polygon(self, point: Point, polygon: List[Point]) -> bool:
        """
        Teste si un point est à l'intérieur d'un polygone (ray casting)
        
        Args:
            point: Point à tester
            polygon: Polygone de test
            
        Returns:
            True si le point est à l'intérieur
        """
        x, y = point.x, point.y
        n = len(polygon)
        inside = False
        
        p1x, p1y = polygon[0].x, polygon[0].y
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n].x, polygon[i % n].y
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        
        return inside

    def _generate_turn(self, last_point: Point, next_y: float, 
                      config: PatternConfig, going_right: bool) -> List[Point]:
        """
        Génère les points de virage entre deux lignes
        
        Args:
            last_point: Dernier point de la ligne précédente
            next_y: Y de la ligne suivante
            config: Configuration
            going_right: Direction de la prochaine ligne
            
        Returns:
            Points de virage
        """
        turn_points = []
        
        # Point intermédiaire de virage
        if going_right:
            # Virage vers la droite
            turn_x = last_point.x + config.turn_radius
        else:
            # Virage vers la gauche
            turn_x = last_point.x - config.turn_radius
            
        # Points de virage en arc
        mid_point = Point(x=turn_x, y=next_y, z=config.altitude)
        turn_points.append(mid_point)
        
        return turn_points

    def _generate_rounded_turn(self, start: Point, end: Point, 
                              radius: float) -> List[Point]:
        """
        Génère un virage arrondi entre deux points
        
        Args:
            start: Point de début
            end: Point de fin
            radius: Rayon du virage
            
        Returns:
            Points du virage arrondi
        """
        points = []
        
        # Calcul du centre du virage
        dx = end.x - start.x
        dy = end.y - start.y
        distance = math.sqrt(dx**2 + dy**2)
        
        if distance < 2 * radius:
            # Virage direct si distance trop courte
            return [end]
        
        # Génération d'un arc
        num_points = 8
        for i in range(1, num_points):
            t = i / num_points
            x = start.x + t * dx
            y = start.y + t * dy
            points.append(Point(x=x, y=y, z=start.z))
        
        return points

    def _optimize_grid_path(self, points: List[Point]) -> List[Point]:
        """
        Optimise l'ordre de visite des points d'une grille (TSP simplifié)
        
        Args:
            points: Points à ordonner
            
        Returns:
            Points ordonnés pour minimiser la distance
        """
        if len(points) <= 2:
            return points
        
        # Algorithme glouton simple (nearest neighbor)
        optimized = [points[0]]
        remaining = points[1:]
        
        while remaining:
            current = optimized[-1]
            nearest_idx = 0
            min_distance = float('inf')
            
            for i, point in enumerate(remaining):
                distance = math.sqrt(
                    (current.x - point.x)**2 + (current.y - point.y)**2
                )
                if distance < min_distance:
                    min_distance = distance
                    nearest_idx = i
            
            optimized.append(remaining.pop(nearest_idx))
        
        return optimized

    def _calculate_path_distance(self, waypoints: List[Point]) -> float:
        """
        Calcule la distance totale d'un chemin
        
        Args:
            waypoints: Points du chemin
            
        Returns:
            Distance totale en mètres
        """
        if len(waypoints) < 2:
            return 0.0
        
        total_distance = 0.0
        for i in range(1, len(waypoints)):
            dx = waypoints[i].x - waypoints[i-1].x
            dy = waypoints[i].y - waypoints[i-1].y
            dz = waypoints[i].z - waypoints[i-1].z
            total_distance += math.sqrt(dx**2 + dy**2 + dz**2)
        
        return total_distance

    def get_pattern_statistics(self) -> Dict[str, Any]:
        """
        Retourne les statistiques de génération de patterns
        
        Returns:
            Dictionnaire des statistiques
        """
        return self._stats.copy()

    def reset_statistics(self):
        """Remet à zéro les statistiques"""
        self._stats = {
            'patterns_generated': 0,
            'total_distance': 0.0,
            'zones_covered': 0
        }
