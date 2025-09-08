#!/usr/bin/env python3

"""
Planificateur de trajectoires avancé pour drone autonome

Implémente différents algorithmes de planification de chemins et patterns
de couverture pour missions de pollinisation optimisées.

Algorithmes inclus:
- A* pathfinding pour navigation avec obstacles
- RRT* pour espaces complexes  
- Dijkstra pour chemins optimaux
- Bézier curves pour lissage
- Dubins paths pour contraintes de virage

Exemples d'utilisation:
    planner = TrajectoryPlanner(logger, config)
    result = planner.plan_path(start_point, goal_point, PlanningAlgorithm.A_STAR)
    if result.success:
        print(f"Chemin trouvé avec coût {result.cost:.2f}m")

Auteur: Adama Komi
Version: 1.1.0
"""

import time
import math
import random
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass
from enum import Enum

import numpy as np
from scipy.spatial.distance import euclidean
from scipy.interpolate import splprep, splev
import matplotlib.pyplot as plt

from geometry_msgs.msg import Point, Pose
from nav_msgs.msg import Path


class PlanningAlgorithm(Enum):
    """Algorithmes de planification disponibles"""
    A_STAR = "a_star"
    RRT_STAR = "rrt_star"
    DIJKSTRA = "dijkstra"
    STRAIGHT_LINE = "straight_line"
    BEZIER = "bezier"
    DUBINS = "dubins"


@dataclass
class PlanningNode:
    """Nœud pour algorithmes de planification
    
    Attributes:
        x: Coordonnée x en mètres
        y: Coordonnée y en mètres
        z: Coordonnée z en mètres
        cost: Coût pour atteindre ce nœud
        parent: Nœud parent dans le chemin
    """
    x: float
    y: float
    z: float
    cost: float = 0.0
    parent: Optional['PlanningNode'] = None
    
    def distance_to(self, other: 'PlanningNode') -> float:
        """
        Calcule la distance euclidienne vers un autre nœud
        
        Args:
            other: Autre nœud
            
        Returns:
            Distance en mètres
        """
        return math.sqrt((self.x - other.x)**2 + (self.y - other.y)**2 + (self.z - other.z)**2)


@dataclass
class Obstacle:
    """Obstacle pour planification
    
    Attributes:
        center_x: Coordonnée x du centre en mètres
        center_y: Coordonnée y du centre en mètres
        center_z: Coordonnée z du centre en mètres
        radius: Rayon de l'obstacle en mètres
        height: Hauteur de l'obstacle en mètres
    """
    center_x: float
    center_y: float
    center_z: float
    radius: float
    height: float = 100.0  # Hauteur par défaut


@dataclass
class PlanningResult:
    """Résultat de planification
    
    Attributes:
        success: Indique si la planification a réussi
        path: Chemin planifié (liste de Points)
        cost: Coût total du chemin en mètres
        planning_time: Temps d'exécution en secondes
        algorithm_used: Algorithme utilisé
        nodes_explored: Nombre de nœuds explorés
        message: Message d'information ou d'erreur
    """
    success: bool
    path: List[Point]
    cost: float
    planning_time: float
    algorithm_used: PlanningAlgorithm
    nodes_explored: int = 0
    message: str = ""


class TrajectoryPlanner:
    """
    Planificateur de trajectoires avancé avec multiple algorithmes
    
    Fournit des capacités de planification optimisées pour missions
    de pollinisation incluant évitement d'obstacles et contraintes
    de vol dynamiques.
    
    Args:
        logger: Logger ROS2 pour la journalisation
        config: Configuration du planificateur avec paramètres par défaut
    """
    
    def __init__(self, logger, config: Dict[str, Any]):
        """
        Initialise le planificateur de trajectoires
        
        Args:
            logger: Logger ROS2
            config: Configuration du planificateur
        """
        self._logger = logger
        self._config = config
        
        # Paramètres de planification
        self._planning_params = config.get('planning', {})
        self._lookahead_distance = self._planning_params.get('lookahead_distance', 10.0)
        self._path_resolution = self._planning_params.get('path_resolution', 1.0)
        self._smoothing_factor = self._planning_params.get('smoothing_factor', 0.8)
        self._turn_radius_min = self._planning_params.get('turn_radius_min', 2.0)
        
        # Limites de l'espace de recherche
        self._bounds = {
            'x_min': -1000.0, 'x_max': 1000.0,
            'y_min': -1000.0, 'y_max': 1000.0,
            'z_min': 1.0, 'z_max': 150.0
        }
        
        # Obstacles détectés
        self._obstacles: List[Obstacle] = []
        
        # Cache des chemins calculés
        self._path_cache: Dict[str, PlanningResult] = {}
        
        self._logger.info("TrajectoryPlanner initialisé")

    def plan_path(self, start: Point, goal: Point, 
                  algorithm: PlanningAlgorithm = PlanningAlgorithm.A_STAR,
                  constraints: Optional[Dict] = None) -> PlanningResult:
        """
        Planifie un chemin entre deux points
        
        Args:
            start: Point de départ (Point avec x, y, z en mètres)
            goal: Point d'arrivée (Point avec x, y, z en mètres)
            algorithm: Algorithme à utiliser
            constraints: Contraintes supplémentaires (optionnel)
            
        Returns:
            Résultat de la planification
            
        Raises:
            ValueError: Si les points sont invalides
        """
        start_time = time.time()
        
        try:
            # Vérification de validité des points
            if not self._is_point_valid(start) or not self._is_point_valid(goal):
                error_msg = "Points de départ ou d'arrivée invalides"
                self._logger.error(error_msg)
                raise ValueError(error_msg)
            
            # Sélection de l'algorithme
            if algorithm == PlanningAlgorithm.A_STAR:
                result = self._plan_a_star(start, goal, constraints)
            elif algorithm == PlanningAlgorithm.RRT_STAR:
                result = self._plan_rrt_star(start, goal, constraints)
            elif algorithm == PlanningAlgorithm.DIJKSTRA:
                result = self._plan_dijkstra(start, goal, constraints)
            elif algorithm == PlanningAlgorithm.STRAIGHT_LINE:
                result = self._plan_straight_line(start, goal, constraints)
            elif algorithm == PlanningAlgorithm.BEZIER:
                result = self._plan_bezier(start, goal, constraints)
            else:
                result = self._plan_straight_line(start, goal, constraints)
            
            result.planning_time = time.time() - start_time
            result.algorithm_used = algorithm
            
            # Lissage du chemin si demandé
            if result.success and self._smoothing_factor > 0:
                result.path = self._smooth_path(result.path)
            
            return result
            
        except ValueError as e:
            self._logger.error(f"Erreur de valeur lors de la planification: {e}")
            raise
        except RuntimeError as e:
            self._logger.error(f"Erreur d'exécution lors de la planification: {e}")
            raise
        except Exception as e:
            error_msg = f"Erreur inattendue lors de la planification: {e}"
            self._logger.error(error_msg)
            return PlanningResult(
                success=False,
                path=[],
                cost=0.0,
                planning_time=time.time() - start_time,
                algorithm_used=algorithm,
                message=error_msg
            )

    def _plan_a_star(self, start: Point, goal: Point, 
                     constraints: Optional[Dict] = None) -> PlanningResult:
        """
        Planification A* avec évitement d'obstacles
        
        Args:
            start: Point de départ
            goal: Point d'arrivée
            constraints: Contraintes supplémentaires
            
        Returns:
            Résultat de la planification A*
        """
        # Conversion en nœuds de planification
        start_node = PlanningNode(start.x, start.y, start.z)
        goal_node = PlanningNode(goal.x, goal.y, goal.z)
        
        # Ensembles open et closed
        open_set = [start_node]
        closed_set = []
        
        # Coûts g (distance parcourue) et f (coût total estimé)
        g_scores = {start_node: 0.0}
        f_scores = {start_node: self._heuristic(start_node, goal_node)}
        
        nodes_explored = 0
        
        while open_set:
            nodes_explored += 1
            
            # Sélection du nœud avec le plus petit f-score
            current = min(open_set, key=lambda n: f_scores.get(n, float('inf')))
            
            # Vérification si on a atteint le but
            if current.distance_to(goal_node) < self._path_resolution:
                path = self._reconstruct_path(current)
                cost = g_scores[current]
                
                return PlanningResult(
                    success=True,
                    path=path,
                    cost=cost,
                    planning_time=0.0,
                    algorithm_used=PlanningAlgorithm.A_STAR,
                    nodes_explored=nodes_explored,
                    message="Chemin A* trouvé avec succès"
                )
            
            # Déplacement du nœud courant vers closed_set
            open_set.remove(current)
            closed_set.append(current)
            
            # Exploration des voisins
            neighbors = self._get_neighbors(current)
            
            for neighbor in neighbors:
                if neighbor in closed_set:
                    continue
                
                # Vérification des collisions
                if self._check_collision(current, neighbor):
                    continue
                
                # Calcul du coût tentative
                tentative_g = g_scores[current] + current.distance_to(neighbor)
                
                if neighbor not in open_set:
                    open_set.append(neighbor)
                elif tentative_g >= g_scores.get(neighbor, float('inf')):
                    continue
                
                # Mise à jour du chemin optimal vers ce voisin
                neighbor.parent = current
                g_scores[neighbor] = tentative_g
                f_scores[neighbor] = tentative_g + self._heuristic(neighbor, goal_node)
        
        # Aucun chemin trouvé
        return PlanningResult(
            success=False,
            path=[],
            cost=0.0,
            planning_time=0.0,
            algorithm_used=PlanningAlgorithm.A_STAR,
            nodes_explored=nodes_explored,
            message="Aucun chemin A* trouvé"
        )

    def _plan_rrt_star(self, start: Point, goal: Point,
                       constraints: Optional[Dict] = None) -> PlanningResult:
        """
        Planification RRT* (Rapidly-exploring Random Tree)
        
        Args:
            start: Point de départ
            goal: Point d'arrivée  
            constraints: Contraintes supplémentaires
            
        Returns:
            Résultat de la planification RRT*
        """
        # Paramètres RRT*
        max_iterations = constraints.get('max_iterations', 1000) if constraints else 1000
        step_size = constraints.get('step_size', 5.0) if constraints else 5.0
        goal_bias = constraints.get('goal_bias', 0.1) if constraints else 0.1
        
        # Initialisation
        start_node = PlanningNode(start.x, start.y, start.z)
        goal_node = PlanningNode(goal.x, goal.y, goal.z)
        
        tree = [start_node]
        best_goal_node = None
        nodes_explored = 0
        
        for iteration in range(max_iterations):
            nodes_explored += 1
            
            # Échantillonnage d'un point aléatoire (avec biais vers le but)
            if random.random() < goal_bias:
                sample = goal_node
            else:
                sample = self._sample_random_point()
            
            # Recherche du nœud le plus proche dans l'arbre
            nearest = min(tree, key=lambda n: n.distance_to(sample))
            
            # Extension vers le sample
            new_node = self._steer(nearest, sample, step_size)
            
            # Vérification des collisions
            if self._check_collision(nearest, new_node):
                continue
            
            # Recherche des voisins pour connexion optimale (RRT*)
            near_nodes = self._get_near_nodes(tree, new_node, step_size)
            
            # Sélection du parent optimal
            min_cost = nearest.cost + nearest.distance_to(new_node)
            min_node = nearest
            
            for near_node in near_nodes:
                cost = near_node.cost + near_node.distance_to(new_node)
                if cost < min_cost and not self._check_collision(near_node, new_node):
                    min_cost = cost
                    min_node = near_node
            
            # Ajout du nouveau nœud
            new_node.parent = min_node
            new_node.cost = min_cost
            tree.append(new_node)
            
            # Rewiring des nœuds voisins
            for near_node in near_nodes:
                cost = new_node.cost + new_node.distance_to(near_node)
                if cost < near_node.cost and not self._check_collision(new_node, near_node):
                    near_node.parent = new_node
                    near_node.cost = cost
            
            # Vérification si on peut atteindre le but
            if new_node.distance_to(goal_node) < step_size:
                if not self._check_collision(new_node, goal_node):
                    goal_node.parent = new_node
                    goal_node.cost = new_node.cost + new_node.distance_to(goal_node)
                    
                    if best_goal_node is None or goal_node.cost < best_goal_node.cost:
                        best_goal_node = goal_node
        
        # Construction du résultat
        if best_goal_node:
            path = self._reconstruct_path(best_goal_node)
            return PlanningResult(
                success=True,
                path=path,
                cost=best_goal_node.cost,
                planning_time=0.0,
                algorithm_used=PlanningAlgorithm.RRT_STAR,
                nodes_explored=nodes_explored,
                message="Chemin RRT* trouvé avec succès"
            )
        else:
            return PlanningResult(
                success=False,
                path=[],
                cost=0.0,
                planning_time=0.0,
                algorithm_used=PlanningAlgorithm.RRT_STAR,
                nodes_explored=nodes_explored,
                message="Aucun chemin RRT* trouvé"
            )

    def _plan_dijkstra(self, start: Point, goal: Point,
                       constraints: Optional[Dict] = None) -> PlanningResult:
        """
        Planification Dijkstra pour chemin optimal
        
        Args:
            start: Point de départ
            goal: Point d'arrivée
            constraints: Contraintes supplémentaires
            
        Returns:
            Résultat de la planification Dijkstra
        """
        # Création d'une grille de nœuds
        grid_resolution = constraints.get('grid_resolution', 2.0) if constraints else 2.0
        nodes = self._create_grid(start, goal, grid_resolution)
        
        # Initialisation
        start_node = min(nodes, key=lambda n: n.distance_to(PlanningNode(start.x, start.y, start.z)))
        goal_node = min(nodes, key=lambda n: n.distance_to(PlanningNode(goal.x, goal.y, goal.z)))
        
        distances = {node: float('inf') for node in nodes}
        distances[start_node] = 0.0
        previous = {}
        unvisited = set(nodes)
        
        nodes_explored = 0
        
        while unvisited:
            nodes_explored += 1
            
            # Sélection du nœud non visité avec la plus petite distance
            current = min(unvisited, key=lambda n: distances[n])
            
            if current == goal_node:
                break
            
            unvisited.remove(current)
            
            # Mise à jour des distances des voisins
            for neighbor in self._get_grid_neighbors(current, nodes, grid_resolution):
                if neighbor not in unvisited:
                    continue
                
                if self._check_collision(current, neighbor):
                    continue
                
                distance = distances[current] + current.distance_to(neighbor)
                
                if distance < distances[neighbor]:
                    distances[neighbor] = distance
                    previous[neighbor] = current
        
        # Reconstruction du chemin
        if goal_node in previous or start_node == goal_node:
            path = []
            current = goal_node
            
            while current in previous:
                point = Point()
                point.x, point.y, point.z = current.x, current.y, current.z
                path.append(point)
                current = previous[current]
            
            # Ajout du point de départ
            point = Point()
            point.x, point.y, point.z = start_node.x, start_node.y, start_node.z
            path.append(point)
            
            path.reverse()
            
            return PlanningResult(
                success=True,
                path=path,
                cost=distances[goal_node],
                planning_time=0.0,
                algorithm_used=PlanningAlgorithm.DIJKSTRA,
                nodes_explored=nodes_explored,
                message="Chemin Dijkstra trouvé avec succès"
            )
        else:
            return PlanningResult(
                success=False,
                path=[],
                cost=0.0,
                planning_time=0.0,
                algorithm_used=PlanningAlgorithm.DIJKSTRA,
                nodes_explored=nodes_explored,
                message="Aucun chemin Dijkstra trouvé"
            )

    def _plan_straight_line(self, start: Point, goal: Point,
                            constraints: Optional[Dict] = None) -> PlanningResult:
        """
        Planification en ligne droite (fallback simple)
        
        Args:
            start: Point de départ
            goal: Point d'arrivée
            constraints: Contraintes supplémentaires
            
        Returns:
            Résultat de la planification en ligne droite
        """
        # Vérification des collisions sur la ligne droite
        if self._check_line_collision(start, goal):
            return PlanningResult(
                success=False,
                path=[],
                cost=0.0,
                planning_time=0.0,
                algorithm_used=PlanningAlgorithm.STRAIGHT_LINE,
                message="Collision détectée sur la ligne droite"
            )
        
        # Calcul de la distance
        distance = math.sqrt((goal.x - start.x)**2 + (goal.y - start.y)**2 + (goal.z - start.z)**2)
        
        # Génération de points intermédiaires
        num_points = max(2, int(distance / self._path_resolution))
        path = []
        
        for i in range(num_points):
            t = i / (num_points - 1)
            point = Point()
            point.x = start.x + t * (goal.x - start.x)
            point.y = start.y + t * (goal.y - start.y)
            point.z = start.z + t * (goal.z - start.z)
            path.append(point)
        
        return PlanningResult(
            success=True,
            path=path,
            cost=distance,
            planning_time=0.0,
            algorithm_used=PlanningAlgorithm.STRAIGHT_LINE,
            nodes_explored=2,
            message="Chemin en ligne droite généré"
        )

    def _plan_bezier(self, start: Point, goal: Point,
                     constraints: Optional[Dict] = None) -> PlanningResult:
        """
        Planification avec courbes de Bézier pour lissage
        
        Args:
            start: Point de départ
            goal: Point d'arrivée
            constraints: Contraintes avec points de contrôle
            
        Returns:
            Résultat de la planification Bézier
            
        Note:
            Cette implémentation utilise des courbes de Bézier cubiques
            avec points de contrôle optionnels
        """
        try:
            # Points de contrôle (peuvent être fournis dans les contraintes)
            if constraints and 'control_points' in constraints:
                control_points = constraints['control_points']
            else:
                # Génération automatique de points de contrôle
                mid_x = (start.x + goal.x) / 2
                mid_y = (start.y + goal.y) / 2
                mid_z = max(start.z, goal.z) + 10.0  # Élévation pour éviter obstacles
                
                control_points = [
                    [start.x, start.y, start.z],
                    [mid_x, mid_y + 10.0, mid_z],  # Décalage latéral
                    [mid_x, mid_y - 10.0, mid_z],
                    [goal.x, goal.y, goal.z]
                ]
            
            # Génération de la courbe de Bézier
            t_values = np.linspace(0, 1, 50)
            path = []
            
            for t in t_values:
                point = self._bezier_point(control_points, t)
                p = Point()
                p.x, p.y, p.z = point[0], point[1], point[2]
                path.append(p)
            
            # Calcul du coût (longueur de la courbe)
            cost = 0.0
            for i in range(len(path) - 1):
                dx = path[i+1].x - path[i].x
                dy = path[i+1].y - path[i].y
                dz = path[i+1].z - path[i].z
                cost += math.sqrt(dx*dx + dy*dy + dz*dz)
            
            return PlanningResult(
                success=True,
                path=path,
                cost=cost,
                planning_time=0.0,
                algorithm_used=PlanningAlgorithm.BEZIER,
                nodes_explored=len(t_values),
                message="Courbe de Bézier générée avec succès"
            )
            
        except Exception as e:
            error_msg = f"Erreur lors de la génération de la courbe de Bézier: {e}"
            self._logger.error(error_msg)
            return PlanningResult(
                success=False,
                path=[],
                cost=0.0,
                planning_time=0.0,
                algorithm_used=PlanningAlgorithm.BEZIER,
                message=error_msg
            )

    def _bezier_point(self, control_points: List[List[float]], t: float) -> List[float]:
        """
        Calcule un point sur une courbe de Bézier
        
        Args:
            control_points: Points de contrôle de la courbe
            t: Paramètre de la courbe (0 à 1)
            
        Returns:
            Point calculé [x, y, z]
        """
        n = len(control_points) - 1
        point = [0.0, 0.0, 0.0]
        
        for i, cp in enumerate(control_points):
            # Coefficient binomial
            binom_coeff = math.comb(n, i)
            # Polynôme de Bernstein
            bernstein = binom_coeff * (t ** i) * ((1 - t) ** (n - i))
            
            for j in range(3):  # x, y, z
                point[j] += bernstein * cp[j]
        
        return point

    # Méthodes utilitaires
    def _is_point_valid(self, point: Point) -> bool:
        """
        Vérifie si un point est dans les limites valides
        
        Args:
            point: Point à vérifier
            
        Returns:
            True si le point est valide, False sinon
        """
        return (self._bounds['x_min'] <= point.x <= self._bounds['x_max'] and
                self._bounds['y_min'] <= point.y <= self._bounds['y_max'] and
                self._bounds['z_min'] <= point.z <= self._bounds['z_max'])

    def _heuristic(self, node1: PlanningNode, node2: PlanningNode) -> float:
        """
        Calcule l'heuristique (distance euclidienne) entre deux nœuds
        
        Args:
            node1: Premier nœud
            node2: Deuxième nœud
            
        Returns:
            Distance euclidienne en mètres
        """
        return node1.distance_to(node2)

    def _get_neighbors(self, node: PlanningNode) -> List[PlanningNode]:
        """
        Génère les voisins d'un nœud pour A*
        
        Args:
            node: Nœud de référence
            
        Returns:
            Liste des nœuds voisins
        """
        neighbors = []
        step = self._path_resolution
        
        # 26 directions en 3D (cube)
        directions = [
            (dx, dy, dz) 
            for dx in [-step, 0, step]
            for dy in [-step, 0, step] 
            for dz in [-step, 0, step]
            if not (dx == 0 and dy == 0 and dz == 0)
        ]
        
        for dx, dy, dz in directions:
            neighbor = PlanningNode(node.x + dx, node.y + dy, node.z + dz)
            
            # Vérification des limites
            if self._is_point_valid(Point(x=neighbor.x, y=neighbor.y, z=neighbor.z)):
                neighbors.append(neighbor)
        
        return neighbors

    def _check_collision(self, node1: PlanningNode, node2: PlanningNode) -> bool:
        """
        Vérifie s'il y a collision entre deux nœuds
        
        Args:
            node1: Premier nœud
            node2: Deuxième nœud
            
        Returns:
            True si collision détectée, False sinon
        """
        # Vérification avec tous les obstacles
        for obstacle in self._obstacles:
            if self._line_intersects_obstacle(node1, node2, obstacle):
                return True
        return False

    def _check_line_collision(self, start: Point, goal: Point) -> bool:
        """
        Vérifie les collisions sur une ligne droite
        
        Args:
            start: Point de départ
            goal: Point d'arrivée
            
        Returns:
            True si collision détectée, False sinon
        """
        start_node = PlanningNode(start.x, start.y, start.z)
        goal_node = PlanningNode(goal.x, goal.y, goal.z)
        return self._check_collision(start_node, goal_node)

    def _line_intersects_obstacle(self, node1: PlanningNode, node2: PlanningNode, 
                                  obstacle: Obstacle) -> bool:
        """
        Vérifie si une ligne intersecte un obstacle cylindrique
        
        Args:
            node1: Premier nœud de la ligne
            node2: Deuxième nœud de la ligne
            obstacle: Obstacle à vérifier
            
        Returns:
            True si intersection détectée, False sinon
        """
        # Distance du point obstacle à la ligne
        # Utilisation de la formule de distance point-ligne en 3D
        
        # Vecteur de la ligne
        line_vec = np.array([node2.x - node1.x, node2.y - node1.y, node2.z - node1.z])
        line_length = np.linalg.norm(line_vec)
        
        if line_length == 0:
            return False
        
        line_unit = line_vec / line_length
        
        # Vecteur du point 1 vers l'obstacle
        to_obstacle = np.array([obstacle.center_x - node1.x, 
                               obstacle.center_y - node1.y,
                               obstacle.center_z - node1.z])
        
        # Projection sur la ligne
        projection_length = np.dot(to_obstacle, line_unit)
        projection_length = max(0, min(line_length, projection_length))
        
        # Point le plus proche sur la ligne
        closest_point = np.array([node1.x, node1.y, node1.z]) + projection_length * line_unit
        
        # Distance à l'obstacle
        distance = np.linalg.norm([closest_point[0] - obstacle.center_x,
                                  closest_point[1] - obstacle.center_y])
        
        # Vérification de collision (cylindre)
        if distance <= obstacle.radius:
            # Vérification de la hauteur
            if (obstacle.center_z <= closest_point[2] <= 
                obstacle.center_z + obstacle.height):
                return True
        
        return False

    def _reconstruct_path(self, node: PlanningNode) -> List[Point]:
        """
        Reconstruit le chemin depuis un nœud final
        
        Args:
            node: Nœud final du chemin
            
        Returns:
            Chemin reconstruit (liste de Points)
        """
        path = []
        current = node
        
        while current is not None:
            point = Point()
            point.x = current.x
            point.y = current.y
            point.z = current.z
            path.append(point)
            current = current.parent
        
        path.reverse()
        return path

    def _sample_random_point(self) -> PlanningNode:
        """
        Échantillonne un point aléatoire dans l'espace
        
        Returns:
            Nœud aléatoire dans les limites définies
        """
        x = random.uniform(self._bounds['x_min'], self._bounds['x_max'])
        y = random.uniform(self._bounds['y_min'], self._bounds['y_max'])
        z = random.uniform(self._bounds['z_min'], self._bounds['z_max'])
        return PlanningNode(x, y, z)

    def _steer(self, from_node: PlanningNode, to_node: PlanningNode, 
               step_size: float) -> PlanningNode:
        """
        Dirige depuis un nœud vers un autre avec une taille de pas limitée
        
        Args:
            from_node: Nœud de départ
            to_node: Nœud cible
            step_size: Taille de pas maximale
            
        Returns:
            Nouveau nœud dans la direction cible
        """
        distance = from_node.distance_to(to_node)
        
        if distance <= step_size:
            return to_node
        
        # Limitation de la distance
        ratio = step_size / distance
        new_x = from_node.x + ratio * (to_node.x - from_node.x)
        new_y = from_node.y + ratio * (to_node.y - from_node.y)
        new_z = from_node.z + ratio * (to_node.z - from_node.z)
        
        return PlanningNode(new_x, new_y, new_z)

    def _get_near_nodes(self, tree: List[PlanningNode], node: PlanningNode, 
                        radius: float) -> List[PlanningNode]:
        """
        Trouve les nœuds dans un rayon donné
        
        Args:
            tree: Arbre de nœuds
            node: Nœud de référence
            radius: Rayon de recherche
            
        Returns:
            Liste des nœuds dans le rayon
        """
        near_nodes = []
        for tree_node in tree:
            if tree_node.distance_to(node) <= radius:
                near_nodes.append(tree_node)
        return near_nodes

    def _create_grid(self, start: Point, goal: Point, resolution: float) -> List[PlanningNode]:
        """
        Crée une grille de nœuds pour Dijkstra
        
        Args:
            start: Point de départ
            goal: Point d'arrivée
            resolution: Résolution de la grille
            
        Returns:
            Liste des nœuds de la grille
        """
        # Calcul des limites de la grille
        min_x = min(start.x, goal.x) - 50
        max_x = max(start.x, goal.x) + 50
        min_y = min(start.y, goal.y) - 50
        max_y = max(start.y, goal.y) + 50
        min_z = min(start.z, goal.z)
        max_z = max(start.z, goal.z) + 20
        
        nodes = []
        
        x = min_x
        while x <= max_x:
            y = min_y
            while y <= max_y:
                z = min_z
                while z <= max_z:
                    node = PlanningNode(x, y, z)
                    if self._is_point_valid(Point(x=x, y=y, z=z)):
                        nodes.append(node)
                    z += resolution
                y += resolution
            x += resolution
        
        return nodes

    def _get_grid_neighbors(self, node: PlanningNode, all_nodes: List[PlanningNode], 
                            resolution: float) -> List[PlanningNode]:
        """
        Trouve les voisins d'un nœud dans une grille
        
        Args:
            node: Nœud de référence
            all_nodes: Tous les nœuds de la grille
            resolution: Résolution de la grille
            
        Returns:
            Liste des nœuds voisins
        """
        neighbors = []
        
        for other_node in all_nodes:
            distance = node.distance_to(other_node)
            # Voisins directs (distance = résolution) ou diagonaux
            if resolution <= distance <= resolution * 1.8:  # Diagonale 3D ≈ 1.73 * resolution
                neighbors.append(other_node)
        
        return neighbors

    def _smooth_path(self, path: List[Point]) -> List[Point]:
        """
        Lisse un chemin en utilisant l'interpolation spline
        
        Args:
            path: Chemin à lisser
            
        Returns:
            Chemin lissé
        """
        if len(path) < 3:
            return path
        
        try:
            # Extraction des coordonnées
            x_coords = [p.x for p in path]
            y_coords = [p.y for p in path]
            z_coords = [p.z for p in path]
            
            # Paramètres pour l'interpolation
            points = np.array([x_coords, y_coords, z_coords])
            
            # Interpolation spline
            tck, u = splprep(points, s=self._smoothing_factor, k=min(3, len(path)-1))
            
            # Génération du chemin lissé
            u_new = np.linspace(0, 1, len(path) * 2)  # Plus de points pour un chemin plus lisse
            smooth_points = splev(u_new, tck)
            
            # Conversion en Points
            smooth_path = []
            for i in range(len(smooth_points[0])):
                point = Point()
                point.x = float(smooth_points[0][i])
                point.y = float(smooth_points[1][i])
                point.z = float(smooth_points[2][i])
                smooth_path.append(point)
            
            return smooth_path
            
        except Exception as e:
            self._logger.warn(f"Erreur lors du lissage: {e}, retour au chemin original")
            return path

    def update_obstacles(self, obstacles: List[Obstacle]):
        """
        Met à jour la liste des obstacles
        
        Args:
            obstacles: Nouvelle liste d'obstacles
        """
        self._obstacles = obstacles
        # Vidage du cache car les obstacles ont changé
        self._path_cache.clear()

    def set_bounds(self, bounds: Dict[str, float]):
        """
        Définit les limites de l'espace de planification
        
        Args:
            bounds: Dictionnaire des limites (x_min, x_max, y_min, y_max, z_min, z_max)
        """
        self._bounds.update(bounds)

    def get_planning_stats(self) -> Dict[str, Any]:
        """
        Retourne les statistiques de planification
        
        Returns:
            Dictionnaire contenant les statistiques
        """
        return {
            'cache_size': len(self._path_cache),
            'obstacles_count': len(self._obstacles),
            'bounds': self._bounds.copy(),
            'config': self._config.copy()
        }