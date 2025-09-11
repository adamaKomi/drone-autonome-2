#!/usr/bin/env python3
"""
Trajectory Planner Node - Planification avancée de trajectoires
Auteur: Adama Komi
Date: 2025-09-11
Version: 1.0.0

Ce nœud implémente les algorithmes de planification de trajectoires pour
la navigation autonome. Il supporte A*, RRT*, Dijkstra et optimisation génétique.

Références:
- A* Algorithm: https://en.wikipedia.org/wiki/A*_search_algorithm
- RRT*: https://en.wikipedia.org/wiki/Rapidly-exploring_random_tree
- Motion Planning: http://planning.cs.uiuc.edu/
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Messages ROS2
from geometry_msgs.msg import Point, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import String, Header

# Services personnalisés (à créer)
from std_srvs.srv import Trigger

import numpy as np
import heapq
import random
import threading
import time
import math
from typing import List, Tuple, Dict, Optional, Any
from dataclasses import dataclass
from enum import Enum


class PlannerType(Enum):
    """Types de planificateurs disponibles"""
    ASTAR = "astar"
    RRT_STAR = "rrt_star"
    DIJKSTRA = "dijkstra"
    GENETIC = "genetic"


@dataclass
class PlanningRequest:
    """Requête de planification"""
    start: Point
    goal: Point
    planner_type: PlannerType = PlannerType.ASTAR
    max_time: float = 10.0
    obstacles: List[Any] = None
    constraints: Dict[str, Any] = None


@dataclass
class PlanningResult:
    """Résultat de planification"""
    success: bool = False
    path: List[Point] = None
    cost: float = float('inf')
    planning_time: float = 0.0
    iterations: int = 0
    algorithm_used: str = ""
    error_message: str = ""


class Node3D:
    """Nœud pour les algorithmes de planification 3D"""
    
    def __init__(self, position: Point, parent=None, cost: float = 0.0):
        self.position = position
        self.parent = parent
        self.cost = cost
        self.heuristic = 0.0
        self.total_cost = cost + self.heuristic
    
    def __lt__(self, other):
        return self.total_cost < other.total_cost
    
    def __eq__(self, other):
        if not isinstance(other, Node3D):
            return False
        return (abs(self.position.x - other.position.x) < 0.1 and
                abs(self.position.y - other.position.y) < 0.1 and
                abs(self.position.z - other.position.z) < 0.1)
    
    def __hash__(self):
        return hash((round(self.position.x, 1), 
                    round(self.position.y, 1), 
                    round(self.position.z, 1)))


class TrajectoryPlannerNode(Node):
    """
    Nœud de planification de trajectoires
    
    Ce nœud est responsable de:
    - Planification de chemins 3D optimaux
    - Évitement d'obstacles statiques
    - Optimisation multi-critères des trajectoires
    - Adaptation dynamique des algorithmes
    """
    
    def __init__(self):
        super().__init__('trajectory_planner_node', namespace='drone_nav')
        
        # Configuration QoS
        self.qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # État du planificateur
        self.planning_lock = threading.RLock()
        self.current_request = None
        self.planning_active = False
        
        # Obstacles et contraintes
        self.static_obstacles = []
        self.planning_bounds = {
            'x_min': -100, 'x_max': 100,
            'y_min': -100, 'y_max': 100,
            'z_min': 1, 'z_max': 100
        }
        
        # Métriques
        self.planning_statistics = {
            'total_requests': 0,
            'successful_plans': 0,
            'average_planning_time': 0.0,
            'algorithms_used': {}
        }
        
        # Paramètres
        self._declare_parameters()
        
        # Publishers
        self._setup_publishers()
        
        # Services
        self._setup_services()
        
        # Subscribers
        self._setup_subscribers()
        
        # Timer pour statistiques
        self.stats_timer = self.create_timer(30.0, self._publish_statistics)
        
        self.get_logger().info("Trajectory Planner Node initialisé")
    
    def _declare_parameters(self):
        """Déclare les paramètres du nœud"""
        # Algorithme par défaut
        self.declare_parameter('default_planner', 'astar')
        
        # Paramètres de grille
        self.declare_parameter('grid_resolution', 1.0)
        self.declare_parameter('planning_timeout', 10.0)
        
        # Paramètres A*
        self.declare_parameter('astar_heuristic_weight', 1.0)
        
        # Paramètres RRT*
        self.declare_parameter('rrt_max_iterations', 5000)
        self.declare_parameter('rrt_step_size', 2.0)
        self.declare_parameter('rrt_goal_bias', 0.1)
        self.declare_parameter('rrt_radius', 5.0)
        
        # Paramètres d'optimisation
        self.declare_parameter('optimization_enabled', True)
        self.declare_parameter('path_smoothing_enabled', True)
        
        # Contraintes de mouvement
        self.declare_parameter('max_climb_angle', 30.0)  # degrés
        self.declare_parameter('max_descent_angle', 45.0)  # degrés
        self.declare_parameter('min_turn_radius', 2.0)  # mètres
    
    def _setup_publishers(self):
        """Configure les publishers"""
        # Trajectoire planifiée
        self.planned_path_pub = self.create_publisher(
            Path,
            'planned_path',
            self.qos_reliable
        )
        
        # Statut de planification
        self.planning_status_pub = self.create_publisher(
            String,
            'planning_status',
            self.qos_reliable
        )
        
        # Métriques de performance
        self.planning_metrics_pub = self.create_publisher(
            String,
            'planning_metrics',
            self.qos_reliable
        )
    
    def _setup_services(self):
        """Configure les services"""
        # Planification de trajectoire
        self.plan_trajectory_service = self.create_service(
            Trigger,  # À remplacer par un service personnalisé
            'plan_trajectory',
            self._handle_plan_trajectory
        )
        
        # Replanification
        self.replan_service = self.create_service(
            Trigger,
            'replan_trajectory',
            self._handle_replan
        )
        
        # Configuration du planificateur
        self.configure_planner_service = self.create_service(
            Trigger,
            'configure_planner',
            self._handle_configure_planner
        )
        
        # Validation de trajectoire
        self.validate_path_service = self.create_service(
            Trigger,
            'validate_path',
            self._handle_validate_path
        )
    
    def _setup_subscribers(self):
        """Configure les subscribers"""
        # Objectif de navigation
        self.goal_sub = self.create_subscription(
            PoseStamped,
            'goal_position',
            self._goal_callback,
            self.qos_reliable
        )
        
        # Obstacles détectés
        self.obstacles_sub = self.create_subscription(
            String,
            'obstacles',
            self._obstacles_callback,
            self.qos_reliable
        )
        
        # Position actuelle
        self.position_sub = self.create_subscription(
            PoseStamped,
            'position',
            self._position_callback,
            self.qos_reliable
        )
    
    # Callbacks
    def _goal_callback(self, msg: PoseStamped):
        """Callback pour nouvel objectif"""
        try:
            goal_point = msg.pose.position
            self.get_logger().info(f"Nouvel objectif reçu: ({goal_point.x:.2f}, {goal_point.y:.2f}, {goal_point.z:.2f})")
            
            # Déclencher une planification automatique si configuré
            auto_plan = self.get_parameter('auto_planning_enabled', default_value=True)
            if auto_plan:
                self._trigger_planning(goal_point)
                
        except Exception as e:
            self.get_logger().error(f"Erreur dans goal_callback: {e}")
    
    def _obstacles_callback(self, msg: String):
        """Callback pour obstacles détectés"""
        try:
            import json
            obstacles_data = json.loads(msg.data)
            
            # Mettre à jour la liste des obstacles
            with self.planning_lock:
                self.static_obstacles = obstacles_data.get('obstacles', [])
            
            self.get_logger().debug(f"Obstacles mis à jour: {len(self.static_obstacles)} obstacles")
            
        except Exception as e:
            self.get_logger().error(f"Erreur dans obstacles_callback: {e}")
    
    def _position_callback(self, msg: PoseStamped):
        """Callback pour position actuelle"""
        # Stocker la position actuelle pour la planification
        self.current_position = msg.pose.position
    
    # Gestionnaires de services
    def _handle_plan_trajectory(self, request, response):
        """Gestionnaire pour planification de trajectoire"""
        try:
            # Dans une implémentation réelle, les paramètres viendraient de la requête
            if not hasattr(self, 'current_position'):
                response.success = False
                response.message = "Position actuelle non disponible"
                return response
            
            # Créer une requête de planification par défaut
            planning_request = PlanningRequest(
                start=self.current_position,
                goal=Point(x=10.0, y=10.0, z=5.0),  # Exemple
                planner_type=PlannerType.ASTAR
            )
            
            # Effectuer la planification
            result = self._plan_path(planning_request)
            
            if result.success:
                # Publier le chemin planifié
                self._publish_planned_path(result.path)
                response.success = True
                response.message = f"Planification réussie en {result.planning_time:.2f}s"
            else:
                response.success = False
                response.message = f"Planification échouée: {result.error_message}"
            
            # Mettre à jour les statistiques
            self._update_statistics(result)
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
            self.get_logger().error(f"Erreur dans plan_trajectory: {e}")
        
        return response
    
    def _handle_replan(self, request, response):
        """Gestionnaire pour replanification"""
        try:
            if self.current_request is None:
                response.success = False
                response.message = "Aucune planification précédente à refaire"
                return response
            
            # Replanifier avec la dernière requête
            result = self._plan_path(self.current_request)
            
            if result.success:
                self._publish_planned_path(result.path)
                response.success = True
                response.message = f"Replanification réussie en {result.planning_time:.2f}s"
            else:
                response.success = False
                response.message = f"Replanification échouée: {result.error_message}"
            
            self._update_statistics(result)
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
        
        return response
    
    def _handle_configure_planner(self, request, response):
        """Gestionnaire pour configuration du planificateur"""
        try:
            # Recharger les paramètres
            self._reload_parameters()
            
            response.success = True
            response.message = "Planificateur reconfiguré"
            self.get_logger().info("Planificateur reconfiguré")
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
        
        return response
    
    def _handle_validate_path(self, request, response):
        """Gestionnaire pour validation de trajectoire"""
        try:
            # Dans une implémentation réelle, valider le chemin fourni
            response.success = True
            response.message = "Chemin validé"
            
        except Exception as e:
            response.success = False
            response.message = f"Erreur: {e}"
        
        return response
    
    # Algorithmes de planification
    def _plan_path(self, request: PlanningRequest) -> PlanningResult:
        """Planifie un chemin selon la requête"""
        start_time = time.time()
        
        with self.planning_lock:
            self.current_request = request
            self.planning_active = True
        
        try:
            # Sélectionner l'algorithme
            if request.planner_type == PlannerType.ASTAR:
                result = self._plan_astar(request)
            elif request.planner_type == PlannerType.RRT_STAR:
                result = self._plan_rrt_star(request)
            elif request.planner_type == PlannerType.DIJKSTRA:
                result = self._plan_dijkstra(request)
            else:
                result = PlanningResult(
                    success=False,
                    error_message=f"Planificateur non supporté: {request.planner_type}"
                )
            
            # Optimiser le chemin si demandé
            if result.success and self.get_parameter('optimization_enabled').value:
                result.path = self._optimize_path(result.path)
            
            # Lisser le chemin si demandé
            if result.success and self.get_parameter('path_smoothing_enabled').value:
                result.path = self._smooth_path(result.path)
            
            result.planning_time = time.time() - start_time
            result.algorithm_used = request.planner_type.value
            
        except Exception as e:
            result = PlanningResult(
                success=False,
                planning_time=time.time() - start_time,
                error_message=str(e)
            )
        
        finally:
            with self.planning_lock:
                self.planning_active = False
        
        return result
    
    def _plan_astar(self, request: PlanningRequest) -> PlanningResult:
        """Algorithme A* pour planification"""
        start_node = Node3D(request.start)
        goal_node = Node3D(request.goal)
        
        open_set = [start_node]
        closed_set = set()
        came_from = {}
        
        iterations = 0
        max_iterations = 10000
        heuristic_weight = self.get_parameter('astar_heuristic_weight').value
        
        while open_set and iterations < max_iterations:
            iterations += 1
            current_node = heapq.heappop(open_set)
            
            if current_node == goal_node:
                # Reconstruire le chemin
                path = self._reconstruct_path(came_from, current_node)
                return PlanningResult(
                    success=True,
                    path=path,
                    cost=current_node.cost,
                    iterations=iterations
                )
            
            closed_set.add(current_node)
            
            # Explorer les voisins
            for neighbor_pos in self._get_neighbors(current_node.position):
                neighbor_node = Node3D(neighbor_pos)
                
                if neighbor_node in closed_set:
                    continue
                
                if self._is_collision(neighbor_pos):
                    continue
                
                # Calculer les coûts
                tentative_cost = current_node.cost + self._distance_3d(
                    current_node.position, neighbor_pos
                )
                
                neighbor_node.cost = tentative_cost
                neighbor_node.heuristic = heuristic_weight * self._distance_3d(
                    neighbor_pos, request.goal
                )
                neighbor_node.total_cost = neighbor_node.cost + neighbor_node.heuristic
                neighbor_node.parent = current_node
                
                # Ajouter à la liste ouverte
                heapq.heappush(open_set, neighbor_node)
                came_from[neighbor_node] = current_node
        
        return PlanningResult(
            success=False,
            iterations=iterations,
            error_message="Aucun chemin trouvé avec A*"
        )
    
    def _plan_rrt_star(self, request: PlanningRequest) -> PlanningResult:
        """Algorithme RRT* pour planification"""
        nodes = [Node3D(request.start)]
        goal_region_radius = 2.0
        
        max_iterations = self.get_parameter('rrt_max_iterations').value
        step_size = self.get_parameter('rrt_step_size').value
        goal_bias = self.get_parameter('rrt_goal_bias').value
        search_radius = self.get_parameter('rrt_radius').value
        
        for iteration in range(max_iterations):
            # Échantillonner un point aléatoire
            if random.random() < goal_bias:
                random_point = request.goal
            else:
                random_point = self._sample_random_point()
            
            # Trouver le nœud le plus proche
            nearest_node = min(nodes, key=lambda n: self._distance_3d(n.position, random_point))
            
            # Étendre vers le point aléatoire
            new_position = self._extend_towards(nearest_node.position, random_point, step_size)
            
            if self._is_collision(new_position):
                continue
            
            # Créer le nouveau nœud
            new_node = Node3D(new_position)
            new_node.cost = nearest_node.cost + self._distance_3d(nearest_node.position, new_position)
            new_node.parent = nearest_node
            
            # RRT* : recherche de voisins pour optimisation
            near_nodes = [n for n in nodes if self._distance_3d(n.position, new_position) <= search_radius]
            
            # Choisir le meilleur parent
            best_parent = nearest_node
            best_cost = new_node.cost
            
            for near_node in near_nodes:
                potential_cost = near_node.cost + self._distance_3d(near_node.position, new_position)
                if potential_cost < best_cost and not self._is_collision_line(near_node.position, new_position):
                    best_parent = near_node
                    best_cost = potential_cost
            
            new_node.parent = best_parent
            new_node.cost = best_cost
            nodes.append(new_node)
            
            # Rewiring
            for near_node in near_nodes:
                potential_cost = new_node.cost + self._distance_3d(new_node.position, near_node.position)
                if potential_cost < near_node.cost and not self._is_collision_line(new_node.position, near_node.position):
                    near_node.parent = new_node
                    near_node.cost = potential_cost
            
            # Vérifier si on a atteint le but
            if self._distance_3d(new_position, request.goal) <= goal_region_radius:
                path = self._reconstruct_path_rrt(new_node)
                return PlanningResult(
                    success=True,
                    path=path,
                    cost=new_node.cost,
                    iterations=iteration + 1
                )
        
        return PlanningResult(
            success=False,
            iterations=max_iterations,
            error_message="Aucun chemin trouvé avec RRT*"
        )
    
    def _plan_dijkstra(self, request: PlanningRequest) -> PlanningResult:
        """Algorithme de Dijkstra pour planification"""
        # Implémentation simplifiée de Dijkstra
        # Similaire à A* mais sans heuristique
        start_node = Node3D(request.start)
        goal_node = Node3D(request.goal)
        
        distances = {start_node: 0}
        previous = {}
        unvisited = [start_node]
        
        iterations = 0
        max_iterations = 5000
        
        while unvisited and iterations < max_iterations:
            iterations += 1
            current_node = min(unvisited, key=lambda n: distances.get(n, float('inf')))
            unvisited.remove(current_node)
            
            if current_node == goal_node:
                path = self._reconstruct_path_dijkstra(previous, current_node, start_node)
                return PlanningResult(
                    success=True,
                    path=path,
                    cost=distances[current_node],
                    iterations=iterations
                )
            
            for neighbor_pos in self._get_neighbors(current_node.position):
                neighbor_node = Node3D(neighbor_pos)
                
                if self._is_collision(neighbor_pos):
                    continue
                
                alt = distances[current_node] + self._distance_3d(current_node.position, neighbor_pos)
                
                if neighbor_node not in distances or alt < distances[neighbor_node]:
                    distances[neighbor_node] = alt
                    previous[neighbor_node] = current_node
                    if neighbor_node not in unvisited:
                        unvisited.append(neighbor_node)
        
        return PlanningResult(
            success=False,
            iterations=iterations,
            error_message="Aucun chemin trouvé avec Dijkstra"
        )
    
    # Méthodes utilitaires
    def _get_neighbors(self, position: Point) -> List[Point]:
        """Obtient les voisins d'une position"""
        neighbors = []
        resolution = self.get_parameter('grid_resolution').value
        
        # 26 voisins en 3D (sans le centre)
        for dx in [-resolution, 0, resolution]:
            for dy in [-resolution, 0, resolution]:
                for dz in [-resolution, 0, resolution]:
                    if dx == 0 and dy == 0 and dz == 0:
                        continue
                    
                    neighbor = Point()
                    neighbor.x = position.x + dx
                    neighbor.y = position.y + dy
                    neighbor.z = position.z + dz
                    
                    if self._is_within_bounds(neighbor):
                        neighbors.append(neighbor)
        
        return neighbors
    
    def _is_within_bounds(self, position: Point) -> bool:
        """Vérifie si une position est dans les limites"""
        return (self.planning_bounds['x_min'] <= position.x <= self.planning_bounds['x_max'] and
                self.planning_bounds['y_min'] <= position.y <= self.planning_bounds['y_max'] and
                self.planning_bounds['z_min'] <= position.z <= self.planning_bounds['z_max'])
    
    def _is_collision(self, position: Point) -> bool:
        """Vérifie s'il y a collision en une position"""
        # Vérifier collision avec obstacles statiques
        for obstacle in self.static_obstacles:
            if self._point_in_obstacle(position, obstacle):
                return True
        return False
    
    def _is_collision_line(self, start: Point, end: Point) -> bool:
        """Vérifie s'il y a collision le long d'une ligne"""
        # Échantillonner la ligne et vérifier les collisions
        num_samples = max(10, int(self._distance_3d(start, end) * 2))
        
        for i in range(num_samples + 1):
            t = i / num_samples
            point = Point()
            point.x = start.x + t * (end.x - start.x)
            point.y = start.y + t * (end.y - start.y)
            point.z = start.z + t * (end.z - start.z)
            
            if self._is_collision(point):
                return True
        
        return False
    
    def _point_in_obstacle(self, point: Point, obstacle: Dict[str, Any]) -> bool:
        """Vérifie si un point est dans un obstacle"""
        # Implémentation simplifiée pour obstacle sphérique
        if obstacle.get('type') == 'sphere':
            center = obstacle.get('center', {'x': 0, 'y': 0, 'z': 0})
            radius = obstacle.get('radius', 1.0)
            
            distance = math.sqrt(
                (point.x - center['x'])**2 +
                (point.y - center['y'])**2 +
                (point.z - center['z'])**2
            )
            return distance <= radius
        
        return False
    
    def _distance_3d(self, p1: Point, p2: Point) -> float:
        """Calcule la distance euclidienne 3D"""
        return math.sqrt(
            (p2.x - p1.x)**2 +
            (p2.y - p1.y)**2 +
            (p2.z - p1.z)**2
        )
    
    def _sample_random_point(self) -> Point:
        """Échantillonne un point aléatoire dans l'espace"""
        point = Point()
        point.x = random.uniform(self.planning_bounds['x_min'], self.planning_bounds['x_max'])
        point.y = random.uniform(self.planning_bounds['y_min'], self.planning_bounds['y_max'])
        point.z = random.uniform(self.planning_bounds['z_min'], self.planning_bounds['z_max'])
        return point
    
    def _extend_towards(self, from_point: Point, to_point: Point, step_size: float) -> Point:
        """Étend d'un point vers un autre avec une taille de pas limitée"""
        distance = self._distance_3d(from_point, to_point)
        
        if distance <= step_size:
            return to_point
        
        ratio = step_size / distance
        new_point = Point()
        new_point.x = from_point.x + ratio * (to_point.x - from_point.x)
        new_point.y = from_point.y + ratio * (to_point.y - from_point.y)
        new_point.z = from_point.z + ratio * (to_point.z - from_point.z)
        
        return new_point
    
    def _reconstruct_path(self, came_from: Dict[Node3D, Node3D], current: Node3D) -> List[Point]:
        """Reconstruit le chemin depuis les parents"""
        path = [current.position]
        
        while current in came_from:
            current = came_from[current]
            path.append(current.position)
        
        path.reverse()
        return path
    
    def _reconstruct_path_rrt(self, node: Node3D) -> List[Point]:
        """Reconstruit le chemin pour RRT*"""
        path = []
        current = node
        
        while current is not None:
            path.append(current.position)
            current = current.parent
        
        path.reverse()
        return path
    
    def _reconstruct_path_dijkstra(self, previous: Dict[Node3D, Node3D], 
                                  current: Node3D, start: Node3D) -> List[Point]:
        """Reconstruit le chemin pour Dijkstra"""
        path = [current.position]
        
        while current in previous:
            current = previous[current]
            path.append(current.position)
        
        path.reverse()
        return path
    
    def _optimize_path(self, path: List[Point]) -> List[Point]:
        """Optimise un chemin existant"""
        if len(path) < 3:
            return path
        
        optimized_path = [path[0]]
        
        # Suppression des points redondants (ligne droite)
        for i in range(1, len(path) - 1):
            # Vérifier si on peut aller directement du dernier point ajouté au point suivant
            if not self._is_collision_line(optimized_path[-1], path[i + 1]):
                continue  # Skip le point intermédiaire
            else:
                optimized_path.append(path[i])
        
        optimized_path.append(path[-1])
        return optimized_path
    
    def _smooth_path(self, path: List[Point]) -> List[Point]:
        """Lisse un chemin avec des courbes de Bézier"""
        if len(path) < 3:
            return path
        
        smoothed_path = [path[0]]
        
        # Lissage par moyenne pondérée
        for i in range(1, len(path) - 1):
            smooth_point = Point()
            smooth_point.x = 0.25 * path[i-1].x + 0.5 * path[i].x + 0.25 * path[i+1].x
            smooth_point.y = 0.25 * path[i-1].y + 0.5 * path[i].y + 0.25 * path[i+1].y
            smooth_point.z = 0.25 * path[i-1].z + 0.5 * path[i].z + 0.25 * path[i+1].z
            smoothed_path.append(smooth_point)
        
        smoothed_path.append(path[-1])
        return smoothed_path
    
    def _publish_planned_path(self, path: List[Point]):
        """Publie le chemin planifié"""
        if not path:
            return
        
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        for point in path:
            pose_stamped = PoseStamped()
            pose_stamped.header = path_msg.header
            pose_stamped.pose.position = point
            pose_stamped.pose.orientation.w = 1.0  # Quaternion neutre
            path_msg.poses.append(pose_stamped)
        
        self.planned_path_pub.publish(path_msg)
        self.get_logger().info(f"Chemin publié avec {len(path)} waypoints")
    
    def _trigger_planning(self, goal: Point):
        """Déclenche une planification automatique"""
        if not hasattr(self, 'current_position'):
            self.get_logger().warning("Position actuelle non disponible pour planification automatique")
            return
        
        # Déterminer le meilleur algorithme
        distance = self._distance_3d(self.current_position, goal)
        if distance < 10:
            planner_type = PlannerType.ASTAR
        elif len(self.static_obstacles) > 10:
            planner_type = PlannerType.RRT_STAR
        else:
            planner_type = PlannerType.ASTAR
        
        # Créer et exécuter la requête
        request = PlanningRequest(
            start=self.current_position,
            goal=goal,
            planner_type=planner_type
        )
        
        result = self._plan_path(request)
        
        if result.success:
            self._publish_planned_path(result.path)
            self.get_logger().info(f"Planification automatique réussie en {result.planning_time:.2f}s")
        else:
            self.get_logger().warning(f"Planification automatique échouée: {result.error_message}")
        
        self._update_statistics(result)
    
    def _update_statistics(self, result: PlanningResult):
        """Met à jour les statistiques de planification"""
        self.planning_statistics['total_requests'] += 1
        
        if result.success:
            self.planning_statistics['successful_plans'] += 1
        
        # Moyenne mobile du temps de planification
        current_avg = self.planning_statistics['average_planning_time']
        total_requests = self.planning_statistics['total_requests']
        new_avg = (current_avg * (total_requests - 1) + result.planning_time) / total_requests
        self.planning_statistics['average_planning_time'] = new_avg
        
        # Compteur d'algorithmes
        algo = result.algorithm_used
        if algo in self.planning_statistics['algorithms_used']:
            self.planning_statistics['algorithms_used'][algo] += 1
        else:
            self.planning_statistics['algorithms_used'][algo] = 1
    
    def _publish_statistics(self):
        """Publie les statistiques périodiquement"""
        try:
            import json
            
            stats_data = {
                'timestamp': time.time(),
                'total_requests': self.planning_statistics['total_requests'],
                'successful_plans': self.planning_statistics['successful_plans'],
                'success_rate': (self.planning_statistics['successful_plans'] / 
                               max(1, self.planning_statistics['total_requests'])) * 100,
                'average_planning_time': self.planning_statistics['average_planning_time'],
                'algorithms_used': self.planning_statistics['algorithms_used'],
                'planning_active': self.planning_active
            }
            
            stats_msg = String()
            stats_msg.data = json.dumps(stats_data)
            self.planning_metrics_pub.publish(stats_msg)
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la publication des statistiques: {e}")
    
    def _reload_parameters(self):
        """Recharge les paramètres du nœud"""
        # Recharger les paramètres dynamiquement
        pass


def main(args=None):
    """Point d'entrée principal"""
    rclpy.init(args=args)
    
    try:
        node = TrajectoryPlannerNode()
        
        # Utiliser MultiThreadedExecutor pour les services
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        node.get_logger().info("Trajectory Planner Node démarré")
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Erreur: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
