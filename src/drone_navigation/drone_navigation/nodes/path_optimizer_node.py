#!/usr/bin/env python3
"""
Path Optimizer Node - Optimiseur de chemins multi-objectifs pour navigation drone
Architecture: Micro-nœud spécialisé dans l'optimisation de trajectoires
Algorithmes: Génétique, Particle Swarm, Simulated Annealing, A* variants
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import numpy as np
import math
import random
import copy
from typing import List, Tuple, Optional, Dict, Any, Callable
from dataclasses import dataclass, field
from enum import Enum
import heapq
import threading
import time

# Messages ROS2
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import Header, Float64, String, Bool
from sensor_msgs.msg import PointCloud2

# Services personnalisés
from drone_msgs.srv import OptimizePath, GetOptimizationMetrics
from drone_msgs.msg import OptimizationRequest, OptimizationResult, PathMetrics

# Utilitaires scientifiques
from scipy.optimize import differential_evolution, dual_annealing
from scipy.spatial import KDTree, distance_matrix
from sklearn.cluster import KMeans
import matplotlib.pyplot as plt


class OptimizationAlgorithm(Enum):
    """Algorithmes d'optimisation disponibles"""
    GENETIC = "genetic"
    PARTICLE_SWARM = "particle_swarm"  
    SIMULATED_ANNEALING = "simulated_annealing"
    DIFFERENTIAL_EVOLUTION = "differential_evolution"
    GRADIENT_DESCENT = "gradient_descent"
    ANT_COLONY = "ant_colony"
    HYBRID = "hybrid"


class OptimizationObjective(Enum):
    """Objectifs d'optimisation"""
    MINIMIZE_DISTANCE = "distance"
    MINIMIZE_TIME = "time"
    MINIMIZE_ENERGY = "energy"
    MAXIMIZE_COVERAGE = "coverage"
    MINIMIZE_RISK = "risk"
    MULTI_OBJECTIVE = "multi"


@dataclass
class OptimizationParameters:
    """Paramètres de configuration de l'optimisation"""
    algorithm: OptimizationAlgorithm = OptimizationAlgorithm.GENETIC
    objective: OptimizationObjective = OptimizationObjective.MINIMIZE_DISTANCE
    population_size: int = 50
    max_iterations: int = 1000
    convergence_threshold: float = 1e-6
    mutation_rate: float = 0.1
    crossover_rate: float = 0.8
    elite_ratio: float = 0.1
    
    # Paramètres PSO
    inertia_weight: float = 0.9
    cognitive_weight: float = 2.0
    social_weight: float = 2.0
    
    # Paramètres Simulated Annealing
    initial_temperature: float = 1000.0
    cooling_rate: float = 0.95
    min_temperature: float = 1.0
    
    # Contraintes
    max_velocity: float = 5.0
    max_acceleration: float = 2.0
    min_turn_radius: float = 1.0
    safety_distance: float = 1.0
    
    # Pondérations multi-objectifs
    distance_weight: float = 0.4
    time_weight: float = 0.3
    energy_weight: float = 0.2
    risk_weight: float = 0.1


@dataclass
class Individual:
    """Individu pour algorithmes évolutionnaires"""
    genome: List[int]  # Séquence des waypoints
    fitness: float = float('inf')
    metrics: Dict[str, float] = field(default_factory=dict)
    constraints_violated: bool = False


@dataclass
class Particle:
    """Particule pour PSO"""
    position: np.ndarray
    velocity: np.ndarray
    best_position: np.ndarray
    best_fitness: float = float('inf')
    fitness: float = float('inf')


class PathOptimizerNode(Node):
    """
    Nœud d'optimisation de chemins multi-objectifs
    
    Responsabilités:
    - Optimisation de trajectoires existantes
    - Algorithmes évolutionnaires et méta-heuristiques
    - Optimisation multi-objectifs
    - Gestion des contraintes dynamiques
    """
    
    def __init__(self):
        super().__init__('path_optimizer_node')
        
        # Configuration du nœud
        self.declare_parameters()
        self.load_configuration()
        
        # État interne
        self.current_paths: Dict[str, List[Tuple[float, float, float]]] = {}
        self.optimization_history: List[Dict] = []
        self.is_optimizing = False
        self.optimization_thread: Optional[threading.Thread] = None
        
        # Métriques en temps réel
        self.current_metrics = PathMetrics()
        
        # Callback groups
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.publisher_cb_group = ReentrantCallbackGroup()
        
        # Publishers
        self.optimized_path_publisher = self.create_publisher(
            Path,
            '/drone_nav/optimized_path',
            10,
            callback_group=self.publisher_cb_group
        )
        
        self.metrics_publisher = self.create_publisher(
            PathMetrics,
            '/drone_nav/path_metrics',
            10,
            callback_group=self.publisher_cb_group
        )
        
        self.optimization_status_publisher = self.create_publisher(
            String,
            '/drone_nav/optimization_status',
            10,
            callback_group=self.publisher_cb_group
        )
        
        # Subscribers
        self.input_path_subscriber = self.create_subscription(
            Path,
            '/drone_nav/input_path',
            self.input_path_callback,
            10,
            callback_group=self.publisher_cb_group
        )
        
        self.obstacles_subscriber = self.create_subscription(
            PointCloud2,
            '/drone_nav/obstacles',
            self.obstacles_callback,
            10,
            callback_group=self.publisher_cb_group
        )
        
        # Services
        self.optimize_service = self.create_service(
            OptimizePath,
            '/drone_nav/path_optimizer_node/optimize_path',
            self.optimize_path_callback,
            callback_group=self.service_cb_group
        )
        
        self.metrics_service = self.create_service(
            GetOptimizationMetrics,
            '/drone_nav/path_optimizer_node/get_metrics',
            self.get_metrics_callback,
            callback_group=self.service_cb_group
        )
        
        # Timer pour publication périodique
        self.metrics_timer = self.create_timer(
            1.0,  # 1 Hz
            self.publish_metrics,
            callback_group=self.publisher_cb_group
        )
        
        # Obstacles et contraintes dynamiques
        self.obstacles: List[Tuple[float, float, float, float]] = []  # x, y, z, radius
        self.dynamic_constraints: Dict[str, Any] = {}
        
        self.get_logger().info("✅ Path Optimizer Node initialisé")
    
    def declare_parameters(self):
        """Déclaration des paramètres du nœud"""
        self.declare_parameter('default_algorithm', 'genetic')
        self.declare_parameter('default_objective', 'distance')
        self.declare_parameter('population_size', 50)
        self.declare_parameter('max_iterations', 1000)
        self.declare_parameter('convergence_threshold', 1e-6)
        self.declare_parameter('mutation_rate', 0.1)
        self.declare_parameter('crossover_rate', 0.8)
        self.declare_parameter('elite_ratio', 0.1)
        self.declare_parameter('max_velocity', 5.0)
        self.declare_parameter('max_acceleration', 2.0)
        self.declare_parameter('min_turn_radius', 1.0)
        self.declare_parameter('safety_distance', 1.0)
        self.declare_parameter('enable_real_time_optimization', True)
        self.declare_parameter('optimization_timeout', 30.0)
    
    def load_configuration(self):
        """Charge la configuration depuis les paramètres"""
        self.params = OptimizationParameters(
            algorithm=OptimizationAlgorithm(self.get_parameter('default_algorithm').value),
            objective=OptimizationObjective(self.get_parameter('default_objective').value),
            population_size=self.get_parameter('population_size').value,
            max_iterations=self.get_parameter('max_iterations').value,
            convergence_threshold=self.get_parameter('convergence_threshold').value,
            mutation_rate=self.get_parameter('mutation_rate').value,
            crossover_rate=self.get_parameter('crossover_rate').value,
            elite_ratio=self.get_parameter('elite_ratio').value,
            max_velocity=self.get_parameter('max_velocity').value,
            max_acceleration=self.get_parameter('max_acceleration').value,
            min_turn_radius=self.get_parameter('min_turn_radius').value,
            safety_distance=self.get_parameter('safety_distance').value
        )
        
        self.enable_real_time = self.get_parameter('enable_real_time_optimization').value
        self.optimization_timeout = self.get_parameter('optimization_timeout').value
    
    def input_path_callback(self, msg: Path):
        """Callback pour réception d'un nouveau chemin à optimiser"""
        if self.enable_real_time and not self.is_optimizing:
            waypoints = self.extract_waypoints_from_path(msg)
            path_id = f"auto_{len(self.current_paths)}"
            self.current_paths[path_id] = waypoints
            
            # Démarrer optimisation automatique
            self.start_background_optimization(path_id, waypoints)
    
    def obstacles_callback(self, msg: PointCloud2):
        """Callback pour mise à jour des obstacles"""
        # TODO: Convertir PointCloud2 en obstacles
        # Pour l'instant, simulation d'obstacles
        self.obstacles = [
            (10.0, 10.0, 5.0, 2.0),  # x, y, z, radius
            (20.0, 15.0, 5.0, 1.5),
        ]
        
        self.get_logger().debug(f"🚧 Obstacles mis à jour: {len(self.obstacles)}")
    
    async def optimize_path_callback(self, request, response):
        """Service d'optimisation de chemin"""
        try:
            self.get_logger().info(f"🔧 Optimisation demandée: {request.algorithm}")
            
            # Extraction des waypoints
            waypoints = self.extract_waypoints_from_path(request.input_path)
            
            if len(waypoints) < 2:
                raise ValueError("Chemin insuffisant (< 2 waypoints)")
            
            # Configuration personnalisée
            params = self.build_params_from_request(request)
            
            # Optimisation
            start_time = time.time()
            optimized_waypoints, metrics = await self.optimize_path(
                waypoints, params, request.path_id
            )
            optimization_time = time.time() - start_time
            
            # Stockage du résultat
            self.current_paths[request.path_id] = optimized_waypoints
            
            # Construction de la réponse
            response.success = True
            response.optimized_path = self.build_path_message(optimized_waypoints)
            response.optimization_time = optimization_time
            response.distance_improvement = metrics.get('distance_improvement', 0.0)
            response.energy_savings = metrics.get('energy_savings', 0.0)
            response.iterations_completed = metrics.get('iterations', 0)
            response.final_fitness = metrics.get('final_fitness', 0.0)
            
            # Publication du chemin optimisé
            self.optimized_path_publisher.publish(response.optimized_path)
            
            self.get_logger().info(
                f"✅ Optimisation terminée: "
                f"{response.distance_improvement:.1f}% amélioration distance, "
                f"{optimization_time:.2f}s"
            )
            
        except Exception as e:
            self.get_logger().error(f"❌ Erreur optimisation: {str(e)}")
            response.success = False
            response.error_message = str(e)
        
        return response
    
    async def get_metrics_callback(self, request, response):
        """Service de récupération des métriques d'optimisation"""
        try:
            path_id = request.path_id
            
            if path_id in self.current_paths:
                waypoints = self.current_paths[path_id]
                metrics = self.calculate_path_metrics(waypoints)
                
                response.success = True
                response.metrics = metrics
                response.optimization_history = json.dumps(self.optimization_history[-10:])
            else:
                response.success = False
                response.error_message = f"Chemin {path_id} non trouvé"
                
        except Exception as e:
            response.success = False
            response.error_message = str(e)
        
        return response
    
    def extract_waypoints_from_path(self, path_msg: Path) -> List[Tuple[float, float, float]]:
        """Extrait les waypoints d'un message Path"""
        waypoints = []
        for pose in path_msg.poses:
            x = pose.pose.position.x
            y = pose.pose.position.y
            z = pose.pose.position.z
            waypoints.append((x, y, z))
        return waypoints
    
    def build_params_from_request(self, request) -> OptimizationParameters:
        """Construit les paramètres depuis une requête"""
        params = copy.deepcopy(self.params)
        
        if hasattr(request, 'algorithm'):
            params.algorithm = OptimizationAlgorithm(request.algorithm)
        if hasattr(request, 'objective'):
            params.objective = OptimizationObjective(request.objective)
        if hasattr(request, 'max_iterations'):
            params.max_iterations = request.max_iterations
        if hasattr(request, 'population_size'):
            params.population_size = request.population_size
        
        return params
    
    async def optimize_path(
        self, 
        waypoints: List[Tuple[float, float, float]], 
        params: OptimizationParameters,
        path_id: str
    ) -> Tuple[List[Tuple[float, float, float]], Dict[str, float]]:
        """Optimise un chemin selon l'algorithme spécifié"""
        
        original_metrics = self.calculate_path_metrics(waypoints)
        
        if params.algorithm == OptimizationAlgorithm.GENETIC:
            optimized_waypoints = await self.genetic_algorithm_optimization(waypoints, params)
        elif params.algorithm == OptimizationAlgorithm.PARTICLE_SWARM:
            optimized_waypoints = await self.particle_swarm_optimization(waypoints, params)
        elif params.algorithm == OptimizationAlgorithm.SIMULATED_ANNEALING:
            optimized_waypoints = await self.simulated_annealing_optimization(waypoints, params)
        elif params.algorithm == OptimizationAlgorithm.DIFFERENTIAL_EVOLUTION:
            optimized_waypoints = await self.differential_evolution_optimization(waypoints, params)
        elif params.algorithm == OptimizationAlgorithm.ANT_COLONY:
            optimized_waypoints = await self.ant_colony_optimization(waypoints, params)
        elif params.algorithm == OptimizationAlgorithm.HYBRID:
            optimized_waypoints = await self.hybrid_optimization(waypoints, params)
        else:
            optimized_waypoints = waypoints  # Pas d'optimisation
        
        # Calcul des améliorations
        optimized_metrics = self.calculate_path_metrics(optimized_waypoints)
        
        metrics = {
            'original_distance': original_metrics['total_distance'],
            'optimized_distance': optimized_metrics['total_distance'],
            'distance_improvement': (
                (original_metrics['total_distance'] - optimized_metrics['total_distance']) /
                original_metrics['total_distance'] * 100
            ),
            'original_energy': original_metrics['estimated_energy'],
            'optimized_energy': optimized_metrics['estimated_energy'],
            'energy_savings': (
                (original_metrics['estimated_energy'] - optimized_metrics['estimated_energy']) /
                original_metrics['estimated_energy'] * 100
            ),
            'final_fitness': optimized_metrics['fitness'],
            'iterations': params.max_iterations  # À adapter selon l'algorithme
        }
        
        # Historique
        self.optimization_history.append({
            'timestamp': time.time(),
            'path_id': path_id,
            'algorithm': params.algorithm.value,
            'metrics': metrics
        })
        
        return optimized_waypoints, metrics
    
    async def genetic_algorithm_optimization(
        self, 
        waypoints: List[Tuple[float, float, float]], 
        params: OptimizationParameters
    ) -> List[Tuple[float, float, float]]:
        """Optimisation par algorithme génétique"""
        
        if len(waypoints) <= 2:
            return waypoints
        
        # Initialisation de la population
        population = self.initialize_population(waypoints, params.population_size)
        
        best_individual = None
        best_fitness = float('inf')
        
        for generation in range(params.max_iterations):
            # Évaluation de la population
            for individual in population:
                individual.fitness = self.evaluate_fitness(
                    self.decode_individual(individual, waypoints), params
                )
                
                if individual.fitness < best_fitness:
                    best_fitness = individual.fitness
                    best_individual = copy.deepcopy(individual)
            
            # Test de convergence
            if generation % 50 == 0:
                self.publish_optimization_status(
                    f"Génération {generation}, Fitness: {best_fitness:.2f}"
                )
            
            # Sélection
            selected = self.selection(population, params)
            
            # Croisement et mutation
            new_population = []
            
            # Élitisme
            elite_count = int(params.population_size * params.elite_ratio)
            population.sort(key=lambda x: x.fitness)
            new_population.extend(population[:elite_count])
            
            # Génération de nouveaux individus
            while len(new_population) < params.population_size:
                parent1, parent2 = random.sample(selected, 2)
                
                if random.random() < params.crossover_rate:
                    child1, child2 = self.crossover(parent1, parent2)
                else:
                    child1, child2 = copy.deepcopy(parent1), copy.deepcopy(parent2)
                
                if random.random() < params.mutation_rate:
                    self.mutate(child1)
                if random.random() < params.mutation_rate:
                    self.mutate(child2)
                
                new_population.extend([child1, child2])
            
            population = new_population[:params.population_size]
        
        # Retour du meilleur individu
        if best_individual:
            return self.decode_individual(best_individual, waypoints)
        else:
            return waypoints
    
    def initialize_population(
        self, 
        waypoints: List[Tuple[float, float, float]], 
        population_size: int
    ) -> List[Individual]:
        """Initialise une population pour l'algorithme génétique"""
        population = []
        
        # Le premier individu est la séquence originale
        original_genome = list(range(len(waypoints)))
        population.append(Individual(genome=original_genome))
        
        # Génération d'individus aléatoires
        for _ in range(population_size - 1):
            genome = list(range(len(waypoints)))
            
            # Garder le premier et dernier waypoint fixes
            if len(genome) > 2:
                middle_points = genome[1:-1]
                random.shuffle(middle_points)
                genome = [genome[0]] + middle_points + [genome[-1]]
            
            population.append(Individual(genome=genome))
        
        return population
    
    def decode_individual(
        self, 
        individual: Individual, 
        original_waypoints: List[Tuple[float, float, float]]
    ) -> List[Tuple[float, float, float]]:
        """Décode un individu en séquence de waypoints"""
        return [original_waypoints[i] for i in individual.genome]
    
    def evaluate_fitness(
        self, 
        waypoints: List[Tuple[float, float, float]], 
        params: OptimizationParameters
    ) -> float:
        """Évalue la fitness d'une solution"""
        
        if params.objective == OptimizationObjective.MINIMIZE_DISTANCE:
            return self.calculate_total_distance(waypoints)
        
        elif params.objective == OptimizationObjective.MINIMIZE_TIME:
            return self.calculate_travel_time(waypoints, params.max_velocity)
        
        elif params.objective == OptimizationObjective.MINIMIZE_ENERGY:
            return self.calculate_energy_consumption(waypoints, params)
        
        elif params.objective == OptimizationObjective.MINIMIZE_RISK:
            return self.calculate_risk_factor(waypoints)
        
        elif params.objective == OptimizationObjective.MULTI_OBJECTIVE:
            return self.calculate_multi_objective_fitness(waypoints, params)
        
        else:
            return self.calculate_total_distance(waypoints)
    
    def calculate_total_distance(self, waypoints: List[Tuple[float, float, float]]) -> float:
        """Calcule la distance totale d'un chemin"""
        if len(waypoints) < 2:
            return 0.0
        
        total_distance = 0.0
        for i in range(1, len(waypoints)):
            dx = waypoints[i][0] - waypoints[i-1][0]
            dy = waypoints[i][1] - waypoints[i-1][1]
            dz = waypoints[i][2] - waypoints[i-1][2]
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            total_distance += distance
        
        return total_distance
    
    def calculate_travel_time(
        self, 
        waypoints: List[Tuple[float, float, float]], 
        max_velocity: float
    ) -> float:
        """Calcule le temps de parcours estimé"""
        total_distance = self.calculate_total_distance(waypoints)
        
        # Pénalité pour les virages serrés
        turn_penalty = 0.0
        if len(waypoints) >= 3:
            for i in range(1, len(waypoints) - 1):
                angle = self.calculate_turn_angle(waypoints[i-1], waypoints[i], waypoints[i+1])
                if angle > math.pi / 2:  # Virage > 90°
                    turn_penalty += 2.0  # 2 secondes de pénalité
        
        return total_distance / max_velocity + turn_penalty
    
    def calculate_energy_consumption(
        self, 
        waypoints: List[Tuple[float, float, float]], 
        params: OptimizationParameters
    ) -> float:
        """Calcule la consommation énergétique estimée"""
        
        energy = 0.0
        
        for i in range(1, len(waypoints)):
            dx = waypoints[i][0] - waypoints[i-1][0]
            dy = waypoints[i][1] - waypoints[i-1][1]
            dz = waypoints[i][2] - waypoints[i-1][2]
            
            distance = math.sqrt(dx*dx + dy*dy + dz*dz)
            
            # Énergie de déplacement horizontal
            energy += distance * 1.0  # 1 J/m
            
            # Énergie de changement d'altitude (plus coûteux)
            if dz > 0:  # Montée
                energy += abs(dz) * 5.0  # 5 J/m
            else:  # Descente (récupération partielle)
                energy += abs(dz) * 2.0  # 2 J/m
        
        # Pénalité pour les accélérations/décélérations
        for i in range(1, len(waypoints) - 1):
            turn_angle = self.calculate_turn_angle(waypoints[i-1], waypoints[i], waypoints[i+1])
            energy += turn_angle * 10.0  # Pénalité proportionnelle à l'angle
        
        return energy
    
    def calculate_risk_factor(self, waypoints: List[Tuple[float, float, float]]) -> float:
        """Calcule le facteur de risque d'un chemin"""
        risk = 0.0
        
        for waypoint in waypoints:
            # Distance aux obstacles
            for obstacle in self.obstacles:
                obs_x, obs_y, obs_z, obs_radius = obstacle
                dx = waypoint[0] - obs_x
                dy = waypoint[1] - obs_y
                dz = waypoint[2] - obs_z
                distance = math.sqrt(dx*dx + dy*dy + dz*dz)
                
                if distance < obs_radius + 1.0:  # Zone de sécurité
                    risk += 100.0 / (distance + 0.1)  # Risque inversement proportionnel
        
        return risk
    
    def calculate_multi_objective_fitness(
        self, 
        waypoints: List[Tuple[float, float, float]], 
        params: OptimizationParameters
    ) -> float:
        """Calcule la fitness multi-objectifs pondérée"""
        
        distance = self.calculate_total_distance(waypoints)
        time = self.calculate_travel_time(waypoints, params.max_velocity)
        energy = self.calculate_energy_consumption(waypoints, params)
        risk = self.calculate_risk_factor(waypoints)
        
        # Normalisation (approximative)
        norm_distance = distance / 1000.0  # Supposé max 1km
        norm_time = time / 300.0  # Supposé max 5 minutes
        norm_energy = energy / 10000.0  # Supposé max 10kJ
        norm_risk = risk / 1000.0  # Supposé max 1000
        
        # Combinaison pondérée
        fitness = (
            params.distance_weight * norm_distance +
            params.time_weight * norm_time +
            params.energy_weight * norm_energy +
            params.risk_weight * norm_risk
        )
        
        return fitness
    
    def calculate_turn_angle(
        self, 
        p1: Tuple[float, float, float], 
        p2: Tuple[float, float, float], 
        p3: Tuple[float, float, float]
    ) -> float:
        """Calcule l'angle de virage entre trois points"""
        
        # Vecteurs
        v1 = np.array([p2[0] - p1[0], p2[1] - p1[1]])
        v2 = np.array([p3[0] - p2[0], p3[1] - p2[1]])
        
        # Normalisation
        v1_norm = np.linalg.norm(v1)
        v2_norm = np.linalg.norm(v2)
        
        if v1_norm == 0 or v2_norm == 0:
            return 0.0
        
        v1 = v1 / v1_norm
        v2 = v2 / v2_norm
        
        # Calcul de l'angle
        cos_angle = np.clip(np.dot(v1, v2), -1.0, 1.0)
        angle = math.acos(cos_angle)
        
        return angle
    
    def selection(self, population: List[Individual], params: OptimizationParameters) -> List[Individual]:
        """Sélection par tournoi"""
        selected = []
        tournament_size = max(2, params.population_size // 10)
        
        for _ in range(params.population_size):
            tournament = random.sample(population, tournament_size)
            winner = min(tournament, key=lambda x: x.fitness)
            selected.append(copy.deepcopy(winner))
        
        return selected
    
    def crossover(self, parent1: Individual, parent2: Individual) -> Tuple[Individual, Individual]:
        """Croisement par ordre (OX)"""
        if len(parent1.genome) <= 2:
            return copy.deepcopy(parent1), copy.deepcopy(parent2)
        
        # Points de croisement
        start = random.randint(1, len(parent1.genome) - 2)
        end = random.randint(start + 1, len(parent1.genome) - 1)
        
        # Enfant 1
        child1_genome = [-1] * len(parent1.genome)
        child1_genome[0] = parent1.genome[0]  # Premier point fixe
        child1_genome[-1] = parent1.genome[-1]  # Dernier point fixe
        child1_genome[start:end] = parent1.genome[start:end]
        
        # Compléter avec parent2
        remaining = [x for x in parent2.genome[1:-1] if x not in child1_genome]
        j = 0
        for i in range(1, len(child1_genome) - 1):
            if child1_genome[i] == -1:
                child1_genome[i] = remaining[j]
                j += 1
        
        # Enfant 2 (symétrique)
        child2_genome = [-1] * len(parent2.genome)
        child2_genome[0] = parent2.genome[0]
        child2_genome[-1] = parent2.genome[-1]
        child2_genome[start:end] = parent2.genome[start:end]
        
        remaining = [x for x in parent1.genome[1:-1] if x not in child2_genome]
        j = 0
        for i in range(1, len(child2_genome) - 1):
            if child2_genome[i] == -1:
                child2_genome[i] = remaining[j]
                j += 1
        
        child1 = Individual(genome=child1_genome)
        child2 = Individual(genome=child2_genome)
        
        return child1, child2
    
    def mutate(self, individual: Individual):
        """Mutation par échange de deux points"""
        if len(individual.genome) <= 2:
            return
        
        # Éviter de muter les points de départ et d'arrivée
        available_indices = list(range(1, len(individual.genome) - 1))
        
        if len(available_indices) >= 2:
            i, j = random.sample(available_indices, 2)
            individual.genome[i], individual.genome[j] = individual.genome[j], individual.genome[i]
    
    async def particle_swarm_optimization(
        self, 
        waypoints: List[Tuple[float, float, float]], 
        params: OptimizationParameters
    ) -> List[Tuple[float, float, float]]:
        """Optimisation par essaim particulaire (PSO)"""
        
        if len(waypoints) <= 2:
            return waypoints
        
        # Initialisation des particules
        swarm = self.initialize_swarm(waypoints, params)
        global_best_position = None
        global_best_fitness = float('inf')
        
        for iteration in range(params.max_iterations):
            for particle in swarm:
                # Évaluation
                decoded_waypoints = self.decode_particle_position(particle.position, waypoints)
                particle.fitness = self.evaluate_fitness(decoded_waypoints, params)
                
                # Mise à jour du meilleur personnel
                if particle.fitness < particle.best_fitness:
                    particle.best_fitness = particle.fitness
                    particle.best_position = particle.position.copy()
                
                # Mise à jour du meilleur global
                if particle.fitness < global_best_fitness:
                    global_best_fitness = particle.fitness
                    global_best_position = particle.position.copy()
            
            # Mise à jour des vitesses et positions
            for particle in swarm:
                # Composantes aléatoires
                r1, r2 = random.random(), random.random()
                
                # Mise à jour de la vitesse
                inertia = particle.velocity * params.inertia_weight
                cognitive = params.cognitive_weight * r1 * (particle.best_position - particle.position)
                social = params.social_weight * r2 * (global_best_position - particle.position)
                
                particle.velocity = inertia + cognitive + social
                
                # Limitation de la vitesse
                max_velocity = 0.1 * len(waypoints)
                particle.velocity = np.clip(particle.velocity, -max_velocity, max_velocity)
                
                # Mise à jour de la position
                particle.position += particle.velocity
                
                # Contrainte sur la position (garder dans les limites)
                particle.position = np.clip(particle.position, 0, len(waypoints) - 1)
        
        # Reconstruction du chemin optimal
        if global_best_position is not None:
            return self.decode_particle_position(global_best_position, waypoints)
        else:
            return waypoints
    
    def initialize_swarm(
        self, 
        waypoints: List[Tuple[float, float, float]], 
        params: OptimizationParameters
    ) -> List[Particle]:
        """Initialise l'essaim de particules"""
        swarm = []
        num_variables = len(waypoints) - 2  # Exclure début et fin
        
        for _ in range(params.population_size):
            position = np.random.permutation(range(1, len(waypoints) - 1)).astype(float)
            position = np.concatenate([[0], position, [len(waypoints) - 1]])
            
            velocity = np.random.uniform(-0.1, 0.1, len(position))
            
            particle = Particle(
                position=position,
                velocity=velocity,
                best_position=position.copy()
            )
            
            swarm.append(particle)
        
        return swarm
    
    def decode_particle_position(
        self, 
        position: np.ndarray, 
        waypoints: List[Tuple[float, float, float]]
    ) -> List[Tuple[float, float, float]]:
        """Décode la position d'une particule en séquence de waypoints"""
        # Conversion en indices entiers triés
        indices = np.argsort(position).astype(int)
        return [waypoints[i] for i in indices]
    
    async def simulated_annealing_optimization(
        self, 
        waypoints: List[Tuple[float, float, float]], 
        params: OptimizationParameters
    ) -> List[Tuple[float, float, float]]:
        """Optimisation par recuit simulé"""
        
        if len(waypoints) <= 2:
            return waypoints
        
        # Solution initiale
        current_solution = list(range(len(waypoints)))
        current_fitness = self.evaluate_fitness(waypoints, params)
        
        best_solution = current_solution.copy()
        best_fitness = current_fitness
        
        temperature = params.initial_temperature
        
        for iteration in range(params.max_iterations):
            # Génération d'une solution voisine
            neighbor_solution = self.generate_neighbor_solution(current_solution)
            neighbor_waypoints = [waypoints[i] for i in neighbor_solution]
            neighbor_fitness = self.evaluate_fitness(neighbor_waypoints, params)
            
            # Critère d'acceptation
            delta = neighbor_fitness - current_fitness
            
            if delta < 0 or random.random() < math.exp(-delta / temperature):
                current_solution = neighbor_solution
                current_fitness = neighbor_fitness
                
                # Mise à jour du meilleur
                if current_fitness < best_fitness:
                    best_solution = current_solution.copy()
                    best_fitness = current_fitness
            
            # Refroidissement
            temperature *= params.cooling_rate
            
            if temperature < params.min_temperature:
                break
        
        return [waypoints[i] for i in best_solution]
    
    def generate_neighbor_solution(self, solution: List[int]) -> List[int]:
        """Génère une solution voisine par échange"""
        neighbor = solution.copy()
        
        if len(neighbor) > 2:
            # Éviter les points de départ et d'arrivée
            available_indices = list(range(1, len(neighbor) - 1))
            
            if len(available_indices) >= 2:
                i, j = random.sample(available_indices, 2)
                neighbor[i], neighbor[j] = neighbor[j], neighbor[i]
        
        return neighbor
    
    def calculate_path_metrics(self, waypoints: List[Tuple[float, float, float]]) -> Dict[str, float]:
        """Calcule toutes les métriques d'un chemin"""
        if not waypoints:
            return {}
        
        total_distance = self.calculate_total_distance(waypoints)
        estimated_time = self.calculate_travel_time(waypoints, self.params.max_velocity)
        estimated_energy = self.calculate_energy_consumption(waypoints, self.params)
        risk_factor = self.calculate_risk_factor(waypoints)
        fitness = self.evaluate_fitness(waypoints, self.params)
        
        return {
            'total_distance': total_distance,
            'estimated_time': estimated_time,
            'estimated_energy': estimated_energy,
            'risk_factor': risk_factor,
            'fitness': fitness,
            'num_waypoints': len(waypoints)
        }
    
    def build_path_message(self, waypoints: List[Tuple[float, float, float]]) -> Path:
        """Construit un message Path à partir de waypoints"""
        path_msg = Path()
        path_msg.header = Header()
        path_msg.header.stamp = self.get_clock().now().to_msg()
        path_msg.header.frame_id = "map"
        
        for waypoint in waypoints:
            pose = PoseStamped()
            pose.header = path_msg.header
            pose.pose.position.x = waypoint[0]
            pose.pose.position.y = waypoint[1]
            pose.pose.position.z = waypoint[2]
            pose.pose.orientation.w = 1.0
            path_msg.poses.append(pose)
        
        return path_msg
    
    def start_background_optimization(self, path_id: str, waypoints: List[Tuple[float, float, float]]):
        """Démarre une optimisation en arrière-plan"""
        if self.optimization_thread and self.optimization_thread.is_alive():
            return  # Optimisation déjà en cours
        
        self.is_optimizing = True
        self.optimization_thread = threading.Thread(
            target=self.background_optimization_worker,
            args=(path_id, waypoints)
        )
        self.optimization_thread.start()
    
    def background_optimization_worker(self, path_id: str, waypoints: List[Tuple[float, float, float]]):
        """Worker pour optimisation en arrière-plan"""
        try:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            
            optimized_waypoints, metrics = loop.run_until_complete(
                self.optimize_path(waypoints, self.params, path_id)
            )
            
            # Mise à jour du chemin
            self.current_paths[path_id] = optimized_waypoints
            
            # Publication du résultat
            path_msg = self.build_path_message(optimized_waypoints)
            self.optimized_path_publisher.publish(path_msg)
            
            self.publish_optimization_status(f"✅ Optimisation terminée: {path_id}")
            
        except Exception as e:
            self.get_logger().error(f"❌ Erreur optimisation arrière-plan: {str(e)}")
        finally:
            self.is_optimizing = False
    
    def publish_optimization_status(self, status: str):
        """Publie le statut de l'optimisation"""
        msg = String()
        msg.data = status
        self.optimization_status_publisher.publish(msg)
    
    def publish_metrics(self):
        """Publie les métriques périodiquement"""
        if self.current_paths:
            # Métriques du dernier chemin
            latest_path = list(self.current_paths.values())[-1]
            metrics = self.calculate_path_metrics(latest_path)
            
            self.current_metrics.header.stamp = self.get_clock().now().to_msg()
            self.current_metrics.total_distance = metrics.get('total_distance', 0.0)
            self.current_metrics.estimated_time = metrics.get('estimated_time', 0.0)
            self.current_metrics.estimated_energy = metrics.get('estimated_energy', 0.0)
            self.current_metrics.risk_factor = metrics.get('risk_factor', 0.0)
            self.current_metrics.fitness_score = metrics.get('fitness', 0.0)
            self.current_metrics.num_waypoints = metrics.get('num_waypoints', 0)
            self.current_metrics.is_optimizing = self.is_optimizing
            
            self.metrics_publisher.publish(self.current_metrics)


def main(args=None):
    """Point d'entrée principal"""
    rclpy.init(args=args)
    
    try:
        node = PathOptimizerNode()
        
        # Utilisation d'un exécuteur multi-threadé
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        node.get_logger().info("🚀 Path Optimizer Node démarré")
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
