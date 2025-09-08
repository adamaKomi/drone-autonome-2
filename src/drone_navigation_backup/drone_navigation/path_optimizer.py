#!/usr/bin/env python3

"""
Optimiseur de chemins pour drone autonome

Implémente des algorithmes d'optimisation de trajectoires pour
améliorer l'efficacité des missions de pollinisation et réduire
la consommation énergétique.

Algorithmes d'optimisation:
- Algorithme génétique pour TSP
- Optimisation par essaims particulaires
- Lissage de trajectoire
- Réduction de waypoints redondants

Auteur: Adama Komi
Version: 1.0.0
"""

import math
import random
import numpy as np
from typing import List, Tuple, Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum
from geometry_msgs.msg import Point, Vector3
from copy import deepcopy


class OptimizationMethod(Enum):
    """Méthodes d'optimisation disponibles"""
    GENETIC_ALGORITHM = "genetic_algorithm"
    PARTICLE_SWARM = "particle_swarm"
    SIMULATED_ANNEALING = "simulated_annealing"
    NEAREST_NEIGHBOR = "nearest_neighbor"
    TWO_OPT = "two_opt"


@dataclass
class OptimizationConfig:
    """Configuration pour l'optimisation"""
    method: OptimizationMethod = OptimizationMethod.GENETIC_ALGORITHM
    max_iterations: int = 100
    population_size: int = 50
    mutation_rate: float = 0.1
    crossover_rate: float = 0.8
    convergence_threshold: float = 0.001
    smoothing_factor: float = 0.5
    waypoint_reduction_threshold: float = 0.1  # mètres


@dataclass
class OptimizationResult:
    """Résultat d'une optimisation"""
    optimized_path: List[Point]
    original_distance: float
    optimized_distance: float
    improvement_percentage: float
    computation_time: float
    iterations_used: int
    success: bool = True
    message: str = ""


class PathOptimizer:
    """
    Optimiseur de chemins pour navigation drone
    
    Optimise les trajectoires pour:
    - Minimiser la distance totale
    - Réduire les changements de direction brusques
    - Éliminer les waypoints redondants
    - Lisser les trajectoires
    """
    
    def __init__(self, node):
        """
        Initialise l'optimiseur de chemins
        
        Args:
            node: Nœud ROS2 parent
        """
        self._node = node
        self._logger = node.get_logger()
        
        # Configuration par défaut (utilisation de l'algorithme plus rapide)
        self._default_config = OptimizationConfig(
            method=OptimizationMethod.NEAREST_NEIGHBOR,  # Plus rapide que l'algorithme génétique
            max_iterations=50,  # Réduit pour de meilleures performances
            population_size=20,
            mutation_rate=0.1,
            crossover_rate=0.8,
            convergence_threshold=0.001,
            smoothing_factor=0.5,
            waypoint_reduction_threshold=0.1
        )
        
        # Statistiques
        self._stats = {
            'optimizations_performed': 0,
            'total_distance_saved': 0.0,
            'average_improvement': 0.0,
            'computation_time_total': 0.0
        }
        
        self._logger.info("PathOptimizer initialisé")

    def optimize_path(self, waypoints: List[Point], 
                     config: Optional[OptimizationConfig] = None) -> OptimizationResult:
        """
        Optimise un chemin selon la méthode configurée
        
        Args:
            waypoints: Points du chemin à optimiser
            config: Configuration d'optimisation
            
        Returns:
            Résultat de l'optimisation
        """
        if config is None:
            config = self._default_config
            
        if len(waypoints) < 2:
            return OptimizationResult(
                optimized_path=waypoints,
                original_distance=0.0,
                optimized_distance=0.0,
                improvement_percentage=0.0,
                computation_time=0.0,
                iterations_used=0,
                success=False,
                message="Chemin trop court pour optimisation"
            )
        
        start_time = time.time()
        original_distance = self._calculate_path_distance(waypoints)
        
        # Sélection de la méthode d'optimisation
        if config.method == OptimizationMethod.GENETIC_ALGORITHM:
            result = self._genetic_algorithm_optimization(waypoints, config)
        elif config.method == OptimizationMethod.NEAREST_NEIGHBOR:
            result = self._nearest_neighbor_optimization(waypoints, config)
        elif config.method == OptimizationMethod.TWO_OPT:
            result = self._two_opt_optimization(waypoints, config)
        elif config.method == OptimizationMethod.SIMULATED_ANNEALING:
            result = self._simulated_annealing_optimization(waypoints, config)
        else:
            result = self._nearest_neighbor_optimization(waypoints, config)
        
        # Post-traitement: lissage et réduction
        if result.success:
            result.optimized_path = self._smooth_path(result.optimized_path, config)
            result.optimized_path = self._reduce_waypoints(result.optimized_path, config)
        
        # Calcul des métriques finales
        computation_time = time.time() - start_time
        optimized_distance = self._calculate_path_distance(result.optimized_path)
        improvement = ((original_distance - optimized_distance) / original_distance) * 100
        
        result.original_distance = original_distance
        result.optimized_distance = optimized_distance
        result.improvement_percentage = improvement
        result.computation_time = computation_time
        
        # Mise à jour des statistiques
        self._update_statistics(result)
        
        self._logger.info(
            f"Optimisation terminée: {improvement:.1f}% d'amélioration "
            f"en {computation_time:.3f}s"
        )
        
        return result

    def _genetic_algorithm_optimization(self, waypoints: List[Point], 
                                      config: OptimizationConfig) -> OptimizationResult:
        """
        Optimisation par algorithme génétique
        
        Args:
            waypoints: Points à optimiser
            config: Configuration
            
        Returns:
            Résultat de l'optimisation
        """
        if len(waypoints) <= 2:
            return OptimizationResult(
                optimized_path=waypoints,
                original_distance=0.0,
                optimized_distance=0.0,
                improvement_percentage=0.0,
                computation_time=0.0,
                iterations_used=0
            )
        
        # Séparation du point de départ et d'arrivée
        start_point = waypoints[0]
        end_point = waypoints[-1]
        middle_points = waypoints[1:-1]
        
        if len(middle_points) <= 1:
            return OptimizationResult(
                optimized_path=waypoints,
                original_distance=self._calculate_path_distance(waypoints),
                optimized_distance=self._calculate_path_distance(waypoints),
                improvement_percentage=0.0,
                computation_time=0.0,
                iterations_used=0
            )
        
        # Génération de la population initiale
        population = []
        for _ in range(config.population_size):
            individual = middle_points.copy()
            random.shuffle(individual)
            population.append(individual)
        
        best_individual = None
        best_fitness = float('inf')
        
        # Évolution génétique
        for generation in range(config.max_iterations):
            # Évaluation de la fitness
            fitness_scores = []
            for individual in population:
                full_path = [start_point] + individual + [end_point]
                distance = self._calculate_path_distance(full_path)
                fitness_scores.append(distance)
                
                if distance < best_fitness:
                    best_fitness = distance
                    best_individual = individual.copy()
            
            # Sélection par tournoi
            new_population = []
            for _ in range(config.population_size):
                parent1 = self._tournament_selection(population, fitness_scores)
                parent2 = self._tournament_selection(population, fitness_scores)
                
                # Croisement
                if random.random() < config.crossover_rate:
                    child = self._crossover(parent1, parent2)
                else:
                    child = parent1.copy()
                
                # Mutation
                if random.random() < config.mutation_rate:
                    child = self._mutate(child)
                
                new_population.append(child)
            
            population = new_population
        
        # Construction du chemin final
        optimized_path = [start_point] + best_individual + [end_point]
        
        return OptimizationResult(
            optimized_path=optimized_path,
            original_distance=0.0,  # Sera calculé plus tard
            optimized_distance=0.0, # Sera calculé plus tard
            improvement_percentage=0.0,
            computation_time=0.0,
            iterations_used=config.max_iterations,
            success=True
        )

    def _nearest_neighbor_optimization(self, waypoints: List[Point], 
                                     config: OptimizationConfig) -> OptimizationResult:
        """
        Optimisation par plus proche voisin
        
        Args:
            waypoints: Points à optimiser
            config: Configuration
            
        Returns:
            Résultat de l'optimisation
        """
        if len(waypoints) <= 2:
            return OptimizationResult(
                optimized_path=waypoints,
                original_distance=0.0,
                optimized_distance=0.0,
                improvement_percentage=0.0,
                computation_time=0.0,
                iterations_used=1,
                success=True
            )
        
        # Algorithme du plus proche voisin
        start_point = waypoints[0]
        unvisited = waypoints[1:].copy()
        optimized_path = [start_point]
        current = start_point
        
        while unvisited:
            nearest_idx = 0
            min_distance = float('inf')
            
            for i, point in enumerate(unvisited):
                distance = self._distance_between_points(current, point)
                if distance < min_distance:
                    min_distance = distance
                    nearest_idx = i
            
            nearest_point = unvisited.pop(nearest_idx)
            optimized_path.append(nearest_point)
            current = nearest_point
        
        return OptimizationResult(
            optimized_path=optimized_path,
            original_distance=0.0,
            optimized_distance=0.0,
            improvement_percentage=0.0,
            computation_time=0.0,
            iterations_used=1,
            success=True
        )

    def _two_opt_optimization(self, waypoints: List[Point], 
                            config: OptimizationConfig) -> OptimizationResult:
        """
        Optimisation 2-opt pour TSP
        
        Args:
            waypoints: Points à optimiser
            config: Configuration
            
        Returns:
            Résultat de l'optimisation
        """
        if len(waypoints) <= 3:
            return OptimizationResult(
                optimized_path=waypoints,
                original_distance=0.0,
                optimized_distance=0.0,
                improvement_percentage=0.0,
                computation_time=0.0,
                iterations_used=0,
                success=True
            )
        
        tour = waypoints.copy()
        improved = True
        iterations = 0
        
        while improved and iterations < config.max_iterations:
            improved = False
            iterations += 1
            
            for i in range(1, len(tour) - 2):
                for j in range(i + 1, len(tour)):
                    if j - i == 1:
                        continue  # Skip adjacent edges
                    
                    # Calcul de l'amélioration potentielle
                    old_distance = (
                        self._distance_between_points(tour[i-1], tour[i]) +
                        self._distance_between_points(tour[j-1], tour[j])
                    )
                    
                    new_distance = (
                        self._distance_between_points(tour[i-1], tour[j-1]) +
                        self._distance_between_points(tour[i], tour[j])
                    )
                    
                    if new_distance < old_distance:
                        # Inverser le segment
                        tour[i:j] = tour[i:j][::-1]
                        improved = True
        
        return OptimizationResult(
            optimized_path=tour,
            original_distance=0.0,
            optimized_distance=0.0,
            improvement_percentage=0.0,
            computation_time=0.0,
            iterations_used=iterations,
            success=True
        )

    def _simulated_annealing_optimization(self, waypoints: List[Point], 
                                        config: OptimizationConfig) -> OptimizationResult:
        """
        Optimisation par recuit simulé
        
        Args:
            waypoints: Points à optimiser
            config: Configuration
            
        Returns:
            Résultat de l'optimisation
        """
        if len(waypoints) <= 2:
            return OptimizationResult(
                optimized_path=waypoints,
                original_distance=0.0,
                optimized_distance=0.0,
                improvement_percentage=0.0,
                computation_time=0.0,
                iterations_used=0,
                success=True
            )
        
        current_tour = waypoints.copy()
        current_distance = self._calculate_path_distance(current_tour)
        best_tour = current_tour.copy()
        best_distance = current_distance
        
        # Paramètres du recuit
        temperature = 1000.0
        cooling_rate = 0.995
        min_temperature = 1.0
        
        iterations = 0
        
        while temperature > min_temperature and iterations < config.max_iterations:
            # Génération d'un voisin (échange de deux points)
            new_tour = current_tour.copy()
            if len(new_tour) > 3:
                i, j = random.sample(range(1, len(new_tour) - 1), 2)
                new_tour[i], new_tour[j] = new_tour[j], new_tour[i]
            
            new_distance = self._calculate_path_distance(new_tour)
            
            # Critère d'acceptation
            delta = new_distance - current_distance
            if delta < 0 or random.random() < math.exp(-delta / temperature):
                current_tour = new_tour
                current_distance = new_distance
                
                if current_distance < best_distance:
                    best_tour = current_tour.copy()
                    best_distance = current_distance
            
            temperature *= cooling_rate
            iterations += 1
        
        return OptimizationResult(
            optimized_path=best_tour,
            original_distance=0.0,
            optimized_distance=0.0,
            improvement_percentage=0.0,
            computation_time=0.0,
            iterations_used=iterations,
            success=True
        )

    def _smooth_path(self, waypoints: List[Point], 
                    config: OptimizationConfig) -> List[Point]:
        """
        Lisse une trajectoire pour réduire les changements brusques
        
        Args:
            waypoints: Points à lisser
            config: Configuration
            
        Returns:
            Points lissés
        """
        if len(waypoints) <= 2:
            return waypoints
        
        smoothed = [waypoints[0]]  # Garder le premier point
        
        for i in range(1, len(waypoints) - 1):
            prev_point = waypoints[i - 1]
            current_point = waypoints[i]
            next_point = waypoints[i + 1]
            
            # Lissage par moyenne pondérée
            smoothed_x = (
                (1 - config.smoothing_factor) * current_point.x +
                config.smoothing_factor * (prev_point.x + next_point.x) / 2
            )
            smoothed_y = (
                (1 - config.smoothing_factor) * current_point.y +
                config.smoothing_factor * (prev_point.y + next_point.y) / 2
            )
            smoothed_z = current_point.z  # Garder l'altitude
            
            smoothed.append(Point(x=smoothed_x, y=smoothed_y, z=smoothed_z))
        
        smoothed.append(waypoints[-1])  # Garder le dernier point
        return smoothed

    def _reduce_waypoints(self, waypoints: List[Point], 
                         config: OptimizationConfig) -> List[Point]:
        """
        Réduit le nombre de waypoints en éliminant les redondants
        
        Args:
            waypoints: Points à réduire
            config: Configuration
            
        Returns:
            Points réduits
        """
        if len(waypoints) <= 2:
            return waypoints
        
        reduced = [waypoints[0]]
        
        for i in range(1, len(waypoints) - 1):
            current = waypoints[i]
            last_kept = reduced[-1]
            next_point = waypoints[i + 1]
            
            # Vérifier si le point est nécessaire
            direct_distance = self._distance_between_points(last_kept, next_point)
            path_distance = (
                self._distance_between_points(last_kept, current) +
                self._distance_between_points(current, next_point)
            )
            
            # Garder le point si la différence est significative
            if abs(path_distance - direct_distance) > config.waypoint_reduction_threshold:
                reduced.append(current)
        
        reduced.append(waypoints[-1])
        return reduced

    def _tournament_selection(self, population: List[List[Point]], 
                            fitness_scores: List[float]) -> List[Point]:
        """Sélection par tournoi pour algorithme génétique"""
        tournament_size = 3
        tournament_indices = random.sample(range(len(population)), tournament_size)
        best_idx = min(tournament_indices, key=lambda i: fitness_scores[i])
        return population[best_idx].copy()

    def _crossover(self, parent1: List[Point], parent2: List[Point]) -> List[Point]:
        """Croisement pour algorithme génétique (Order Crossover)"""
        if len(parent1) <= 2:
            return parent1.copy()
        
        size = len(parent1)
        start, end = sorted(random.sample(range(size), 2))
        
        child = [None] * size
        child[start:end] = parent1[start:end]
        
        remaining = [item for item in parent2 if item not in child]
        j = 0
        for i in range(size):
            if child[i] is None:
                child[i] = remaining[j]
                j += 1
        
        return child

    def _mutate(self, individual: List[Point]) -> List[Point]:
        """Mutation pour algorithme génétique"""
        if len(individual) <= 2:
            return individual
        
        mutated = individual.copy()
        i, j = random.sample(range(len(mutated)), 2)
        mutated[i], mutated[j] = mutated[j], mutated[i]
        return mutated

    def _distance_between_points(self, p1: Point, p2: Point) -> float:
        """Calcule la distance euclidienne entre deux points"""
        return math.sqrt(
            (p1.x - p2.x)**2 + (p1.y - p2.y)**2 + (p1.z - p2.z)**2
        )

    def _calculate_path_distance(self, waypoints: List[Point]) -> float:
        """Calcule la distance totale d'un chemin"""
        if len(waypoints) < 2:
            return 0.0
        
        total_distance = 0.0
        for i in range(1, len(waypoints)):
            total_distance += self._distance_between_points(waypoints[i-1], waypoints[i])
        
        return total_distance

    def _update_statistics(self, result: OptimizationResult):
        """Met à jour les statistiques d'optimisation"""
        self._stats['optimizations_performed'] += 1
        
        if result.success and result.improvement_percentage > 0:
            distance_saved = result.original_distance - result.optimized_distance
            self._stats['total_distance_saved'] += distance_saved
            
            # Moyenne mobile de l'amélioration
            current_avg = self._stats['average_improvement']
            n = self._stats['optimizations_performed']
            self._stats['average_improvement'] = (
                (current_avg * (n - 1) + result.improvement_percentage) / n
            )
        
        self._stats['computation_time_total'] += result.computation_time

    def get_optimization_statistics(self) -> Dict[str, Any]:
        """Retourne les statistiques d'optimisation"""
        return self._stats.copy()

    def reset_statistics(self):
        """Remet à zéro les statistiques"""
        self._stats = {
            'optimizations_performed': 0,
            'total_distance_saved': 0.0,
            'average_improvement': 0.0,
            'computation_time_total': 0.0
        }


# Import manquant ajouté
import time
