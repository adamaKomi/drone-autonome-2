#!/usr/bin/env python3

"""
Script de réglage automatique des paramètres PID

Utilise des algorithmes d'optimisation pour ajuster automatiquement
les gains PID du contrôleur de position pour des performances optimales.

Auteur: Adama Komi  
Version: 1.1.0
"""

import argparse
import time
import json
import numpy as np
import sys
import math
from typing import Dict, List, Tuple, Optional, Any
from dataclasses import dataclass
from enum import Enum

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import Point, PoseStamped, Twist
from std_msgs.msg import String, Header, Float32
from sensor_msgs.msg import NavSatFix, Imu
from mavros_msgs.msg import State, PositionTarget
from mavros_msgs.srv import CommandBool, SetMode, ParamSet, ParamGet


class TuningMethod(Enum):
    """Méthodes de réglage PID"""
    ZIEGLER_NICHOLS = "ziegler_nichols"
    GENETIC_ALGORITHM = "genetic_algorithm"
    PARTICLE_SWARM = "particle_swarm"
    MANUAL_STEP = "manual_step"


@dataclass
class PIDGains:
    """Structure pour les gains PID"""
    kp: float
    ki: float
    kd: float
    
    def to_dict(self) -> Dict[str, float]:
        return {'kp': self.kp, 'ki': self.ki, 'kd': self.kd}
        
    @classmethod
    def from_dict(cls, data: Dict[str, float]):
        return cls(kp=data['kp'], ki=data['ki'], kd=data['kd'])
    
    def __str__(self):
        return f"Kp={self.kp:.4f}, Ki={self.ki:.4f}, Kd={self.kd:.4f}"


@dataclass
class TuningResult:
    """Résultat de réglage PID"""
    gains: PIDGains
    performance_metrics: Dict[str, Any]
    success: bool
    method: str
    iterations: int
    tuning_time: float
    

class PIDTuner(Node):
    """Réglage automatique des PID pour drone navigation"""
    
    def __init__(self):
        super().__init__('pid_tuner')
        
        # Configuration QoS
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            depth=10
        )
        
        self.current_position = None
        self.current_velocity = None
        self.mavros_state = None
        self.imu_data = None
        
        self.position_history = []
        self.velocity_history = []
        self.error_history = []
        self.command_history = []
        
        # Subscribers
        self.position_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', 
            self.position_callback, qos_profile)
        self.velocity_sub = self.create_subscription(
            Twist, '/mavros/local_position/velocity_local',
            self.velocity_callback, qos_profile)
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, qos_profile)
        self.imu_sub = self.create_subscription(
            Imu, '/mavros/imu/data', self.imu_callback, qos_profile)
            
        # Publishers
        self.position_target_pub = self.create_publisher(
            PositionTarget, '/mavros/setpoint_position/local', 10)
        self.status_pub = self.create_publisher(
            String, '/drone_nav/pid_tuning_status', 10)
        self.performance_pub = self.create_publisher(
            Float32, '/drone_nav/performance_score', 10)
            
        # Services
        self.arm_service = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_service = self.create_client(SetMode, '/mavros/set_mode')
        self.param_set_service = self.create_client(ParamSet, '/mavros/param/set')
        self.param_get_service = self.create_client(ParamGet, '/mavros/param/get')
        
        # Configuration par défaut
        self.default_gains = PIDGains(kp=1.0, ki=0.1, kd=0.05)
        self.current_gains = self.default_gains
        
        # Paramètres de tuning
        self.test_duration = 15.0  # secondes
        self.settling_threshold = 0.2  # mètres
        self.max_overshoot_percent = 20.0  # %
        
        self.get_logger().info("PID Tuner initialized")
        
    def position_callback(self, msg: PoseStamped):
        """Callback position locale"""
        self.current_position = msg.pose.position
        
    def velocity_callback(self, msg: Twist):
        """Callback vélocité locale"""
        self.current_velocity = msg
        
    def state_callback(self, msg: State):
        """Callback état MAVROS"""
        self.mavros_state = msg
        
    def imu_callback(self, msg: Imu):
        """Callback données IMU"""
        self.imu_data = msg
        
    def wait_for_connections(self, timeout: float = 30.0) -> bool:
        """Attendre connexions MAVROS
        
        Args:
            timeout: Délai maximum d'attente en secondes
            
        Returns:
            bool: True si toutes les connexions sont établies
        """
        self.get_logger().info("En attente des connexions MAVROS...")
        
        start_time = time.time()
        while time.time() - start_time < timeout:
            if (self.mavros_state is not None and 
                self.current_position is not None and
                self.current_velocity is not None):
                self.get_logger().info("Toutes les connexions sont établies")
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
            
        self.get_logger().error("Timeout lors de l'attente des connexions")
        return False
        
    def set_pid_gains(self, gains: PIDGains) -> bool:
        """Définir les gains PID sur le contrôleur de vol
        
        Args:
            gains: Gains PID à appliquer
            
        Returns:
            bool: True si la configuration a réussi
        """
        try:
            # Dans une implémentation réelle, on utiliserait les services param
            # Pour cette démo, on simule la configuration
            self.current_gains = gains
            self.get_logger().info(f"Gains PID définis: {gains}")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Erreur configuration PID: {e}")
            return False
            
    def send_position_target(self, position: Point) -> bool:
        """Envoyer une consigne de position
        
        Args:
            position: Position cible
            
        Returns:
            bool: True si la commande a été envoyée
        """
        try:
            msg = PositionTarget()
            msg.header = Header()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.header.frame_id = 'map'
            
            msg.type_mask = PositionTarget.IGNORE_VELOCITY | PositionTarget.IGNORE_ACCELERATION
            msg.position = position
            msg.yaw = 0.0
            
            self.position_target_pub.publish(msg)
            return True
            
        except Exception as e:
            self.get_logger().error(f"Erreur envoi consigne: {e}")
            return False
            
    def evaluate_pid_performance(self, gains: PIDGains, test_duration: float = None) -> Dict[str, Any]:
        """Évaluer performance des gains PID
        
        Args:
            gains: Gains PID à tester
            test_duration: Durée du test en secondes
            
        Returns:
            Dict: Métriques de performance
        """
        if test_duration is None:
            test_duration = self.test_duration
            
        self.get_logger().info(f"Test des gains PID: {gains}")
        
        # Configurer les gains
        if not self.set_pid_gains(gains):
            return {'error': 'Échec configuration des gains'}
        
        # Définir la position cible pour le test
        if not self.current_position:
            return {'error': 'Position actuelle non disponible'}
            
        start_pos = Point(
            x=self.current_position.x,
            y=self.current_position.y,
            z=self.current_position.z
        )
        
        # Déplacement de 5m en X, 3m en Y, 2m en Z
        target_pos = Point(
            x=start_pos.x + 5.0,
            y=start_pos.y + 3.0, 
            z=start_pos.z + 2.0
        )
        
        # Réinitialiser les historiques
        self.position_history.clear()
        self.velocity_history.clear()
        self.error_history.clear()
        self.command_history.clear()
        
        # Envoyer la consigne initiale
        if not self.send_position_target(target_pos):
            return {'error': 'Échec envoi consigne'}
        
        # Boucle de test
        start_time = time.time()
        dt = 0.05  # 20Hz
        
        while time.time() - start_time < test_duration:
            if self.current_position:
                # Calculer l'erreur
                error = Point()
                error.x = target_pos.x - self.current_position.x
                error.y = target_pos.y - self.current_position.y
                error.z = target_pos.z - self.current_position.z
                
                error_magnitude = math.sqrt(error.x**2 + error.y**2 + error.z**2)
                
                # Enregistrer les données
                current_time = time.time()
                self.position_history.append({
                    'x': self.current_position.x, 
                    'y': self.current_position.y,
                    'z': self.current_position.z,
                    'timestamp': current_time
                })
                
                self.error_history.append({
                    'x': error.x, 'y': error.y, 'z': error.z,
                    'magnitude': error_magnitude,
                    'timestamp': current_time
                })
                
                # Envoyer périodiquement la consigne
                if len(self.position_history) % 10 == 0:
                    self.send_position_target(target_pos)
                    
            rclpy.spin_once(self, timeout_sec=dt)
            
        # Analyser les performances
        return self._analyze_performance_metrics(target_pos)
        
    def _analyze_performance_metrics(self, target_pos: Point) -> Dict[str, Any]:
        """Analyser métriques de performance
        
        Args:
            target_pos: Position cible
            
        Returns:
            Dict: Métriques de performance détaillées
        """
        if not self.error_history:
            return {'error': 'Aucune donnée de performance collectée'}
            
        # Extraire les erreurs
        errors = [e['magnitude'] for e in self.error_history]
        timestamps = [e['timestamp'] for e in self.error_history]
        
        metrics = {}
        
        # 1. Temps de stabilisation
        settling_time = None
        for i, error in enumerate(errors):
            if error < self.settling_threshold:
                # Vérifier que l'erreur reste sous le seuil
                remaining_errors = errors[i:min(i+20, len(errors))]
                if all(e < self.settling_threshold for e in remaining_errors):
                    settling_time = timestamps[i] - timestamps[0]
                    break
                    
        metrics['settling_time'] = settling_time
        
        # 2. Overshoot
        if self.position_history:
            distances = [
                math.sqrt((p['x'] - target_pos.x)**2 + 
                         (p['y'] - target_pos.y)**2 +
                         (p['z'] - target_pos.z)**2)
                for p in self.position_history
            ]
            max_distance = max(distances) if distances else 0
            target_distance = math.sqrt(5.0**2 + 3.0**2 + 2.0**2)  # Norme du déplacement
            overshoot = max(0, max_distance - target_distance)
            metrics['overshoot_percent'] = (overshoot / target_distance) * 100 if target_distance > 0 else 0
        else:
            metrics['overshoot_percent'] = 0
            
        # 3. Erreur en régime permanent
        if len(errors) > 20:
            steady_errors = errors[-20:]  # Dernières 20 mesures
            metrics['steady_state_error'] = np.mean(steady_errors)
            metrics['steady_state_std'] = np.std(steady_errors)
        else:
            metrics['steady_state_error'] = np.mean(errors) if errors else float('inf')
            metrics['steady_state_std'] = np.std(errors) if errors else 0
            
        # 4. Oscillations
        oscillations = 0
        if len(errors) > 2:
            for i in range(1, len(errors)-1):
                if (errors[i] > errors[i-1] and errors[i] > errors[i+1]) or \
                   (errors[i] < errors[i-1] and errors[i] < errors[i+1]):
                    oscillations += 1
        metrics['oscillations'] = oscillations
        
        # 5. Score de performance global
        settling_penalty = 50.0 if metrics['settling_time'] is None else metrics['settling_time']
        overshoot_penalty = metrics['overshoot_percent'] * 2.0
        sse_penalty = metrics['steady_state_error'] * 100.0
        oscillation_penalty = metrics['oscillations'] * 5.0
        
        performance_score = (
            settling_penalty +
            overshoot_penalty + 
            sse_penalty +
            oscillation_penalty
        )
        
        metrics['performance_score'] = performance_score
        metrics['data_points'] = len(self.error_history)
        
        # Publier le score de performance
        score_msg = Float32()
        score_msg.data = float(performance_score)
        self.performance_pub.publish(score_msg)
        
        return metrics
        
    def tune_ziegler_nichols(self) -> TuningResult:
        """Méthode de Ziegler-Nichols pour réglage PID
        
        Returns:
            TuningResult: Résultat du réglage
        """
        self.get_logger().info("Démarrage réglage Ziegler-Nichols...")
        start_time = time.time()
        
        # Étape 1: Trouver le gain critique Ku
        kp_test = 0.1
        ku = None
        iteration = 0
        
        for iteration in range(15):  # Maximum 15 itérations
            test_gains = PIDGains(kp=kp_test, ki=0.0, kd=0.0)
            metrics = self.evaluate_pid_performance(test_gains, 12.0)
            
            if 'error' in metrics:
                self.get_logger().error(f"Erreur réglage ZN: {metrics['error']}")
                break
                
            # Vérifier les oscillations
            if metrics.get('oscillations', 0) >= 3:  # Oscillations détectées
                ku = kp_test
                self.get_logger().info(f"Gain critique Ku trouvé: {ku:.3f}")
                break
                
            # Augmenter progressivement le gain
            kp_test *= 1.3
            iteration += 1
            
        if ku is None:
            self.get_logger().warning("Gain critique non trouvé, utilisation des gains par défaut")
            final_metrics = self.evaluate_pid_performance(self.default_gains)
            return TuningResult(
                gains=self.default_gains,
                performance_metrics=final_metrics,
                success=False,
                method='ziegler_nichols',
                iterations=iteration,
                tuning_time=time.time() - start_time
            )
            
        # Étape 2: Calculer les gains optimaux selon ZN
        pu = 2.0  # Période d'oscillation estimée (secondes)
        
        optimal_gains = PIDGains(
            kp=0.6 * ku,
            ki=1.2 * ku / pu,
            kd=0.075 * ku * pu
        )
        
        # Test final
        final_metrics = self.evaluate_pid_performance(optimal_gains)
        
        return TuningResult(
            gains=optimal_gains,
            performance_metrics=final_metrics,
            success=True,
            method='ziegler_nichols',
            iterations=iteration + 1,
            tuning_time=time.time() - start_time
        )
        
    def tune_genetic_algorithm(self, population_size: int = 15, generations: int = 8) -> TuningResult:
        """Algorithme génétique pour optimisation PID
        
        Args:
            population_size: Taille de la population
            generations: Nombre de générations
            
        Returns:
            TuningResult: Résultat du réglage
        """
        self.get_logger().info(f"Démarrage algorithme génétique ({generations} générations)...")
        start_time = time.time()
        total_iterations = 0
        
        # Plages de valeurs raisonnables pour les gains PID
        kp_range = (0.1, 5.0)
        ki_range = (0.01, 1.0)
        kd_range = (0.001, 0.5)
        
        # Initialiser la population
        population = []
        for _ in range(population_size):
            gains = PIDGains(
                kp=np.random.uniform(*kp_range),
                ki=np.random.uniform(*ki_range),
                kd=np.random.uniform(*kd_range)
            )
            population.append(gains)
            
        best_gains = None
        best_score = float('inf')
        best_metrics = {}
        
        for generation in range(generations):
            self.get_logger().info(f"Génération {generation + 1}/{generations}")
            
            # Évaluer chaque individu
            fitness_scores = []
            for i, gains in enumerate(population):
                metrics = self.evaluate_pid_performance(gains, 10.0)
                total_iterations += 1
                
                if 'error' in metrics:
                    score = float('inf')
                else:
                    score = metrics.get('performance_score', float('inf'))
                    
                fitness_scores.append(score)
                
                # Suivre le meilleur individu
                if score < best_score:
                    best_score = score
                    best_gains = gains
                    best_metrics = metrics
                    
                self.get_logger().info(f"Individu {i+1}: Score={score:.2f}, Gains={gains}")
                
            # Reproduction pour la prochaine génération
            if generation < generations - 1:
                population = self._genetic_reproduction(population, fitness_scores, 
                                                      kp_range, ki_range, kd_range)
                
        # Évaluation finale
        final_metrics = self.evaluate_pid_performance(best_gains, 15.0) if best_gains else {}
        total_iterations += 1
        
        return TuningResult(
            gains=best_gains or self.default_gains,
            performance_metrics=final_metrics or best_metrics,
            success=best_gains is not None,
            method='genetic_algorithm',
            iterations=total_iterations,
            tuning_time=time.time() - start_time
        )
        
    def _genetic_reproduction(self, population: List[PIDGains], fitness_scores: List[float],
                            kp_range: Tuple[float, float], ki_range: Tuple[float, float], 
                            kd_range: Tuple[float, float]) -> List[PIDGains]:
        """Reproduction génétique avec élitisme, croisement et mutation"""
        new_population = []
        population_size = len(population)
        
        # Élitisme: garder les 20% meilleurs
        elite_size = max(1, population_size // 5)
        elite_indices = np.argsort(fitness_scores)[:elite_size]
        for idx in elite_indices:
            new_population.append(population[idx])
            
        # Remplir le reste de la population
        while len(new_population) < population_size:
            # Sélection des parents
            parent1 = self._tournament_selection(population, fitness_scores)
            parent2 = self._tournament_selection(population, fitness_scores)
            
            # Croisement
            child = self._crossover(parent1, parent2)
            
            # Mutation
            child = self._mutate(child, kp_range, ki_range, kd_range)
            
            new_population.append(child)
            
        return new_population
        
    def _tournament_selection(self, population: List[PIDGains], 
                             fitness_scores: List[float], tournament_size: int = 3) -> PIDGains:
        """Sélection par tournoi"""
        indices = np.random.choice(len(population), tournament_size, replace=False)
        tournament_fitness = [fitness_scores[i] for i in indices]
        winner_idx = indices[np.argmin(tournament_fitness)]
        return population[winner_idx]
        
    def _crossover(self, parent1: PIDGains, parent2: PIDGains) -> PIDGains:
        """Croisement par recombinaison arithmétique"""
        alpha = np.random.random()
        return PIDGains(
            kp=alpha * parent1.kp + (1 - alpha) * parent2.kp,
            ki=alpha * parent1.ki + (1 - alpha) * parent2.ki,
            kd=alpha * parent1.kd + (1 - alpha) * parent2.kd
        )
        
    def _mutate(self, gains: PIDGains, kp_range: Tuple[float, float], 
               ki_range: Tuple[float, float], kd_range: Tuple[float, float],
               mutation_rate: float = 0.2) -> PIDGains:
        """Mutation gaussienne avec limites"""
        if np.random.random() < mutation_rate:
            gains.kp = np.clip(gains.kp * np.random.normal(1.0, 0.15), *kp_range)
        if np.random.random() < mutation_rate:
            gains.ki = np.clip(gains.ki * np.random.normal(1.0, 0.15), *ki_range)
        if np.random.random() < mutation_rate:
            gains.kd = np.clip(gains.kd * np.random.normal(1.0, 0.15), *kd_range)
        return gains
        
    def manual_step_tuning(self) -> TuningResult:
        """Réglage manuel par étapes (mode interactif)"""
        self.get_logger().info("Démarrage réglage manuel...")
        start_time = time.time()
        
        current_gains = self.default_gains
        iterations = 0
        
        print("\n=== Mode Réglage Manuel PID ===")
        print("Commandes disponibles:")
        print("  +kp/-kp : Augmenter/diminuer gain proportionnel")
        print("  +ki/-ki : Augmenter/diminuer gain intégral") 
        print("  +kd/-kd : Augmenter/diminuer gain dérivé")
        print("  test    : Tester les gains actuels")
        print("  save    : Sauvegarder les gains actuels")
        print("  quit    : Quitter le mode manuel")
        print("  help    : Afficher cette aide")
        
        while True:
            print(f"\nGains actuels: {current_gains}")
            command = input("Commande: ").strip().lower()
            
            if command == 'quit':
                break
            elif command == 'test':
                metrics = self.evaluate_pid_performance(current_gains)
                if 'error' not in metrics:
                    print(f"Score performance: {metrics.get('performance_score', 'N/A'):.2f}")
                    print(f"Temps stabilisation: {metrics.get('settling_time', 'N/A'):.1f}s")
                    print(f"Overshoot: {metrics.get('overshoot_percent', 'N/A'):.1f}%")
                    print(f"Erreur régime permanent: {metrics.get('steady_state_error', 'N/A'):.3f}m")
                iterations += 1
            elif command == '+kp':
                current_gains.kp *= 1.2
                print(f"Kp augmenté à: {current_gains.kp:.4f}")
            elif command == '-kp':
                current_gains.kp /= 1.2
                print(f"Kp diminué à: {current_gains.kp:.4f}")
            elif command == '+ki':
                current_gains.ki *= 1.2
                print(f"Ki augmenté à: {current_gains.ki:.4f}")
            elif command == '-ki':
                current_gains.ki /= 1.2
                print(f"Ki diminué à: {current_gains.ki:.4f}")
            elif command == '+kd':
                current_gains.kd *= 1.2
                print(f"Kd augmenté à: {current_gains.kd:.4f}")
            elif command == '-kd':
                current_gains.kd /= 1.2
                print(f"Kd diminué à: {current_gains.kd:.4f}")
            elif command == 'help':
                print("Commandes: +kp, -kp, +ki, -ki, +kd, -kd, test, save, quit, help")
            else:
                print("Commande non reconnue. Tapez 'help' pour l'aide.")
                
        # Test final
        final_metrics = self.evaluate_pid_performance(current_gains)
        iterations += 1
        
        return TuningResult(
            gains=current_gains,
            performance_metrics=final_metrics,
            success=True,
            method='manual_step',
            iterations=iterations,
            tuning_time=time.time() - start_time
        )
        
    def save_tuning_results(self, result: TuningResult, filename: str):
        """Sauvegarder résultats de réglage
        
        Args:
            result: Résultat à sauvegarder
            filename: Nom du fichier de sortie
        """
        data = {
            'timestamp': time.strftime('%Y-%m-%d %H:%M:%S'),
            'method': result.method,
            'success': result.success,
            'iterations': result.iterations,
            'tuning_time_seconds': result.tuning_time,
            'gains': result.gains.to_dict(),
            'performance_metrics': result.performance_metrics,
            'performance_score': result.performance_metrics.get('performance_score', float('inf'))
        }
        
        try:
            with open(filename, 'w') as f:
                json.dump(data, f, indent=2, ensure_ascii=False)
            self.get_logger().info(f"Résultats sauvegardés dans: {filename}")
        except Exception as e:
            self.get_logger().error(f"Erreur sauvegarde résultats: {e}")


def main():
    """Point d'entrée principal"""
    parser = argparse.ArgumentParser(description='Réglage automatique PID pour navigation drone')
    parser.add_argument('--method', choices=['zn', 'ga', 'manual'], 
                       default='zn', help='Méthode de réglage (zn=Ziegler-Nichols, ga=Algorithme Génétique)')
    parser.add_argument('--output', '-o', help='Fichier de sortie pour les résultats')
    parser.add_argument('--generations', type=int, default=8,
                       help='Nombre de générations pour algorithme génétique')
    parser.add_argument('--population', type=int, default=15,
                       help='Taille de population pour algorithme génétique')
    parser.add_argument('--duration', type=float, default=15.0,
                       help='Durée des tests en secondes')
    
    args = parser.parse_args()
    
    rclpy.init()
    tuner = PIDTuner()
    
    try:
        if not tuner.wait_for_connections():
            tuner.get_logger().error("Échec connexion MAVROS")
            return 1
            
        # Configurer la durée des tests
        if args.duration:
            tuner.test_duration = args.duration
            
        # Exécuter la méthode de réglage
        if args.method == 'zn':
            result = tuner.tune_ziegler_nichols()
        elif args.method == 'ga':
            result = tuner.tune_genetic_algorithm(args.population, args.generations)
        elif args.method == 'manual':
            result = tuner.manual_step_tuning()
        else:
            tuner.get_logger().error(f"Méthode inconnue: {args.method}")
            return 1
            
        # Afficher les résultats
        print(f"\n=== RÉSULTATS RÉGLAGE PID ({result.method.upper()}) ===")
        print(f"Succès: {'OUI' if result.success else 'NON'}")
        print(f"Itérations: {result.iterations}")
        print(f"Temps de réglage: {result.tuning_time:.1f}s")
        print(f"Gains optimaux: {result.gains}")
        
        if 'performance_score' in result.performance_metrics:
            print(f"Score de performance: {result.performance_metrics['performance_score']:.2f}")
        if 'settling_time' in result.performance_metrics and result.performance_metrics['settling_time']:
            print(f"Temps de stabilisation: {result.performance_metrics['settling_time']:.1f}s")
        if 'overshoot_percent' in result.performance_metrics:
            print(f"Overshoot: {result.performance_metrics['overshoot_percent']:.1f}%")
        if 'steady_state_error' in result.performance_metrics:
            print(f"Erreur régime permanent: {result.performance_metrics['steady_state_error']:.3f}m")
            
        # Sauvegarder les résultats
        if args.output:
            tuner.save_tuning_results(result, args.output)
            
        return 0 if result.success else 1
        
    except KeyboardInterrupt:
        tuner.get_logger().info("Réglage interrompu par l'utilisateur")
        return 1
    except Exception as e:
        tuner.get_logger().error(f"Erreur lors du réglage: {e}")
        return 1
    finally:
        tuner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())