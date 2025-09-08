#!/usr/bin/env python3

"""
Contrôleur de position PID avancé pour drone autonome

Implémente des contrôleurs PID multi-axes avec anti-windup,
modes de contrôle adaptatifs et approche de précision pour
missions de pollinisation.

Exemples d'utilisation:
    controller = PositionController(node)
    controller.set_control_mode(ControlMode.PRECISION_APPROACH)
    controller.set_target_position(target_point)
    output = controller.update(current_position, target_position, current_velocity)

Auteur: Adama Komi
Version: 1.2.0
"""
import numpy as np
from dataclasses import dataclass
from geometry_msgs.msg import Point, Vector3, Quaternion
from mavros_msgs.msg import PositionTarget, State
from drone_msgs.msg import Waypoint, Velocity3D

import time
import math
from typing import Dict, Any, Optional, Tuple
from enum import Enum

import numpy as np
from geometry_msgs.msg import Point, Vector3, Quaternion
from mavros_msgs.msg import PositionTarget, State


class ControlMode(Enum):
    """Modes de contrôle disponibles"""
    POSITION_HOLD = "position_hold"
    GUIDED = "guided"
    FOLLOW_TRAJECTORY = "follow_trajectory"
    PRECISION_APPROACH = "precision_approach"
    EMERGENCY_LAND = "emergency_land"


@dataclass
class PIDGains:
    """Gains PID pour un axe
    
    Attributes:
        kp: Gain proportionnel
        ki: Gain intégral
        kd: Gain dérivé
        max_output: Sortie maximale
        max_windup: Limite anti-windup
        deadband: Zone morte
    """
    kp: float = 1.0  # Gain proportionnel
    ki: float = 0.1  # Gain intégral
    kd: float = 0.05 # Gain dérivé
    max_output: float = 10.0    # Sortie maximale
    max_windup: float = 5.0     # Anti-windup maximum
    deadband: float = 0.1       # Zone morte


@dataclass
class ControlState:
    """État du contrôleur pour un axe
    
    Attributes:
        setpoint: Consigne actuelle
        feedback: Retour d'état actuel
        error: Erreur actuelle
        error_prev: Erreur précédente
        error_integral: Intégrale de l'erreur
        error_derivative: Dérivée de l'erreur
        output: Sortie du contrôleur
        last_time: Dernière mise à jour en secondes
    """
    setpoint: float = 0.0       # Consigne
    feedback: float = 0.0       # Retour
    error: float = 0.0          # Erreur actuelle
    error_prev: float = 0.0     # Erreur précédente
    error_integral: float = 0.0 # Intégrale de l'erreur
    error_derivative: float = 0.0 # Dérivée de l'erreur
    output: float = 0.0         # Sortie du contrôleur
    last_time: float = 0.0      # Dernière mise à jour


@dataclass
class VelocityLimits:
    """Limites de vitesse et accélération
    
    Attributes:
        max_velocity_xy: Vitesse maximale horizontale en m/s
        max_velocity_z: Vitesse maximale verticale en m/s
        max_acceleration: Accélération maximale en m/s²
        max_jerk: Jerk maximal en m/s³
    """
    max_velocity_xy: float = 10.0   # m/s
    max_velocity_z: float = 5.0     # m/s
    max_acceleration: float = 5.0   # m/s²
    max_jerk: float = 10.0          # m/s³


@dataclass
class ControlOutput:
    """Sortie du contrôleur de position
    
    Attributes:
        position_target: Position cible en mètres
        velocity_target: Vitesse cible en m/s
        acceleration_target: Accélération cible en m/s²
        yaw_target: Angle de cap cible en radians
        success: Indique si le calcul a réussi
        message: Message d'information ou d'erreur
    """
    position_target: Point
    velocity_target: Vector3
    acceleration_target: Vector3
    yaw_target: float
    success: bool = True
    message: str = ""


class PIDController:
    """
    Contrôleur PID simple pour un axe
    
    Implémente un contrôleur PID avec anti-windup et limitation de sortie
    pour contrôle de position, vitesse ou autres variables continues.
    """
    
    def __init__(self, kp: float = 1.0, ki: float = 0.0, kd: float = 0.0,
                 output_limits: Tuple[float, float] = (-float('inf'), float('inf')),
                 integral_limits: Tuple[float, float] = (-float('inf'), float('inf')),
                 derivative_filter: float = 0.0):
        """
        Initialise le contrôleur PID
        
        Args:
            kp: Gain proportionnel
            ki: Gain intégral
            kd: Gain dérivé
            output_limits: Limites de sortie (min, max)
            integral_limits: Limites de l'intégrateur (min, max)
            derivative_filter: Coefficient de filtrage dérivé (0-1)
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_limits = output_limits
        self.integral_limits = integral_limits
        self.derivative_filter = derivative_filter
        
        # État interne
        self._prev_error = 0.0
        self._integral = 0.0
        self._prev_derivative = 0.0
        self._prev_time = None
        
    def reset(self):
        """Remet à zéro l'état interne du contrôleur"""
        self._prev_error = 0.0
        self._integral = 0.0
        self._prev_derivative = 0.0
        self._prev_time = None
        
    def update(self, error: float, dt: float) -> float:
        """
        Met à jour le contrôleur PID
        
        Args:
            error: Erreur (setpoint - measurement)
            dt: Intervalle de temps depuis la dernière mise à jour en secondes
            
        Returns:
            Sortie du contrôleur
            
        Raises:
            ValueError: Si l'intervalle de temps est invalide
        """
        if dt <= 0.0:
            raise ValueError("Intervalle de temps doit être positif")
            
        # Terme proportionnel
        proportional = self.kp * error
        
        # Terme intégral avec anti-windup
        self._integral += error * dt
        self._integral = np.clip(self._integral, self.integral_limits[0], self.integral_limits[1])
        integral = self.ki * self._integral
        
        # Terme dérivé avec filtrage
        derivative = (error - self._prev_error) / dt
        if self.derivative_filter > 0.0:
            derivative = (self.derivative_filter * self._prev_derivative + 
                         (1.0 - self.derivative_filter) * derivative)
        derivative_term = self.kd * derivative
        
        # Sortie totale
        output = proportional + integral + derivative_term
        
        # Application des limites de sortie
        output = np.clip(output, self.output_limits[0], self.output_limits[1])
        
        # Sauvegarde pour la prochaine itération
        self._prev_error = error
        self._prev_derivative = derivative
        
        return output
        
    def set_gains(self, kp: float, ki: float, kd: float):
        """
        Modifie les gains du contrôleur
        
        Args:
            kp: Nouveau gain proportionnel
            ki: Nouveau gain intégral
            kd: Nouveau gain dérivé
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        
    def get_state(self) -> Dict[str, float]:
        """
        Retourne l'état interne du contrôleur
        
        Returns:
            Dictionnaire contenant l'état interne
        """
        return {
            'integral': self._integral,
            'derivative': self._prev_derivative,
            'prev_error': self._prev_error
        }


class PositionController:
    """
    Contrôleur de position PID avancé multi-axes
    
    Fournit un contrôle de position précis avec:
    - Contrôleurs PID séparés pour X, Y, Z
    - Anti-windup et limitation de sortie
    - Modes de contrôle adaptatifs
    - Approche de précision pour pollinisation
    - Gestion des contraintes dynamiques
    
    Args:
        logger: Logger ROS2 pour la journalisation
        config: Configuration du contrôleur avec paramètres par défaut
    """
    
    def __init__(self, node):
        """
        Initialise le contrôleur de position
        
        Args:
            node: Nœud ROS2 NavigationNode
        """
        self._node = node
        self._logger = node.get_logger()
        
        # Configuration depuis les paramètres du nœud
        self._config = self._load_config_from_node()
        
        # Configuration des gains PID
        self._load_pid_gains()
        
        # États des contrôleurs
        self._position_state_x = ControlState()
        self._position_state_y = ControlState()
        self._position_state_z = ControlState()
        
        self._velocity_state_x = ControlState()
        self._velocity_state_y = ControlState()
        self._velocity_state_z = ControlState()
        
        # Mode de contrôle actuel
        self._control_mode = ControlMode.POSITION_HOLD
        
        # Limites de vitesse et accélération
        self._velocity_limits = self._load_velocity_limits()
        
        # Position et vitesse actuelles
        self._current_position: Optional[Point] = None
        self._current_velocity: Optional[Vector3] = None
        self._target_position: Optional[Point] = None
        
        # Variables pour l'approche de précision
        self._precision_mode = False
        self._precision_radius = 0.5  # m
        self._approach_speed = 1.0    # m/s
        self._stabilization_time = 2.0 # s
        
        # Filtrage et lissage
        self._velocity_filter_alpha = 0.8
        self._filtered_velocity = Vector3()
        
        # Statistiques
        self._stats = {
            'control_cycles': 0,
            'average_error': 0.0,
            'max_error': 0.0,
            'convergence_time': 0.0
        }
        
        # Tolérance pour waypoint atteint
        self._waypoint_tolerance_xy = 0.5  # m
        self._waypoint_tolerance_z = 0.3   # m
        
        self._logger.info("PositionController initialisé")

    def _load_config_from_node(self) -> Dict[str, Any]:
        """Charge la configuration depuis les paramètres du nœud"""
        return {
            'pid_gains': {
                'position': self._node.pid_position,
                'velocity': self._node.pid_velocity
            },
            'limits': self._node.limits
        }

    def _load_pid_gains(self):
        """Charge les gains PID depuis la configuration"""
        pid_config = self._config.get('pid_gains', {})
        
        # Gains de position
        pos_config = pid_config.get('position', {})
        self._position_gains_xy = PIDGains(
            kp=pos_config.get('p_xy', 1.0),
            ki=pos_config.get('i_xy', 0.1),
            kd=pos_config.get('d_xy', 0.05),
            max_output=pos_config.get('max_output_xy', 10.0),
            max_windup=pos_config.get('max_windup_xy', 5.0),
            deadband=pos_config.get('deadband_xy', 0.1)
        )
        
        self._position_gains_z = PIDGains(
            kp=pos_config.get('p_z', 1.5),
            ki=pos_config.get('i_z', 0.2),
            kd=pos_config.get('d_z', 0.1),
            max_output=pos_config.get('max_output_z', 5.0),
            max_windup=pos_config.get('max_windup_z', 2.0),
            deadband=pos_config.get('deadband_z', 0.05)
        )
        
        # Gains de vitesse
        vel_config = pid_config.get('velocity', {})
        self._velocity_gains_xy = PIDGains(
            kp=vel_config.get('p_xy', 0.8),
            ki=vel_config.get('i_xy', 0.05),
            kd=vel_config.get('d_xy', 0.02),
            max_output=vel_config.get('max_output_xy', 5.0),
            max_windup=vel_config.get('max_windup_xy', 2.5),
            deadband=vel_config.get('deadband_xy', 0.05)
        )
        
        self._velocity_gains_z = PIDGains(
            kp=vel_config.get('p_z', 1.0),
            ki=vel_config.get('i_z', 0.1),
            kd=vel_config.get('d_z', 0.05),
            max_output=vel_config.get('max_output_z', 3.0),
            max_windup=vel_config.get('max_windup_z', 1.5),
            deadband=vel_config.get('deadband_z', 0.02)
        )

    def _load_velocity_limits(self) -> VelocityLimits:
        """Charge les limites de vitesse depuis la configuration"""
        limits_config = self._config.get('limits', {})
        
        return VelocityLimits(
            max_velocity_xy=limits_config.get('max_velocity_xy', 10.0),
            max_velocity_z=limits_config.get('max_velocity_z', 5.0),
            max_acceleration=limits_config.get('max_acceleration', 5.0),
            max_jerk=limits_config.get('max_jerk', 10.0)
        )

    def set_control_mode(self, mode: ControlMode):
        """
        Définit le mode de contrôle
        
        Args:
            mode: Nouveau mode de contrôle
        """
        if mode != self._control_mode:
            self._logger.info(f"Changement de mode: {self._control_mode.value} -> {mode.value}")
            self._control_mode = mode
            
            # Réinitialisation des intégrales lors du changement de mode
            self._reset_integral_terms()

    def set_target_position(self, target: Point):
        """
        Définit la position cible
        
        Args:
            target: Position cible (Point avec x, y, z en mètres)
        """
        self._target_position = target
        
        # Mode précision si on s'approche d'une cible de pollinisation
        if self._control_mode == ControlMode.PRECISION_APPROACH:
            self._precision_mode = True

    def update_current_state(self, position: Point, velocity: Optional[Vector3] = None):
        """
        Met à jour l'état actuel du drone
        
        Args:
            position: Position actuelle (Point avec x, y, z en mètres)
            velocity: Vitesse actuelle (Vector3 avec x, y, z en m/s, optionnelle)
        """
        self._current_position = position
        
        if velocity:
            # Filtrage de la vitesse pour réduire le bruit
            self._filtered_velocity.x = (self._velocity_filter_alpha * self._filtered_velocity.x + 
                                       (1 - self._velocity_filter_alpha) * velocity.x)
            self._filtered_velocity.y = (self._velocity_filter_alpha * self._filtered_velocity.y + 
                                       (1 - self._velocity_filter_alpha) * velocity.y)
            self._filtered_velocity.z = (self._velocity_filter_alpha * self._filtered_velocity.z + 
                                       (1 - self._velocity_filter_alpha) * velocity.z)
            self._current_velocity = self._filtered_velocity

    def update(self, current_position: Point, target_position: Point, 
               current_velocity: Optional[Vector3] = None) -> ControlOutput:
        """
        Met à jour le contrôleur et calcule la sortie
        
        Args:
            current_position: Position actuelle (Point avec x, y, z en mètres)
            target_position: Position cible (Point avec x, y, z en mètres)
            current_velocity: Vitesse actuelle (Vector3 avec x, y, z en m/s, optionnelle)
            
        Returns:
            Sortie du contrôleur
            
        Raises:
            RuntimeError: Si une erreur survient pendant le calcul
        """
        current_time = time.time()
        
        try:
            # Mise à jour de l'état
            self.update_current_state(current_position, current_velocity)
            self.set_target_position(target_position)
            
            # Calcul de la sortie selon le mode
            if self._control_mode == ControlMode.PRECISION_APPROACH:
                return self._precision_approach_control(current_time)
            elif self._control_mode == ControlMode.FOLLOW_TRAJECTORY:
                return self._trajectory_following_control(current_time)
            elif self._control_mode == ControlMode.EMERGENCY_LAND:
                return self._emergency_landing_control(current_time)
            else:
                return self._position_hold_control(current_time)
                
        except ValueError as e:
            self._logger.error(f"Erreur de valeur dans le contrôleur: {e}")
            raise
        except RuntimeError as e:
            self._logger.error(f"Erreur d'exécution dans le contrôleur: {e}")
            raise
        except Exception as e:
            error_msg = f"Erreur inattendue dans le contrôleur: {e}"
            self._logger.error(error_msg)
            return ControlOutput(
                position_target=current_position,
                velocity_target=Vector3(),
                acceleration_target=Vector3(),
                yaw_target=0.0,
                success=False,
                message=error_msg
            )

    def _position_hold_control(self, current_time: float) -> ControlOutput:
        """
        Contrôle de maintien de position standard
        
        Args:
            current_time: Temps actuel en secondes
            
        Returns:
            Sortie du contrôleur
            
        Raises:
            RuntimeError: Si la position ou la cible est manquante
        """
        if not self._current_position or not self._target_position:
            raise RuntimeError("Position ou cible manquante pour le contrôle de position")
        
        # Contrôle PID de position pour chaque axe
        vel_x = self._compute_position_pid(
            self._target_position.x, self._current_position.x,
            self._position_state_x, self._position_gains_xy, current_time
        )
        
        vel_y = self._compute_position_pid(
            self._target_position.y, self._current_position.y,
            self._position_state_y, self._position_gains_xy, current_time
        )
        
        vel_z = self._compute_position_pid(
            self._target_position.z, self._current_position.z,
            self._position_state_z, self._position_gains_z, current_time
        )
        
        # Limitation des vitesses
        velocity_target = self._limit_velocity(Vector3(x=vel_x, y=vel_y, z=vel_z))
        
        # Calcul de l'accélération cible (dérivée de la vitesse)
        acceleration_target = self._compute_acceleration_target(velocity_target, current_time)
        
        # Mise à jour des statistiques
        self._update_statistics()
        
        return ControlOutput(
            position_target=self._target_position,
            velocity_target=velocity_target,
            acceleration_target=acceleration_target,
            yaw_target=0.0,  # Pas de contrôle de yaw pour l'instant
            success=True,
            message="Contrôle de position actif"
        )

    def _precision_approach_control(self, current_time: float) -> ControlOutput:
        """
        Contrôle d'approche de précision pour pollinisation
        
        Args:
            current_time: Temps actuel en secondes
            
        Returns:
            Sortie du contrôleur optimisée pour la précision
            
        Raises:
            RuntimeError: Si la position ou la cible est manquante
        """
        if not self._current_position or not self._target_position:
            raise RuntimeError("Position ou cible manquante pour l'approche de précision")
        
        # Calcul de la distance à la cible
        distance = self._calculate_distance_to_target()
        
        # Ajustement des gains pour la précision
        precision_gains_xy = PIDGains(
            kp=self._position_gains_xy.kp * 1.5,  # Gains plus élevés pour précision
            ki=self._position_gains_xy.ki * 0.8,  # Intégrale réduite pour éviter oscillations
            kd=self._position_gains_xy.kd * 2.0,  # Dérivée augmentée pour stabilité
            max_output=self._approach_speed,      # Vitesse limitée
            max_windup=self._position_gains_xy.max_windup * 0.5,
            deadband=0.05  # Zone morte réduite
        )
        
        precision_gains_z = PIDGains(
            kp=self._position_gains_z.kp * 1.2,
            ki=self._position_gains_z.ki * 0.8,
            kd=self._position_gains_z.kd * 1.5,
            max_output=self._approach_speed * 0.5,  # Vitesse verticale plus lente
            max_windup=self._position_gains_z.max_windup * 0.5,
            deadband=0.02
        )
        
        # Contrôle PID avec gains de précision
        vel_x = self._compute_position_pid(
            self._target_position.x, self._current_position.x,
            self._position_state_x, precision_gains_xy, current_time
        )
        
        vel_y = self._compute_position_pid(
            self._target_position.y, self._current_position.y,
            self._position_state_y, precision_gains_xy, current_time
        )
        
        vel_z = self._compute_position_pid(
            self._target_position.z, self._current_position.z,
            self._position_state_z, precision_gains_z, current_time
        )
        
        # Réduction progressive de la vitesse près de la cible
        if distance < self._precision_radius * 2:
            speed_factor = max(0.1, distance / (self._precision_radius * 2))
            vel_x *= speed_factor
            vel_y *= speed_factor
            vel_z *= speed_factor
        
        velocity_target = Vector3(x=vel_x, y=vel_y, z=vel_z)
        acceleration_target = self._compute_acceleration_target(velocity_target, current_time)
        
        # Vérification de convergence
        converged = distance < self._precision_radius / 10  # 5cm de précision
        message = f"Approche précision - Distance: {distance:.3f}m"
        if converged:
            message += " - Position atteinte"
        
        return ControlOutput(
            position_target=self._target_position,
            velocity_target=velocity_target,
            acceleration_target=acceleration_target,
            yaw_target=0.0,
            success=True,
            message=message
        )

    def _trajectory_following_control(self, current_time: float) -> ControlOutput:
        """
        Contrôle de suivi de trajectoire
        
        Args:
            current_time: Temps actuel en secondes
            
        Returns:
            Sortie du contrôleur pour suivi de trajectoire
            
        Note:
            Implémentation simplifiée - utilise le contrôle de position standard
            en attendant une implémentation complète de suivi de trajectoire
        """
        # TODO: Implémenter le suivi de trajectoire avec anticipation
        self._logger.debug("Utilisation du contrôle de position standard (trajectoire non implémenté)")
        return self._position_hold_control(current_time)

    def _emergency_landing_control(self, current_time: float) -> ControlOutput:
        """
        Contrôle d'atterrissage d'urgence
        
        Args:
            current_time: Temps actuel en secondes
            
        Returns:
            Sortie du contrôleur pour atterrissage d'urgence
            
        Raises:
            RuntimeError: Si la position actuelle est manquante
        """
        if not self._current_position:
            raise RuntimeError("Position actuelle manquante pour l'atterrissage d'urgence")
        
        # Maintien X, Y et descente contrôlée en Z
        emergency_target = Point()
        emergency_target.x = self._current_position.x
        emergency_target.y = self._current_position.y
        emergency_target.z = 0.0  # Atterrissage
        
        # Vitesse de descente contrôlée
        descent_rate = -2.0  # m/s
        
        velocity_target = Vector3(x=0.0, y=0.0, z=descent_rate)
        acceleration_target = Vector3()
        
        return ControlOutput(
            position_target=emergency_target,
            velocity_target=velocity_target,
            acceleration_target=acceleration_target,
            yaw_target=0.0,
            success=True,
            message="Atterrissage d'urgence en cours"
        )

    def _compute_position_pid(self, setpoint: float, feedback: float,
                             state: ControlState, gains: PIDGains,
                             current_time: float) -> float:
        """
        Calcule la sortie PID pour un axe
        
        Args:
            setpoint: Consigne en mètres
            feedback: Retour en mètres
            state: État du contrôleur
            gains: Gains PID
            current_time: Temps actuel en secondes
            
        Returns:
            Sortie du contrôleur PID en m/s
        """
        # Calcul du pas de temps
        dt = current_time - state.last_time if state.last_time > 0 else 0.02
        dt = max(0.001, min(0.1, dt))  # Limitation du dt
        
        # Mise à jour de l'état
        state.setpoint = setpoint
        state.feedback = feedback
        state.error_prev = state.error
        state.error = setpoint - feedback
        
        # Zone morte
        if abs(state.error) < gains.deadband:
            state.error = 0.0
        
        # Terme proportionnel
        p_term = gains.kp * state.error
        
        # Terme intégral avec anti-windup
        state.error_integral += state.error * dt
        
        # Anti-windup - limitation de l'intégrale
        if abs(state.error_integral) > gains.max_windup:
            state.error_integral = math.copysign(gains.max_windup, state.error_integral)
        
        # Reset de l'intégrale si changement de signe de l'erreur
        if state.error * state.error_prev < 0:
            state.error_integral *= 0.5
        
        i_term = gains.ki * state.error_integral
        
        # Terme dérivé
        state.error_derivative = (state.error - state.error_prev) / dt
        d_term = gains.kd * state.error_derivative
        
        # Sortie totale
        output = p_term + i_term + d_term
        
        # Limitation de la sortie
        output = max(-gains.max_output, min(gains.max_output, output))
        
        # Mise à jour de l'état
        state.output = output
        state.last_time = current_time
        
        return output

    def _limit_velocity(self, velocity: Vector3) -> Vector3:
        """
        Limite la vitesse selon les contraintes configurées
        
        Args:
            velocity: Vitesse non limitée en m/s
            
        Returns:
            Vitesse limitée en m/s
        """
        # Limitation XY
        xy_magnitude = math.sqrt(velocity.x**2 + velocity.y**2)
        if xy_magnitude > self._velocity_limits.max_velocity_xy:
            factor = self._velocity_limits.max_velocity_xy / xy_magnitude
            velocity.x *= factor
            velocity.y *= factor
        
        # Limitation Z
        velocity.z = max(-self._velocity_limits.max_velocity_z, 
                        min(self._velocity_limits.max_velocity_z, velocity.z))
        
        return velocity

    def _compute_acceleration_target(self, velocity_target: Vector3, 
                                   current_time: float) -> Vector3:
        """
        Calcule l'accélération cible basée sur la dérivée de la vitesse
        
        Args:
            velocity_target: Vitesse cible en m/s
            current_time: Temps actuel en secondes
            
        Returns:
            Accélération cible en m/s²
        """
        acceleration = Vector3()
        
        if self._current_velocity and hasattr(self, '_last_velocity_time'):
            dt = current_time - self._last_velocity_time
            if dt > 0:
                acceleration.x = (velocity_target.x - self._current_velocity.x) / dt
                acceleration.y = (velocity_target.y - self._current_velocity.y) / dt
                acceleration.z = (velocity_target.z - self._current_velocity.z) / dt
                
                # Limitation de l'accélération
                acc_magnitude = math.sqrt(acceleration.x**2 + acceleration.y**2 + acceleration.z**2)
                if acc_magnitude > self._velocity_limits.max_acceleration:
                    factor = self._velocity_limits.max_acceleration / acc_magnitude
                    acceleration.x *= factor
                    acceleration.y *= factor
                    acceleration.z *= factor
        
        self._last_velocity_time = current_time
        return acceleration

    def _calculate_distance_to_target(self) -> float:
        """
        Calcule la distance euclidienne à la cible
        
        Returns:
            Distance en mètres ou infini si position/cible manquante
        """
        if not self._current_position or not self._target_position:
            return float('inf')
        
        dx = self._target_position.x - self._current_position.x
        dy = self._target_position.y - self._current_position.y
        dz = self._target_position.z - self._current_position.z
        
        return math.sqrt(dx*dx + dy*dy + dz*dz)

    def _create_zero_output(self, message: str) -> ControlOutput:
        """
        Crée une sortie nulle avec un message d'erreur
        
        Args:
            message: Message d'erreur
            
        Returns:
            Sortie de contrôle avec erreur
        """
        return ControlOutput(
            position_target=Point(),
            velocity_target=Vector3(),
            acceleration_target=Vector3(),
            yaw_target=0.0,
            success=False,
            message=message
        )

    def _reset_integral_terms(self):
        """Remet à zéro les termes intégraux"""
        self._position_state_x.error_integral = 0.0
        self._position_state_y.error_integral = 0.0
        self._position_state_z.error_integral = 0.0
        self._velocity_state_x.error_integral = 0.0
        self._velocity_state_y.error_integral = 0.0
        self._velocity_state_z.error_integral = 0.0

    def _update_statistics(self):
        """Met à jour les statistiques du contrôleur"""
        self._stats['control_cycles'] += 1
        
        if self._current_position and self._target_position:
            error = self._calculate_distance_to_target()
            
            # Moyenne mobile de l'erreur
            alpha = 0.1
            self._stats['average_error'] = (alpha * error + 
                                          (1 - alpha) * self._stats['average_error'])
            
            # Erreur maximale
            self._stats['max_error'] = max(self._stats['max_error'], error)

    def tune_gains(self, axis: str, gains: Dict[str, float]):
        """
        Ajuste les gains PID pour un axe spécifique
        
        Args:
            axis: 'position_xy', 'position_z', 'velocity_xy', 'velocity_z'
            gains: Dictionnaire des nouveaux gains
            
        Raises:
            ValueError: Si l'axe est inconnu ou les gains sont invalides
        """
        if axis == 'position_xy':
            target_gains = self._position_gains_xy
        elif axis == 'position_z':
            target_gains = self._position_gains_z
        elif axis == 'velocity_xy':
            target_gains = self._velocity_gains_xy
        elif axis == 'velocity_z':
            target_gains = self._velocity_gains_z
        else:
            raise ValueError(f"Axe inconnu pour tuning: {axis}")
        
        # Mise à jour des gains
        for key, value in gains.items():
            if hasattr(target_gains, key):
                setattr(target_gains, key, value)
                self._logger.info(f"Gain {axis}.{key} mis à jour: {value}")
        
        # Reset des termes intégraux après changement de gains
        self._reset_integral_terms()

    def get_control_status(self) -> Dict[str, Any]:
        """
        Retourne l'état complet du contrôleur
        
        Returns:
            Dictionnaire contenant l'état du contrôleur
        """
        return {
            'mode': self._control_mode.value,
            'precision_mode': self._precision_mode,
            'current_position': {
                'x': self._current_position.x if self._current_position else 0,
                'y': self._current_position.y if self._current_position else 0,
                'z': self._current_position.z if self._current_position else 0
            } if self._current_position else None,
            'target_position': {
                'x': self._target_position.x if self._target_position else 0,
                'y': self._target_position.y if self._target_position else 0,
                'z': self._target_position.z if self._target_position else 0
            } if self._target_position else None,
            'distance_to_target': self._calculate_distance_to_target(),
            'statistics': self._stats.copy(),
            'position_errors': {
                'x': self._position_state_x.error,
                'y': self._position_state_y.error,
                'z': self._position_state_z.error
            }
        }

    def reset(self):
        """Remet à zéro le contrôleur"""
        self._reset_integral_terms()
        self._precision_mode = False
        self._current_position = None
        self._current_velocity = None
        self._target_position = None
        self._filtered_velocity = Vector3()
        
        # Reset des statistiques
        self._stats = {
            'control_cycles': 0,
            'average_error': 0.0,
            'max_error': 0.0,
            'convergence_time': 0.0
        }

    def compute_velocity(self, current_position: Point, target_waypoint: Waypoint) -> Velocity3D:
        """
        Calcule la vitesse nécessaire pour atteindre un waypoint
        
        Args:
            current_position: Position actuelle du drone
            target_waypoint: Waypoint cible
            
        Returns:
            Vitesse cible sous forme de Velocity3D
        """
        # Conversion du waypoint en Point
        target_position = Point()
        target_position.x = target_waypoint.latitude   # Supposant que c'est en coordonnées locales
        target_position.y = target_waypoint.longitude  # pour le moment
        target_position.z = target_waypoint.altitude
        
        # Mise à jour de l'état
        self.update_current_state(current_position)
        self.set_target_position(target_position)
        
        # Calcul de la sortie du contrôleur
        control_output = self.update(current_position, target_position)
        
        # Conversion en Velocity3D
        velocity = Velocity3D()
        velocity.x = control_output.velocity_target.x
        velocity.y = control_output.velocity_target.y
        velocity.z = control_output.velocity_target.z
        
        return velocity

    def is_waypoint_reached(self, current_position: Point, target_waypoint: Waypoint) -> bool:
        """
        Vérifie si un waypoint a été atteint
        
        Args:
            current_position: Position actuelle du drone
            target_waypoint: Waypoint cible
            
        Returns:
            True si le waypoint est atteint, False sinon
        """
        # Calcul de la distance horizontale
        dx = target_waypoint.latitude - current_position.x   # Coordonnées locales
        dy = target_waypoint.longitude - current_position.y
        dz = target_waypoint.altitude - current_position.z
        
        distance_xy = math.sqrt(dx*dx + dy*dy)
        distance_z = abs(dz)
        
        # Vérification des tolérances
        xy_reached = distance_xy < self._waypoint_tolerance_xy
        z_reached = distance_z < self._waypoint_tolerance_z
        
        return xy_reached and z_reached