#!/usr/bin/env python3

"""
Contrôleur de position PID avancé pour drone autonome

Implémente des contrôleurs PID multi-axes avec anti-windup,
modes de contrôle adaptatifs et approche de précision pour
missions de pollinisation.

Exemples d'utilisation:
    controller = PositionController(node)
    controller.set_control_mode(ControlMode.PRECISION_APPROACH)
    velocity = controller.compute_velocity(current_pos, target_waypoint)

Auteur: Adama Komi
Version: 1.2.0
"""
import time
import math
import numpy as np
from typing import Dict, Any, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
from geometry_msgs.msg import Point, Vector3, Quaternion
from mavros_msgs.msg import PositionTarget, State
from drone_msgs.msg import Waypoint, Velocity3D


class ControlMode(Enum):
    """Modes de contrôle disponibles"""
    POSITION_HOLD = "position_hold"
    GUIDED = "guided"
    FOLLOW_TRAJECTORY = "follow_trajectory"
    PRECISION_APPROACH = "precision_approach"
    EMERGENCY_LAND = "emergency_land"


@dataclass
class PIDGains:
    """Gains PID pour un axe"""
    kp: float = 1.0
    ki: float = 0.1
    kd: float = 0.05
    max_output: float = 10.0
    max_windup: float = 5.0
    deadband: float = 0.1


@dataclass
class ControlState:
    """État du contrôleur pour un axe"""
    setpoint: float = 0.0
    feedback: float = 0.0
    error: float = 0.0
    error_prev: float = 0.0
    error_integral: float = 0.0
    error_derivative: float = 0.0
    output: float = 0.0
    last_time: float = 0.0


@dataclass
class VelocityLimits:
    """Limites de vitesse et accélération"""
    max_velocity_xy: float = 10.0   # m/s
    max_velocity_z: float = 5.0     # m/s
    max_acceleration: float = 5.0   # m/s²
    max_jerk: float = 10.0          # m/s³


class PositionController:
    """
    Contrôleur de position PID avancé multi-axes
    
    Fournit un contrôle de position précis avec:
    - Contrôleurs PID séparés pour X, Y, Z
    - Anti-windup et limitation de sortie
    - Modes de contrôle adaptatifs
    - Interface cohérente avec NavigationNode
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
        
        # Filtrage et lissage
        self._velocity_filter_alpha = 0.8
        self._filtered_velocity = Vector3()
        
        # Tolérance pour waypoint atteint
        self._waypoint_tolerance_xy = 0.5  # m
        self._waypoint_tolerance_z = 0.3   # m
        
        # Statistiques
        self._stats = {
            'control_cycles': 0,
            'average_error': 0.0,
            'max_error': 0.0,
            'convergence_time': 0.0
        }
        
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
            max_output=10.0,
            max_windup=5.0,
            deadband=0.1
        )
        
        self._position_gains_z = PIDGains(
            kp=pos_config.get('p_z', 1.5),
            ki=pos_config.get('i_z', 0.2),
            kd=pos_config.get('d_z', 0.1),
            max_output=5.0,
            max_windup=2.0,
            deadband=0.05
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

    def compute_velocity(self, current_position: Point, target_waypoint: Waypoint) -> Velocity3D:
        """
        Calcule la vitesse nécessaire pour atteindre un waypoint
        
        Args:
            current_position: Position actuelle du drone
            target_waypoint: Waypoint cible
            
        Returns:
            Vitesse cible sous forme de Velocity3D
        """
        # Conversion du waypoint en Point (coordonnées locales)
        target_position = Point()
        target_position.x = target_waypoint.latitude   # Supposant coordonnées locales
        target_position.y = target_waypoint.longitude
        target_position.z = target_waypoint.altitude
        
        # Mise à jour de l'état
        self._current_position = current_position
        self._target_position = target_position
        
        current_time = time.time()
        
        # Contrôle PID de position pour chaque axe
        vel_x = self._compute_position_pid(
            target_position.x, current_position.x,
            self._position_state_x, self._position_gains_xy, current_time
        )
        
        vel_y = self._compute_position_pid(
            target_position.y, current_position.y,
            self._position_state_y, self._position_gains_xy, current_time
        )
        
        vel_z = self._compute_position_pid(
            target_position.z, current_position.z,
            self._position_state_z, self._position_gains_z, current_time
        )
        
        # Limitation des vitesses
        velocity_target = self._limit_velocity(Vector3(x=vel_x, y=vel_y, z=vel_z))
        
        # Conversion en Velocity3D
        velocity = Velocity3D()
        velocity.x = velocity_target.x
        velocity.y = velocity_target.y
        velocity.z = velocity_target.z
        
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
            state.error_integral = 0.0
        
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
            scale = self._velocity_limits.max_velocity_xy / xy_magnitude
            velocity.x *= scale
            velocity.y *= scale
        
        # Limitation Z
        velocity.z = max(-self._velocity_limits.max_velocity_z, 
                        min(self._velocity_limits.max_velocity_z, velocity.z))
        
        return velocity

    def _reset_integral_terms(self):
        """Remet à zéro les termes intégraux"""
        self._position_state_x.error_integral = 0.0
        self._position_state_y.error_integral = 0.0
        self._position_state_z.error_integral = 0.0

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

    def tune_gains(self, axis: str, gains: Dict[str, float]):
        """
        Ajuste les gains PID pour un axe spécifique
        
        Args:
            axis: 'position_xy' ou 'position_z'
            gains: Dictionnaire des nouveaux gains
            
        Raises:
            ValueError: Si l'axe est inconnu ou les gains sont invalides
        """
        if axis == 'position_xy':
            target_gains = self._position_gains_xy
        elif axis == 'position_z':
            target_gains = self._position_gains_z
        else:
            raise ValueError(f"Axe inconnu: {axis}")
        
        # Mise à jour des gains
        for key, value in gains.items():
            if hasattr(target_gains, key):
                setattr(target_gains, key, value)
            else:
                raise ValueError(f"Gain inconnu: {key}")
        
        # Reset des termes intégraux après changement de gains
        self._reset_integral_terms()
        
        self._logger.info(f"Gains PID mis à jour pour {axis}: {gains}")
