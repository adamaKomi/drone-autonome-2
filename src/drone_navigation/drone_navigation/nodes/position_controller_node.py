#!/usr/bin/env python3
"""
Position Controller Node - Contrôleur de position PID avancé pour drone autonome
Architecture: Micro-nœud spécialisé dans le contrôle de position précis
Algorithmes: PID multi-axes, Contrôle adaptatif, Anti-windup, Feed-forward
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
import threading
import time

# Messages ROS2
from geometry_msgs.msg import Point, PoseStamped, Twist, Vector3, Quaternion
from nav_msgs.msg import Odometry
from std_msgs.msg import Header, Float64, String, Bool
from sensor_msgs.msg import Imu

# Services personnalisés
from drone_msgs.srv import SetControlParameters, GetControllerStatus, TuneController
from drone_msgs.msg import ControllerConfig, ControllerStatus, PIDGains

# Utilitaires de contrôle
from scipy.signal import butter, filtfilt, lfilter
from scipy.optimize import minimize
import control as ctrl


class ControlMode(Enum):
    """Modes de contrôle disponibles"""
    POSITION = "position"
    VELOCITY = "velocity"
    ACCELERATION = "acceleration"
    ATTITUDE = "attitude"
    HYBRID = "hybrid"


class ControllerType(Enum):
    """Types de contrôleurs disponibles"""
    PID = "pid"
    PID_ADAPTIVE = "pid_adaptive"
    MODEL_PREDICTIVE = "mpc"
    SLIDING_MODE = "sliding_mode"
    FUZZY = "fuzzy"
    NEURAL = "neural"


@dataclass
class PIDConfiguration:
    """Configuration d'un contrôleur PID"""
    kp: float = 1.0
    ki: float = 0.1
    kd: float = 0.05
    
    # Limites
    output_min: float = -10.0
    output_max: float = 10.0
    integral_min: float = -5.0
    integral_max: float = 5.0
    
    # Anti-windup
    enable_anti_windup: bool = True
    windup_threshold: float = 0.8
    
    # Filtrage dérivé
    derivative_filter_cutoff: float = 50.0  # Hz
    
    # Feed-forward
    enable_feedforward: bool = True
    feedforward_gain: float = 0.5


@dataclass
class ControllerState:
    """État interne d'un contrôleur PID"""
    previous_error: float = 0.0
    integral: float = 0.0
    derivative: float = 0.0
    filtered_derivative: float = 0.0
    previous_measurement: float = 0.0
    output: float = 0.0
    last_time: float = 0.0
    
    # Historique pour analyse
    error_history: List[float] = field(default_factory=list)
    output_history: List[float] = field(default_factory=list)


@dataclass
class PositionControllerConfig:
    """Configuration globale du contrôleur de position"""
    # Contrôleurs PID pour chaque axe
    x_pid: PIDConfiguration = field(default_factory=PIDConfiguration)
    y_pid: PIDConfiguration = field(default_factory=PIDConfiguration)
    z_pid: PIDConfiguration = field(default_factory=PIDConfiguration)
    yaw_pid: PIDConfiguration = field(default_factory=PIDConfiguration)
    
    # Paramètres dynamiques
    control_frequency: float = 50.0  # Hz
    max_velocity: float = 5.0  # m/s
    max_acceleration: float = 3.0  # m/s²
    max_jerk: float = 5.0  # m/s³
    
    # Seuils de précision
    position_tolerance: float = 0.1  # m
    velocity_tolerance: float = 0.1  # m/s
    yaw_tolerance: float = 0.05  # rad
    
    # Mode de contrôle
    control_mode: ControlMode = ControlMode.POSITION
    controller_type: ControllerType = ControllerType.PID
    
    # Auto-tuning
    enable_auto_tuning: bool = False
    tuning_method: str = "ziegler_nichols"
    
    # Adaptation
    enable_adaptation: bool = True
    adaptation_rate: float = 0.01


class PositionControllerNode(Node):
    """
    Nœud de contrôle de position multi-axes pour drone autonome
    
    Responsabilités:
    - Contrôle PID précis sur X, Y, Z, Yaw
    - Adaptation dynamique des gains
    - Filtrage et limitation des commandes
    - Auto-tuning des paramètres
    """
    
    def __init__(self):
        super().__init__('position_controller_node')
        
        # Configuration du nœud
        self.declare_parameters()
        self.load_configuration()
        
        # État interne des contrôleurs
        self.x_controller = ControllerState()
        self.y_controller = ControllerState()
        self.z_controller = ControllerState()
        self.yaw_controller = ControllerState()
        
        # État du système
        self.current_position: Optional[Tuple[float, float, float]] = None
        self.current_velocity: Optional[Tuple[float, float, float]] = None
        self.current_yaw: float = 0.0
        self.target_position: Optional[Tuple[float, float, float]] = None
        self.target_yaw: float = 0.0
        
        # Filtres pour les mesures
        self.position_filter = self.create_butterworth_filter(10.0)  # 10 Hz
        self.velocity_filter = self.create_butterworth_filter(20.0)  # 20 Hz
        
        # Historique pour adaptation
        self.performance_history: List[Dict] = []
        self.last_control_time = time.time()
        
        # Threading
        self.control_lock = threading.Lock()
        
        # Callback groups
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        self.publisher_cb_group = ReentrantCallbackGroup()
        self.subscriber_cb_group = ReentrantCallbackGroup()
        
        # Publishers
        self.velocity_command_publisher = self.create_publisher(
            Twist,
            '/drone_nav/cmd_vel',
            10,
            callback_group=self.publisher_cb_group
        )
        
        self.controller_status_publisher = self.create_publisher(
            ControllerStatus,
            '/drone_nav/controller_status',
            10,
            callback_group=self.publisher_cb_group
        )
        
        self.debug_publisher = self.create_publisher(
            String,
            '/drone_nav/controller_debug',
            10,
            callback_group=self.publisher_cb_group
        )
        
        # Subscribers
        self.position_subscriber = self.create_subscription(
            PoseStamped,
            '/drone_nav/current_pose',
            self.position_callback,
            10,
            callback_group=self.subscriber_cb_group
        )
        
        self.velocity_subscriber = self.create_subscription(
            Twist,
            '/drone_nav/current_velocity',
            self.velocity_callback,
            10,
            callback_group=self.subscriber_cb_group
        )
        
        self.target_subscriber = self.create_subscription(
            PoseStamped,
            '/drone_nav/target_pose',
            self.target_callback,
            10,
            callback_group=self.subscriber_cb_group
        )
        
        self.imu_subscriber = self.create_subscription(
            Imu,
            '/drone_nav/imu',
            self.imu_callback,
            10,
            callback_group=self.subscriber_cb_group
        )
        
        # Services
        self.set_params_service = self.create_service(
            SetControlParameters,
            '/drone_nav/position_controller_node/set_parameters',
            self.set_parameters_callback,
            callback_group=self.service_cb_group
        )
        
        self.get_status_service = self.create_service(
            GetControllerStatus,
            '/drone_nav/position_controller_node/get_status',
            self.get_status_callback,
            callback_group=self.service_cb_group
        )
        
        self.tune_service = self.create_service(
            TuneController,
            '/drone_nav/position_controller_node/tune',
            self.tune_callback,
            callback_group=self.service_cb_group
        )
        
        # Timer de contrôle principal
        self.control_timer = self.create_timer(
            1.0 / self.config.control_frequency,
            self.control_loop,
            callback_group=self.publisher_cb_group
        )
        
        # Timer de publication du statut
        self.status_timer = self.create_timer(
            0.1,  # 10 Hz
            self.publish_status,
            callback_group=self.publisher_cb_group
        )
        
        self.get_logger().info("✅ Position Controller Node initialisé")
    
    def declare_parameters(self):
        """Déclaration des paramètres du nœud"""
        # PID X
        self.declare_parameter('x_kp', 2.0)
        self.declare_parameter('x_ki', 0.1)
        self.declare_parameter('x_kd', 0.5)
        
        # PID Y
        self.declare_parameter('y_kp', 2.0)
        self.declare_parameter('y_ki', 0.1)
        self.declare_parameter('y_kd', 0.5)
        
        # PID Z
        self.declare_parameter('z_kp', 3.0)
        self.declare_parameter('z_ki', 0.2)
        self.declare_parameter('z_kd', 0.8)
        
        # PID Yaw
        self.declare_parameter('yaw_kp', 1.5)
        self.declare_parameter('yaw_ki', 0.05)
        self.declare_parameter('yaw_kd', 0.3)
        
        # Paramètres généraux
        self.declare_parameter('control_frequency', 50.0)
        self.declare_parameter('max_velocity', 5.0)
        self.declare_parameter('max_acceleration', 3.0)
        self.declare_parameter('position_tolerance', 0.1)
        self.declare_parameter('enable_auto_tuning', False)
        self.declare_parameter('enable_adaptation', True)
    
    def load_configuration(self):
        """Charge la configuration depuis les paramètres"""
        self.config = PositionControllerConfig(
            x_pid=PIDConfiguration(
                kp=self.get_parameter('x_kp').value,
                ki=self.get_parameter('x_ki').value,
                kd=self.get_parameter('x_kd').value
            ),
            y_pid=PIDConfiguration(
                kp=self.get_parameter('y_kp').value,
                ki=self.get_parameter('y_ki').value,
                kd=self.get_parameter('y_kd').value
            ),
            z_pid=PIDConfiguration(
                kp=self.get_parameter('z_kp').value,
                ki=self.get_parameter('z_ki').value,
                kd=self.get_parameter('z_kd').value
            ),
            yaw_pid=PIDConfiguration(
                kp=self.get_parameter('yaw_kp').value,
                ki=self.get_parameter('yaw_ki').value,
                kd=self.get_parameter('yaw_kd').value
            ),
            control_frequency=self.get_parameter('control_frequency').value,
            max_velocity=self.get_parameter('max_velocity').value,
            max_acceleration=self.get_parameter('max_acceleration').value,
            position_tolerance=self.get_parameter('position_tolerance').value,
            enable_auto_tuning=self.get_parameter('enable_auto_tuning').value,
            enable_adaptation=self.get_parameter('enable_adaptation').value
        )
    
    def position_callback(self, msg: PoseStamped):
        """Callback pour mise à jour de la position"""
        with self.control_lock:
            self.current_position = (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            )
            
            # Extraction du yaw depuis le quaternion
            self.current_yaw = self.quaternion_to_yaw(msg.pose.orientation)
    
    def velocity_callback(self, msg: Twist):
        """Callback pour mise à jour de la vitesse"""
        with self.control_lock:
            self.current_velocity = (
                msg.linear.x,
                msg.linear.y,
                msg.linear.z
            )
    
    def target_callback(self, msg: PoseStamped):
        """Callback pour nouveau point cible"""
        with self.control_lock:
            self.target_position = (
                msg.pose.position.x,
                msg.pose.position.y,
                msg.pose.position.z
            )
            
            self.target_yaw = self.quaternion_to_yaw(msg.pose.orientation)
            
            # Reset des intégrales lors d'un nouveau target
            self.x_controller.integral = 0.0
            self.y_controller.integral = 0.0
            self.z_controller.integral = 0.0
            self.yaw_controller.integral = 0.0
            
            self.get_logger().info(
                f"🎯 Nouveau target: ({msg.pose.position.x:.2f}, "
                f"{msg.pose.position.y:.2f}, {msg.pose.position.z:.2f})"
            )
    
    def imu_callback(self, msg: Imu):
        """Callback pour données IMU (pour améliorer la stabilité)"""
        # TODO: Utiliser l'IMU pour améliorer l'estimation de l'état
        pass
    
    def control_loop(self):
        """Boucle de contrôle principale"""
        if not self.is_ready_for_control():
            return
        
        with self.control_lock:
            current_time = time.time()
            dt = current_time - self.last_control_time
            
            if dt < 1.0 / self.config.control_frequency * 0.5:
                return  # Éviter les fréquences trop élevées
            
            # Calcul des erreurs
            pos_error_x = self.target_position[0] - self.current_position[0]
            pos_error_y = self.target_position[1] - self.current_position[1]
            pos_error_z = self.target_position[2] - self.current_position[2]
            yaw_error = self.normalize_angle(self.target_yaw - self.current_yaw)
            
            # Contrôle PID pour chaque axe
            vel_cmd_x = self.pid_controller(
                pos_error_x, self.x_controller, self.config.x_pid, dt
            )
            vel_cmd_y = self.pid_controller(
                pos_error_y, self.y_controller, self.config.y_pid, dt
            )
            vel_cmd_z = self.pid_controller(
                pos_error_z, self.z_controller, self.config.z_pid, dt
            )
            yaw_rate_cmd = self.pid_controller(
                yaw_error, self.yaw_controller, self.config.yaw_pid, dt
            )
            
            # Limitation des commandes
            vel_cmd_x = self.limit_velocity(vel_cmd_x, self.config.max_velocity)
            vel_cmd_y = self.limit_velocity(vel_cmd_y, self.config.max_velocity)
            vel_cmd_z = self.limit_velocity(vel_cmd_z, self.config.max_velocity)
            yaw_rate_cmd = self.limit_velocity(yaw_rate_cmd, math.pi)  # rad/s
            
            # Publication de la commande
            cmd_msg = Twist()
            cmd_msg.linear.x = vel_cmd_x
            cmd_msg.linear.y = vel_cmd_y
            cmd_msg.linear.z = vel_cmd_z
            cmd_msg.angular.z = yaw_rate_cmd
            
            self.velocity_command_publisher.publish(cmd_msg)
            
            # Mise à jour de l'historique
            self.update_performance_history(
                current_time, 
                [pos_error_x, pos_error_y, pos_error_z, yaw_error],
                [vel_cmd_x, vel_cmd_y, vel_cmd_z, yaw_rate_cmd]
            )
            
            # Adaptation si activée
            if self.config.enable_adaptation and len(self.performance_history) > 50:
                self.adaptive_tuning()
            
            self.last_control_time = current_time
    
    def is_ready_for_control(self) -> bool:
        """Vérifie si le contrôleur est prêt"""
        return (
            self.current_position is not None and
            self.target_position is not None
        )
    
    def pid_controller(
        self, 
        error: float, 
        controller_state: ControllerState, 
        config: PIDConfiguration, 
        dt: float
    ) -> float:
        """Implémentation d'un contrôleur PID avancé"""
        
        if dt <= 0:
            return 0.0
        
        # Terme proportionnel
        proportional = config.kp * error
        
        # Terme intégral avec anti-windup
        controller_state.integral += error * dt
        
        if config.enable_anti_windup:
            # Limitation de l'intégrale
            controller_state.integral = np.clip(
                controller_state.integral,
                config.integral_min,
                config.integral_max
            )
            
            # Anti-windup conditionnel
            if abs(controller_state.output) > config.windup_threshold * config.output_max:
                if (error > 0 and controller_state.integral > 0) or \
                   (error < 0 and controller_state.integral < 0):
                    controller_state.integral -= error * dt  # Annule l'accumulation
        
        integral = config.ki * controller_state.integral
        
        # Terme dérivé avec filtrage
        if dt > 0:
            raw_derivative = (error - controller_state.previous_error) / dt
            
            # Filtre dérivé (filtre passe-bas)
            alpha = dt / (dt + 1.0 / (2 * math.pi * config.derivative_filter_cutoff))
            controller_state.filtered_derivative = (
                alpha * raw_derivative + 
                (1 - alpha) * controller_state.filtered_derivative
            )
            
            derivative = config.kd * controller_state.filtered_derivative
        else:
            derivative = 0.0
        
        # Calcul de la sortie
        output = proportional + integral + derivative
        
        # Feed-forward si activé
        if config.enable_feedforward:
            # Simple feed-forward basé sur l'erreur
            feedforward = config.feedforward_gain * error
            output += feedforward
        
        # Limitation de la sortie
        output = np.clip(output, config.output_min, config.output_max)
        
        # Mise à jour de l'état
        controller_state.previous_error = error
        controller_state.derivative = derivative
        controller_state.output = output
        
        # Historique pour analyse
        controller_state.error_history.append(error)
        controller_state.output_history.append(output)
        
        # Limitation de l'historique
        if len(controller_state.error_history) > 1000:
            controller_state.error_history = controller_state.error_history[-500:]
        if len(controller_state.output_history) > 1000:
            controller_state.output_history = controller_state.output_history[-500:]
        
        return output
    
    def limit_velocity(self, velocity: float, max_velocity: float) -> float:
        """Limite la vitesse avec saturation douce"""
        return np.clip(velocity, -max_velocity, max_velocity)
    
    def normalize_angle(self, angle: float) -> float:
        """Normalise un angle entre -π et π"""
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    
    def quaternion_to_yaw(self, quaternion: Quaternion) -> float:
        """Convertit un quaternion en angle yaw"""
        # Conversion quaternion -> yaw (rotation autour de Z)
        w = quaternion.w
        x = quaternion.x
        y = quaternion.y
        z = quaternion.z
        
        # Formule pour yaw
        yaw = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return yaw
    
    def create_butterworth_filter(self, cutoff_frequency: float):
        """Crée un filtre de Butterworth"""
        nyquist = self.config.control_frequency / 2
        normalized_cutoff = cutoff_frequency / nyquist
        b, a = butter(2, normalized_cutoff, btype='low', analog=False)
        return {'b': b, 'a': a, 'zi': None}
    
    def apply_filter(self, signal: float, filter_config: Dict) -> float:
        """Applique un filtre à un signal"""
        if filter_config['zi'] is None:
            # Initialisation
            from scipy.signal import lfilter_zi
            filter_config['zi'] = lfilter_zi(filter_config['b'], filter_config['a']) * signal
        
        filtered_signal, filter_config['zi'] = lfilter(
            filter_config['b'], 
            filter_config['a'], 
            [signal], 
            zi=filter_config['zi']
        )
        
        return filtered_signal[0]
    
    def update_performance_history(
        self, 
        timestamp: float, 
        errors: List[float], 
        outputs: List[float]
    ):
        """Met à jour l'historique de performance"""
        performance_data = {
            'timestamp': timestamp,
            'errors': errors.copy(),
            'outputs': outputs.copy(),
            'rms_error': math.sqrt(sum(e*e for e in errors) / len(errors))
        }
        
        self.performance_history.append(performance_data)
        
        # Limitation de l'historique
        if len(self.performance_history) > 1000:
            self.performance_history = self.performance_history[-500:]
    
    def adaptive_tuning(self):
        """Adaptation automatique des gains PID"""
        if len(self.performance_history) < 100:
            return
        
        # Analyse des performances récentes
        recent_data = self.performance_history[-50:]
        avg_rms_error = sum(d['rms_error'] for d in recent_data) / len(recent_data)
        
        # Calcul des variations d'erreur
        error_variance = np.var([d['rms_error'] for d in recent_data])
        
        # Adaptation simple basée sur l'erreur RMS
        adaptation_factor = self.config.adaptation_rate
        
        if avg_rms_error > self.config.position_tolerance * 2:
            # Erreur trop élevée -> augmenter Kp
            self.config.x_pid.kp *= (1 + adaptation_factor)
            self.config.y_pid.kp *= (1 + adaptation_factor)
            self.config.z_pid.kp *= (1 + adaptation_factor)
            
            self.get_logger().debug(f"🔧 Adaptation: Kp augmenté (erreur: {avg_rms_error:.3f})")
        
        elif error_variance > 0.01:
            # Oscillations -> réduire Kd
            self.config.x_pid.kd *= (1 - adaptation_factor)
            self.config.y_pid.kd *= (1 - adaptation_factor)
            self.config.z_pid.kd *= (1 - adaptation_factor)
            
            self.get_logger().debug(f"🔧 Adaptation: Kd réduit (variance: {error_variance:.4f})")
        
        # Limitation des gains
        self.limit_pid_gains()
    
    def limit_pid_gains(self):
        """Limite les gains PID dans des plages raisonnables"""
        configs = [self.config.x_pid, self.config.y_pid, self.config.z_pid, self.config.yaw_pid]
        
        for config in configs:
            config.kp = np.clip(config.kp, 0.1, 10.0)
            config.ki = np.clip(config.ki, 0.0, 2.0)
            config.kd = np.clip(config.kd, 0.0, 2.0)
    
    async def set_parameters_callback(self, request, response):
        """Service de configuration des paramètres"""
        try:
            # Mise à jour des gains PID
            if hasattr(request, 'x_gains'):
                self.config.x_pid.kp = request.x_gains.kp
                self.config.x_pid.ki = request.x_gains.ki
                self.config.x_pid.kd = request.x_gains.kd
            
            if hasattr(request, 'y_gains'):
                self.config.y_pid.kp = request.y_gains.kp
                self.config.y_pid.ki = request.y_gains.ki
                self.config.y_pid.kd = request.y_gains.kd
            
            if hasattr(request, 'z_gains'):
                self.config.z_pid.kp = request.z_gains.kp
                self.config.z_pid.ki = request.z_gains.ki
                self.config.z_pid.kd = request.z_gains.kd
            
            if hasattr(request, 'yaw_gains'):
                self.config.yaw_pid.kp = request.yaw_gains.kp
                self.config.yaw_pid.ki = request.yaw_gains.ki
                self.config.yaw_pid.kd = request.yaw_gains.kd
            
            # Reset des états internes
            self.reset_controller_states()
            
            response.success = True
            response.message = "Paramètres mis à jour avec succès"
            
            self.get_logger().info("✅ Paramètres PID mis à jour")
            
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"❌ Erreur mise à jour paramètres: {str(e)}")
        
        return response
    
    async def get_status_callback(self, request, response):
        """Service de récupération du statut"""
        try:
            with self.control_lock:
                response.success = True
                response.is_active = self.is_ready_for_control()
                
                if self.current_position and self.target_position:
                    # Calcul des erreurs actuelles
                    pos_error = math.sqrt(
                        (self.target_position[0] - self.current_position[0])**2 +
                        (self.target_position[1] - self.current_position[1])**2 +
                        (self.target_position[2] - self.current_position[2])**2
                    )
                    
                    response.position_error = pos_error
                    response.is_at_target = pos_error < self.config.position_tolerance
                    
                    # Gains actuels
                    response.current_gains.x_kp = self.config.x_pid.kp
                    response.current_gains.x_ki = self.config.x_pid.ki
                    response.current_gains.x_kd = self.config.x_pid.kd
                    
                    response.current_gains.y_kp = self.config.y_pid.kp
                    response.current_gains.y_ki = self.config.y_pid.ki
                    response.current_gains.y_kd = self.config.y_pid.kd
                    
                    response.current_gains.z_kp = self.config.z_pid.kp
                    response.current_gains.z_ki = self.config.z_pid.ki
                    response.current_gains.z_kd = self.config.z_pid.kd
                    
                    # Performance
                    if self.performance_history:
                        recent_performance = self.performance_history[-10:]
                        avg_error = sum(p['rms_error'] for p in recent_performance) / len(recent_performance)
                        response.average_error = avg_error
                
        except Exception as e:
            response.success = False
            response.error_message = str(e)
        
        return response
    
    async def tune_callback(self, request, response):
        """Service d'auto-tuning des contrôleurs"""
        try:
            tuning_method = request.method
            
            if tuning_method == "ziegler_nichols":
                await self.ziegler_nichols_tuning()
            elif tuning_method == "cohen_coon":
                await self.cohen_coon_tuning()
            elif tuning_method == "genetic":
                await self.genetic_tuning()
            else:
                raise ValueError(f"Méthode de tuning non supportée: {tuning_method}")
            
            response.success = True
            response.message = f"Auto-tuning {tuning_method} terminé"
            
            self.get_logger().info(f"✅ Auto-tuning {tuning_method} terminé")
            
        except Exception as e:
            response.success = False
            response.message = str(e)
            self.get_logger().error(f"❌ Erreur auto-tuning: {str(e)}")
        
        return response
    
    def reset_controller_states(self):
        """Reset des états internes des contrôleurs"""
        for controller in [self.x_controller, self.y_controller, 
                          self.z_controller, self.yaw_controller]:
            controller.integral = 0.0
            controller.previous_error = 0.0
            controller.derivative = 0.0
            controller.filtered_derivative = 0.0
    
    async def ziegler_nichols_tuning(self):
        """Auto-tuning par méthode Ziegler-Nichols"""
        # Implémentation simplifiée de Ziegler-Nichols
        # Dans une implémentation complète, il faudrait:
        # 1. Augmenter Kp jusqu'à oscillation
        # 2. Mesurer la période d'oscillation
        # 3. Calculer les gains selon les formules ZN
        
        self.get_logger().info("🔧 Démarrage auto-tuning Ziegler-Nichols")
        
        # Pour cette démo, application de valeurs conservatrices
        for config in [self.config.x_pid, self.config.y_pid, self.config.z_pid]:
            config.kp = 2.0
            config.ki = 0.1
            config.kd = 0.5
        
        self.config.yaw_pid.kp = 1.5
        self.config.yaw_pid.ki = 0.05
        self.config.yaw_pid.kd = 0.3
        
        self.reset_controller_states()
    
    def publish_status(self):
        """Publication du statut du contrôleur"""
        status_msg = ControllerStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        
        status_msg.is_active = self.is_ready_for_control()
        status_msg.control_frequency = self.config.control_frequency
        
        if self.current_position and self.target_position:
            pos_error = math.sqrt(
                (self.target_position[0] - self.current_position[0])**2 +
                (self.target_position[1] - self.current_position[1])**2 +
                (self.target_position[2] - self.current_position[2])**2
            )
            status_msg.position_error = pos_error
            status_msg.is_at_target = pos_error < self.config.position_tolerance
        
        # États des contrôleurs
        status_msg.x_integral = self.x_controller.integral
        status_msg.y_integral = self.y_controller.integral
        status_msg.z_integral = self.z_controller.integral
        status_msg.yaw_integral = self.yaw_controller.integral
        
        self.controller_status_publisher.publish(status_msg)


def main(args=None):
    """Point d'entrée principal"""
    rclpy.init(args=args)
    
    try:
        node = PositionControllerNode()
        
        # Utilisation d'un exécuteur multi-threadé
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        node.get_logger().info("🚀 Position Controller Node démarré")
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
