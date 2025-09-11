#!/usr/bin/env python3
"""
Parameter Manager Node - Gestion centralisée des paramètres système
Auteur: Adama Komi
Date: 2025-09-11
Version: 1.0.0

Ce nœud gère la configuration centralisée de tous les paramètres du système
de navigation. Il charge les fichiers YAML et distribue les paramètres aux
autres nœuds via des services ROS2.

Références:
- ROS2 Parameters: https://docs.ros.org/en/humble/Concepts/About-ROS-2-Parameters.html
- YAML Configuration: https://pyyaml.org/wiki/PyYAMLDocumentation
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.msg import ParameterDescriptor, ParameterType
from rcl_interfaces.srv import SetParameters, GetParameters
from std_msgs.msg import String
from std_srvs.srv import Trigger

import yaml
import os
import threading
from typing import Dict, Any, Optional
from dataclasses import dataclass, field
from pathlib import Path


@dataclass
class ParameterConfig:
    """Configuration centralisée des paramètres"""
    # Navigation
    max_velocity: float = 10.0
    max_acceleration: float = 3.0
    planning_frequency: float = 10.0
    control_frequency: float = 50.0
    
    # Sécurité
    min_altitude: float = 2.0
    max_altitude: float = 120.0
    safety_distance: float = 5.0
    emergency_descent_rate: float = 2.0
    
    # Contrôle PID
    pid_position_x: Dict[str, float] = field(default_factory=lambda: {
        'kp': 1.2, 'ki': 0.1, 'kd': 0.05, 'max_output': 5.0
    })
    pid_position_y: Dict[str, float] = field(default_factory=lambda: {
        'kp': 1.2, 'ki': 0.1, 'kd': 0.05, 'max_output': 5.0
    })
    pid_position_z: Dict[str, float] = field(default_factory=lambda: {
        'kp': 2.0, 'ki': 0.2, 'kd': 0.08, 'max_output': 3.0
    })
    
    # Algorithmes
    default_planner: str = "astar"
    optimization_enabled: bool = True
    obstacle_avoidance_method: str = "potential_field"


class ParameterManagerNode(Node):
    """
    Nœud de gestion centralisée des paramètres système
    
    Ce nœud est responsable de:
    - Charger les configurations depuis les fichiers YAML
    - Distribuer les paramètres aux autres nœuds
    - Gérer la reconfiguration dynamique
    - Valider la cohérence des paramètres
    """
    
    def __init__(self):
        super().__init__('parameter_manager_node', namespace='drone_nav')
        
        # Configuration
        self.config = ParameterConfig()
        self.config_file_paths = []
        self.parameter_cache = {}
        self.lock = threading.RLock()
        
        # Déclarer les paramètres du nœud
        self._declare_node_parameters()
        
        # Charger la configuration initiale
        self._load_configurations()
        
        # Services
        self.set_param_service = self.create_service(
            SetParameters, 
            'set_parameter',
            self._handle_set_parameter
        )
        
        self.get_param_service = self.create_service(
            GetParameters,
            'get_parameter', 
            self._handle_get_parameter
        )
        
        self.reload_config_service = self.create_service(
            Trigger,
            'reload_config',
            self._handle_reload_config
        )
        
        # Publisher pour les mises à jour
        self.param_updates_pub = self.create_publisher(
            String,
            'parameter_updates',
            10
        )
        
        # Timer pour validation périodique
        self.validation_timer = self.create_timer(
            30.0,  # 30 secondes
            self._validate_parameters
        )
        
        self.get_logger().info("Parameter Manager Node initialisé")
        self._log_current_configuration()
    
    def _declare_node_parameters(self):
        """Déclare les paramètres internes du nœud"""
        # Chemins des fichiers de configuration
        self.declare_parameter(
            'config_files',
            ['config/navigation_params.yaml', 'config/pid_tuning.yaml'],
            ParameterDescriptor(
                description="Liste des fichiers de configuration YAML",
                type=ParameterType.PARAMETER_STRING_ARRAY
            )
        )
        
        # Mode de validation
        self.declare_parameter(
            'validation_enabled',
            True,
            ParameterDescriptor(
                description="Activer la validation des paramètres",
                type=ParameterType.PARAMETER_BOOL
            )
        )
        
        # Sauvegarde automatique
        self.declare_parameter(
            'auto_save_enabled',
            False,
            ParameterDescriptor(
                description="Sauvegarde automatique des modifications",
                type=ParameterType.PARAMETER_BOOL
            )
        )
    
    def _load_configurations(self):
        """Charge toutes les configurations depuis les fichiers YAML"""
        try:
            with self.lock:
                config_files = self.get_parameter('config_files').value
                
                for config_file in config_files:
                    self._load_single_config(config_file)
                
                # Appliquer la configuration chargée
                self._apply_loaded_config()
                
            self.get_logger().info(
                f"Configuration chargée depuis {len(config_files)} fichiers"
            )
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors du chargement: {e}")
            self._load_default_config()
    
    def _load_single_config(self, config_file: str):
        """Charge un fichier de configuration individuel"""
        try:
            # Construire le chemin absolu
            if not os.path.isabs(config_file):
                package_path = self._get_package_path()
                config_path = os.path.join(package_path, config_file)
            else:
                config_path = config_file
            
            if not os.path.exists(config_path):
                self.get_logger().warning(f"Fichier non trouvé: {config_path}")
                return
            
            # Charger le YAML
            with open(config_path, 'r', encoding='utf-8') as f:
                yaml_data = yaml.safe_load(f)
            
            # Mettre à jour le cache
            self._update_parameter_cache(yaml_data, config_file)
            self.config_file_paths.append(config_path)
            
            self.get_logger().debug(f"Configuration chargée: {config_file}")
            
        except Exception as e:
            self.get_logger().error(
                f"Erreur lors du chargement de {config_file}: {e}"
            )
    
    def _update_parameter_cache(self, yaml_data: Dict[str, Any], source_file: str):
        """Met à jour le cache des paramètres avec les données YAML"""
        def flatten_dict(d: Dict[str, Any], parent_key: str = '', sep: str = '.') -> Dict[str, Any]:
            """Aplatit un dictionnaire hiérarchique"""
            items = []
            for k, v in d.items():
                new_key = f"{parent_key}{sep}{k}" if parent_key else k
                if isinstance(v, dict):
                    items.extend(flatten_dict(v, new_key, sep=sep).items())
                else:
                    items.append((new_key, v))
            return dict(items)
        
        # Aplatir la configuration YAML
        flat_params = flatten_dict(yaml_data)
        
        # Mettre à jour le cache
        for key, value in flat_params.items():
            self.parameter_cache[key] = {
                'value': value,
                'source': source_file,
                'type': type(value).__name__
            }
    
    def _apply_loaded_config(self):
        """Applique la configuration chargée à l'objet config"""
        try:
            # Navigation
            self.config.max_velocity = self._get_cached_param(
                'navigation.max_velocity', 10.0
            )
            self.config.max_acceleration = self._get_cached_param(
                'navigation.max_acceleration', 3.0
            )
            self.config.planning_frequency = self._get_cached_param(
                'navigation.planning_frequency', 10.0
            )
            self.config.control_frequency = self._get_cached_param(
                'navigation.control_frequency', 50.0
            )
            
            # Sécurité
            self.config.min_altitude = self._get_cached_param(
                'safety.min_altitude', 2.0
            )
            self.config.max_altitude = self._get_cached_param(
                'safety.max_altitude', 120.0
            )
            self.config.safety_distance = self._get_cached_param(
                'safety.safety_distance', 5.0
            )
            self.config.emergency_descent_rate = self._get_cached_param(
                'safety.emergency_descent_rate', 2.0
            )
            
            # Contrôleurs PID
            self._load_pid_config('position_x', self.config.pid_position_x)
            self._load_pid_config('position_y', self.config.pid_position_y)
            self._load_pid_config('position_z', self.config.pid_position_z)
            
            # Algorithmes
            self.config.default_planner = self._get_cached_param(
                'algorithms.default_planner', "astar"
            )
            self.config.optimization_enabled = self._get_cached_param(
                'algorithms.optimization_enabled', True
            )
            self.config.obstacle_avoidance_method = self._get_cached_param(
                'algorithms.obstacle_avoidance', "potential_field"
            )
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors de l'application de la config: {e}")
        
        # Synchroniser avec les paramètres ROS2
        self._sync_ros2_parameters()
    
    def _sync_ros2_parameters(self):
        """Synchronise le cache avec les paramètres ROS2"""
        try:
            # Déclarer tous les paramètres du cache comme paramètres ROS2
            for param_name, param_data in self.parameter_cache.items():
                value = param_data['value']
                
                # Déterminer le type de paramètre
                if isinstance(value, bool):
                    param_type = ParameterType.PARAMETER_BOOL
                elif isinstance(value, int):
                    param_type = ParameterType.PARAMETER_INTEGER
                elif isinstance(value, float):
                    param_type = ParameterType.PARAMETER_DOUBLE
                elif isinstance(value, str):
                    param_type = ParameterType.PARAMETER_STRING
                else:
                    continue  # Ignorer les types non supportés
                
                # Déclarer le paramètre ROS2
                try:
                    self.declare_parameter(
                        param_name,
                        value,
                        ParameterDescriptor(
                            description=f"Paramètre de navigation: {param_name}",
                            type=param_type
                        )
                    )
                except Exception:
                    # Le paramètre existe déjà, le mettre à jour
                    if isinstance(value, bool):
                        param_value = Parameter(param_name, Parameter.Type.BOOL, value)
                    elif isinstance(value, int):
                        param_value = Parameter(param_name, Parameter.Type.INTEGER, value)
                    elif isinstance(value, float):
                        param_value = Parameter(param_name, Parameter.Type.DOUBLE, value)
                    elif isinstance(value, str):
                        param_value = Parameter(param_name, Parameter.Type.STRING, value)
                    else:
                        continue  # Ignorer les types non supportés
                    
                    self.set_parameters([param_value])
                    
            self.get_logger().debug(f"Synchronisé {len(self.parameter_cache)} paramètres avec ROS2")
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la synchronisation ROS2: {e}")
    
    def _load_pid_config(self, controller_name: str, pid_dict: Dict[str, float]):
        """Charge la configuration d'un contrôleur PID"""
        base_key = f"pid_controllers.{controller_name}"
        
        pid_dict['kp'] = self._get_cached_param(f"{base_key}.kp", pid_dict.get('kp', 1.0))
        pid_dict['ki'] = self._get_cached_param(f"{base_key}.ki", pid_dict.get('ki', 0.1))
        pid_dict['kd'] = self._get_cached_param(f"{base_key}.kd", pid_dict.get('kd', 0.05))
        pid_dict['max_output'] = self._get_cached_param(
            f"{base_key}.max_output", pid_dict.get('max_output', 5.0)
        )
    
    def _get_cached_param(self, key: str, default_value: Any) -> Any:
        """Récupère un paramètre du cache avec valeur par défaut"""
        if key in self.parameter_cache:
            return self.parameter_cache[key]['value']
        else:
            self.get_logger().warning(
                f"Paramètre {key} non trouvé, utilisation de la valeur par défaut: {default_value}"
            )
            # Ajouter la valeur par défaut au cache pour les services
            self.parameter_cache[key] = {
                'value': default_value,
                'source': 'default',
                'type': type(default_value).__name__
            }
            return default_value
    
    def _load_default_config(self):
        """Charge la configuration par défaut en cas d'erreur"""
        self.get_logger().warning("Chargement de la configuration par défaut")
        self.config = ParameterConfig()
    
    def _get_package_path(self) -> str:
        """Obtient le chemin du package ROS2"""
        try:
            import ament_index_python
            return ament_index_python.get_package_share_directory('drone_navigation')
        except:
            # Fallback: utiliser le répertoire courant
            return os.getcwd()
    
    def _handle_set_parameter(self, request, response):
        """Gestionnaire du service set_parameter"""
        try:
            with self.lock:
                for param in request.parameters:
                    # Valider le paramètre
                    if self._validate_single_parameter(param):
                        # Mettre à jour le cache
                        self.parameter_cache[param.name] = {
                            'value': self._parameter_value_to_python(param),
                            'source': 'dynamic_update',
                            'type': self._get_parameter_type_name(param)
                        }
                        
                        # Mettre à jour la configuration
                        self._update_config_from_parameter(param)
                        
                        self.get_logger().info(f"Paramètre mis à jour: {param.name}")
                    else:
                        response.results.append(
                            f"Validation échouée pour {param.name}"
                        )
                        continue
                
                # Publier la mise à jour
                self._publish_parameter_update("parameters_updated")
                
                response.successful = True
                
        except Exception as e:
            self.get_logger().error(f"Erreur dans set_parameter: {e}")
            response.successful = False
            response.reason = str(e)
        
        return response
    
    def _handle_get_parameter(self, request, response):
        """Gestionnaire du service get_parameter"""
        try:
            with self.lock:
                for param_name in request.names:
                    if param_name in self.parameter_cache:
                        param_info = self.parameter_cache[param_name]
                        value = param_info['value']
                        
                        # Créer le parameter value
                        param_value = self._python_value_to_parameter_value(value)
                        response.values.append(param_value)
                    else:
                        # Paramètre non trouvé
                        from rcl_interfaces.msg import ParameterValue
                        empty_value = ParameterValue()
                        empty_value.type = ParameterType.PARAMETER_NOT_SET
                        response.values.append(empty_value)
        
        except Exception as e:
            self.get_logger().error(f"Erreur dans get_parameter: {e}")
        
        return response
    
    def _handle_reload_config(self, request, response):
        """Gestionnaire du service reload_config"""
        try:
            self.get_logger().info("Rechargement de la configuration...")
            
            # Nettoyer le cache
            with self.lock:
                self.parameter_cache.clear()
                self.config_file_paths.clear()
            
            # Recharger la configuration
            self._load_configurations()
            
            # Publier la mise à jour
            self._publish_parameter_update("configuration_reloaded")
            
            response.success = True
            response.message = "Configuration rechargée avec succès"
            
            self.get_logger().info("Configuration rechargée avec succès")
            
        except Exception as e:
            self.get_logger().error(f"Erreur lors du rechargement: {e}")
            response.success = False
            response.message = f"Erreur: {e}"
        
        return response
    
    def _validate_parameters(self):
        """Validation périodique des paramètres"""
        if not self.get_parameter('validation_enabled').value:
            return
        
        try:
            validation_errors = []
            
            # Valider les limites physiques
            if self.config.min_altitude >= self.config.max_altitude:
                validation_errors.append("min_altitude >= max_altitude")
            
            if self.config.max_velocity <= 0:
                validation_errors.append("max_velocity doit être > 0")
            
            if self.config.control_frequency <= 0:
                validation_errors.append("control_frequency doit être > 0")
            
            # Valider les gains PID
            for pid_name, pid_config in [
                ('position_x', self.config.pid_position_x),
                ('position_y', self.config.pid_position_y),
                ('position_z', self.config.pid_position_z)
            ]:
                if pid_config['kp'] < 0:
                    validation_errors.append(f"PID {pid_name}: kp < 0")
                if pid_config['max_output'] <= 0:
                    validation_errors.append(f"PID {pid_name}: max_output <= 0")
            
            # Logger les erreurs
            if validation_errors:
                self.get_logger().warning(
                    f"Erreurs de validation: {', '.join(validation_errors)}"
                )
            else:
                self.get_logger().debug("Validation des paramètres: OK")
                
        except Exception as e:
            self.get_logger().error(f"Erreur lors de la validation: {e}")
    
    def _validate_single_parameter(self, param: Parameter) -> bool:
        """Valide un paramètre individuel"""
        try:
            value = self._parameter_value_to_python(param)
            
            # Validation basée sur le nom
            if 'altitude' in param.name.lower() and isinstance(value, (int, float)):
                return 0 <= value <= 1000  # Limite raisonnable
            
            if 'velocity' in param.name.lower() and isinstance(value, (int, float)):
                return 0 <= value <= 50  # Limite de sécurit
            
            if 'frequency' in param.name.lower() and isinstance(value, (int, float)):
                return 0 < value <= 1000  # Fréquence positive
            
            # Validation des gains PID
            if param.name.endswith('.kp') and isinstance(value, (int, float)):
                return value >= 0
            
            if param.name.endswith('.max_output') and isinstance(value, (int, float)):
                return value > 0
            
            return True  # Validation passée par défaut
            
        except Exception as e:
            self.get_logger().error(f"Erreur validation paramètre {param.name}: {e}")
            return False
    
    def _parameter_value_to_python(self, param: Parameter) -> Any:
        """Convertit un ParameterValue ROS2 en valeur Python"""
        if param.type_ == ParameterType.PARAMETER_BOOL:
            return param.value
        elif param.type_ == ParameterType.PARAMETER_INTEGER:
            return param.value
        elif param.type_ == ParameterType.PARAMETER_DOUBLE:
            return param.value
        elif param.type_ == ParameterType.PARAMETER_STRING:
            return param.value
        elif param.type_ == ParameterType.PARAMETER_BYTE_ARRAY:
            return param.value
        elif param.type_ == ParameterType.PARAMETER_BOOL_ARRAY:
            return param.value
        elif param.type_ == ParameterType.PARAMETER_INTEGER_ARRAY:
            return param.value
        elif param.type_ == ParameterType.PARAMETER_DOUBLE_ARRAY:
            return param.value
        elif param.type_ == ParameterType.PARAMETER_STRING_ARRAY:
            return param.value
        else:
            return None
    
    def _python_value_to_parameter_value(self, value: Any):
        """Convertit une valeur Python en ParameterValue ROS2"""
        from rcl_interfaces.msg import ParameterValue
        
        param_value = ParameterValue()
        
        if isinstance(value, bool):
            param_value.type = ParameterType.PARAMETER_BOOL
            param_value.bool_value = value
        elif isinstance(value, int):
            param_value.type = ParameterType.PARAMETER_INTEGER
            param_value.integer_value = value
        elif isinstance(value, float):
            param_value.type = ParameterType.PARAMETER_DOUBLE
            param_value.double_value = value
        elif isinstance(value, str):
            param_value.type = ParameterType.PARAMETER_STRING
            param_value.string_value = value
        elif isinstance(value, list):
            if all(isinstance(x, bool) for x in value):
                param_value.type = ParameterType.PARAMETER_BOOL_ARRAY
                param_value.bool_array_value = value
            elif all(isinstance(x, int) for x in value):
                param_value.type = ParameterType.PARAMETER_INTEGER_ARRAY
                param_value.integer_array_value = value
            elif all(isinstance(x, float) for x in value):
                param_value.type = ParameterType.PARAMETER_DOUBLE_ARRAY
                param_value.double_array_value = value
            elif all(isinstance(x, str) for x in value):
                param_value.type = ParameterType.PARAMETER_STRING_ARRAY
                param_value.string_array_value = value
        
        return param_value
    
    def _get_parameter_type_name(self, param: Parameter) -> str:
        """Obtient le nom du type d'un paramètre"""
        type_map = {
            ParameterType.PARAMETER_BOOL: 'bool',
            ParameterType.PARAMETER_INTEGER: 'int',
            ParameterType.PARAMETER_DOUBLE: 'float',
            ParameterType.PARAMETER_STRING: 'str',
            ParameterType.PARAMETER_BOOL_ARRAY: 'bool_array',
            ParameterType.PARAMETER_INTEGER_ARRAY: 'int_array',
            ParameterType.PARAMETER_DOUBLE_ARRAY: 'float_array',
            ParameterType.PARAMETER_STRING_ARRAY: 'str_array'
        }
        return type_map.get(param.type_, 'unknown')
    
    def _update_config_from_parameter(self, param: Parameter):
        """Met à jour l'objet config depuis un paramètre"""
        value = self._parameter_value_to_python(param)
        
        # Mapper les noms de paramètres aux attributs de config
        param_mapping = {
            'navigation.max_velocity': 'max_velocity',
            'navigation.max_acceleration': 'max_acceleration',
            'navigation.planning_frequency': 'planning_frequency',
            'navigation.control_frequency': 'control_frequency',
            'safety.min_altitude': 'min_altitude',
            'safety.max_altitude': 'max_altitude',
            'safety.safety_distance': 'safety_distance',
            'safety.emergency_descent_rate': 'emergency_descent_rate',
            'algorithms.default_planner': 'default_planner',
            'algorithms.optimization_enabled': 'optimization_enabled',
            'algorithms.obstacle_avoidance': 'obstacle_avoidance_method'
        }
        
        if param.name in param_mapping:
            setattr(self.config, param_mapping[param.name], value)
        elif param.name.startswith('pid_controllers.'):
            self._update_pid_config_from_parameter(param.name, value)
    
    def _update_pid_config_from_parameter(self, param_name: str, value: Any):
        """Met à jour la configuration PID depuis un paramètre"""
        parts = param_name.split('.')
        if len(parts) >= 3:
            controller_name = parts[1]  # position_x, position_y, position_z
            param_key = parts[2]        # kp, ki, kd, max_output
            
            if controller_name == 'position_x':
                self.config.pid_position_x[param_key] = value
            elif controller_name == 'position_y':
                self.config.pid_position_y[param_key] = value
            elif controller_name == 'position_z':
                self.config.pid_position_z[param_key] = value
    
    def _publish_parameter_update(self, event_type: str):
        """Publie une notification de mise à jour des paramètres"""
        import json
        
        update_msg = String()
        update_data = {
            'event_type': event_type,
            'timestamp': self.get_clock().now().nanoseconds,
            'parameter_count': len(self.parameter_cache)
        }
        update_msg.data = json.dumps(update_data)
        
        self.param_updates_pub.publish(update_msg)
    
    def _log_current_configuration(self):
        """Affiche la configuration actuelle dans les logs"""
        self.get_logger().info("=== Configuration actuelle ===")
        self.get_logger().info(f"Navigation: max_vel={self.config.max_velocity}, "
                             f"max_acc={self.config.max_acceleration}")
        self.get_logger().info(f"Sécurité: alt_min={self.config.min_altitude}, "
                             f"alt_max={self.config.max_altitude}")
        self.get_logger().info(f"Contrôle: freq={self.config.control_frequency}Hz")
        self.get_logger().info(f"Planificateur: {self.config.default_planner}")
        self.get_logger().info("==============================")
    
    def get_config(self) -> ParameterConfig:
        """Retourne la configuration actuelle (thread-safe)"""
        with self.lock:
            return self.config


def main(args=None):
    """Point d'entrée principal"""
    rclpy.init(args=args)
    
    try:
        node = ParameterManagerNode()
        
        # Utiliser MultiThreadedExecutor pour les services
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        node.get_logger().info("Parameter Manager Node démarré")
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
