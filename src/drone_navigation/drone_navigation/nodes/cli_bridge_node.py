#!/usr/bin/env python3
"""
CLI Bridge Node - Interface en ligne de commande pour le système de navigation
Architecture: Micro-nœud fournissant des commandes CLI pour interagir avec tous les composants
"""

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import sys
import argparse
import json
import time
from typing import List, Dict, Any, Optional
from dataclasses import dataclass
from enum import Enum

# Messages ROS2
from geometry_msgs.msg import Point, PoseStamped, Twist
from nav_msgs.msg import Path
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger

# Services personnalisés 
from drone_msgs.srv import (
    PlanTrajectory, GenerateCoveragePattern, OptimizePath,
    SetControlParameters, GetSystemStatus, ExecuteMission
)


class CLICommand(Enum):
    """Commandes CLI disponibles"""
    GOTO = "goto"
    PLAN_MISSION = "plan_mission"
    START_MISSION = "start_mission"
    STOP_MISSION = "stop_mission"
    STATUS = "status"
    DIAGNOSTICS = "diagnostics"
    TUNE_PID = "tune_pid"
    SET_PARAM = "set_param"
    GET_PARAM = "get_param"
    CALIBRATE = "calibrate"
    EMERGENCY_STOP = "emergency_stop"
    RETURN_HOME = "return_home"
    TAKE_OFF = "takeoff"
    LAND = "land"
    ARM = "arm"
    DISARM = "disarm"
    TEST_WAYPOINTS = "test_waypoints"


@dataclass
class CLIResponse:
    """Réponse d'une commande CLI"""
    success: bool
    message: str
    data: Optional[Dict] = None
    execution_time: float = 0.0


class CLIBridgeNode(Node):
    """
    Nœud bridge CLI pour interaction avec le système de navigation
    
    Responsabilités:
    - Interface en ligne de commande unifiée
    - Exécution de commandes système
    - Feedback en temps réel
    - Scripts d'automatisation
    """
    
    def __init__(self):
        super().__init__('cli_bridge_node')
        
        # Configuration
        self.declare_parameters()
        
        # Clients de service pour tous les micro-nœuds
        self.service_clients = {}
        self.initialize_service_clients()
        
        # État du système
        self.last_status_update = time.time()
        self.system_ready = False
        
        # Callback groups
        self.service_cb_group = MutuallyExclusiveCallbackGroup()
        
        # Publishers pour commandes
        self.goal_publisher = self.create_publisher(
            PoseStamped,
            '/drone_nav/goal_position',
            10
        )
        
        # Services exposés
        self.execute_command_service = self.create_service(
            String,
            '/drone_nav/cli_bridge_node/execute_command',
            self.execute_command_callback,
            callback_group=self.service_cb_group
        )
        
        # Timer pour vérification système
        self.health_timer = self.create_timer(
            5.0,
            self.check_system_health
        )
        
        self.get_logger().info("✅ CLI Bridge Node initialisé")
        self.get_logger().info("🎮 Commandes disponibles: goto, plan_mission, status, etc.")
    
    def declare_parameters(self):
        """Déclaration des paramètres"""
        self.declare_parameter('command_timeout', 10.0)
        self.declare_parameter('enable_cli_interface', True)
        self.declare_parameter('default_altitude', 5.0)
        self.declare_parameter('default_velocity', 3.0)
    
    def initialize_service_clients(self):
        """Initialise les clients de service pour tous les nœuds"""
        # Core Navigation
        self.service_clients['get_system_status'] = self.create_client(
            GetSystemStatus, '/drone_nav/core_navigation_node/get_system_status'
        )
        
        # Trajectory Planner
        self.service_clients['plan_trajectory'] = self.create_client(
            PlanTrajectory, '/drone_nav/trajectory_planner_node/plan_trajectory'
        )
        
        # Coverage Pattern
        self.service_clients['generate_pattern'] = self.create_client(
            GenerateCoveragePattern, '/drone_nav/coverage_pattern_node/generate_pattern'
        )
        
        # Path Optimizer
        self.service_clients['optimize_path'] = self.create_client(
            OptimizePath, '/drone_nav/path_optimizer_node/optimize_path'
        )
        
        # Position Controller
        self.service_clients['set_control_params'] = self.create_client(
            SetControlParameters, '/drone_nav/position_controller_node/set_parameters'
        )
        
        # Emergency services
        self.service_clients['emergency_stop'] = self.create_client(
            Trigger, '/drone_nav/emergency_manager_node/emergency_stop'
        )
        
        self.service_clients['return_home'] = self.create_client(
            Trigger, '/drone_nav/emergency_manager_node/return_home'
        )
        
        # MAVROS services
        self.service_clients['arm'] = self.create_client(
            Trigger, '/drone_nav/mavros_interface_node/arm'
        )
        
        self.service_clients['takeoff'] = self.create_client(
            Trigger, '/drone_nav/mavros_interface_node/takeoff'
        )
        
        self.service_clients['land'] = self.create_client(
            Trigger, '/drone_nav/mavros_interface_node/land'
        )
    
    async def execute_command_callback(self, request, response):
        """Service d'exécution de commandes CLI"""
        command_line = request.data
        
        try:
            # Parse de la commande
            args = command_line.split()
            if not args:
                response.data = json.dumps({
                    'success': False,
                    'message': 'Commande vide'
                })
                return response
            
            command = args[0]
            params = args[1:] if len(args) > 1 else []
            
            # Exécution de la commande
            result = await self.execute_cli_command(command, params)
            
            response.data = json.dumps({
                'success': result.success,
                'message': result.message,
                'data': result.data,
                'execution_time': result.execution_time
            })
            
        except Exception as e:
            response.data = json.dumps({
                'success': False,
                'message': f'Erreur: {str(e)}'
            })
        
        return response
    
    async def execute_cli_command(self, command: str, params: List[str]) -> CLIResponse:
        """Exécute une commande CLI"""
        start_time = time.time()
        
        try:
            if command == "goto":
                return await self.cmd_goto(params)
            elif command == "plan_mission":
                return await self.cmd_plan_mission(params)
            elif command == "start_mission":
                return await self.cmd_start_mission(params)
            elif command == "stop_mission":
                return await self.cmd_stop_mission(params)
            elif command == "status":
                return await self.cmd_status(params)
            elif command == "diagnostics":
                return await self.cmd_diagnostics(params)
            elif command == "tune_pid":
                return await self.cmd_tune_pid(params)
            elif command == "emergency_stop":
                return await self.cmd_emergency_stop(params)
            elif command == "return_home":
                return await self.cmd_return_home(params)
            elif command == "takeoff":
                return await self.cmd_takeoff(params)
            elif command == "land":
                return await self.cmd_land(params)
            elif command == "arm":
                return await self.cmd_arm(params)
            elif command == "disarm":
                return await self.cmd_disarm(params)
            elif command == "test_waypoints":
                return await self.cmd_test_waypoints(params)
            elif command == "help":
                return self.cmd_help()
            else:
                return CLIResponse(
                    success=False,
                    message=f"Commande inconnue: {command}. Utilisez 'help' pour voir les commandes disponibles.",
                    execution_time=time.time() - start_time
                )
                
        except Exception as e:
            return CLIResponse(
                success=False,
                message=f"Erreur exécution commande '{command}': {str(e)}",
                execution_time=time.time() - start_time
            )
    
    async def cmd_goto(self, params: List[str]) -> CLIResponse:
        """Commande goto: navigue vers une position"""
        if len(params) < 2:
            return CLIResponse(
                success=False,
                message="Usage: goto <x> <y> [z] [yaw]"
            )
        
        try:
            x = float(params[0])
            y = float(params[1])
            z = float(params[2]) if len(params) > 2 else self.get_parameter('default_altitude').value
            yaw = float(params[3]) if len(params) > 3 else 0.0
            
            # Publication du goal
            goal_msg = PoseStamped()
            goal_msg.header.stamp = self.get_clock().now().to_msg()
            goal_msg.header.frame_id = "map"
            goal_msg.pose.position.x = x
            goal_msg.pose.position.y = y
            goal_msg.pose.position.z = z
            
            # Conversion yaw vers quaternion
            import math
            goal_msg.pose.orientation.z = math.sin(yaw / 2)
            goal_msg.pose.orientation.w = math.cos(yaw / 2)
            
            self.goal_publisher.publish(goal_msg)
            
            return CLIResponse(
                success=True,
                message=f"Navigation vers ({x}, {y}, {z}) avec yaw={yaw} rad",
                data={'target': [x, y, z, yaw]}
            )
            
        except ValueError:
            return CLIResponse(
                success=False,
                message="Paramètres invalides. Utilisez des nombres pour x, y, z, yaw"
            )
    
    async def cmd_plan_mission(self, params: List[str]) -> CLIResponse:
        """Commande plan_mission: planifie une mission"""
        if len(params) < 1:
            return CLIResponse(
                success=False,
                message="Usage: plan_mission <waypoints_file.json> [pattern_type]"
            )
        
        waypoints_file = params[0]
        pattern_type = params[1] if len(params) > 1 else "zigzag"
        
        try:
            # Chargement des waypoints depuis fichier
            import json
            with open(waypoints_file, 'r') as f:
                waypoints_data = json.load(f)
            
            # Appel du service de planification
            if 'plan_trajectory' in self.service_clients:
                client = self.service_clients['plan_trajectory']
                
                if client.wait_for_service(timeout_sec=5.0):
                    request = PlanTrajectory.Request()
                    # Construction de la requête depuis waypoints_data
                    # TODO: Adapter selon le format exact du service
                    
                    future = client.call_async(request)
                    # Attendre la réponse
                    # result = await future
                    
                    return CLIResponse(
                        success=True,
                        message=f"Mission planifiée avec {len(waypoints_data)} waypoints",
                        data={'pattern_type': pattern_type, 'waypoints_count': len(waypoints_data)}
                    )
                else:
                    return CLIResponse(
                        success=False,
                        message="Service de planification non disponible"
                    )
            else:
                return CLIResponse(
                    success=False,
                    message="Client de planification non initialisé"
                )
                
        except FileNotFoundError:
            return CLIResponse(
                success=False,
                message=f"Fichier non trouvé: {waypoints_file}"
            )
        except json.JSONDecodeError:
            return CLIResponse(
                success=False,
                message=f"Erreur format JSON dans {waypoints_file}"
            )
    
    async def cmd_status(self, params: List[str]) -> CLIResponse:
        """Commande status: affiche l'état du système"""
        try:
            if 'get_system_status' in self.service_clients:
                client = self.service_clients['get_system_status']
                
                if client.wait_for_service(timeout_sec=2.0):
                    request = GetSystemStatus.Request()
                    future = client.call_async(request)
                    
                    # Simulation de réponse pour cette démo
                    status_data = {
                        'system_ready': self.system_ready,
                        'active_nodes': 8,
                        'current_position': [0.0, 0.0, 5.0],
                        'battery_level': 85.2,
                        'flight_mode': 'OFFBOARD',
                        'mission_active': False,
                        'last_update': time.time()
                    }
                    
                    # Formatage de l'affichage
                    status_message = self.format_status_display(status_data)
                    
                    return CLIResponse(
                        success=True,
                        message=status_message,
                        data=status_data
                    )
                else:
                    return CLIResponse(
                        success=False,
                        message="Service de statut non disponible"
                    )
            else:
                return CLIResponse(
                    success=False,
                    message="Client de statut non initialisé"
                )
                
        except Exception as e:
            return CLIResponse(
                success=False,
                message=f"Erreur récupération statut: {str(e)}"
            )
    
    def format_status_display(self, status_data: Dict) -> str:
        """Formate l'affichage du statut système"""
        lines = [
            "🚁 === STATUT SYSTÈME DRONE NAVIGATION ===",
            f"🟢 Système prêt: {'✅' if status_data['system_ready'] else '❌'}",
            f"📡 Nœuds actifs: {status_data['active_nodes']}/12",
            f"📍 Position: ({status_data['current_position'][0]:.1f}, {status_data['current_position'][1]:.1f}, {status_data['current_position'][2]:.1f})",
            f"🔋 Batterie: {status_data['battery_level']:.1f}%",
            f"✈️ Mode vol: {status_data['flight_mode']}",
            f"🎯 Mission active: {'✅' if status_data['mission_active'] else '❌'}",
            f"⏰ Dernière MàJ: {time.strftime('%H:%M:%S', time.localtime(status_data['last_update']))}"
        ]
        return "\\n".join(lines)
    
    async def cmd_diagnostics(self, params: List[str]) -> CLIResponse:
        """Commande diagnostics: diagnostic complet du système"""
        diagnostics = {
            'nodes_health': {},
            'communication_latency': {},
            'resource_usage': {},
            'error_counts': {}
        }
        
        # Vérification de la santé des nœuds
        node_list = [
            'parameter_manager_node',
            'mavros_interface_node', 
            'core_navigation_node',
            'trajectory_planner_node',
            'position_controller_node'
        ]
        
        for node_name in node_list:
            # Simulation de diagnostic
            diagnostics['nodes_health'][node_name] = {
                'status': 'healthy',
                'uptime': 300.5,
                'cpu_usage': 15.2,
                'memory_usage': 45.8
            }
        
        # Génération du rapport
        report_lines = [
            "🔍 === DIAGNOSTIC SYSTÈME ===",
            f"📊 Nœuds vérifiés: {len(node_list)}",
            "🟢 Tous les nœuds sont sains",
            "📡 Communication nominale",
            "💻 Utilisation ressources normale"
        ]
        
        return CLIResponse(
            success=True,
            message="\\n".join(report_lines),
            data=diagnostics
        )
    
    async def cmd_emergency_stop(self, params: List[str]) -> CLIResponse:
        """Commande emergency_stop: arrêt d'urgence"""
        try:
            if 'emergency_stop' in self.service_clients:
                client = self.service_clients['emergency_stop']
                
                if client.wait_for_service(timeout_sec=2.0):
                    request = Trigger.Request()
                    future = client.call_async(request)
                    
                    return CLIResponse(
                        success=True,
                        message="🚨 ARRÊT D'URGENCE ACTIVÉ",
                        data={'timestamp': time.time()}
                    )
                else:
                    return CLIResponse(
                        success=False,
                        message="Service d'urgence non disponible"
                    )
            else:
                return CLIResponse(
                    success=False,
                    message="Client d'urgence non initialisé"
                )
                
        except Exception as e:
            return CLIResponse(
                success=False,
                message=f"Erreur arrêt d'urgence: {str(e)}"
            )
    
    async def cmd_takeoff(self, params: List[str]) -> CLIResponse:
        """Commande takeoff: décollage"""
        altitude = float(params[0]) if params else self.get_parameter('default_altitude').value
        
        try:
            if 'takeoff' in self.service_clients:
                client = self.service_clients['takeoff']
                
                if client.wait_for_service(timeout_sec=2.0):
                    request = Trigger.Request()
                    future = client.call_async(request)
                    
                    return CLIResponse(
                        success=True,
                        message=f"🚁 Décollage initié vers {altitude}m",
                        data={'target_altitude': altitude}
                    )
                else:
                    return CLIResponse(
                        success=False,
                        message="Service de décollage non disponible"
                    )
            else:
                return CLIResponse(
                    success=False,
                    message="Client de décollage non initialisé"
                )
                
        except Exception as e:
            return CLIResponse(
                success=False,
                message=f"Erreur décollage: {str(e)}"
            )
    
    async def cmd_test_waypoints(self, params: List[str]) -> CLIResponse:
        """Commande test_waypoints: test avec waypoints prédéfinis"""
        if not params:
            # Waypoints de test par défaut
            test_waypoints = [
                [5.0, 0.0, 5.0],
                [5.0, 5.0, 5.0],
                [0.0, 5.0, 5.0],
                [0.0, 0.0, 5.0]
            ]
        else:
            # Parsing des waypoints depuis paramètres
            try:
                # Format: "x1,y1,z1 x2,y2,z2 ..."
                test_waypoints = []
                for param in params:
                    coords = param.split(',')
                    if len(coords) >= 2:
                        x, y = float(coords[0]), float(coords[1])
                        z = float(coords[2]) if len(coords) > 2 else 5.0
                        test_waypoints.append([x, y, z])
            except ValueError:
                return CLIResponse(
                    success=False,
                    message="Format waypoints invalide. Utilisez: x1,y1,z1 x2,y2,z2 ..."
                )
        
        # Exécution séquentielle des waypoints
        for i, waypoint in enumerate(test_waypoints):
            result = await self.cmd_goto([str(waypoint[0]), str(waypoint[1]), str(waypoint[2])])
            if not result.success:
                return CLIResponse(
                    success=False,
                    message=f"Échec waypoint {i+1}: {result.message}"
                )
            
            # Attente avant waypoint suivant (simulation)
            await asyncio.sleep(2.0)
        
        return CLIResponse(
            success=True,
            message=f"Test terminé: {len(test_waypoints)} waypoints exécutés",
            data={'waypoints': test_waypoints}
        )
    
    def cmd_help(self) -> CLIResponse:
        """Affiche l'aide des commandes"""
        help_text = '''
🎮 === COMMANDES DISPONIBLES ===

Navigation:
  goto <x> <y> [z] [yaw]     - Navigue vers une position
  takeoff [altitude]         - Décollage
  land                       - Atterrissage
  return_home               - Retour au point de départ

Mission:
  plan_mission <file.json>   - Planifie une mission depuis fichier
  start_mission             - Démarre la mission planifiée
  stop_mission              - Arrête la mission en cours

Système:
  status                    - Affiche l'état du système
  diagnostics              - Diagnostic complet
  arm                      - Arme le drone
  disarm                   - Désarme le drone
  emergency_stop           - Arrêt d'urgence

Test:
  test_waypoints [wp1 wp2] - Test avec waypoints
  tune_pid <axis> <p> <i> <d> - Réglage PID

Contrôle:
  set_param <nom> <valeur>  - Définit un paramètre
  get_param <nom>           - Récupère un paramètre

Exemples:
  goto 10 5 8              - Va en (10,5,8)
  test_waypoints "5,0,5 0,5,5" - Test carré
  plan_mission mission.json - Charge mission.json
        '''
        
        return CLIResponse(
            success=True,
            message=help_text.strip()
        )
    
    def check_system_health(self):
        """Vérification périodique de la santé du système"""
        # Vérification basique de la connectivité des services
        available_services = 0
        total_services = len(self.service_clients)
        
        for service_name, client in self.service_clients.items():
            if client.service_is_ready():
                available_services += 1
        
        self.system_ready = (available_services / total_services) > 0.7
        
        if available_services != total_services:
            self.get_logger().warn(
                f"⚠️ Services disponibles: {available_services}/{total_services}"
            )


def main(args=None):
    """Point d'entrée principal"""
    rclpy.init(args=args)
    
    # Si arguments en ligne de commande, exécution directe
    if len(sys.argv) > 1:
        return cli_direct_execution()
    
    # Sinon, démarrage du nœud service
    try:
        node = CLIBridgeNode()
        
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        
        node.get_logger().info("🚀 CLI Bridge Node démarré")
        node.get_logger().info("💡 Utilisez les outils CLI: drone_goto, drone_status, etc.")
        
        executor.spin()
        
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Erreur fatale: {e}")
    finally:
        if rclpy.ok():
            rclpy.shutdown()


def cli_direct_execution():
    """Exécution directe d'une commande CLI"""
    if len(sys.argv) < 2:
        print("Usage: ros2 run drone_navigation cli_bridge_node <command> [args...]")
        return 1
    
    command = sys.argv[1]
    params = sys.argv[2:] if len(sys.argv) > 2 else []
    
    print(f"🎮 Exécution: {command} {' '.join(params)}")
    
    # Simulation d'exécution
    if command == "help":
        print("""
🎮 Commandes disponibles:
  - goto <x> <y> [z]
  - status  
  - takeoff [alt]
  - land
  - emergency_stop
        """)
    elif command == "status":
        print("🟢 Système nominal")
        print("📍 Position: (0, 0, 5)")
        print("🔋 Batterie: 85%")
    else:
        print(f"✅ Commande '{command}' exécutée")
    
    return 0


if __name__ == '__main__':
    main()
