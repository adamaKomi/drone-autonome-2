#!/usr/bin/env python3
"""
=============================================================================
DRONE MISSION NODE - Nœud de gestion et exécution de missions
=============================================================================
Auteur: Assistant IA
Date: 2025-08-29
Version: 1.0.0

Description:
    Nœud ROS2 spécialisé pour la planification et l'exécution de missions.
    Gère les missions complexes, la logique de décision et l'orchestration.
    
Responsabilités:
    - Planification de missions complexes
    - Exécution séquentielle de tâches
    - Gestion des conditions et événements
    - Interface avec les nœuds navigation et interface
    - Sauvegarde et chargement de missions

Utilisation:
    ros2 run drone_mission mission_node
=============================================================================
"""

import json
import time
import threading
import os
from enum import Enum
from typing import Optional, Tuple, List, Dict, Any, Union
from dataclasses import dataclass, asdict
import uuid

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from rclpy.parameter import Parameter
from rclpy.callback_groups import ReentrantCallbackGroup

# Message types
from std_msgs.msg import String, Bool, Float64
from std_srvs.srv import SetBool, Trigger
from geometry_msgs.msg import Point, PoseStamped
import geometry_msgs.msg

# Services personnalisés
from drone_mission.srv import MissionControl, LoadMission, CreateMission
from drone_mission.msg import MissionStatus


class MissionState(Enum):
    """États possibles d'une mission"""
    IDLE = "IDLE"                    # En attente
    LOADING = "LOADING"              # Chargement mission
    PLANNING = "PLANNING"            # Planification
    READY = "READY"                  # Prête à exécuter
    EXECUTING = "EXECUTING"          # En cours d'exécution
    PAUSED = "PAUSED"               # En pause
    COMPLETED = "COMPLETED"          # Terminée avec succès
    FAILED = "FAILED"               # Échec
    ABORTED = "ABORTED"             # Abandonnée


class TaskType(Enum):
    """Types de tâches possibles"""
    ARM = "ARM"                      # Armer le drone
    DISARM = "DISARM"               # Désarmer le drone
    TAKEOFF = "TAKEOFF"             # Décollage
    LAND = "LAND"                   # Atterrissage
    GOTO = "GOTO"                   # Aller à position
    WAYPOINT = "WAYPOINT"           # Waypoint avec paramètres
    SET_MODE = "SET_MODE"           # Changer mode de vol
    WAIT = "WAIT"                   # Attendre (durée)
    WAIT_FOR_CONDITION = "WAIT_FOR_CONDITION"  # Attendre condition
    RTL = "RTL"                     # Retour au point de départ
    SURVEY = "SURVEY"               # Mission de reconnaissance
    ORBIT = "ORBIT"                 # Vol en orbite
    CUSTOM = "CUSTOM"               # Tâche personnalisée
    # Nouvelles tâches pour la pollinisation
    DETECT_FLOWERS = "DETECT_FLOWERS"           # Détecter des fleurs
    APPROACH_FLOWER = "APPROACH_FLOWER"         # Approche précise d'une fleur
    POLLINATE = "POLLINATE"                     # Action de pollinisation
    COLLECT_DATA = "COLLECT_DATA"               # Collecter des données
    SCAN_AREA = "SCAN_AREA"                     # Scanner une zone pour fleurs


class TaskStatus(Enum):
    """États d'une tâche"""
    PENDING = "PENDING"              # En attente
    EXECUTING = "EXECUTING"          # En cours
    COMPLETED = "COMPLETED"          # Terminée
    FAILED = "FAILED"               # Échec
    SKIPPED = "SKIPPED"             # Ignorée


@dataclass
class MissionTask:
    """Structure d'une tâche de mission"""
    id: str                          # ID unique
    type: TaskType                   # Type de tâche
    parameters: Dict[str, Any]       # Paramètres spécifiques
    description: str = ""            # Description
    status: TaskStatus = TaskStatus.PENDING
    timeout: float = 60.0           # Timeout en secondes
    retry_count: int = 0            # Nombre de tentatives
    max_retries: int = 3            # Nombre max de tentatives
    required: bool = True           # Tâche obligatoire
    conditions: Dict[str, Any] = None  # Conditions d'exécution
    created_at: float = 0.0
    started_at: float = 0.0
    completed_at: float = 0.0
    
    def __post_init__(self):
        if self.created_at == 0.0:
            self.created_at = time.time()
        if self.conditions is None:
            self.conditions = {}


@dataclass
class Mission:
    """Structure d'une mission complète"""
    id: str                          # ID unique de la mission
    name: str                        # Nom de la mission
    description: str = ""            # Description
    tasks: List[MissionTask] = None  # Liste des tâches
    state: MissionState = MissionState.IDLE
    created_at: float = 0.0
    started_at: float = 0.0
    completed_at: float = 0.0
    current_task_index: int = 0
    metadata: Dict[str, Any] = None  # Métadonnées additionnelles
    
    def __post_init__(self):
        if self.tasks is None:
            self.tasks = []
        if self.created_at == 0.0:
            self.created_at = time.time()
        if self.metadata is None:
            self.metadata = {}


class MissionExecutor:
    """Exécuteur de missions"""
    
    def __init__(self, logger, interface_client, navigation_client):
        self.logger = logger
        self.interface_client = interface_client
        self.navigation_client = navigation_client
        
        self.current_mission: Optional[Mission] = None
        self.execution_thread: Optional[threading.Thread] = None
        self.stop_execution = False
        self.pause_execution = False
        
        # État du drone
        self.drone_state = "UNKNOWN"
        self.drone_armed = False
        self.drone_mode = "UNKNOWN"
        self.drone_connected = False
        self.drone_position = [0.0, 0.0, 0.0]
        self.drone_battery = 100.0
        
        # Callbacks pour événements
        self.mission_callbacks = []
        self.task_callbacks = []
        
    def load_mission(self, mission: Mission) -> bool:
        """Charge une mission pour exécution"""
        if self.current_mission and self.current_mission.state == MissionState.EXECUTING:
            self.logger.warn("⚠️ Mission en cours, arrêt requis avant chargement")
            return False
            
        self.current_mission = mission
        self.current_mission.state = MissionState.READY
        self.logger.info(f"📋 Mission '{mission.name}' chargée ({len(mission.tasks)} tâches)")
        
        self._notify_mission_callbacks("LOADED", mission)
        return True
        
    def start_mission(self) -> bool:
        """Démarre l'exécution de la mission"""
        if not self.current_mission:
            self.logger.error("❌ Aucune mission chargée")
            return False
            
        if self.current_mission.state != MissionState.READY:
            self.logger.error(f"❌ Mission non prête (état: {self.current_mission.state})")
            return False
            
        # Vérifications préalables
        if not self.drone_connected:
            self.logger.error("❌ Drone non connecté")
            return False
            
        self.current_mission.state = MissionState.EXECUTING
        self.current_mission.started_at = time.time()
        self.current_mission.current_task_index = 0
        self.stop_execution = False
        self.pause_execution = False
        
        # Démarrer le thread d'exécution
        self.execution_thread = threading.Thread(target=self._execute_mission)
        self.execution_thread.start()
        
        self.logger.info(f"🚀 Exécution mission '{self.current_mission.name}' démarrée")
        self._notify_mission_callbacks("STARTED", self.current_mission)
        return True
        
    def pause_mission(self):
        """Met en pause la mission"""
        if self.current_mission and self.current_mission.state == MissionState.EXECUTING:
            self.pause_execution = True
            self.current_mission.state = MissionState.PAUSED
            self.logger.info("⏸️ Mission mise en pause")
            self._notify_mission_callbacks("PAUSED", self.current_mission)
            
    def resume_mission(self):
        """Reprend la mission"""
        if self.current_mission and self.current_mission.state == MissionState.PAUSED:
            self.pause_execution = False
            self.current_mission.state = MissionState.EXECUTING
            self.logger.info("▶️ Mission reprise")
            self._notify_mission_callbacks("RESUMED", self.current_mission)
            
    def stop_mission(self):
        """Arrête la mission"""
        if self.current_mission and self.current_mission.state in [MissionState.EXECUTING, MissionState.PAUSED]:
            self.stop_execution = True
            self.current_mission.state = MissionState.ABORTED
            self.logger.info("🛑 Mission arrêtée")
            self._notify_mission_callbacks("STOPPED", self.current_mission)
            
    def emergency_abort(self):
        """Abandon d'urgence de la mission"""
        self.stop_execution = True
        if self.current_mission:
            self.current_mission.state = MissionState.ABORTED
            self.logger.warn("🚨 ABANDON D'URGENCE de la mission")
            self._notify_mission_callbacks("EMERGENCY_ABORT", self.current_mission)
            
    def update_drone_state(self, state: str, armed: bool, mode: str, connected: bool,
                          position: List[float], battery: float):
        """Met à jour l'état du drone"""
        self.drone_state = state
        self.drone_armed = armed
        self.drone_mode = mode
        self.drone_connected = connected
        self.drone_position = position.copy()
        self.drone_battery = battery
        
    def _execute_mission(self):
        """Thread principal d'exécution de mission"""
        try:
            mission = self.current_mission
            self.logger.info(f"🎯 Début exécution mission '{mission.name}'")
            
            while (mission.current_task_index < len(mission.tasks) and 
                   not self.stop_execution):
                
                # Attendre si en pause
                while self.pause_execution and not self.stop_execution:
                    time.sleep(0.1)
                    
                if self.stop_execution:
                    break
                    
                # Exécuter la tâche courante
                task = mission.tasks[mission.current_task_index]
                success = self._execute_task(task)
                
                if success:
                    mission.current_task_index += 1
                    self.logger.info(f"✅ Tâche {task.type.value} terminée ({mission.current_task_index}/{len(mission.tasks)})")
                else:
                    if task.required:
                        self.logger.error(f"❌ Tâche critique {task.type.value} échouée, arrêt mission")
                        mission.state = MissionState.FAILED
                        break
                    else:
                        self.logger.warn(f"⚠️ Tâche optionnelle {task.type.value} échouée, continuation")
                        mission.current_task_index += 1
                        
            # Fin de mission
            if self.stop_execution:
                mission.state = MissionState.ABORTED
                self.logger.info("🛑 Mission abandonnée")
            elif mission.current_task_index >= len(mission.tasks):
                mission.state = MissionState.COMPLETED
                mission.completed_at = time.time()
                duration = mission.completed_at - mission.started_at
                self.logger.info(f"🎉 Mission '{mission.name}' terminée avec succès en {duration:.1f}s")
            else:
                mission.state = MissionState.FAILED
                self.logger.error(f"💥 Mission '{mission.name}' échouée")
                
            self._notify_mission_callbacks("FINISHED", mission)
            
        except Exception as e:
            self.logger.error(f"❌ Erreur fatale dans exécution mission: {e}")
            if self.current_mission:
                self.current_mission.state = MissionState.FAILED
                self._notify_mission_callbacks("ERROR", self.current_mission)
                
    def _execute_task(self, task: MissionTask) -> bool:
        """Exécute une tâche individuelle"""
        self.logger.info(f"🔄 Exécution tâche: {task.type.value} - {task.description}")
        
        task.status = TaskStatus.EXECUTING
        task.started_at = time.time()
        self._notify_task_callbacks("STARTED", task)
        
        try:
            # Vérifier les conditions d'exécution
            if not self._check_task_conditions(task):
                task.status = TaskStatus.SKIPPED
                self.logger.info(f"⏭️ Tâche {task.type.value} ignorée (conditions non remplies)")
                return True
                
            # Exécuter selon le type
            success = False
            if task.type == TaskType.ARM:
                success = self._execute_arm_task(task)
            elif task.type == TaskType.DISARM:
                success = self._execute_disarm_task(task)
            elif task.type == TaskType.TAKEOFF:
                success = self._execute_takeoff_task(task)
            elif task.type == TaskType.LAND:
                success = self._execute_land_task(task)
            elif task.type == TaskType.GOTO:
                success = self._execute_goto_task(task)
            elif task.type == TaskType.WAYPOINT:
                success = self._execute_waypoint_task(task)
            elif task.type == TaskType.SET_MODE:
                success = self._execute_set_mode_task(task)
            elif task.type == TaskType.WAIT:
                success = self._execute_wait_task(task)
            elif task.type == TaskType.WAIT_FOR_CONDITION:
                success = self._execute_wait_condition_task(task)
            elif task.type == TaskType.RTL:
                success = self._execute_rtl_task(task)
            elif task.type == TaskType.SURVEY:
                success = self._execute_survey_task(task)
            elif task.type == TaskType.ORBIT:
                success = self._execute_orbit_task(task)
            else:
                self.logger.error(f"❌ Type de tâche non supporté: {task.type.value}")
                success = False
                
            # Mise à jour du statut
            if success:
                task.status = TaskStatus.COMPLETED
                task.completed_at = time.time()
                self._notify_task_callbacks("COMPLETED", task)
            else:
                task.retry_count += 1
                if task.retry_count < task.max_retries:
                    self.logger.warn(f"🔄 Nouvelle tentative {task.retry_count + 1}/{task.max_retries} pour {task.type.value}")
                    return self._execute_task(task)  # Récursion pour retry
                else:
                    task.status = TaskStatus.FAILED
                    task.completed_at = time.time()
                    self._notify_task_callbacks("FAILED", task)
                    
            return success
            
        except Exception as e:
            self.logger.error(f"❌ Erreur dans exécution tâche {task.type.value}: {e}")
            task.status = TaskStatus.FAILED
            task.completed_at = time.time()
            self._notify_task_callbacks("ERROR", task)
            return False
            
    def _check_task_conditions(self, task: MissionTask) -> bool:
        """Vérifie les conditions d'exécution d'une tâche"""
        conditions = task.conditions
        
        # Vérification de l'état du drone
        if 'drone_state' in conditions:
            if self.drone_state != conditions['drone_state']:
                return False
                
        if 'armed' in conditions:
            if self.drone_armed != conditions['armed']:
                return False
                
        if 'mode' in conditions:
            if self.drone_mode != conditions['mode']:
                return False
                
        # Vérification de batterie minimum
        if 'min_battery' in conditions:
            if self.drone_battery < conditions['min_battery']:
                self.logger.warn(f"⚠️ Batterie trop faible: {self.drone_battery}% < {conditions['min_battery']}%")
                return False
                
        return True
        
    def _execute_arm_task(self, task: MissionTask) -> bool:
        """Exécute une tâche d'armement"""
        return self._call_interface_service("ARM")
        
    def _execute_disarm_task(self, task: MissionTask) -> bool:
        """Exécute une tâche de désarmement"""
        return self._call_interface_service("DISARM")
        
    def _execute_takeoff_task(self, task: MissionTask) -> bool:
        """Exécute une tâche de décollage"""
        altitude = task.parameters.get('altitude', 3.0)
        return self._call_interface_service(f"TAKEOFF:{altitude}")
        
    def _execute_land_task(self, task: MissionTask) -> bool:
        """Exécute une tâche d'atterrissage"""
        return self._call_interface_service("LAND")
        
    def _execute_goto_task(self, task: MissionTask) -> bool:
        """Exécute une tâche de navigation vers position"""
        x = task.parameters.get('x', 0.0)
        y = task.parameters.get('y', 0.0)
        z = task.parameters.get('z', 3.0)
        yaw = task.parameters.get('yaw', 0.0)
        
        return self._call_navigation_service("goto", f"{x}:{y}:{z}:{yaw}")
        
    def _execute_waypoint_task(self, task: MissionTask) -> bool:
        """Exécute une tâche de waypoint avec paramètres"""
        x = task.parameters.get('x', 0.0)
        y = task.parameters.get('y', 0.0)
        z = task.parameters.get('z', 3.0)
        yaw = task.parameters.get('yaw', 0.0)
        tolerance = task.parameters.get('tolerance', 0.5)
        hold_time = task.parameters.get('hold_time', 0.0)
        max_vel = task.parameters.get('max_velocity', 2.0)
        
        waypoint_cmd = f"{x}:{y}:{z}:{yaw}:{tolerance}:{hold_time}:{max_vel}"
        return self._call_navigation_service("add_waypoint", waypoint_cmd)
        
    def _execute_set_mode_task(self, task: MissionTask) -> bool:
        """Exécute un changement de mode"""
        mode = task.parameters.get('mode', 'GUIDED')
        return self._call_interface_service(f"SET_MODE:{mode}")
        
    def _execute_wait_task(self, task: MissionTask) -> bool:
        """Exécute une attente temporisée"""
        duration = task.parameters.get('duration', 1.0)
        self.logger.info(f"⏱️ Attente {duration}s...")
        
        start_time = time.time()
        while time.time() - start_time < duration:
            if self.stop_execution:
                return False
            time.sleep(0.1)
            
        return True
        
    def _execute_wait_condition_task(self, task: MissionTask) -> bool:
        """Exécute une attente conditionnelle"""
        # TODO: Implémenter selon les besoins spécifiques
        self.logger.info("⏳ Attente de condition...")
        return True
        
    def _execute_rtl_task(self, task: MissionTask) -> bool:
        """Exécute un retour au point de départ"""
        return self._call_interface_service("SET_MODE:RTL")
        
    def _execute_survey_task(self, task: MissionTask) -> bool:
        """Exécute une mission de reconnaissance"""
        # TODO: Implémenter la logique de reconnaissance
        self.logger.info("📸 Mission de reconnaissance...")
        return True
        
    def _execute_orbit_task(self, task: MissionTask) -> bool:
        """Exécute un vol en orbite"""
        # TODO: Implémenter la logique d'orbite
        self.logger.info("🔄 Vol en orbite...")
        return True
        
    def _call_interface_service(self, command: str) -> bool:
        """Appelle un service du nœud interface"""
        try:
            if command == "ARM":
                # Appel au service d'armement
                from std_srvs.srv import SetBool
                client = self.create_client(SetBool, '/drone_interface/arm')
                if not client.wait_for_service(timeout_sec=5.0):
                    self.logger.error("❌ Service /drone_interface/arm non disponible")
                    return False
                
                request = SetBool.Request()
                request.data = True
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                
                if future.result() and future.result().success:
                    self.logger.info("✅ Drone armé avec succès")
                    return True
                else:
                    self.logger.error("❌ Échec armement drone")
                    return False
                    
            elif command == "DISARM":
                # Appel au service de désarmement
                from std_srvs.srv import SetBool
                client = self.create_client(SetBool, '/drone_interface/disarm')
                if not client.wait_for_service(timeout_sec=5.0):
                    return False
                    
                request = SetBool.Request()
                request.data = True
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                
                return future.result() and future.result().success
                
            elif command.startswith("TAKEOFF:"):
                altitude = float(command.split(":")[1])
                # Utiliser le service de navigation pour takeoff
                return self._call_navigation_service("set_position", f"0:0:{altitude}:0")
                
            elif command.startswith("SET_MODE:"):
                mode = command.split(":")[1]
                from drone_interface.srv import SetFlightMode
                client = self.create_client(SetFlightMode, '/drone_interface/set_flight_mode')
                if not client.wait_for_service(timeout_sec=5.0):
                    return False
                    
                request = SetFlightMode.Request()
                request.mode = mode
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                
                return future.result() and future.result().success
                
            else:
                self.logger.error(f"❌ Commande interface inconnue: {command}")
                return False
                
        except Exception as e:
            self.logger.error(f"❌ Erreur service interface: {e}")
            return False
            
    def _call_navigation_service(self, service_type: str, command: str) -> bool:
        """Appelle un service du nœud navigation"""
        try:
            if service_type == "goto" or service_type == "set_position":
                # Parse coordinates: "x:y:z:yaw"
                coords = command.split(":")
                x, y, z = float(coords[0]), float(coords[1]), float(coords[2])
                yaw = float(coords[3]) if len(coords) > 3 else 0.0
                
                from drone_navigation.srv import SetPosition
                client = self.create_client(SetPosition, '/navigation/set_position')
                if not client.wait_for_service(timeout_sec=5.0):
                    self.logger.error("❌ Service /navigation/set_position non disponible")
                    return False
                
                request = SetPosition.Request()
                request.position.x = x
                request.position.y = y
                request.position.z = z
                request.yaw = yaw
                
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
                
                if future.result() and future.result().success:
                    self.logger.info(f"✅ Navigation vers ({x:.1f}, {y:.1f}, {z:.1f}) démarrée")
                    
                    # Attendre que le drone atteigne la position (avec timeout)
                    return self._wait_for_position_reached(x, y, z, tolerance=1.0, timeout=30.0)
                else:
                    self.logger.error("❌ Échec commande de navigation")
                    return False
                    
            elif service_type == "add_waypoint":
                # Parse waypoint: "x:y:z:yaw:tolerance:hold_time:max_vel"
                parts = command.split(":")
                x, y, z = float(parts[0]), float(parts[1]), float(parts[2])
                
                from drone_navigation.srv import AddWaypoint
                client = self.create_client(AddWaypoint, '/navigation/set_waypoints')
                if not client.wait_for_service(timeout_sec=5.0):
                    return False
                
                request = AddWaypoint.Request()
                waypoint = geometry_msgs.msg.Point()
                waypoint.x = x
                waypoint.y = y  
                waypoint.z = z
                request.waypoints = [waypoint]
                
                future = client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                
                return future.result() and future.result().success
                
            else:
                self.logger.error(f"❌ Type de service navigation inconnu: {service_type}")
                return False
                
        except Exception as e:
            self.logger.error(f"❌ Erreur service navigation: {e}")
            return False
            
    def _wait_for_position_reached(self, target_x: float, target_y: float, target_z: float, 
                                  tolerance: float = 1.0, timeout: float = 30.0) -> bool:
        """Attend que le drone atteigne la position cible"""
        start_time = time.time()
        
        while time.time() - start_time < timeout and not self.stop_execution:
            # Obtenir la position actuelle depuis /mavros/local_position/pose
            try:
                from geometry_msgs.msg import PoseStamped
                import rclpy
                
                # Créer un subscriber temporaire pour obtenir la position
                current_pos = None
                
                def pos_callback(msg):
                    nonlocal current_pos
                    current_pos = msg
                
                temp_sub = self.create_subscription(
                    PoseStamped, '/mavros/local_position/pose', pos_callback, 1
                )
                
                # Attendre de recevoir la position
                start_wait = time.time()
                while current_pos is None and time.time() - start_wait < 2.0:
                    rclpy.spin_once(self, timeout_sec=0.1)
                
                self.destroy_subscription(temp_sub)
                
                if current_pos:
                    distance = math.sqrt(
                        (target_x - current_pos.pose.position.x)**2 +
                        (target_y - current_pos.pose.position.y)**2 +
                        (target_z - current_pos.pose.position.z)**2
                    )
                    
                    if distance <= tolerance:
                        self.logger.info(f"✅ Position cible atteinte (distance: {distance:.2f}m)")
                        return True
                        
                    self.logger.debug(f"🧭 Distance à la cible: {distance:.2f}m")
                    
            except Exception as e:
                self.logger.error(f"❌ Erreur vérification position: {e}")
                
            time.sleep(0.5)
            
        self.logger.warn(f"⚠️ Timeout atteint pour atteindre position ({target_x}, {target_y}, {target_z})")
        return False
            
    def add_mission_callback(self, callback):
        """Ajoute un callback pour les événements de mission"""
        self.mission_callbacks.append(callback)
        
    def add_task_callback(self, callback):
        """Ajoute un callback pour les événements de tâche"""
        self.task_callbacks.append(callback)
        
    def _notify_mission_callbacks(self, event: str, mission: Mission):
        """Notifie les callbacks de mission"""
        for callback in self.mission_callbacks:
            try:
                callback(event, mission)
            except Exception as e:
                self.logger.error(f"❌ Erreur callback mission: {e}")
                
    def _notify_task_callbacks(self, event: str, task: MissionTask):
        """Notifie les callbacks de tâche"""
        for callback in self.task_callbacks:
            try:
                callback(event, task)
            except Exception as e:
                self.logger.error(f"❌ Erreur callback tâche: {e}")
                
    def get_mission_status(self) -> Dict[str, Any]:
        """Retourne l'état actuel de la mission"""
        if not self.current_mission:
            return {"state": "NO_MISSION"}
            
        mission = self.current_mission
        current_task = None
        if 0 <= mission.current_task_index < len(mission.tasks):
            current_task = mission.tasks[mission.current_task_index]
            
        return {
            "mission_id": mission.id,
            "mission_name": mission.name,
            "state": mission.state.value,
            "progress": f"{mission.current_task_index}/{len(mission.tasks)}",
            "current_task": current_task.type.value if current_task else None,
            "current_task_status": current_task.status.value if current_task else None
        }


class MissionNode(Node):
    """
    Nœud de gestion des missions pour le drone
    """
    
    def __init__(self):
        super().__init__('drone_mission')
        
        self.logger = self.get_logger()
        self.logger.info("📋 Initialisation du nœud de mission...")
        
        # Configuration QoS
        self._setup_qos_profiles()
        
        # Initialisation de l'exécuteur
        self.mission_executor = MissionExecutor(self.logger, None, None)
        
        # Configuration des souscripteurs
        self._setup_subscribers()
        
        # Configuration des publishers
        self._setup_publishers()
        
        # Configuration des services
        self._setup_services()
        
        # Timer pour publication d'état
        self.status_timer = self.create_timer(1.0, self._publish_status)
        
        # Répertoire des missions
        self.mission_directory = os.path.expanduser("~/ros2_ws/src/drone_mission/missions")
        os.makedirs(self.mission_directory, exist_ok=True)
        
        self.logger.info("✅ Nœud de mission initialisé!")
        self._print_node_info()
        
    def _setup_qos_profiles(self):
        """Configure les profils QoS"""
        self.qos_critical = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
    def _setup_subscribers(self):
        """Configure les souscripteurs"""
        # État du drone depuis interface
        self.drone_status_sub = self.create_subscription(
            String,
            '/drone/status',
            self._drone_status_callback,
            self.qos_critical
        )
        
        # Batterie du drone
        self.battery_sub = self.create_subscription(
            Float64,
            '/drone/battery_percentage',
            self._battery_callback,
            self.qos_critical
        )
        
    def _setup_publishers(self):
        """Configure les publishers"""
        # État de la mission avec message personnalisé
        self.mission_status_pub = self.create_publisher(
            MissionStatus,
            '/drone/mission/status',
            self.qos_critical
        )
        
        # Progression de mission (format simple)
        self.mission_progress_pub = self.create_publisher(
            String,
            '/drone/mission/progress',
            self.qos_critical
        )
        
    def _setup_services(self):
        """Configure les services"""
        # Service de contrôle de mission  
        self.mission_control_service = self.create_service(
            MissionControl,
            '/drone/mission/control',
            self._handle_mission_control
        )
        
        # Service de chargement de mission
        self.load_mission_service = self.create_service(
            LoadMission,
            '/drone/mission/load',
            self._handle_load_mission
        )
        
        # Service de création de mission
        self.create_mission_service = self.create_service(
            CreateMission,
            '/drone/mission/create',
            self._handle_create_mission
        )
        
    def _drone_status_callback(self, msg: String):
        """Callback pour l'état du drone"""
        # Parse: "STATE:MODE:ARMED:CONNECTED"
        parts = msg.data.split(':')
        if len(parts) >= 4:
            state = parts[0]
            mode = parts[1]
            armed = parts[2] == 'True'
            connected = parts[3] == 'True'
            
            # Position par défaut (devrait venir d'un autre topic)
            position = [0.0, 0.0, 0.0]
            battery = getattr(self, '_last_battery', 100.0)
            
            self.mission_executor.update_drone_state(
                state, armed, mode, connected, position, battery
            )
            
    def _battery_callback(self, msg: Float64):
        """Callback pour la batterie"""
        self._last_battery = msg.data
        
    def _handle_mission_control(self, request, response):
        """Gère le contrôle de mission"""
        try:
            command = request.command.upper()
            
            if command == "START":
                success = self.mission_executor.start_mission()
                response.message = "Mission démarrée" if success else "Impossible de démarrer"
                response.success = success
                
            elif command == "PAUSE":
                self.mission_executor.pause_mission()
                response.message = "Mission en pause"
                response.success = True
                
            elif command == "RESUME":
                self.mission_executor.resume_mission()
                response.message = "Mission reprise"
                response.success = True
                
            elif command == "STOP":
                self.mission_executor.stop_mission()
                response.message = "Mission arrêtée"
                response.success = True
                
            elif command == "EMERGENCY":
                self.mission_executor.emergency_abort()
                response.message = "Abandon d'urgence"
                response.success = True
                
            else:
                response.message = f"Commande inconnue: {command}"
                response.success = False
                
        except Exception as e:
            self.logger.error(f"❌ Erreur mission_control: {e}")
            response.message = f"ERREUR: {str(e)}"
            response.success = False
            
        return response
        
    def _handle_load_mission(self, request, response):
        """Gère le chargement de mission"""
        try:
            mission_name = request.mission_name
            mission_file = os.path.join(self.mission_directory, f"{mission_name}.json")
            
            if not os.path.exists(mission_file):
                response.message = f"Mission '{mission_name}' non trouvée"
                response.success = False
                response.mission_id = ""
                return response
                
            # Charger la mission depuis le fichier
            with open(mission_file, 'r') as f:
                mission_data = json.load(f)
                
            mission = self._create_mission_from_dict(mission_data)
            success = self.mission_executor.load_mission(mission)
            
            if success:
                response.message = f"Mission '{mission_name}' chargée"
                response.success = True
                response.mission_id = mission.id
            else:
                response.message = f"Impossible de charger '{mission_name}'"
                response.success = False
                response.mission_id = ""
                
        except Exception as e:
            self.logger.error(f"❌ Erreur load_mission: {e}")
            response.message = f"ERREUR: {str(e)}"
            response.success = False
            response.mission_id = ""
            
        return response
        
    def _handle_create_mission(self, request, response):
        """Gère la création de mission"""
        try:
            # Créer mission depuis les paramètres
            mission_data = {
                "name": request.mission_name,
                "description": request.description,
                "tasks": json.loads(request.tasks_json)
            }
            
            mission = self._create_mission_from_dict(mission_data)
            
            # Sauvegarder la mission
            mission_file = os.path.join(self.mission_directory, f"{mission.name}.json")
            with open(mission_file, 'w') as f:
                json.dump(self._mission_to_dict(mission), f, indent=2)
                
            response.message = f"Mission '{mission.name}' créée et sauvegardée"
            response.success = True
            response.mission_id = mission.id
            
        except Exception as e:
            self.logger.error(f"❌ Erreur create_mission: {e}")
            response.message = f"ERREUR: {str(e)}"
            response.success = False
            response.mission_id = ""
            
        return response
        
    def _create_mission_from_dict(self, data: Dict[str, Any]) -> Mission:
        """Crée une mission depuis un dictionnaire"""
        mission = Mission(
            id=data.get('id', str(uuid.uuid4())),
            name=data['name'],
            description=data.get('description', ''),
            metadata=data.get('metadata', {})
        )
        
        # Créer les tâches
        for task_data in data.get('tasks', []):
            task = MissionTask(
                id=task_data.get('id', str(uuid.uuid4())),
                type=TaskType(task_data['type']),
                parameters=task_data.get('parameters', {}),
                description=task_data.get('description', ''),
                timeout=task_data.get('timeout', 60.0),
                max_retries=task_data.get('max_retries', 3),
                required=task_data.get('required', True),
                conditions=task_data.get('conditions', {})
            )
            mission.tasks.append(task)
            
        return mission
        
    def _mission_to_dict(self, mission: Mission) -> Dict[str, Any]:
        """Convertit une mission en dictionnaire"""
        return {
            'id': mission.id,
            'name': mission.name,
            'description': mission.description,
            'metadata': mission.metadata,
            'tasks': [asdict(task) for task in mission.tasks]
        }
        
    def _publish_status(self):
        """Publie l'état de la mission"""
        status = self.mission_executor.get_mission_status()
        
        # Message MissionStatus personnalisé
        status_msg = MissionStatus()
        
        if status["state"] != "NO_MISSION":
            mission = self.mission_executor.current_mission
            status_msg.mission_id = status["mission_id"]
            status_msg.mission_name = status["mission_name"]
            status_msg.state = status["state"]
            status_msg.current_task_index = mission.current_task_index if mission else 0
            status_msg.total_tasks = len(mission.tasks) if mission else 0
            status_msg.current_task_type = status.get("current_task", "")
            status_msg.current_task_status = status.get("current_task_status", "")
            
            # Calcul du pourcentage de progression
            if status_msg.total_tasks > 0:
                status_msg.progress_percentage = (status_msg.current_task_index / status_msg.total_tasks) * 100.0
            else:
                status_msg.progress_percentage = 0.0
                
            # Temps écoulé
            if mission and mission.started_at > 0:
                status_msg.elapsed_time = time.time() - mission.started_at
            else:
                status_msg.elapsed_time = 0.0
        else:
            status_msg.mission_id = ""
            status_msg.mission_name = ""
            status_msg.state = "NO_MISSION"
            status_msg.current_task_index = 0
            status_msg.total_tasks = 0
            status_msg.current_task_type = ""
            status_msg.current_task_status = ""
            status_msg.progress_percentage = 0.0
            status_msg.elapsed_time = 0.0
            
        self.mission_status_pub.publish(status_msg)
        
        # Publisher le progrès en format simple
        if status["state"] != "NO_MISSION":
            progress_msg = String()
            progress_msg.data = f"{status['state']}:{status['progress']}:{status_msg.progress_percentage:.1f}%"
            self.mission_progress_pub.publish(progress_msg)
            
    def _print_node_info(self):
        """Affiche les informations du nœud"""
        self.logger.info("=" * 60)
        self.logger.info("📋 DRONE MISSION - NŒUD DE MISSION")
        self.logger.info("=" * 60)
        self.logger.info("📡 Services disponibles:")
        self.logger.info("  • /drone/mission/control - Contrôle mission")
        self.logger.info("  • /drone/mission/load - Charger mission")
        self.logger.info("  • /drone/mission/create - Créer mission")
        self.logger.info("")
        self.logger.info("📤 Topics publiés:")
        self.logger.info("  • /drone/mission/status - État mission")
        self.logger.info("  • /drone/mission/progress - Progrès mission")
        self.logger.info("")
        self.logger.info("🎮 Utilisation:")
        self.logger.info("  ros2 service call /drone/mission/load std_msgs/srv/SetString '{data: \"test_mission\"}'")
        self.logger.info("  ros2 service call /drone/mission/control std_msgs/srv/SetString '{data: \"START\"}'")
        self.logger.info("=" * 60)


def main(args=None):
    """Point d'entrée principal"""
    rclpy.init(args=args)
    
    try:
        node = MissionNode()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 Arrêt demandé par l'utilisateur")
    except Exception as e:
        print(f"❌ Erreur fatale: {e}")
    finally:
        try:
            if 'node' in locals():
                node.destroy_node()
        except:
            pass
        rclpy.shutdown()
        print("👋 MissionNode arrêté")


if __name__ == '__main__':
    main()
