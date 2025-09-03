#!/usr/bin/env python3
"""
=============================================================================
ENHANCED MISSION NODE - Robust Mission Orchestration System
=============================================================================
Auteur: Adama Komi
Date: 2025-09-02
Version: 2.0.0

Description:
    Enhanced mission orchestrator with improved state management,
    error recovery, and integration with custom drone_msgs.
    
Key Improvements:
    - Robust state management with proper transitions
    - Enhanced error handling and recovery mechanisms
    - Integration with custom message types
    - Event-driven architecture for better responsiveness
    - Comprehensive logging and monitoring

Architecture:
    - MissionOrchestrator: Main coordination logic
    - StateManager: Mission state transitions
    - TaskExecutor: Enhanced task execution with recovery
    - EventBus: Inter-component communication
=============================================================================
"""

import asyncio
import time
import threading
from enum import Enum, auto
from typing import Dict, List, Optional, Callable, Any
from dataclasses import dataclass, field
import uuid
import json
import os
from pathlib import Path

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.action import ActionServer, ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup

# Standard messages
from std_msgs.msg import String, Bool, Float64
from geometry_msgs.msg import Point

# Custom messages (drone_msgs)
from drone_msgs.msg import MissionStatus, DroneStatus, PollinationTask
from drone_msgs.srv import SetMissionPlan
from drone_msgs.action import ExecuteMission

# Other packages
from drone_navigation.action import NavigateToGoal
from std_srvs.srv import Trigger


class MissionState(Enum):
    """Enhanced mission states with clear transitions"""
    IDLE = auto()
    PLANNING = auto()
    VALIDATING = auto()
    READY = auto()
    EXECUTING = auto()
    PAUSED = auto()
    RECOVERING = auto()
    COMPLETING = auto()
    COMPLETED = auto()
    FAILED = auto()
    ABORTED = auto()


class TaskStatus(Enum):
    """Task execution states"""
    PENDING = auto()
    EXECUTING = auto()
    COMPLETED = auto()
    FAILED = auto()
    SKIPPED = auto()
    RETRYING = auto()


@dataclass
class MissionEvent:
    """Event for mission state machine"""
    event_type: str
    timestamp: float
    data: Dict[str, Any] = field(default_factory=dict)
    source: str = "system"


@dataclass
class EnhancedMissionTask:
    """Enhanced task with better state management"""
    id: str
    task_type: str
    parameters: Dict[str, Any]
    status: TaskStatus = TaskStatus.PENDING
    max_retries: int = 3
    retry_count: int = 0
    timeout: float = 60.0
    required: bool = True
    dependencies: List[str] = field(default_factory=list)
    created_at: float = field(default_factory=time.time)
    started_at: Optional[float] = None
    completed_at: Optional[float] = None
    error_message: Optional[str] = None


class EventBus:
    """Simple event bus for component communication"""
    
    def __init__(self):
        self.subscribers: Dict[str, List[Callable]] = {}
        
    def subscribe(self, event_type: str, callback: Callable):
        """Subscribe to an event type"""
        if event_type not in self.subscribers:
            self.subscribers[event_type] = []
        self.subscribers[event_type].append(callback)
        
    def publish(self, event: MissionEvent):
        """Publish an event to all subscribers"""
        if event.event_type in self.subscribers:
            for callback in self.subscribers[event.event_type]:
                try:
                    callback(event)
                except Exception as e:
                    print(f"Error in event callback: {e}")


class MissionStateManager:
    """Manages mission state transitions and validation"""
    
    def __init__(self, event_bus: EventBus, logger):
        self.current_state = MissionState.IDLE
        self.event_bus = event_bus
        self.logger = logger
        self.state_history: List[tuple] = []
        
        # Define valid state transitions
        self.valid_transitions = {
            MissionState.IDLE: [MissionState.PLANNING],
            MissionState.PLANNING: [MissionState.VALIDATING, MissionState.FAILED],
            MissionState.VALIDATING: [MissionState.READY, MissionState.FAILED],
            MissionState.READY: [MissionState.EXECUTING, MissionState.IDLE],
            MissionState.EXECUTING: [MissionState.PAUSED, MissionState.RECOVERING, 
                                   MissionState.COMPLETING, MissionState.ABORTED],
            MissionState.PAUSED: [MissionState.EXECUTING, MissionState.ABORTED],
            MissionState.RECOVERING: [MissionState.EXECUTING, MissionState.FAILED],
            MissionState.COMPLETING: [MissionState.COMPLETED, MissionState.FAILED],
            MissionState.COMPLETED: [MissionState.IDLE],
            MissionState.FAILED: [MissionState.IDLE],
            MissionState.ABORTED: [MissionState.IDLE]
        }
        
    def transition_to(self, new_state: MissionState, reason: str = "") -> bool:
        """Attempt to transition to a new state"""
        if new_state in self.valid_transitions.get(self.current_state, []):
            old_state = self.current_state
            self.current_state = new_state
            self.state_history.append((time.time(), old_state, new_state, reason))
            
            # Publish state change event
            event = MissionEvent(
                event_type="state_changed",
                timestamp=time.time(),
                data={
                    "old_state": old_state.name,
                    "new_state": new_state.name,
                    "reason": reason
                }
            )
            self.event_bus.publish(event)
            
            self.logger.info(f"🔄 Mission state: {old_state.name} → {new_state.name} ({reason})")
            return True
        else:
            self.logger.error(f"❌ Invalid state transition: {self.current_state.name} → {new_state.name}")
            return False


class EnhancedMissionOrchestrator(Node):
    """
    Enhanced mission orchestrator with robust state management
    """
    
    def __init__(self):
        super().__init__('enhanced_mission_orchestrator')
        
        self.logger = self.get_logger()
        self.logger.info("🚀 Initialisation du Mission Orchestrator Enhanced...")
        
        # Core components
        self.event_bus = EventBus()
        self.state_manager = MissionStateManager(self.event_bus, self.logger)
        
        # Mission data
        self.current_mission: Optional[Dict[str, Any]] = None
        self.mission_tasks: List[EnhancedMissionTask] = []
        self.current_task_index: int = 0
        
        # System state
        self.drone_status: Optional[DroneStatus] = None
        self.system_ready: bool = False
        
        # Configuration
        self.config = self._load_configuration()
        
        # Setup ROS2 components
        self._setup_qos_profiles()
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_services()
        self._setup_action_servers()
        self._setup_action_clients()
        
        # Event subscriptions
        self._setup_event_handlers()
        
        # Timers
        self.status_timer = self.create_timer(1.0, self._publish_mission_status)
        self.health_timer = self.create_timer(5.0, self._health_check)
        
        self.logger.info("✅ Enhanced Mission Orchestrator initialisé!")
        
    def _load_configuration(self) -> Dict[str, Any]:
        """Load configuration from file"""
        config_path = Path.home() / "ros2_ws" / "config" / "mission_config.yaml"
        
        # Default configuration
        default_config = {
            "mission_defaults": {
                "timeout": 300.0,
                "max_retries": 3,
                "safety_checks": True
            },
            "task_timeouts": {
                "ARM": 30.0,
                "TAKEOFF": 60.0,
                "GOTO": 120.0,
                "POLLINATE": 30.0,
                "LAND": 60.0
            },
            "recovery": {
                "enable_auto_recovery": True,
                "max_recovery_attempts": 3,
                "recovery_timeout": 60.0
            }
        }
        
        # TODO: Load from YAML file if exists
        return default_config
        
    def _setup_qos_profiles(self):
        """Setup QoS profiles for different types of communication"""
        self.qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
    def _setup_subscribers(self):
        """Setup ROS2 subscribers"""
        # Drone status updates
        self.drone_status_sub = self.create_subscription(
            DroneStatus,
            '/drone/status_enhanced',
            self._drone_status_callback,
            self.qos_sensor
        )
        
    def _setup_publishers(self):
        """Setup ROS2 publishers"""
        # Mission status publication
        self.mission_status_pub = self.create_publisher(
            MissionStatus,
            '/mission/status_enhanced',
            self.qos_reliable
        )
        
        # System events
        self.events_pub = self.create_publisher(
            String,
            '/mission/events',
            10
        )
        
    def _setup_services(self):
        """Setup ROS2 services"""
        # Mission planning service
        self.mission_plan_service = self.create_service(
            SetMissionPlan,
            '/mission/set_plan_enhanced',
            self._handle_set_mission_plan
        )
        
        # Mission control services
        self.start_service = self.create_service(
            Trigger,
            '/mission/start_enhanced',
            self._handle_start_mission
        )
        
        self.pause_service = self.create_service(
            Trigger,
            '/mission/pause_enhanced',
            self._handle_pause_mission
        )
        
        self.abort_service = self.create_service(
            Trigger,
            '/mission/abort_enhanced',
            self._handle_abort_mission
        )
        
    def _setup_action_servers(self):
        """Setup ROS2 action servers"""
        self.callback_group = ReentrantCallbackGroup()
        
        # Mission execution action
        self.execute_mission_server = ActionServer(
            self,
            ExecuteMission,
            '/mission/execute_enhanced',
            self._execute_mission_callback,
            callback_group=self.callback_group
        )
        
    def _setup_action_clients(self):
        """Setup ROS2 action clients for other services"""
        # Navigation client
        self.navigation_client = ActionClient(
            self,
            NavigateToGoal,
            '/navigation/navigate_to_goal'
        )
        
    def _setup_event_handlers(self):
        """Setup internal event handlers"""
        self.event_bus.subscribe("state_changed", self._handle_state_change)
        self.event_bus.subscribe("task_completed", self._handle_task_completion)
        self.event_bus.subscribe("task_failed", self._handle_task_failure)
        
    def _drone_status_callback(self, msg: DroneStatus):
        """Process drone status updates"""
        self.drone_status = msg
        
        # Check if system becomes ready
        if not self.system_ready and msg.connected and msg.state == "CONNECTED":
            self.system_ready = True
            self.logger.info("✅ Système prêt pour les missions")
            
    def _handle_set_mission_plan(self, request, response):
        """Handle mission planning requests"""
        try:
            # Validate mission can be planned
            if not self.state_manager.transition_to(MissionState.PLANNING, "New mission plan requested"):
                response.success = False
                response.message = "Cannot start planning in current state"
                return response
                
            # Create mission structure
            mission_id = str(uuid.uuid4())[:8]
            mission = {
                "id": mission_id,
                "name": request.mission_name,
                "description": request.mission_description,
                "waypoints": request.waypoints,
                "target_colors": request.target_flower_colors,
                "altitude": request.mission_altitude,
                "max_duration": request.max_mission_duration,
                "created_at": time.time()
            }
            
            # Generate tasks from mission plan
            tasks = self._generate_mission_tasks(mission)
            
            # Validate mission
            if self._validate_mission(mission, tasks):
                self.current_mission = mission
                self.mission_tasks = tasks
                self.state_manager.transition_to(MissionState.READY, "Mission validated and ready")
                
                response.success = True
                response.message = f"Mission '{request.mission_name}' planned successfully"
                response.mission_id = mission_id
            else:
                self.state_manager.transition_to(MissionState.FAILED, "Mission validation failed")
                response.success = False
                response.message = "Mission validation failed"
                response.mission_id = ""
                
        except Exception as e:
            self.logger.error(f"❌ Error in mission planning: {e}")
            self.state_manager.transition_to(MissionState.FAILED, f"Planning error: {str(e)}")
            response.success = False
            response.message = f"Planning error: {str(e)}"
            response.mission_id = ""
            
        return response
        
    def _generate_mission_tasks(self, mission: Dict[str, Any]) -> List[EnhancedMissionTask]:
        """Generate executable tasks from mission plan"""
        tasks = []
        
        # Pre-flight tasks
        tasks.append(EnhancedMissionTask(
            id=f"arm_{mission['id']}",
            task_type="ARM",
            parameters={},
            timeout=self.config["task_timeouts"]["ARM"]
        ))
        
        tasks.append(EnhancedMissionTask(
            id=f"takeoff_{mission['id']}",
            task_type="TAKEOFF",
            parameters={"altitude": mission["altitude"]},
            timeout=self.config["task_timeouts"]["TAKEOFF"],
            dependencies=[f"arm_{mission['id']}"]
        ))
        
        # Waypoint navigation tasks
        for i, waypoint in enumerate(mission["waypoints"]):
            tasks.append(EnhancedMissionTask(
                id=f"goto_{i}_{mission['id']}",
                task_type="GOTO",
                parameters={
                    "x": waypoint.x,
                    "y": waypoint.y,
                    "z": waypoint.z,
                    "target_colors": mission["target_colors"]
                },
                timeout=self.config["task_timeouts"]["GOTO"]
            ))
            
            # Add flower detection and pollination
            tasks.append(EnhancedMissionTask(
                id=f"detect_flowers_{i}_{mission['id']}",
                task_type="DETECT_FLOWERS",
                parameters={
                    "colors": mission["target_colors"],
                    "scan_duration": 10.0
                },
                timeout=30.0,
                dependencies=[f"goto_{i}_{mission['id']}"]
            ))
        
        # Landing task
        tasks.append(EnhancedMissionTask(
            id=f"land_{mission['id']}",
            task_type="LAND",
            parameters={},
            timeout=self.config["task_timeouts"]["LAND"]
        ))
        
        return tasks
        
    def _validate_mission(self, mission: Dict[str, Any], tasks: List[EnhancedMissionTask]) -> bool:
        """Validate mission feasibility"""
        # Check system readiness
        if not self.system_ready:
            self.logger.error("❌ System not ready for mission")
            return False
            
        # Check drone status
        if not self.drone_status or not self.drone_status.connected:
            self.logger.error("❌ Drone not connected")
            return False
            
        # Validate waypoints
        for waypoint in mission["waypoints"]:
            if waypoint.z < 1.0 or waypoint.z > 50.0:
                self.logger.error(f"❌ Invalid altitude: {waypoint.z}m")
                return False
                
        # Check battery level
        if self.drone_status.battery_percentage < 50.0:
            self.logger.error(f"❌ Insufficient battery: {self.drone_status.battery_percentage}%")
            return False
            
        # Validate task dependencies
        task_ids = {task.id for task in tasks}
        for task in tasks:
            for dep in task.dependencies:
                if dep not in task_ids:
                    self.logger.error(f"❌ Invalid task dependency: {dep}")
                    return False
                    
        self.logger.info("✅ Mission validation passed")
        return True
        
    def _handle_start_mission(self, request, response):
        """Handle mission start requests"""
        if self.state_manager.current_state == MissionState.READY:
            if self.state_manager.transition_to(MissionState.EXECUTING, "Mission start requested"):
                response.success = True
                response.message = "Mission started successfully"
                
                # Start mission execution in background
                threading.Thread(target=self._execute_mission_loop, daemon=True).start()
            else:
                response.success = False
                response.message = "Failed to start mission"
        else:
            response.success = False
            response.message = f"Cannot start mission in state: {self.state_manager.current_state.name}"
            
        return response
        
    def _execute_mission_loop(self):
        """Main mission execution loop"""
        self.logger.info("🚀 Starting mission execution loop")
        
        try:
            self.current_task_index = 0
            
            while (self.current_task_index < len(self.mission_tasks) and 
                   self.state_manager.current_state == MissionState.EXECUTING):
                
                task = self.mission_tasks[self.current_task_index]
                
                # Check task dependencies
                if not self._check_task_dependencies(task):
                    self.logger.warning(f"⚠️ Task dependencies not met: {task.id}")
                    time.sleep(1.0)
                    continue
                    
                # Execute task
                self.logger.info(f"🔄 Executing task: {task.task_type} ({task.id})")
                task.status = TaskStatus.EXECUTING
                task.started_at = time.time()
                
                success = self._execute_single_task(task)
                
                if success:
                    task.status = TaskStatus.COMPLETED
                    task.completed_at = time.time()
                    self.logger.info(f"✅ Task completed: {task.id}")
                    
                    # Publish task completion event
                    event = MissionEvent(
                        event_type="task_completed",
                        timestamp=time.time(),
                        data={"task_id": task.id, "task_type": task.task_type}
                    )
                    self.event_bus.publish(event)
                    
                    self.current_task_index += 1
                    
                else:
                    # Handle task failure
                    task.retry_count += 1
                    if task.retry_count < task.max_retries:
                        task.status = TaskStatus.RETRYING
                        self.logger.warning(f"🔄 Retrying task: {task.id} ({task.retry_count}/{task.max_retries})")
                        time.sleep(2.0)  # Brief delay before retry
                    else:
                        task.status = TaskStatus.FAILED
                        self.logger.error(f"❌ Task failed permanently: {task.id}")
                        
                        if task.required:
                            self.state_manager.transition_to(MissionState.FAILED, f"Required task failed: {task.id}")
                            break
                        else:
                            # Skip optional task and continue
                            task.status = TaskStatus.SKIPPED
                            self.current_task_index += 1
                            
            # Mission completion
            if self.current_task_index >= len(self.mission_tasks):
                self.state_manager.transition_to(MissionState.COMPLETING, "All tasks completed")
                time.sleep(2.0)  # Allow for cleanup
                self.state_manager.transition_to(MissionState.COMPLETED, "Mission successful")
                
        except Exception as e:
            self.logger.error(f"❌ Error in mission execution: {e}")
            self.state_manager.transition_to(MissionState.FAILED, f"Execution error: {str(e)}")
            
    def _execute_single_task(self, task: EnhancedMissionTask) -> bool:
        """Execute a single mission task"""
        try:
            if task.task_type == "ARM":
                return self._execute_arm_task(task)
            elif task.task_type == "TAKEOFF":
                return self._execute_takeoff_task(task)
            elif task.task_type == "GOTO":
                return self._execute_goto_task(task)
            elif task.task_type == "DETECT_FLOWERS":
                return self._execute_detect_flowers_task(task)
            elif task.task_type == "POLLINATE":
                return self._execute_pollinate_task(task)
            elif task.task_type == "LAND":
                return self._execute_land_task(task)
            else:
                self.logger.error(f"❌ Unknown task type: {task.task_type}")
                return False
                
        except Exception as e:
            self.logger.error(f"❌ Error executing task {task.id}: {e}")
            task.error_message = str(e)
            return False
            
    def _execute_arm_task(self, task: EnhancedMissionTask) -> bool:
        """Execute ARM task"""
        # TODO: Call drone interface arm service
        self.logger.info("🔐 Arming drone...")
        time.sleep(2.0)  # Simulate arm time
        return True
        
    def _execute_takeoff_task(self, task: EnhancedMissionTask) -> bool:
        """Execute TAKEOFF task"""
        altitude = task.parameters.get("altitude", 3.0)
        self.logger.info(f"🚁 Taking off to {altitude}m...")
        time.sleep(5.0)  # Simulate takeoff time
        return True
        
    def _execute_goto_task(self, task: EnhancedMissionTask) -> bool:
        """Execute GOTO task"""
        x = task.parameters.get("x", 0.0)
        y = task.parameters.get("y", 0.0)
        z = task.parameters.get("z", 3.0)
        self.logger.info(f"🧭 Navigating to ({x}, {y}, {z})...")
        
        # TODO: Use navigation action client
        time.sleep(8.0)  # Simulate navigation time
        return True
        
    def _execute_detect_flowers_task(self, task: EnhancedMissionTask) -> bool:
        """Execute DETECT_FLOWERS task"""
        colors = task.parameters.get("colors", ["red", "yellow"])
        duration = task.parameters.get("scan_duration", 10.0)
        
        self.logger.info(f"🌸 Scanning for flowers: {colors} for {duration}s...")
        
        # TODO: Activate vision system and wait for detections
        time.sleep(duration)
        return True
        
    def _execute_pollinate_task(self, task: EnhancedMissionTask) -> bool:
        """Execute POLLINATE task"""
        flower_id = task.parameters.get("flower_id", "unknown")
        method = task.parameters.get("method", "HOVER")
        
        self.logger.info(f"🐝 Pollinating flower {flower_id} using {method}...")
        
        # TODO: Execute pollination sequence
        time.sleep(3.0)  # Simulate pollination time
        return True
        
    def _execute_land_task(self, task: EnhancedMissionTask) -> bool:
        """Execute LAND task"""
        self.logger.info("🛬 Landing drone...")
        time.sleep(10.0)  # Simulate landing time
        return True
        
    def _check_task_dependencies(self, task: EnhancedMissionTask) -> bool:
        """Check if all task dependencies are satisfied"""
        for dep_id in task.dependencies:
            # Find dependency task
            dep_task = next((t for t in self.mission_tasks if t.id == dep_id), None)
            if not dep_task or dep_task.status != TaskStatus.COMPLETED:
                return False
        return True
        
    def _publish_mission_status(self):
        """Publish current mission status"""
        if not self.current_mission:
            return
            
        status_msg = MissionStatus()
        status_msg.header.stamp = self.get_clock().now().to_msg()
        status_msg.mission_id = self.current_mission["id"]
        status_msg.mission_name = self.current_mission["name"]
        status_msg.mission_description = self.current_mission["description"]
        status_msg.state = self.state_manager.current_state.name
        
        # Calculate progress
        completed_tasks = sum(1 for task in self.mission_tasks if task.status == TaskStatus.COMPLETED)
        total_tasks = len(self.mission_tasks)
        status_msg.progress_percentage = (completed_tasks / total_tasks * 100.0) if total_tasks > 0 else 0.0
        
        status_msg.current_task_index = self.current_task_index
        status_msg.total_tasks = total_tasks
        
        # Current task info
        if self.current_task_index < len(self.mission_tasks):
            current_task = self.mission_tasks[self.current_task_index]
            status_msg.current_task_id = current_task.id
            status_msg.current_task_type = current_task.task_type
            status_msg.current_task_description = f"Executing {current_task.task_type}"
        
        # Mission metrics
        if self.current_mission.get("created_at"):
            status_msg.start_time = self.current_mission["created_at"]
            status_msg.elapsed_time = time.time() - self.current_mission["created_at"]
        
        status_msg.tasks_completed = completed_tasks
        status_msg.tasks_failed = sum(1 for task in self.mission_tasks if task.status == TaskStatus.FAILED)
        
        self.mission_status_pub.publish(status_msg)
        
    def _health_check(self):
        """Perform system health checks"""
        if self.state_manager.current_state == MissionState.EXECUTING:
            # Check drone connectivity
            if not self.drone_status or not self.drone_status.connected:
                self.logger.error("❌ Lost drone connection during mission")
                self.state_manager.transition_to(MissionState.RECOVERING, "Lost drone connection")
                
            # Check battery level
            if self.drone_status and self.drone_status.battery_percentage < 25.0:
                self.logger.warning(f"⚠️ Low battery during mission: {self.drone_status.battery_percentage}%")
                # Could trigger emergency landing or mission abort
                
    def _handle_state_change(self, event: MissionEvent):
        """Handle mission state changes"""
        self.logger.info(f"🔄 State changed: {event.data['old_state']} → {event.data['new_state']}")
        
    def _handle_task_completion(self, event: MissionEvent):
        """Handle task completion events"""
        task_id = event.data["task_id"]
        self.logger.info(f"✅ Task completed: {task_id}")
        
    def _handle_task_failure(self, event: MissionEvent):
        """Handle task failure events"""
        task_id = event.data["task_id"]
        self.logger.error(f"❌ Task failed: {task_id}")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        orchestrator = EnhancedMissionOrchestrator()
        
        # Use MultiThreadedExecutor for better concurrency
        from rclpy.executors import MultiThreadedExecutor
        executor = MultiThreadedExecutor()
        executor.add_node(orchestrator)
        
        orchestrator.logger.info("🚀 Enhanced Mission Orchestrator démarré!")
        executor.spin()
        
    except KeyboardInterrupt:
        orchestrator.logger.info("👋 Arrêt du Mission Orchestrator")
    finally:
        if 'orchestrator' in locals():
            orchestrator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
