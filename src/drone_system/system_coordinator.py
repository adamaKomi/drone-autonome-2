#!/usr/bin/env python3
"""
=============================================================================
SYSTEM COORDINATOR - Central Coordination for Drone Pollination System
=============================================================================
Auteur: Adama Komi  
Date: 2025-09-02
Version: 1.0.0

Description:
    Central coordinator that manages communication between all drone packages.
    Provides service discovery, state synchronization, and system health monitoring.
    
Responsibilities:
    - Service discovery and registration
    - Cross-package state synchronization
    - System health monitoring
    - Event routing and coordination
    - Configuration management
    - Failure detection and recovery coordination

Architecture:
    - ServiceRegistry: Discover and manage available services
    - StateSynchronizer: Keep all packages in sync
    - HealthMonitor: Monitor system health
    - EventRouter: Route events between packages
=============================================================================
"""

import time
import threading
from typing import Dict, List, Optional, Any, Callable
from dataclasses import dataclass, field
from enum import Enum, auto
import json

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy, QoSHistoryPolicy
from rclpy.service import Service

# Standard messages
from std_msgs.msg import String, Bool
from std_srvs.srv import Trigger

# Custom messages
from drone_msgs.msg import DroneStatus, MissionStatus


class SystemComponent(Enum):
    """System components that can be registered"""
    DRONE_INTERFACE = "drone_interface"
    DRONE_NAVIGATION = "drone_navigation"
    DRONE_MISSION = "drone_mission"
    DRONE_VISION = "drone_vision"
    DATA_COLLECTOR = "data_collector"


class ComponentState(Enum):
    """States for system components"""
    UNKNOWN = auto()
    STARTING = auto()
    READY = auto()
    ACTIVE = auto()
    ERROR = auto()
    STOPPED = auto()


@dataclass
class ServiceInfo:
    """Information about a registered service"""
    name: str
    service_type: str
    component: SystemComponent
    node_name: str
    last_seen: float
    health_status: str = "unknown"


@dataclass
class ComponentInfo:
    """Information about a system component"""
    name: SystemComponent
    node_name: str
    state: ComponentState
    services: List[str] = field(default_factory=list)
    topics: List[str] = field(default_factory=list)
    last_heartbeat: float = 0.0
    error_count: int = 0
    start_time: float = field(default_factory=time.time)


@dataclass
class SystemEvent:
    """System-wide event"""
    event_type: str
    source_component: SystemComponent
    timestamp: float
    data: Dict[str, Any] = field(default_factory=dict)
    priority: str = "normal"  # low, normal, high, critical


class ServiceRegistry:
    """Registry for system services and their health"""
    
    def __init__(self, logger):
        self.logger = logger
        self.services: Dict[str, ServiceInfo] = {}
        self.components: Dict[SystemComponent, ComponentInfo] = {}
        self.lock = threading.Lock()
        
    def register_service(self, service_info: ServiceInfo):
        """Register a service with the registry"""
        with self.lock:
            self.services[service_info.name] = service_info
            self.logger.info(f"📝 Service registered: {service_info.name} ({service_info.component.value})")
            
    def register_component(self, component_info: ComponentInfo):
        """Register a system component"""
        with self.lock:
            self.components[component_info.name] = component_info
            self.logger.info(f"🔧 Component registered: {component_info.name.value}")
            
    def update_component_state(self, component: SystemComponent, new_state: ComponentState):
        """Update component state"""
        with self.lock:
            if component in self.components:
                old_state = self.components[component].state
                self.components[component].state = new_state
                self.components[component].last_heartbeat = time.time()
                self.logger.info(f"🔄 Component state: {component.value} {old_state.name} → {new_state.name}")
                
    def get_available_services(self, component: Optional[SystemComponent] = None) -> List[ServiceInfo]:
        """Get list of available services, optionally filtered by component"""
        with self.lock:
            if component:
                return [svc for svc in self.services.values() if svc.component == component]
            return list(self.services.values())
            
    def is_component_ready(self, component: SystemComponent) -> bool:
        """Check if a component is ready"""
        with self.lock:
            if component not in self.components:
                return False
            comp_info = self.components[component]
            return comp_info.state in [ComponentState.READY, ComponentState.ACTIVE]
            
    def get_system_health(self) -> Dict[str, Any]:
        """Get overall system health status"""
        with self.lock:
            total_components = len(self.components)
            ready_components = sum(1 for comp in self.components.values() 
                                 if comp.state in [ComponentState.READY, ComponentState.ACTIVE])
            error_components = sum(1 for comp in self.components.values() 
                                 if comp.state == ComponentState.ERROR)
            
            return {
                "total_components": total_components,
                "ready_components": ready_components,
                "error_components": error_components,
                "system_ready": ready_components == total_components and total_components > 0,
                "health_percentage": (ready_components / total_components * 100) if total_components > 0 else 0
            }


class StateSynchronizer:
    """Synchronizes state across all system components"""
    
    def __init__(self, logger):
        self.logger = logger
        self.state_cache: Dict[str, Any] = {}
        self.subscribers: Dict[str, List[Callable]] = {}
        self.lock = threading.Lock()
        
    def publish_state(self, state_name: str, state_data: Any):
        """Publish state update to all subscribers"""
        with self.lock:
            self.state_cache[state_name] = {
                "data": state_data,
                "timestamp": time.time()
            }
            
            # Notify subscribers
            if state_name in self.subscribers:
                for callback in self.subscribers[state_name]:
                    try:
                        callback(state_data)
                    except Exception as e:
                        self.logger.error(f"❌ Error in state callback: {e}")
                        
    def subscribe_to_state(self, state_name: str, callback: Callable):
        """Subscribe to state updates"""
        with self.lock:
            if state_name not in self.subscribers:
                self.subscribers[state_name] = []
            self.subscribers[state_name].append(callback)
            
            # Send current state if available
            if state_name in self.state_cache:
                try:
                    callback(self.state_cache[state_name]["data"])
                except Exception as e:
                    self.logger.error(f"❌ Error in initial state callback: {e}")
                    
    def get_state(self, state_name: str) -> Optional[Any]:
        """Get current state value"""
        with self.lock:
            return self.state_cache.get(state_name, {}).get("data")


class EventRouter:
    """Routes events between system components"""
    
    def __init__(self, logger):
        self.logger = logger
        self.event_handlers: Dict[str, List[Callable]] = {}
        self.event_history: List[SystemEvent] = []
        self.lock = threading.Lock()
        
    def publish_event(self, event: SystemEvent):
        """Publish an event to all handlers"""
        with self.lock:
            self.event_history.append(event)
            
            # Keep only last 100 events
            if len(self.event_history) > 100:
                self.event_history = self.event_history[-100:]
                
            # Route to handlers
            if event.event_type in self.event_handlers:
                for handler in self.event_handlers[event.event_type]:
                    try:
                        handler(event)
                    except Exception as e:
                        self.logger.error(f"❌ Error in event handler: {e}")
                        
        self.logger.debug(f"📡 Event routed: {event.event_type} from {event.source_component.value}")
        
    def subscribe_to_event(self, event_type: str, handler: Callable):
        """Subscribe to specific event type"""
        with self.lock:
            if event_type not in self.event_handlers:
                self.event_handlers[event_type] = []
            self.event_handlers[event_type].append(handler)
            
    def get_recent_events(self, count: int = 10) -> List[SystemEvent]:
        """Get recent events"""
        with self.lock:
            return self.event_history[-count:]


class SystemCoordinator(Node):
    """
    Central system coordinator for the drone pollination system
    """
    
    def __init__(self):
        super().__init__('system_coordinator')
        
        self.logger = self.get_logger()
        self.logger.info("🎛️ Initialisation du System Coordinator...")
        
        # Core components
        self.service_registry = ServiceRegistry(self.logger)
        self.state_synchronizer = StateSynchronizer(self.logger)
        self.event_router = EventRouter(self.logger)
        
        # System state
        self.system_start_time = time.time()
        self.coordination_active = True
        
        # Setup ROS2 components
        self._setup_qos_profiles()
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_services()
        
        # Setup event handlers
        self._setup_event_handlers()
        
        # Initialize component discovery
        self._discover_system_components()
        
        # Timers
        self.health_timer = self.create_timer(5.0, self._monitor_system_health)
        self.discovery_timer = self.create_timer(10.0, self._discover_services)
        self.heartbeat_timer = self.create_timer(1.0, self._publish_system_heartbeat)
        
        self.logger.info("✅ System Coordinator initialisé!")
        
    def _setup_qos_profiles(self):
        """Setup QoS profiles"""
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
        # Subscribe to component heartbeats
        self.heartbeat_sub = self.create_subscription(
            String,
            '/system/component_heartbeat',
            self._component_heartbeat_callback,
            10
        )
        
        # Subscribe to component state updates
        self.component_state_sub = self.create_subscription(
            String,
            '/system/component_state',
            self._component_state_callback,
            10
        )
        
        # Subscribe to drone status for coordination
        self.drone_status_sub = self.create_subscription(
            DroneStatus,
            '/drone/status_enhanced',
            self._drone_status_callback,
            self.qos_sensor
        )
        
        # Subscribe to mission status
        self.mission_status_sub = self.create_subscription(
            MissionStatus,
            '/mission/status_enhanced',
            self._mission_status_callback,
            self.qos_reliable
        )
        
    def _setup_publishers(self):
        """Setup ROS2 publishers"""
        # System-wide state publication
        self.system_state_pub = self.create_publisher(
            String,
            '/system/state',
            self.qos_reliable
        )
        
        # System events publication
        self.system_events_pub = self.create_publisher(
            String,
            '/system/events',
            10
        )
        
        # System health publication
        self.system_health_pub = self.create_publisher(
            String,
            '/system/health',
            self.qos_reliable
        )
        
    def _setup_services(self):
        """Setup ROS2 services"""
        # System control services
        self.shutdown_service = self.create_service(
            Trigger,
            '/system/shutdown',
            self._handle_system_shutdown
        )
        
        self.restart_service = self.create_service(
            Trigger,
            '/system/restart',
            self._handle_system_restart
        )
        
        self.health_check_service = self.create_service(
            Trigger,
            '/system/health_check',
            self._handle_health_check
        )
        
    def _setup_event_handlers(self):
        """Setup internal event handlers"""
        self.event_router.subscribe_to_event("component_error", self._handle_component_error)
        self.event_router.subscribe_to_event("mission_failed", self._handle_mission_failure)
        self.event_router.subscribe_to_event("system_warning", self._handle_system_warning)
        
    def _discover_system_components(self):
        """Initialize discovery of system components"""
        expected_components = [
            SystemComponent.DRONE_INTERFACE,
            SystemComponent.DRONE_NAVIGATION,
            SystemComponent.DRONE_MISSION,
            SystemComponent.DRONE_VISION
        ]
        
        for component in expected_components:
            comp_info = ComponentInfo(
                name=component,
                node_name=f"{component.value}_node",
                state=ComponentState.UNKNOWN
            )
            self.service_registry.register_component(comp_info)
            
    def _discover_services(self):
        """Discover available services in the system"""
        # Get list of available services
        service_names = self.get_service_names_and_types()
        
        for service_name, service_types in service_names:
            # Parse service to determine component
            component = self._identify_service_component(service_name)
            if component:
                service_info = ServiceInfo(
                    name=service_name,
                    service_type=service_types[0] if service_types else "unknown",
                    component=component,
                    node_name="unknown",
                    last_seen=time.time()
                )
                self.service_registry.register_service(service_info)
                
    def _identify_service_component(self, service_name: str) -> Optional[SystemComponent]:
        """Identify which component a service belongs to"""
        if '/drone/' in service_name:
            return SystemComponent.DRONE_INTERFACE
        elif '/navigation/' in service_name:
            return SystemComponent.DRONE_NAVIGATION
        elif '/mission/' in service_name:
            return SystemComponent.DRONE_MISSION
        elif '/vision/' in service_name:
            return SystemComponent.DRONE_VISION
        elif '/data_collector/' in service_name:
            return SystemComponent.DATA_COLLECTOR
        return None
        
    def _component_heartbeat_callback(self, msg: String):
        """Process component heartbeat messages"""
        try:
            heartbeat_data = json.loads(msg.data)
            component_name = heartbeat_data.get("component")
            
            if component_name:
                component = SystemComponent(component_name)
                self.service_registry.update_component_state(component, ComponentState.ACTIVE)
                
        except Exception as e:
            self.logger.error(f"❌ Error processing heartbeat: {e}")
            
    def _component_state_callback(self, msg: String):
        """Process component state updates"""
        try:
            state_data = json.loads(msg.data)
            component_name = state_data.get("component")
            state_name = state_data.get("state")
            
            if component_name and state_name:
                component = SystemComponent(component_name)
                state = ComponentState[state_name.upper()]
                self.service_registry.update_component_state(component, state)
                
        except Exception as e:
            self.logger.error(f"❌ Error processing component state: {e}")
            
    def _drone_status_callback(self, msg: DroneStatus):
        """Process drone status updates"""
        # Synchronize drone state across system
        self.state_synchronizer.publish_state("drone_status", {
            "state": msg.state,
            "armed": msg.armed,
            "connected": msg.connected,
            "battery": msg.battery_percentage,
            "position": [msg.position.x, msg.position.y, msg.position.z]
        })
        
    def _mission_status_callback(self, msg: MissionStatus):
        """Process mission status updates"""
        # Synchronize mission state across system
        self.state_synchronizer.publish_state("mission_status", {
            "mission_id": msg.mission_id,
            "state": msg.state,
            "progress": msg.progress_percentage,
            "current_task": msg.current_task_type
        })
        
    def _monitor_system_health(self):
        """Monitor overall system health"""
        health = self.service_registry.get_system_health()
        
        # Publish health status
        health_msg = String()
        health_msg.data = json.dumps({
            "timestamp": time.time(),
            "system_uptime": time.time() - self.system_start_time,
            **health
        })
        self.system_health_pub.publish(health_msg)
        
        # Check for critical issues
        if health["error_components"] > 0:
            self.logger.warning(f"⚠️ System health warning: {health['error_components']} components in error")
            
        if health["health_percentage"] < 80:
            self.logger.error(f"🚨 System health critical: {health['health_percentage']:.1f}% healthy")
            
    def _publish_system_heartbeat(self):
        """Publish system coordinator heartbeat"""
        heartbeat = {
            "component": "system_coordinator",
            "timestamp": time.time(),
            "uptime": time.time() - self.system_start_time,
            "coordination_active": self.coordination_active
        }
        
        msg = String()
        msg.data = json.dumps(heartbeat)
        self.system_state_pub.publish(msg)
        
    def _handle_component_error(self, event: SystemEvent):
        """Handle component error events"""
        self.logger.error(f"🚨 Component error: {event.source_component.value} - {event.data}")
        
        # Update component state
        self.service_registry.update_component_state(event.source_component, ComponentState.ERROR)
        
        # Trigger recovery if needed
        if event.priority == "critical":
            self._initiate_system_recovery(event.source_component)
            
    def _handle_mission_failure(self, event: SystemEvent):
        """Handle mission failure events"""
        self.logger.error(f"🚨 Mission failure: {event.data}")
        
        # Coordinate system response to mission failure
        # This could include emergency landing, data saving, etc.
        
    def _handle_system_warning(self, event: SystemEvent):
        """Handle system warning events"""
        self.logger.warning(f"⚠️ System warning: {event.source_component.value} - {event.data}")
        
    def _initiate_system_recovery(self, failed_component: SystemComponent):
        """Initiate recovery for a failed component"""
        self.logger.info(f"🔧 Initiating recovery for {failed_component.value}")
        
        # Recovery logic depends on the component
        # For now, just log the attempt
        
    def _handle_system_shutdown(self, request, response):
        """Handle system shutdown request"""
        self.logger.info("🛑 System shutdown requested")
        
        # Publish shutdown event
        shutdown_event = SystemEvent(
            event_type="system_shutdown",
            source_component=SystemComponent.DRONE_INTERFACE,  # Using as system component
            timestamp=time.time(),
            data={"reason": "user_request"},
            priority="high"
        )
        self.event_router.publish_event(shutdown_event)
        
        response.success = True
        response.message = "System shutdown initiated"
        
        # Set flag to stop coordination
        self.coordination_active = False
        
        return response
        
    def _handle_system_restart(self, request, response):
        """Handle system restart request"""
        self.logger.info("🔄 System restart requested")
        
        response.success = True
        response.message = "System restart initiated"
        return response
        
    def _handle_health_check(self, request, response):
        """Handle health check request"""
        health = self.service_registry.get_system_health()
        
        response.success = health["system_ready"]
        response.message = f"System health: {health['health_percentage']:.1f}% ({health['ready_components']}/{health['total_components']} components ready)"
        
        return response


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)
    
    try:
        coordinator = SystemCoordinator()
        
        coordinator.logger.info("🎛️ System Coordinator démarré!")
        rclpy.spin(coordinator)
        
    except KeyboardInterrupt:
        coordinator.logger.info("👋 Arrêt du System Coordinator")
    finally:
        if 'coordinator' in locals():
            coordinator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
