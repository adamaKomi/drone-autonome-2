#!/usr/bin/env python3
"""
=============================================================================
HEALTH MONITOR - Moniteur de santé système pour drone autonome
=============================================================================
Auteur: Adama Komi
Date: 2025-09-03
Version: 3.0.0

Description:
    Moniteur de santé système pour surveiller l'état des services,
    connexions et performances du système drone.
=============================================================================
"""

import time
import threading
from dataclasses import dataclass, field
from typing import List, TYPE_CHECKING

# Import conditionnel pour éviter l'import circulaire
if TYPE_CHECKING:
    from .state_manager import DroneStatusData


@dataclass
class SystemHealth:
    """État de santé système"""
    mavros_connection: bool = False
    services_ready: bool = False
    topics_active: bool = False
    parameter_server: bool = False
    last_check: float = 0.0
    error_count: int = 0
    warnings: List[str] = field(default_factory=list)


class HealthMonitor:
    """Moniteur de santé système"""
    
    def __init__(self, logger):
        self.logger = logger
        self.health = SystemHealth()
        self._lock = threading.RLock()
        
    def check_mavros_services(self, node) -> bool:
        """Vérifie la disponibilité des services MAVROS"""
        try:
            services = [
                getattr(node, 'arm_client', None),
                getattr(node, 'mode_client', None),
                getattr(node, 'takeoff_client', None)
            ]
            
            # Filtrer les services None
            valid_services = [s for s in services if s is not None]
            
            if not valid_services:
                self.logger.warning("No MAVROS services available for checking")
                return False
            
            all_ready = all(service.service_is_ready() for service in valid_services)
            
            with self._lock:
                self.health.services_ready = all_ready
                if all_ready:
                    self.logger.debug("All MAVROS services ready")
                else:
                    self.logger.warning("Some MAVROS services not ready")
                
            return all_ready
            
        except Exception as e:
            self.logger.error(f"Error checking MAVROS services: {e}")
            return False
            
    def update_health(self, status):
        """Met à jour l'état de santé global"""
        with self._lock:
            self.health.mavros_connection = status.connected
            self.health.last_check = time.time()
            
            # Reset compteur d'erreurs si tout va bien
            if status.connected and len(status.safety_messages) == 0:
                if self.health.error_count > 0:
                    self.logger.info("System health improved, resetting error count")
                self.health.error_count = 0
            elif len(status.safety_messages) > 0:
                self.health.error_count += 1
                self.logger.warning(f"Health issues detected: {status.safety_messages}")
                
    def get_health(self) -> SystemHealth:
        """Retourne l'état de santé"""
        with self._lock:
            # Copie pour éviter les modifications concurrentes
            return SystemHealth(
                mavros_connection=self.health.mavros_connection,
                services_ready=self.health.services_ready,
                topics_active=self.health.topics_active,
                parameter_server=self.health.parameter_server,
                last_check=self.health.last_check,
                error_count=self.health.error_count,
                warnings=self.health.warnings.copy()
            )
            
    def check_system_resources(self) -> tuple[float, float]:
        """Vérifie l'utilisation des ressources système"""
        try:
            import psutil
            
            # CPU usage
            cpu_percent = psutil.cpu_percent(interval=0.1)
            
            # Memory usage
            memory = psutil.virtual_memory()
            memory_percent = memory.percent
            
            return cpu_percent, memory_percent
            
        except ImportError:
            self.logger.warning("psutil not available for system monitoring")
            return 0.0, 0.0
        except Exception as e:
            self.logger.error(f"Error checking system resources: {e}")
            return 0.0, 0.0
            
    def add_warning(self, message: str):
        """Ajoute un avertissement au système"""
        with self._lock:
            if message not in self.health.warnings:
                self.health.warnings.append(message)
                self.logger.warning(f"Health warning added: {message}")
                
    def clear_warnings(self):
        """Efface tous les avertissements"""
        with self._lock:
            if self.health.warnings:
                self.logger.info("Clearing all health warnings")
                self.health.warnings.clear()
                
    def is_healthy(self) -> bool:
        """Retourne True si le système est en bonne santé"""
        with self._lock:
            return (self.health.mavros_connection and 
                   self.health.services_ready and 
                   self.health.error_count < 5 and
                   len(self.health.warnings) < 3)
