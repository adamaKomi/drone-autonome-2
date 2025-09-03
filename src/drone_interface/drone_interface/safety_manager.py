#!/usr/bin/env python3
"""
=============================================================================
SAFETY MANAGER - Gestionnaire de sécurité pour drone autonome
=============================================================================
Auteur: Adama Komi
Date: 2025-09-03
Version: 3.0.0

Description:
    Gestionnaire de sécurité avancé pour surveiller et valider l'état du drone.
    Implémente des vérifications pré-vol, en vol et d'urgence.
=============================================================================
"""

import time
import threading
from enum import IntEnum
from typing import Tuple, List, TYPE_CHECKING
from dataclasses import dataclass, field

# Import conditionnel pour éviter l'import circulaire
if TYPE_CHECKING:
    from .state_manager import DroneStatusData


class SafetyLevel(IntEnum):
    """Niveaux de sécurité"""
    SAFE = 0
    WARNING = 1
    CRITICAL = 2
    EMERGENCY = 3


class SafetyManager:
    """Gestionnaire de sécurité avancé"""
    
    def __init__(self, logger):
        self.logger = logger
        self._lock = threading.RLock()
        
        # Limites de sécurité
        self.min_battery_voltage = 10.5  # V
        self.min_battery_percentage = 20.0  # %
        self.max_wind_speed = 15.0  # m/s
        self.max_altitude = 120.0  # m
        self.min_gps_satellites = 6
        self.max_hdop = 2.0
        
        # Historique pour détection de tendances
        self._battery_history = []
        self._gps_history = []
        
    def check_pre_arm_conditions(self, status) -> Tuple[bool, List[str], List[str]]:
        """Vérifications avant armement"""
        errors = []
        warnings = []
        
        with self._lock:
            # Connexion
            if not status.connected:
                errors.append("Drone non connecté")
                
            # GPS
            if not status.gps_fix:
                errors.append("Pas de fix GPS")
            elif status.gps_satellites < self.min_gps_satellites:
                warnings.append(f"Peu de satellites GPS ({status.gps_satellites})")
            elif status.gps_hdop > self.max_hdop:
                warnings.append(f"HDOP élevé ({status.gps_hdop:.2f})")
                
            # Batterie
            if status.battery_voltage > 0:
                if status.battery_voltage < self.min_battery_voltage:
                    errors.append(f"Batterie faible ({status.battery_voltage:.1f}V)")
                    
            # Mode
            if not status.guided:
                errors.append("Mode GUIDED requis")
                
            # Connexion récente
            if time.time() - status.last_heartbeat > 3.0:
                errors.append("Heartbeat trop ancien")
                
        can_arm = len(errors) == 0
        return can_arm, errors, warnings
        
    def check_flight_safety(self, status) -> Tuple[SafetyLevel, List[str], List[str]]:
        """Vérification continue pendant le vol"""
        errors = []
        warnings = []
        safety_level = SafetyLevel.SAFE
        
        with self._lock:
            # Batterie critique
            if status.battery_percentage > 0 and status.battery_percentage < 10:
                errors.append("Batterie critique (<10%)")
                safety_level = max(safety_level, SafetyLevel.CRITICAL)
            elif status.battery_percentage < 20:
                warnings.append("Batterie faible (<20%)")
                safety_level = max(safety_level, SafetyLevel.WARNING)
                
            # Perte GPS
            if status.armed and not status.gps_fix:
                errors.append("Perte GPS en vol")
                safety_level = max(safety_level, SafetyLevel.EMERGENCY)
                
            # Altitude excessive
            if status.position[2] > self.max_altitude:
                errors.append(f"Altitude excessive ({status.position[2]:.1f}m)")
                safety_level = max(safety_level, SafetyLevel.CRITICAL)
                
            # Perte de connexion
            if time.time() - status.last_heartbeat > 5.0:
                errors.append("Perte de connexion")
                safety_level = max(safety_level, SafetyLevel.EMERGENCY)
                
        return safety_level, errors, warnings
        
    def update_battery_trend(self, voltage: float, percentage: float):
        """Met à jour l'historique batterie pour prédiction"""
        with self._lock:
            current_time = time.time()
            self._battery_history.append((current_time, voltage, percentage))
            # Garde seulement les 5 dernières minutes
            cutoff = current_time - 300
            self._battery_history = [h for h in self._battery_history if h[0] > cutoff]
            
    def get_battery_trend(self) -> str:
        """Analyse la tendance de décharge de la batterie"""
        with self._lock:
            if len(self._battery_history) < 2:
                return "Données insuffisantes"
                
            # Calcul de la dérivée approximative
            recent = self._battery_history[-5:]  # 5 derniers points
            if len(recent) < 2:
                return "OK"
                
            # Calcul de la pente
            time_diff = recent[-1][0] - recent[0][0]
            if time_diff > 0:
                voltage_diff = recent[-1][1] - recent[0][1]
                rate = voltage_diff / time_diff * 60  # V/min
                
                if rate < -0.5:  # Décharge rapide
                    return "Décharge rapide détectée"
                elif rate < -0.2:
                    return "Décharge modérée"
                else:
                    return "Décharge normale"
            
            return "OK"
