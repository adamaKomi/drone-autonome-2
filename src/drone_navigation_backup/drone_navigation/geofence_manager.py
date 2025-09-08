#!/usr/bin/env python3

"""
Gestionnaire de géobarrières pour drone autonome

Implémente la gestion des zones autorisées et interdites pour
assurer la sécurité des vols et le respect des réglementations.

Fonctionnalités:
- Définition de zones d'inclusion/exclusion
- Vérification en temps réel des violations
- Actions automatiques (RTL, arrêt, contournement)
- Support des géobarrières 3D avec limites d'altitude

Auteur: Adama Komi
Version: 1.0.0
"""

import math
import time
from typing import List, Dict, Any, Optional, Tuple
from dataclasses import dataclass
from enum import Enum
from geometry_msgs.msg import Point, Polygon, Point32


class GeofenceType(Enum):
    """Types de géobarrières"""
    INCLUSION = "inclusion"    # Zone autorisée
    EXCLUSION = "exclusion"    # Zone interdite
    WARNING = "warning"        # Zone d'avertissement
    ALTITUDE_LIMIT = "altitude_limit"  # Limite d'altitude


class ViolationAction(Enum):
    """Actions en cas de violation"""
    STOP = "stop"              # Arrêt immédiat
    RTL = "rtl"               # Retour à la base
    LAND = "land"             # Atterrissage immédiat
    AVOID = "avoid"           # Contournement automatique
    WARNING_ONLY = "warning"  # Avertissement seulement


@dataclass
class GeofenceZone:
    """Définition d'une zone de géobarrière"""
    id: str
    name: str
    zone_type: GeofenceType
    boundary: List[Point]      # Périmètre de la zone (polygone)
    min_altitude: float = 0.0  # Altitude minimale (mètres)
    max_altitude: float = 120.0 # Altitude maximale (mètres)
    action: ViolationAction = ViolationAction.WARNING_ONLY
    enabled: bool = True
    buffer_distance: float = 5.0  # Distance de sécurité (mètres)
    description: str = ""


@dataclass
class ViolationEvent:
    """Événement de violation de géobarrière"""
    timestamp: float
    zone_id: str
    zone_name: str
    violation_type: GeofenceType
    position: Point
    distance_to_boundary: float
    action_taken: ViolationAction
    severity: str = "medium"  # low, medium, high, critical


class GeofenceManager:
    """
    Gestionnaire de géobarrières pour navigation drone
    
    Gère les zones de vol autorisées/interdites et déclenche
    les actions appropriées en cas de violation.
    """
    
    def __init__(self, node):
        """
        Initialise le gestionnaire de géobarrières
        
        Args:
            node: Nœud ROS2 parent
        """
        self._node = node
        self._logger = node.get_logger()
        
        # Zones de géobarrières actives
        self._zones: Dict[str, GeofenceZone] = {}
        
        # Historique des violations
        self._violation_history: List[ViolationEvent] = []
        
        # Configuration
        self._global_enabled = True
        self._check_interval = 0.1  # secondes
        self._last_check_time = 0.0
        
        # Statistiques
        self._stats = {
            'total_checks': 0,
            'violations_detected': 0,
            'zones_active': 0,
            'last_violation_time': 0.0
        }
        
        # Zone de sécurité par défaut (rectangle autour du point de départ)
        self._create_default_safety_zone()
        
        self._logger.info("GeofenceManager initialisé")

    def add_zone(self, zone: GeofenceZone) -> bool:
        """
        Ajoute une zone de géobarrière
        
        Args:
            zone: Zone à ajouter
            
        Returns:
            True si ajoutée avec succès
        """
        if not self._validate_zone(zone):
            self._logger.error(f"Zone invalide: {zone.id}")
            return False
        
        self._zones[zone.id] = zone
        self._stats['zones_active'] = len([z for z in self._zones.values() if z.enabled])
        
        self._logger.info(f"Zone de géobarrière ajoutée: {zone.name} ({zone.zone_type.value})")
        return True

    def remove_zone(self, zone_id: str) -> bool:
        """
        Supprime une zone de géobarrière
        
        Args:
            zone_id: ID de la zone à supprimer
            
        Returns:
            True si supprimée avec succès
        """
        if zone_id in self._zones:
            removed_zone = self._zones.pop(zone_id)
            self._stats['zones_active'] = len([z for z in self._zones.values() if z.enabled])
            self._logger.info(f"Zone supprimée: {removed_zone.name}")
            return True
        
        self._logger.warning(f"Zone non trouvée: {zone_id}")
        return False

    def enable_zone(self, zone_id: str, enabled: bool = True) -> bool:
        """
        Active/désactive une zone
        
        Args:
            zone_id: ID de la zone
            enabled: État d'activation
            
        Returns:
            True si modifiée avec succès
        """
        if zone_id in self._zones:
            self._zones[zone_id].enabled = enabled
            self._stats['zones_active'] = len([z for z in self._zones.values() if z.enabled])
            status = "activée" if enabled else "désactivée"
            self._logger.info(f"Zone {status}: {self._zones[zone_id].name}")
            return True
        
        return False

    def check_violation(self, current_position: Point) -> Optional[ViolationEvent]:
        """
        Vérifie les violations de géobarrières
        
        Args:
            current_position: Position actuelle du drone
            
        Returns:
            Événement de violation si détecté, None sinon
        """
        if not self._global_enabled:
            return None
        
        current_time = time.time()
        if current_time - self._last_check_time < self._check_interval:
            return None
        
        self._last_check_time = current_time
        self._stats['total_checks'] += 1
        
        # Vérification de chaque zone active
        for zone in self._zones.values():
            if not zone.enabled:
                continue
            
            violation = self._check_zone_violation(current_position, zone)
            if violation:
                self._handle_violation(violation)
                return violation
        
        return None

    def _check_zone_violation(self, position: Point, zone: GeofenceZone) -> Optional[ViolationEvent]:
        """
        Vérifie la violation d'une zone spécifique
        
        Args:
            position: Position à vérifier
            zone: Zone de géobarrière
            
        Returns:
            Événement de violation si détecté
        """
        # Vérification des limites d'altitude
        if position.z < zone.min_altitude or position.z > zone.max_altitude:
            return ViolationEvent(
                timestamp=time.time(),
                zone_id=zone.id,
                zone_name=zone.name,
                violation_type=GeofenceType.ALTITUDE_LIMIT,
                position=position,
                distance_to_boundary=min(
                    abs(position.z - zone.min_altitude),
                    abs(position.z - zone.max_altitude)
                ),
                action_taken=zone.action,
                severity="high"
            )
        
        # Vérification du périmètre horizontal
        if len(zone.boundary) < 3:
            return None
        
        is_inside = self._point_in_polygon(position, zone.boundary)
        distance_to_boundary = self._distance_to_polygon(position, zone.boundary)
        
        # Logique de violation selon le type de zone
        violation_detected = False
        
        if zone.zone_type == GeofenceType.INCLUSION:
            # Zone d'inclusion: violation si à l'extérieur
            if not is_inside or distance_to_boundary < zone.buffer_distance:
                violation_detected = True
        
        elif zone.zone_type == GeofenceType.EXCLUSION:
            # Zone d'exclusion: violation si à l'intérieur
            if is_inside or distance_to_boundary < zone.buffer_distance:
                violation_detected = True
        
        elif zone.zone_type == GeofenceType.WARNING:
            # Zone d'avertissement: toujours signaler la proximité
            if is_inside or distance_to_boundary < zone.buffer_distance * 2:
                violation_detected = True
        
        if violation_detected:
            severity = self._calculate_severity(distance_to_boundary, zone.buffer_distance)
            return ViolationEvent(
                timestamp=time.time(),
                zone_id=zone.id,
                zone_name=zone.name,
                violation_type=zone.zone_type,
                position=position,
                distance_to_boundary=distance_to_boundary,
                action_taken=zone.action,
                severity=severity
            )
        
        return None

    def _handle_violation(self, violation: ViolationEvent):
        """
        Gère une violation détectée
        
        Args:
            violation: Événement de violation
        """
        # Enregistrement dans l'historique
        self._violation_history.append(violation)
        self._stats['violations_detected'] += 1
        self._stats['last_violation_time'] = violation.timestamp
        
        # Limitation de l'historique
        if len(self._violation_history) > 1000:
            self._violation_history = self._violation_history[-500:]
        
        # Log de l'événement
        self._logger.warning(
            f"Violation géobarrière détectée - Zone: {violation.zone_name}, "
            f"Type: {violation.violation_type.value}, "
            f"Distance: {violation.distance_to_boundary:.2f}m, "
            f"Action: {violation.action_taken.value}"
        )
        
        # Déclenchement de l'action selon la configuration
        self._trigger_violation_action(violation)

    def _trigger_violation_action(self, violation: ViolationEvent):
        """
        Déclenche l'action configurée pour une violation
        
        Args:
            violation: Événement de violation
        """
        try:
            if violation.action_taken == ViolationAction.STOP:
                self._logger.critical("Action STOP déclenchée - Arrêt immédiat requis")
                # Ici on pourrait publier un message d'arrêt d'urgence
                
            elif violation.action_taken == ViolationAction.RTL:
                self._logger.critical("Action RTL déclenchée - Retour à la base")
                # Déclencher RTL via MAVROS
                self._node.trigger_emergency_rtl()
                
            elif violation.action_taken == ViolationAction.LAND:
                self._logger.critical("Action LAND déclenchée - Atterrissage d'urgence")
                # Déclencher atterrissage d'urgence
                
            elif violation.action_taken == ViolationAction.AVOID:
                self._logger.warning("Action AVOID - Contournement automatique requis")
                # Ici on pourrait déclencher l'évitement d'obstacles
                
            elif violation.action_taken == ViolationAction.WARNING_ONLY:
                self._logger.warning("Avertissement géobarrière - Surveillance continue")
                
        except Exception as e:
            self._logger.error(f"Erreur lors du déclenchement d'action: {e}")

    def _point_in_polygon(self, point: Point, polygon: List[Point]) -> bool:
        """
        Teste si un point est à l'intérieur d'un polygone (ray casting)
        
        Args:
            point: Point à tester
            polygon: Polygone de test
            
        Returns:
            True si le point est à l'intérieur
        """
        x, y = point.x, point.y
        n = len(polygon)
        inside = False
        
        p1x, p1y = polygon[0].x, polygon[0].y
        for i in range(1, n + 1):
            p2x, p2y = polygon[i % n].x, polygon[i % n].y
            if y > min(p1y, p2y):
                if y <= max(p1y, p2y):
                    if x <= max(p1x, p2x):
                        if p1y != p2y:
                            xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                        if p1x == p2x or x <= xinters:
                            inside = not inside
            p1x, p1y = p2x, p2y
        
        return inside

    def _distance_to_polygon(self, point: Point, polygon: List[Point]) -> float:
        """
        Calcule la distance minimale d'un point au périmètre d'un polygone
        
        Args:
            point: Point de référence
            polygon: Polygone
            
        Returns:
            Distance minimale en mètres
        """
        if len(polygon) < 2:
            return float('inf')
        
        min_distance = float('inf')
        
        # Distance à chaque segment du polygone
        for i in range(len(polygon)):
            p1 = polygon[i]
            p2 = polygon[(i + 1) % len(polygon)]
            
            distance = self._distance_point_to_segment(point, p1, p2)
            min_distance = min(min_distance, distance)
        
        return min_distance

    def _distance_point_to_segment(self, point: Point, seg_start: Point, seg_end: Point) -> float:
        """
        Calcule la distance d'un point à un segment de ligne
        
        Args:
            point: Point de référence
            seg_start: Début du segment
            seg_end: Fin du segment
            
        Returns:
            Distance en mètres
        """
        # Vecteurs
        dx = seg_end.x - seg_start.x
        dy = seg_end.y - seg_start.y
        
        if dx == 0 and dy == 0:
            # Segment dégénéré (point)
            return math.sqrt((point.x - seg_start.x)**2 + (point.y - seg_start.y)**2)
        
        # Paramètre t pour la projection
        t = ((point.x - seg_start.x) * dx + (point.y - seg_start.y) * dy) / (dx**2 + dy**2)
        t = max(0.0, min(1.0, t))  # Limiter à [0,1]
        
        # Point le plus proche sur le segment
        closest_x = seg_start.x + t * dx
        closest_y = seg_start.y + t * dy
        
        # Distance euclidienne
        return math.sqrt((point.x - closest_x)**2 + (point.y - closest_y)**2)

    def _calculate_severity(self, distance: float, buffer: float) -> str:
        """
        Calcule la sévérité d'une violation basée sur la distance
        
        Args:
            distance: Distance à la frontière
            buffer: Distance de buffer
            
        Returns:
            Niveau de sévérité
        """
        if distance <= 0:
            return "critical"
        elif distance <= buffer * 0.25:
            return "high"
        elif distance <= buffer * 0.5:
            return "medium"
        else:
            return "low"

    def _validate_zone(self, zone: GeofenceZone) -> bool:
        """
        Valide la configuration d'une zone
        
        Args:
            zone: Zone à valider
            
        Returns:
            True si valide
        """
        if not zone.id or not zone.name:
            return False
        
        if zone.min_altitude >= zone.max_altitude:
            return False
        
        if len(zone.boundary) < 3:
            return False
        
        if zone.buffer_distance < 0:
            return False
        
        return True

    def _create_default_safety_zone(self):
        """Crée une zone de sécurité par défaut"""
        # Zone d'inclusion de 1km x 1km centrée sur l'origine
        safety_boundary = [
            Point(x=-500.0, y=-500.0, z=0.0),
            Point(x=500.0, y=-500.0, z=0.0),
            Point(x=500.0, y=500.0, z=0.0),
            Point(x=-500.0, y=500.0, z=0.0)
        ]
        
        safety_zone = GeofenceZone(
            id="default_safety",
            name="Zone de sécurité par défaut",
            zone_type=GeofenceType.INCLUSION,
            boundary=safety_boundary,
            min_altitude=5.0,
            max_altitude=120.0,
            action=ViolationAction.RTL,
            enabled=True,
            buffer_distance=10.0,
            description="Zone de sécurité par défaut pour les opérations"
        )
        
        self.add_zone(safety_zone)

    def get_active_zones(self) -> List[GeofenceZone]:
        """Retourne la liste des zones actives"""
        return [zone for zone in self._zones.values() if zone.enabled]

    def get_violation_history(self, limit: int = 50) -> List[ViolationEvent]:
        """
        Retourne l'historique des violations
        
        Args:
            limit: Nombre maximum d'événements à retourner
            
        Returns:
            Liste des violations récentes
        """
        return self._violation_history[-limit:]

    def get_statistics(self) -> Dict[str, Any]:
        """Retourne les statistiques du gestionnaire"""
        return self._stats.copy()

    def clear_violation_history(self):
        """Efface l'historique des violations"""
        self._violation_history.clear()
        self._logger.info("Historique des violations effacé")

    def set_global_enabled(self, enabled: bool):
        """
        Active/désactive globalement les géobarrières
        
        Args:
            enabled: État d'activation globale
        """
        self._global_enabled = enabled
        status = "activées" if enabled else "désactivées"
        self._logger.info(f"Géobarrières globalement {status}")

    def is_position_safe(self, position: Point) -> Tuple[bool, List[str]]:
        """
        Vérifie si une position est sûre
        
        Args:
            position: Position à vérifier
            
        Returns:
            Tuple (is_safe, list_of_violations)
        """
        violations = []
        
        for zone in self._zones.values():
            if not zone.enabled:
                continue
                
            violation = self._check_zone_violation(position, zone)
            if violation:
                violations.append(f"{zone.name}: {violation.violation_type.value}")
        
        return len(violations) == 0, violations
