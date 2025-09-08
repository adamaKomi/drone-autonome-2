#!/usr/bin/env python3

"""
Drone Navigation Package

Package de navigation avancé pour drones autonomes de pollinisation.
Fournit des capacités de planification de trajectoires, contrôle de position,
évitement d'obstacles et gestion de géobarrières pour missions de précision.

Modules principaux:
- navigation_node: Nœud principal avec lifecycle management
- trajectory_planner: Planificateur de trajectoires avancé  
- position_controller: Contrôleur de position PID
- path_optimizer: Optimiseur de chemins intelligents
- obstacle_avoidance: Système d'évitement d'obstacles
- geofence_manager: Gestionnaire de géobarrières
- coverage_patterns: Patterns de couverture (zigzag, spiral)

Auteur: Adama Komi
Version: 1.0.0
Licence: MIT
"""

__version__ = "1.0.0"
__author__ = "Adama Komi"
__email__ = "adama.komi@example.com"
__license__ = "MIT"

# Imports des modules principaux
from .navigation_node import NavigationNode
__all__ = [
    'NavigationNode',
]
