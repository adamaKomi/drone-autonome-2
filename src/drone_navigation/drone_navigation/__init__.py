#!/usr/bin/env python3

"""
Drone Navigation Package

Package de navigation avancé pour drones autonomes de pollinisation.
Fournit des capacités de planification de trajectoires, contrôle de position,
évitement d'obstacles et gestion de géobarrières pour missions de précision.

Modules principaux:
- core_navigation_node: Nœud principal avec lifecycle management
- trajectory_planner_node: Planificateur de trajectoires avancé  
- position_controller_node: Contrôleur de position PID
- path_optimizer_node: Optimiseur de chemins intelligents
- coverage_pattern_node: Patterns de couverture (zigzag, spiral)
- parameter_manager_node: Gestionnaire de paramètres centralisé
- mavros_interface_node: Interface avec MAVROS
- cli_bridge_node: Pont pour outils CLI

Auteur: Adama Komi
Version: 3.0.0
Licence: MIT
"""

__version__ = "3.0.0"
__author__ = "Adama Komi"
__email__ = "adama.komi@example.com"
__license__ = "MIT"

# Le package contient principalement des nœuds ROS2 indépendants
# Les imports spécifiques sont faits dans chaque nœud selon les besoins
__all__ = []
