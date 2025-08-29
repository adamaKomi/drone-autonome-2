#!/usr/bin/env python3
"""
=============================================================================
EXEMPLES ET MISSIONS PRÉDÉFINIES POUR DRONE INTERFACE
=============================================================================
Auteur: Assistant IA
Date: 2025-08-29
Version: 1.0.0

Description:
    Collection d'exemples et de missions prédéfinies pour démontrer
    les capacités du système de contrôle de drone.
    
Utilisation:
    python3 drone_examples.py --mission basic_flight
    python3 drone_examples.py --mission square_pattern
    python3 drone_examples.py --list-missions
=============================================================================
"""

import argparse
import time
import math
from typing import List, Tuple, Dict, Any
from dataclasses import dataclass

import rclpy
from rclpy.node import Node

# Import des modules principaux (en supposant qu'ils sont dans le même package)
from interface_node import DroneInterface, DroneState, FlightMode
from config import create_config, print_config_info


@dataclass
class Waypoint:
    """Point de passage pour une mission"""
    x: float                    # Position X (mètres, frame local)
    y: float                    # Position Y (mètres, frame local)
    z: float                    # Position Z/altitude (mètres)
    yaw: float = 0.0           # Orientation (radians)
    hold_time: float = 2.0     # Temps d'attente au waypoint (secondes)
    tolerance: float = 1.0     # Tolérance de position (mètres)


@dataclass
class Mission:
    """Définition d'une mission complète"""
    name: str                   # Nom de la mission
    description: str            # Description de la mission
    waypoints: List[Waypoint]   # Liste des waypoints
    takeoff_altitude: float = 2.5  # Altitude de décollage
    auto_land: bool = True      # Atterrissage automatique à la fin
    max_duration: float = 300.0 # Durée maximum de la mission (secondes)


class MissionExecutor:
    """Exécuteur de missions pour drone"""
    
    def __init__(self, drone_interface: DroneInterface):
        self.drone_interface = drone_interface
        self.logger = drone_interface.get_logger()
        self.current_mission = None
        self.mission_start_time = None
        self.current_waypoint_index = 0
        self.is_executing = False
        
    def execute_mission(self, mission: Mission) -> bool:
        """
        Exécute une mission complète
        
        Args:
            mission: Mission à exécuter
            
        Returns:
            bool: True si succès, False sinon
        """
        self.logger.info(f"🎯 Début de la mission: {mission.name}")
        self.logger.info(f"📋 Description: {mission.description}")
        self.logger.info(f"📍 Waypoints: {len(mission.waypoints)}")
        
        self.current_mission = mission
        self.mission_start_time = time.time()
        self.current_waypoint_index = 0
        self.is_executing = True
        
        try:
            # Étape 1: Préparation pré-vol
            if not self._preflight_checks():
                return False
                
            # Étape 2: Décollage
            if not self._execute_takeoff(mission.takeoff_altitude):
                return False
                
            # Étape 3: Exécution des waypoints
            if not self._execute_waypoints(mission.waypoints):
                return False
                
            # Étape 4: Atterrissage (si demandé)
            if mission.auto_land:
                if not self._execute_landing():
                    return False
                    
            self.logger.info("✅ Mission terminée avec succès!")
            return True
            
        except Exception as e:
            self.logger.error(f"❌ Erreur pendant la mission: {e}")
            return False
        finally:
            self.is_executing = False
            
    def abort_mission(self):
        """Interrompt la mission en cours"""
        if self.is_executing:
            self.logger.warn("🛑 ARRÊT DE MISSION DEMANDÉ")
            self.is_executing = False
            
            # Activer le mode d'urgence
            self.drone_interface._cmd_emergency()
            
    def _preflight_checks(self) -> bool:
        """Vérifications pré-vol"""
        self.logger.info("🔍 Vérifications pré-vol...")
        
        # Vérifier l'état du drone
        status = self.drone_interface.state_manager.get_status()
        
        if not status.connected:
            self.logger.error("❌ Drone non connecté")
            return False
            
        # Vérifier les conditions de sécurité
        can_arm, reason = self.drone_interface.safety_manager.check_arm_conditions(status)
        if not can_arm:
            self.logger.error(f"❌ Conditions d'armement non remplies: {reason}")
            return False
            
        self.logger.info("✅ Vérifications pré-vol réussies")
        return True
        
    def _execute_takeoff(self, altitude: float) -> bool:
        """Exécute le décollage"""
        self.logger.info(f"🚀 Décollage vers {altitude}m...")
        
        success, msg = self.drone_interface.takeoff_landing_manager.takeoff(altitude)
        
        if success:
            self.logger.info("✅ Décollage réussi")
            return True
        else:
            self.logger.error(f"❌ Échec du décollage: {msg}")
            return False
            
    def _execute_waypoints(self, waypoints: List[Waypoint]) -> bool:
        """Exécute la séquence de waypoints"""
        self.logger.info(f"🗺️ Exécution de {len(waypoints)} waypoints...")
        
        for i, waypoint in enumerate(waypoints):
            if not self.is_executing:
                self.logger.warn("🛑 Mission interrompue")
                return False
                
            # Vérifier le timeout de mission
            if time.time() - self.mission_start_time > self.current_mission.max_duration:
                self.logger.error("⏰ Timeout de mission dépassé")
                return False
                
            self.current_waypoint_index = i
            self.logger.info(f"📍 Waypoint {i+1}/{len(waypoints)}: ({waypoint.x:.1f}, {waypoint.y:.1f}, {waypoint.z:.1f})")
            
            # Aller vers le waypoint
            success = self.drone_interface.position_controller.set_position(
                waypoint.x, waypoint.y, waypoint.z, waypoint.yaw
            )
            
            if not success:
                self.logger.error(f"❌ Impossible de définir le waypoint {i+1}")
                return False
                
            # Attendre d'atteindre le waypoint
            if not self._wait_for_waypoint(waypoint):
                self.logger.error(f"❌ Impossible d'atteindre le waypoint {i+1}")
                return False
                
            # Temps d'attente au waypoint
            if waypoint.hold_time > 0:
                self.logger.info(f"⏳ Attente de {waypoint.hold_time}s au waypoint...")
                time.sleep(waypoint.hold_time)
                
        self.logger.info("✅ Tous les waypoints ont été atteints")
        return True
        
    def _wait_for_waypoint(self, waypoint: Waypoint, timeout: float = 30.0) -> bool:
        """Attend d'atteindre un waypoint"""
        start_time = time.time()
        
        while time.time() - start_time < timeout and self.is_executing:
            status = self.drone_interface.state_manager.get_status()
            current_x, current_y, current_z = status.position
            
            # Calculer la distance au waypoint
            distance = math.sqrt(
                (current_x - waypoint.x)**2 + 
                (current_y - waypoint.y)**2 + 
                (current_z - waypoint.z)**2
            )
            
            if distance <= waypoint.tolerance:
                self.logger.info(f"✅ Waypoint atteint (distance: {distance:.2f}m)")
                return True
                
            time.sleep(0.5)  # Vérifier toutes les 500ms
            
        return False
        
    def _execute_landing(self) -> bool:
        """Exécute l'atterrissage"""
        self.logger.info("🛬 Atterrissage...")
        
        success, msg = self.drone_interface.takeoff_landing_manager.land()
        
        if success:
            self.logger.info("✅ Atterrissage réussi")
            return True
        else:
            self.logger.error(f"❌ Échec de l'atterrissage: {msg}")
            return False


def create_missions() -> Dict[str, Mission]:
    """Crée une collection de missions prédéfinies"""
    
    missions = {}
    
    # Mission 1: Vol de base
    missions["basic_flight"] = Mission(
        name="Vol de Base",
        description="Décollage, vol stationnaire et atterrissage simple",
        waypoints=[
            Waypoint(0.0, 0.0, 3.0, hold_time=5.0)  # Hover à 3m pendant 5s
        ],
        takeoff_altitude=3.0
    )
    
    # Mission 2: Motif carré
    missions["square_pattern"] = Mission(
        name="Motif Carré",
        description="Vol en forme de carré de 10m x 10m",
        waypoints=[
            Waypoint(5.0, 0.0, 3.0, hold_time=3.0),   # Point 1
            Waypoint(5.0, 5.0, 3.0, hold_time=3.0),   # Point 2
            Waypoint(0.0, 5.0, 3.0, hold_time=3.0),   # Point 3
            Waypoint(0.0, 0.0, 3.0, hold_time=3.0),   # Retour au centre
        ],
        takeoff_altitude=3.0
    )
    
    # Mission 3: Vol en triangle
    missions["triangle_pattern"] = Mission(
        name="Motif Triangle",
        description="Vol en forme de triangle équilatéral",
        waypoints=[
            Waypoint(5.0, 0.0, 4.0, hold_time=2.0),                    # Point 1
            Waypoint(-2.5, 4.33, 4.0, hold_time=2.0),                  # Point 2 (60°)
            Waypoint(-2.5, -4.33, 4.0, hold_time=2.0),                 # Point 3 (120°)
            Waypoint(0.0, 0.0, 4.0, hold_time=3.0),                    # Retour au centre
        ],
        takeoff_altitude=4.0
    )
    
    # Mission 4: Vol en cercle
    missions["circle_pattern"] = Mission(
        name="Motif Cercle",
        description="Vol en cercle de 8m de rayon",
        waypoints=[
            Waypoint(8.0, 0.0, 5.0, yaw=math.pi/2, hold_time=1.0),     # 0°
            Waypoint(5.66, 5.66, 5.0, yaw=math.pi, hold_time=1.0),     # 45°
            Waypoint(0.0, 8.0, 5.0, yaw=3*math.pi/2, hold_time=1.0),   # 90°
            Waypoint(-5.66, 5.66, 5.0, yaw=0.0, hold_time=1.0),        # 135°
            Waypoint(-8.0, 0.0, 5.0, yaw=math.pi/2, hold_time=1.0),    # 180°
            Waypoint(-5.66, -5.66, 5.0, yaw=math.pi, hold_time=1.0),   # 225°
            Waypoint(0.0, -8.0, 5.0, yaw=3*math.pi/2, hold_time=1.0),  # 270°
            Waypoint(5.66, -5.66, 5.0, yaw=0.0, hold_time=1.0),        # 315°
            Waypoint(0.0, 0.0, 5.0, hold_time=3.0),                    # Retour centre
        ],
        takeoff_altitude=5.0
    )
    
    # Mission 5: Test d'altitude
    missions["altitude_test"] = Mission(
        name="Test d'Altitude",
        description="Test de montée et descente à différentes altitudes",
        waypoints=[
            Waypoint(0.0, 0.0, 2.0, hold_time=3.0),   # 2m
            Waypoint(0.0, 0.0, 5.0, hold_time=3.0),   # 5m
            Waypoint(0.0, 0.0, 8.0, hold_time=3.0),   # 8m
            Waypoint(0.0, 0.0, 5.0, hold_time=3.0),   # Redescendre à 5m
            Waypoint(0.0, 0.0, 2.0, hold_time=3.0),   # Redescendre à 2m
        ],
        takeoff_altitude=2.0
    )
    
    # Mission 6: Inspection de périmètre
    missions["perimeter_inspection"] = Mission(
        name="Inspection Périmètre",
        description="Inspection d'un périmètre rectangulaire de 20m x 15m",
        waypoints=[
            Waypoint(10.0, 0.0, 6.0, yaw=math.pi/2, hold_time=2.0),    # Est
            Waypoint(10.0, 7.5, 6.0, yaw=math.pi, hold_time=2.0),      # Nord-Est
            Waypoint(0.0, 7.5, 6.0, yaw=3*math.pi/2, hold_time=2.0),   # Nord
            Waypoint(-10.0, 7.5, 6.0, yaw=math.pi, hold_time=2.0),     # Nord-Ouest
            Waypoint(-10.0, 0.0, 6.0, yaw=3*math.pi/2, hold_time=2.0), # Ouest
            Waypoint(-10.0, -7.5, 6.0, yaw=0.0, hold_time=2.0),        # Sud-Ouest
            Waypoint(0.0, -7.5, 6.0, yaw=math.pi/2, hold_time=2.0),    # Sud
            Waypoint(10.0, -7.5, 6.0, yaw=0.0, hold_time=2.0),         # Sud-Est
            Waypoint(0.0, 0.0, 6.0, hold_time=5.0),                    # Retour centre
        ],
        takeoff_altitude=6.0,
        max_duration=600.0  # 10 minutes maximum
    )
    
    # Mission 7: Test de vitesse
    missions["speed_test"] = Mission(
        name="Test de Vitesse",
        description="Test de déplacement rapide entre points distants",
        waypoints=[
            Waypoint(15.0, 0.0, 4.0, hold_time=2.0, tolerance=2.0),    # Point distant Est
            Waypoint(-15.0, 0.0, 4.0, hold_time=2.0, tolerance=2.0),   # Point distant Ouest
            Waypoint(0.0, 15.0, 4.0, hold_time=2.0, tolerance=2.0),    # Point distant Nord
            Waypoint(0.0, -15.0, 4.0, hold_time=2.0, tolerance=2.0),   # Point distant Sud
            Waypoint(0.0, 0.0, 4.0, hold_time=5.0),                    # Retour centre
        ],
        takeoff_altitude=4.0,
        max_duration=300.0
    )
    
    return missions


class DroneExampleNode(Node):
    """Nœud ROS2 pour exécuter les exemples de missions"""
    
    def __init__(self, mission_name: str = None, config_profile: str = "simulation"):
        super().__init__('drone_examples')
        
        self.logger = self.get_logger()
        self.logger.info("🎯 Initialisation du nœud d'exemples de missions...")
        
        # Charger la configuration
        self.config = create_config(config_profile)
        print_config_info(self.config)
        
        # Créer l'interface drone
        self.drone_interface = DroneInterface()
        
        # Créer l'exécuteur de missions
        self.mission_executor = MissionExecutor(self.drone_interface)
        
        # Charger les missions disponibles
        self.missions = create_missions()
        
        self.logger.info("✅ Nœud d'exemples initialisé")
        
        # Exécuter la mission si spécifiée
        if mission_name:
            self.execute_mission(mission_name)
            
    def list_missions(self):
        """Affiche la liste des missions disponibles"""
        self.logger.info("=" * 60)
        self.logger.info("📋 MISSIONS DISPONIBLES")
        self.logger.info("=" * 60)
        
        for name, mission in self.missions.items():
            self.logger.info(f"🎯 {name}:")
            self.logger.info(f"   • Nom: {mission.name}")
            self.logger.info(f"   • Description: {mission.description}")
            self.logger.info(f"   • Waypoints: {len(mission.waypoints)}")
            self.logger.info(f"   • Altitude: {mission.takeoff_altitude}m")
            self.logger.info(f"   • Durée max: {mission.max_duration}s")
            self.logger.info("")
            
    def execute_mission(self, mission_name: str):
        """Exécute une mission spécifique"""
        if mission_name not in self.missions:
            self.logger.error(f"❌ Mission '{mission_name}' introuvable")
            self.list_missions()
            return
            
        mission = self.missions[mission_name]
        
        self.logger.info("=" * 60)
        self.logger.info(f"🚀 EXÉCUTION DE LA MISSION: {mission.name}")
        self.logger.info("=" * 60)
        
        try:
            # Attendre que ROS2 soit prêt
            self.logger.info("⏳ Attente de la connexion MAVROS...")
            timeout = 30.0
            start_time = time.time()
            
            while time.time() - start_time < timeout:
                status = self.drone_interface.state_manager.get_status()
                if status.connected:
                    break
                time.sleep(1.0)
                rclpy.spin_once(self.drone_interface, timeout_sec=0.1)
            else:
                self.logger.error("❌ Timeout: MAVROS non connecté")
                return
                
            self.logger.info("✅ MAVROS connecté")
            
            # Exécuter la mission
            success = self.mission_executor.execute_mission(mission)
            
            if success:
                self.logger.info("🎉 MISSION RÉUSSIE!")
            else:
                self.logger.error("💥 MISSION ÉCHOUÉE!")
                
        except KeyboardInterrupt:
            self.logger.warn("🛑 Interruption utilisateur")
            self.mission_executor.abort_mission()
        except Exception as e:
            self.logger.error(f"💥 Erreur inattendue: {e}")
            self.mission_executor.abort_mission()


def main():
    """Point d'entrée principal"""
    parser = argparse.ArgumentParser(description="Exemples de missions pour drone")
    parser.add_argument('--mission', '-m', help='Nom de la mission à exécuter')
    parser.add_argument('--list-missions', '-l', action='store_true', help='Lister les missions disponibles')
    parser.add_argument('--config', '-c', default='simulation', help='Profil de configuration (default, simulation, etc.)')
    
    args = parser.parse_args()
    
    # Initialiser ROS2
    rclpy.init()
    
    try:
        # Créer le nœud
        node = DroneExampleNode(config_profile=args.config)
        
        if args.list_missions:
            node.list_missions()
        elif args.mission:
            node.execute_mission(args.mission)
        else:
            node.list_missions()
            node.logger.info("💡 Utilisez --mission <nom> pour exécuter une mission")
            node.logger.info("💡 Utilisez --list-missions pour voir toutes les missions")
            
        # Garder le nœud actif
        if not args.list_missions:
            rclpy.spin(node)
            
    except KeyboardInterrupt:
        print("\n🛑 Arrêt demandé par l'utilisateur")
    except Exception as e:
        print(f"❌ Erreur: {e}")
    finally:
        # Nettoyage
        rclpy.shutdown()
        print("👋 Exemples de missions arrêtés")


if __name__ == '__main__':
    main()
