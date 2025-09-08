#!/usr/bin/env python3

"""
Test d'intégration pour le package drone_navigation

Vérifie que tous les modules sont correctement intégrés et
fonctionnent ensemble.
"""

import sys
import traceback
from geometry_msgs.msg import Point

# Mock d'un nœud ROS2 pour les tests
class MockNode:
    def __init__(self):
        self.pid_position = {
            'p_xy': 1.0, 'i_xy': 0.1, 'd_xy': 0.05,
            'p_z': 1.5, 'i_z': 0.2, 'd_z': 0.1
        }
        self.pid_velocity = {
            'p_xy': 0.8, 'i_xy': 0.05, 'd_xy': 0.02
        }
        self.limits = {
            'max_velocity_xy': 10.0,
            'max_velocity_z': 5.0,
            'max_acceleration': 5.0,
            'max_jerk': 10.0
        }
    
    def get_logger(self):
        return MockLogger()
    
    def trigger_emergency_rtl(self):
        print("RTL déclenché")

class MockLogger:
    def info(self, msg):
        print(f"INFO: {msg}")
    
    def warning(self, msg):
        print(f"WARNING: {msg}")
    
    def error(self, msg):
        print(f"ERROR: {msg}")
    
    def critical(self, msg):
        print(f"CRITICAL: {msg}")

def test_integration():
    """Test d'intégration des modules"""
    print("=== Test d'intégration drone_navigation ===\n")
    
    try:
        # Création d'un nœud mock
        node = MockNode()
        print("✓ Mock node créé")
        
        # Test du contrôleur de position
        from drone_navigation.position_controller import PositionController
        controller = PositionController(node)
        print("✓ PositionController initialisé")
        
        # Test des patterns de couverture
        from drone_navigation.coverage_patterns import CoveragePatterns, PatternType, CoverageZone
        patterns = CoveragePatterns(node)
        
        # Création d'une zone de test
        test_zone = CoverageZone(
            boundary=[
                Point(x=0.0, y=0.0, z=0.0),
                Point(x=100.0, y=0.0, z=0.0),
                Point(x=100.0, y=100.0, z=0.0),
                Point(x=0.0, y=100.0, z=0.0)
            ],
            name="Zone de test"
        )
        
        zigzag_pattern = patterns.generate_pattern(PatternType.ZIGZAG, test_zone)
        print(f"✓ Pattern zigzag généré: {len(zigzag_pattern)} waypoints")
        
        # Test de l'optimiseur de chemins
        from drone_navigation.path_optimizer import PathOptimizer
        optimizer = PathOptimizer(node)
        
        if len(zigzag_pattern) > 2:
            result = optimizer.optimize_path(zigzag_pattern)
            print(f"✓ Optimisation de chemin: {result.improvement_percentage:.1f}% d'amélioration")
        
        # Test du gestionnaire de géobarrières
        from drone_navigation.geofence_manager import GeofenceManager
        geofence = GeofenceManager(node)
        
        test_position = Point(x=50.0, y=50.0, z=10.0)
        violation = geofence.check_violation(test_position)
        print(f"✓ Vérification géobarrière: {'violation détectée' if violation else 'position sûre'}")
        
        # Test d'évitement d'obstacles
        from drone_navigation.obstacle_avoidance import ObstacleAvoidance
        avoidance = ObstacleAvoidance(node)
        print("✓ ObstacleAvoidance initialisé")
        
        print("\n=== Tous les tests d'intégration réussis! ===")
        return True
        
    except Exception as e:
        print(f"\n❌ Erreur lors du test d'intégration: {e}")
        print("Trace complète:")
        traceback.print_exc()
        return False

if __name__ == "__main__":
    success = test_integration()
    sys.exit(0 if success else 1)
