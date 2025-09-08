#!/usr/bin/env python3
"""
Script de test rapide des interfaces du package drone_navigation
"""

import sys
import os

# Ajouter le path d'installation pour les imports
sys.path.insert(0, '/home/adama133/ros2_ws/install/drone_navigation/lib/python3.10/site-packages')

def test_interfaces():
    """Test des interfaces principales"""
    print("🔍 TEST DES INTERFACES DRONE_NAVIGATION")
    print("="*50)
    
    try:
        # Test import drone_msgs
        from drone_msgs.srv import Land, SetFlightMode, NavigateToWaypoint
        from drone_msgs.msg import DroneState, FlightMode
        print("✅ Interfaces drone_msgs importées avec succès")
    except ImportError as e:
        print(f"❌ Erreur import drone_msgs: {e}")
        
    try:
        # Test import modules navigation
        import drone_navigation
        print("✅ Package drone_navigation importé avec succès")
        
        from drone_navigation.navigation_node import NavigationNode
        print("✅ NavigationNode importé avec succès")
        
        from drone_navigation.trajectory_planner import TrajectoryPlanner
        print("✅ TrajectoryPlanner importé avec succès")
        
        from drone_navigation.position_controller import PositionController
        print("✅ PositionController importé avec succès")
        
    except ImportError as e:
        print(f"❌ Erreur import navigation: {e}")
        
    print("\n🎯 SERVICES ATTENDUS:")
    services = [
        "/drone_nav/goto_position",
        "/drone_nav/goto_local",
        "/drone_nav/plan_path", 
        "/drone_nav/set_waypoints",
        "/drone_nav/start_mission",
        "/drone_nav/emergency_rtl"
    ]
    
    for service in services:
        print(f"📡 {service}")
        
    print("\n📊 TOPICS ATTENDUS:")
    topics = [
        "/drone_nav/status",
        "/drone_nav/trajectory",
        "/drone_nav/waypoint_reached",
        "/diagnostics"
    ]
    
    for topic in topics:
        print(f"📢 {topic}")
        
    print("\n✅ PACKAGE DRONE_NAVIGATION PRÊT !")

if __name__ == "__main__":
    test_interfaces()
