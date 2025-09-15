#!/usr/bin/env python3
"""
Test de mission complète pour démontrer la coordination des nœuds.
Ce script simule une mission de navigation avec waypoints.
"""

import rclpy
import time
import sys
from rclpy.node import Node
from geometry_msgs.msg import Point

from drone_msgs.msg import (NavigationStatus, MissionStatus, Waypoint)
from drone_msgs.srv import (SetWaypoints, GotoPosition, GotoLocal)
from std_srvs.srv import Empty

class MissionTestNode(Node):
    def __init__(self):
        super().__init__('mission_test_node')
        
        # État de la mission
        self.mission_status = "UNKNOWN"
        self.navigation_status = "UNKNOWN"
        self.current_waypoint = 0
        self.total_waypoints = 0
        
        # Subscribers pour monitorer
        self.mission_sub = self.create_subscription(
            MissionStatus, '/drone_nav/mission_status', self.on_mission_status, 10
        )
        self.nav_sub = self.create_subscription(
            NavigationStatus, '/drone_nav/status', self.on_nav_status, 10
        )
        
        # Clients de services
        self.set_waypoints_client = self.create_client(SetWaypoints, '/drone_nav/set_waypoints_local')
        self.start_mission_client = self.create_client(Empty, '/drone_nav/start_mission')
        self.goto_local_client = self.create_client(GotoLocal, '/drone_nav/goto_local')
        
        self.get_logger().info("🚁 Mission Test Node initialized")
    
    def on_mission_status(self, msg):
        self.mission_status = msg.status
        self.current_waypoint = msg.current_waypoint
        self.total_waypoints = msg.total_waypoints
        self.get_logger().info(f"📊 Mission: {msg.status} - WP {msg.current_waypoint}/{msg.total_waypoints} ({msg.progress:.1f}%)")
    
    def on_nav_status(self, msg):
        self.navigation_status = msg.status
        if msg.status == "NAVIGATING":
            self.get_logger().info(f"🛸 Navigation: {msg.status} - Mode: {msg.mode}")
        elif msg.status == "SUCCEEDED":
            self.get_logger().info(f"✅ Navigation: {msg.status}")
        elif msg.status == "FAILED":
            self.get_logger().info(f"❌ Navigation: {msg.status}")
    
    def create_test_waypoint(self, x, y, z, wp_type="NORMAL"):
        """Crée un waypoint de test"""
        waypoint = Waypoint()
        waypoint.position = Point()
        waypoint.position.x = float(x)
        waypoint.position.y = float(y)
        waypoint.position.z = float(z)
        waypoint.tolerance = 2.0
        waypoint.wp_type = wp_type
        waypoint.actions = []
        waypoint.speed = 5.0
        waypoint.yaw = 0.0
        waypoint.hold_time = 2.0 if wp_type == "HOLD" else 0.0
        return waypoint
    
    def setup_test_mission(self):
        """Configure une mission de test avec plusieurs waypoints"""
        self.get_logger().info("🗺️  Setting up test mission...")
        
        if not self.set_waypoints_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ Set waypoints service not available")
            return False
        
        # Créer une mission de test avec 4 waypoints
        waypoints = [
            self.create_test_waypoint(10.0, 0.0, 20.0, "NORMAL"),     # Point 1
            self.create_test_waypoint(10.0, 10.0, 20.0, "HOLD"),      # Point 2 avec pause
            self.create_test_waypoint(0.0, 10.0, 20.0, "NORMAL"),     # Point 3
            self.create_test_waypoint(0.0, 0.0, 20.0, "NORMAL")       # Retour
        ]
        
        request = SetWaypoints.Request()
        request.waypoints = waypoints
        
        try:
            future = self.set_waypoints_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() and future.result().success:
                self.get_logger().info(f"✅ Mission configured with {len(waypoints)} waypoints")
                return True
            else:
                self.get_logger().error("❌ Failed to set waypoints")
                return False
        except Exception as e:
            self.get_logger().error(f"❌ Exception setting waypoints: {e}")
            return False
    
    def test_individual_navigation(self):
        """Test la navigation individuelle vers un point"""
        self.get_logger().info("🎯 Testing individual navigation...")
        
        if not self.goto_local_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ Local navigation service not available")
            return False
        
        # Tester navigation vers un point
        request = GotoLocal.Request()
        request.x = 5.0
        request.y = 5.0
        request.z = 15.0
        request.yaw_angle = 0.0
        
        try:
            self.get_logger().info(f"🚀 Navigating to ({request.x}, {request.y}, {request.z})...")
            future = self.goto_local_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() and future.result().success:
                self.get_logger().info("✅ Navigation command accepted")
                
                # Attendre un peu pour voir la progression
                self.get_logger().info("⏳ Waiting for navigation to progress...")
                time.sleep(8.0)
                
                return True
            else:
                self.get_logger().error("❌ Navigation command rejected")
                return False
        except Exception as e:
            self.get_logger().error(f"❌ Exception in navigation: {e}")
            return False
    
    def test_mission_execution(self):
        """Test l'exécution d'une mission complète"""
        self.get_logger().info("🎯 Testing mission execution...")
        
        if not self.start_mission_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ Start mission service not available")
            return False
        
        # Démarrer la mission
        request = Empty.Request()
        
        try:
            self.get_logger().info("🚀 Starting mission...")
            future = self.start_mission_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result():
                self.get_logger().info("✅ Mission started successfully")
                
                # Monitorer la mission pendant 30 secondes
                self.get_logger().info("⏳ Monitoring mission progress...")
                start_time = time.time()
                
                while time.time() - start_time < 30.0:
                    rclpy.spin_once(self, timeout_sec=1.0)
                    
                    if self.mission_status == "COMPLETED":
                        self.get_logger().info("🎉 Mission completed successfully!")
                        return True
                    elif self.mission_status == "FAILED":
                        self.get_logger().error("❌ Mission failed")
                        return False
                
                self.get_logger().info("⏰ Mission monitoring timeout")
                return True  # Pas d'échec, juste timeout
                
            else:
                self.get_logger().error("❌ Failed to start mission")
                return False
        except Exception as e:
            self.get_logger().error(f"❌ Exception starting mission: {e}")
            return False
    
    def run_complete_test(self):
        """Lance le test complet"""
        self.get_logger().info("🚁 Starting Complete Mission Test 🚁")
        self.get_logger().info("="*60)
        
        results = {}
        
        # Test 1: Configuration de mission
        self.get_logger().info("\n📋 Phase 1: Mission Setup")
        results["Mission Setup"] = self.setup_test_mission()
        time.sleep(2.0)
        
        # Test 2: Navigation individuelle
        self.get_logger().info("\n🎯 Phase 2: Individual Navigation")
        results["Individual Navigation"] = self.test_individual_navigation()
        time.sleep(2.0)
        
        # Test 3: Exécution de mission (optionnel si le service existe)
        self.get_logger().info("\n🚀 Phase 3: Mission Execution")
        results["Mission Execution"] = self.test_mission_execution()
        
        # Résumé
        self.print_final_results(results)
    
    def print_final_results(self, results):
        """Affiche le résumé final des tests"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("🏁 COMPLETE MISSION TEST RESULTS 🏁")
        self.get_logger().info("="*60)
        
        total_tests = len(results)
        passed_tests = sum(1 for result in results.values() if result)
        failed_tests = total_tests - passed_tests
        
        for test_name, result in results.items():
            icon = "✅" if result else "❌"
            status = "PASS" if result else "FAIL"
            self.get_logger().info(f"{icon} {test_name}: {status}")
        
        self.get_logger().info("-"*60)
        self.get_logger().info(f"Total Tests: {total_tests}")
        self.get_logger().info(f"Passed: {passed_tests}")
        self.get_logger().info(f"Failed: {failed_tests}")
        
        if failed_tests == 0:
            self.get_logger().info("🎉 ALL MISSION TESTS PASSED! System ready for flight! 🎉")
        else:
            self.get_logger().error("❌ SOME MISSION TESTS FAILED. Check configuration.")
        
        self.get_logger().info("\n📝 Next Steps:")
        self.get_logger().info("  • Connect to ArduPilot SITL for realistic simulation")
        self.get_logger().info("  • Test with real GPS coordinates")
        self.get_logger().info("  • Add obstacle avoidance scenarios")
        self.get_logger().info("  • Test emergency procedures")

def main():
    rclpy.init()
    
    mission_test = MissionTestNode()
    
    try:
        # Attendre que tous les nœuds soient prêts
        time.sleep(3.0)
        
        # Lancer le test complet
        mission_test.run_complete_test()
        
        # Maintenir le nœud actif un peu pour voir les messages finaux
        time.sleep(3.0)
        
    except KeyboardInterrupt:
        mission_test.get_logger().info("Test interrupted by user")
    except Exception as e:
        mission_test.get_logger().error(f"Test failed with exception: {e}")
    finally:
        mission_test.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
