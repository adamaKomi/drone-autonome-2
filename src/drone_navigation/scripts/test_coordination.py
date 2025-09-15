#!/usr/bin/env python3
"""
Script de test pour valider la coordination des nœuds de navigation.
Ce script teste les interactions entre tous les nœuds du package drone_navigation.
"""

import rclpy
import time
import sys
from rclpy.node import Node
from geometry_msgs.msg import Point
from std_msgs.msg import Bool, String

from drone_msgs.msg import (NavigationStatus, MissionStatus, PathProgress, 
                           Waypoint, WaypointList)
from drone_msgs.srv import (GotoPosition, GotoLocal, SetWaypoints, 
                           GetWaypoints, PauseMission, ResumeMission)

class NavigationTestNode(Node):
    def __init__(self):
        super().__init__('navigation_test_node')
        
        # État des tests
        self.test_results = {}
        self.received_messages = {}
        
        # Subscribers pour monitorer les topics
        self.status_sub = self.create_subscription(
            NavigationStatus, '/drone_nav/status', self.on_nav_status, 10
        )
        self.mission_status_sub = self.create_subscription(
            MissionStatus, '/drone_nav/mission_status', self.on_mission_status, 10
        )
        self.progress_sub = self.create_subscription(
            PathProgress, '/drone_nav/progress', self.on_progress, 10
        )
        self.safety_sub = self.create_subscription(
            Bool, '/drone_nav/safe_to_navigate', self.on_safety, 10
        )
        self.waypoints_sub = self.create_subscription(
            WaypointList, '/drone_nav/waypoints_list', self.on_waypoints, 10
        )
        
        # Clients pour les services
        self.gps_client = self.create_client(GotoPosition, '/drone_nav/goto_position')
        self.local_client = self.create_client(GotoLocal, '/drone_nav/goto_local')
        self.set_waypoints_client = self.create_client(SetWaypoints, '/drone_nav/set_waypoints_local')
        self.get_waypoints_client = self.create_client(GetWaypoints, '/drone_nav/get_waypoints_local')
        
        self.get_logger().info("Navigation Test Node initialized")
    
    def on_nav_status(self, msg):
        self.received_messages['nav_status'] = msg
        self.get_logger().info(f"Navigation Status: {msg.status} - {msg.mode}")
    
    def on_mission_status(self, msg):
        self.received_messages['mission_status'] = msg
        self.get_logger().info(f"Mission Status: {msg.status} - WP {msg.current_waypoint}/{msg.total_waypoints}")
    
    def on_progress(self, msg):
        self.received_messages['progress'] = msg
        if msg.distance_remaining < float('inf'):
            self.get_logger().info(f"Progress: {msg.progress:.2f} - Distance: {msg.distance_remaining:.2f}m")
    
    def on_safety(self, msg):
        self.received_messages['safety'] = msg
        self.get_logger().info(f"Safe to navigate: {msg.data}")
    
    def on_waypoints(self, msg):
        self.received_messages['waypoints'] = msg
        self.get_logger().info(f"Waypoints list: {len(msg.waypoints)} waypoints")
    
    def test_services_availability(self):
        """Test la disponibilité des services"""
        self.get_logger().info("=== Testing Services Availability ===")
        
        services = [
            (self.gps_client, "GPS Navigation"),
            (self.local_client, "Local Navigation"),
            (self.set_waypoints_client, "Set Waypoints"),
            (self.get_waypoints_client, "Get Waypoints")
        ]
        
        for client, name in services:
            if client.wait_for_service(timeout_sec=3.0):
                self.get_logger().info(f"✓ {name} service available")
                self.test_results[name] = "PASS"
            else:
                self.get_logger().error(f"✗ {name} service NOT available")
                self.test_results[name] = "FAIL"
    
    def test_topic_communication(self):
        """Test la réception des messages sur les topics"""
        self.get_logger().info("=== Testing Topic Communication ===")
        
        # Attendre les messages
        time.sleep(3.0)
        
        expected_topics = [
            ('nav_status', 'Navigation Status'),
            ('mission_status', 'Mission Status'),
            ('safety', 'Safety Status'),
            ('waypoints', 'Waypoints List')
        ]
        
        for topic_key, topic_name in expected_topics:
            if topic_key in self.received_messages:
                self.get_logger().info(f"✓ {topic_name} messages received")
                self.test_results[topic_name] = "PASS"
            else:
                self.get_logger().warning(f"⚠ {topic_name} messages NOT received")
                self.test_results[topic_name] = "PARTIAL"
    
    def test_waypoint_management(self):
        """Test la gestion des waypoints"""
        self.get_logger().info("=== Testing Waypoint Management ===")
        
        # Test set waypoints
        if self.set_waypoints_client.wait_for_service(timeout_sec=3.0):
            waypoints = [
                self.create_test_waypoint(10.0, 20.0, 30.0),
                self.create_test_waypoint(15.0, 25.0, 35.0)
            ]
            
            request = SetWaypoints.Request()
            request.waypoints = waypoints
            
            try:
                future = self.set_waypoints_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.result() and future.result().success:
                    self.get_logger().info("✓ Waypoints set successfully")
                    self.test_results["Set Waypoints"] = "PASS"
                else:
                    self.get_logger().error("✗ Failed to set waypoints")
                    self.test_results["Set Waypoints"] = "FAIL"
            except Exception as e:
                self.get_logger().error(f"✗ Exception setting waypoints: {e}")
                self.test_results["Set Waypoints"] = "FAIL"
        
        # Test get waypoints
        if self.get_waypoints_client.wait_for_service(timeout_sec=3.0):
            request = GetWaypoints.Request()
            
            try:
                future = self.get_waypoints_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.result() and future.result().success:
                    waypoints = future.result().waypoints
                    self.get_logger().info(f"✓ Retrieved {len(waypoints)} waypoints")
                    self.test_results["Get Waypoints"] = "PASS"
                else:
                    self.get_logger().error("✗ Failed to get waypoints")
                    self.test_results["Get Waypoints"] = "FAIL"
            except Exception as e:
                self.get_logger().error(f"✗ Exception getting waypoints: {e}")
                self.test_results["Get Waypoints"] = "FAIL"
    
    def test_local_navigation(self):
        """Test la navigation locale"""
        self.get_logger().info("=== Testing Local Navigation ===")
        
        if self.local_client.wait_for_service(timeout_sec=3.0):
            request = GotoLocal.Request()
            request.x = 5.0
            request.y = 10.0
            request.z = 15.0
            request.yaw_angle = 0.0
            
            try:
                future = self.local_client.call_async(request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
                
                if future.result() and future.result().success:
                    self.get_logger().info("✓ Local navigation command accepted")
                    self.test_results["Local Navigation"] = "PASS"
                else:
                    self.get_logger().error("✗ Local navigation command rejected")
                    self.test_results["Local Navigation"] = "FAIL"
            except Exception as e:
                self.get_logger().error(f"✗ Exception in local navigation: {e}")
                self.test_results["Local Navigation"] = "FAIL"
        else:
            self.test_results["Local Navigation"] = "FAIL"
    
    def create_test_waypoint(self, x, y, z):
        """Crée un waypoint de test"""
        waypoint = Waypoint()
        waypoint.position = Point()
        waypoint.position.x = x
        waypoint.position.y = y
        waypoint.position.z = z
        waypoint.tolerance = 2.0
        waypoint.wp_type = "NORMAL"
        waypoint.actions = []
        waypoint.speed = 5.0
        waypoint.yaw = 0.0
        waypoint.hold_time = 0.0
        return waypoint
    
    def run_all_tests(self):
        """Lance tous les tests"""
        self.get_logger().info("🚁 Starting Navigation System Coordination Tests 🚁")
        
        self.test_services_availability()
        time.sleep(1.0)
        
        self.test_topic_communication()
        time.sleep(1.0)
        
        self.test_waypoint_management()
        time.sleep(1.0)
        
        self.test_local_navigation()
        time.sleep(1.0)
        
        self.print_test_summary()
    
    def print_test_summary(self):
        """Affiche le résumé des tests"""
        self.get_logger().info("="*50)
        self.get_logger().info("🏁 TEST SUMMARY 🏁")
        self.get_logger().info("="*50)
        
        total_tests = len(self.test_results)
        passed_tests = sum(1 for result in self.test_results.values() if result == "PASS")
        partial_tests = sum(1 for result in self.test_results.values() if result == "PARTIAL")
        failed_tests = sum(1 for result in self.test_results.values() if result == "FAIL")
        
        for test_name, result in self.test_results.items():
            icon = "✓" if result == "PASS" else "⚠" if result == "PARTIAL" else "✗"
            self.get_logger().info(f"{icon} {test_name}: {result}")
        
        self.get_logger().info("-"*50)
        self.get_logger().info(f"Total Tests: {total_tests}")
        self.get_logger().info(f"Passed: {passed_tests}")
        self.get_logger().info(f"Partial: {partial_tests}")
        self.get_logger().info(f"Failed: {failed_tests}")
        
        if failed_tests == 0 and partial_tests == 0:
            self.get_logger().info("🎉 ALL TESTS PASSED! Navigation system is ready! 🎉")
        elif failed_tests == 0:
            self.get_logger().info("⚠️  TESTS MOSTLY PASSED but some topics not active")
        else:
            self.get_logger().error("❌ SOME TESTS FAILED. Check node status and retry.")

def main():
    rclpy.init()
    
    test_node = NavigationTestNode()
    
    try:
        # Lancer les tests après un petit délai
        time.sleep(2.0)
        test_node.run_all_tests()
        
        # Maintenir le nœud actif pour quelques secondes pour voir les messages
        time.sleep(3.0)
        
    except KeyboardInterrupt:
        test_node.get_logger().info("Test interrupted by user")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
