#!/usr/bin/env python3
"""
Script de test manuel pour les commandes de navigation.
Permet de tester facilement les commandes individuelles.
"""

import rclpy
import sys
from rclpy.node import Node
from geometry_msgs.msg import Point

from drone_msgs.msg import Waypoint
from drone_msgs.srv import GotoLocal, GotoPosition, SetWaypoints, GetWaypoints

def print_usage():
    print("🚁 Drone Navigation Manual Test Commands")
    print("="*50)
    print("Usage: python3 test_manual.py <command> [args]")
    print("")
    print("Commands:")
    print("  goto_local <x> <y> <z>     - Navigate to local coordinates")
    print("  goto_gps <lat> <lon> <alt> - Navigate to GPS coordinates")
    print("  add_waypoint <x> <y> <z>   - Add a waypoint")
    print("  list_waypoints             - List all waypoints")
    print("  clear_waypoints            - Clear all waypoints")
    print("")
    print("Examples:")
    print("  python3 test_manual.py goto_local 10 20 30")
    print("  python3 test_manual.py goto_gps 45.123 -1.456 100")
    print("  python3 test_manual.py add_waypoint 5 10 15")
    print("  python3 test_manual.py list_waypoints")

class ManualTestNode(Node):
    def __init__(self):
        super().__init__('manual_test_node')
        
        # Clients de services
        self.goto_local_client = self.create_client(GotoLocal, '/drone_nav/goto_local')
        self.goto_gps_client = self.create_client(GotoPosition, '/drone_nav/goto_position')
        self.set_waypoints_client = self.create_client(SetWaypoints, '/drone_nav/set_waypoints_local')
        self.get_waypoints_client = self.create_client(GetWaypoints, '/drone_nav/get_waypoints_local')
        
        self.get_logger().info("Manual Test Node initialized")
    
    def goto_local(self, x, y, z):
        """Navigation locale"""
        if not self.goto_local_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ Local navigation service not available")
            return False
        
        request = GotoLocal.Request()
        request.x = float(x)
        request.y = float(y)
        request.z = float(z)
        request.yaw_angle = 0.0
        
        self.get_logger().info(f"🚀 Navigating to local ({x}, {y}, {z})...")
        
        try:
            future = self.goto_local_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() and future.result().success:
                self.get_logger().info(f"✅ {future.result().message}")
                return True
            else:
                self.get_logger().error(f"❌ {future.result().message if future.result() else 'No response'}")
                return False
        except Exception as e:
            self.get_logger().error(f"❌ Exception: {e}")
            return False
    
    def goto_gps(self, lat, lon, alt):
        """Navigation GPS"""
        if not self.goto_gps_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ GPS navigation service not available")
            return False
        
        request = GotoPosition.Request()
        request.latitude = float(lat)
        request.longitude = float(lon)
        request.altitude = float(alt)
        request.yaw_angle = 0.0
        request.tolerance = 5.0
        
        self.get_logger().info(f"🌍 Navigating to GPS ({lat}, {lon}, {alt})...")
        
        try:
            future = self.goto_gps_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() and future.result().success:
                self.get_logger().info(f"✅ {future.result().message}")
                return True
            else:
                self.get_logger().error(f"❌ {future.result().message if future.result() else 'No response'}")
                return False
        except Exception as e:
            self.get_logger().error(f"❌ Exception: {e}")
            return False
    
    def add_waypoint(self, x, y, z):
        """Ajouter un waypoint"""
        # D'abord récupérer les waypoints existants
        waypoints = self.get_waypoints_list()
        if waypoints is None:
            waypoints = []
        
        # Créer le nouveau waypoint
        waypoint = Waypoint()
        waypoint.position = Point()
        waypoint.position.x = float(x)
        waypoint.position.y = float(y)
        waypoint.position.z = float(z)
        waypoint.tolerance = 2.0
        waypoint.wp_type = "NORMAL"
        waypoint.actions = []
        waypoint.speed = 5.0
        waypoint.yaw = 0.0
        waypoint.hold_time = 0.0
        
        waypoints.append(waypoint)
        
        # Mettre à jour la liste
        if not self.set_waypoints_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ Set waypoints service not available")
            return False
        
        request = SetWaypoints.Request()
        request.waypoints = waypoints
        
        self.get_logger().info(f"📍 Adding waypoint ({x}, {y}, {z})...")
        
        try:
            future = self.set_waypoints_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() and future.result().success:
                self.get_logger().info(f"✅ Waypoint added. Total: {len(waypoints)} waypoints")
                return True
            else:
                self.get_logger().error(f"❌ Failed to add waypoint")
                return False
        except Exception as e:
            self.get_logger().error(f"❌ Exception: {e}")
            return False
    
    def get_waypoints_list(self):
        """Récupérer la liste des waypoints"""
        if not self.get_waypoints_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ Get waypoints service not available")
            return None
        
        request = GetWaypoints.Request()
        
        try:
            future = self.get_waypoints_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() and future.result().success:
                return future.result().waypoints
            else:
                return []
        except Exception as e:
            self.get_logger().error(f"❌ Exception getting waypoints: {e}")
            return None
    
    def list_waypoints(self):
        """Lister les waypoints"""
        self.get_logger().info("📋 Listing waypoints...")
        
        waypoints = self.get_waypoints_list()
        if waypoints is None:
            return False
        
        if len(waypoints) == 0:
            self.get_logger().info("📍 No waypoints configured")
        else:
            self.get_logger().info(f"📍 {len(waypoints)} waypoints configured:")
            for i, wp in enumerate(waypoints):
                pos = wp.position
                self.get_logger().info(f"  {i+1}: ({pos.x:.1f}, {pos.y:.1f}, {pos.z:.1f}) - {wp.wp_type}")
        
        return True
    
    def clear_waypoints(self):
        """Vider la liste des waypoints"""
        if not self.set_waypoints_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("❌ Set waypoints service not available")
            return False
        
        request = SetWaypoints.Request()
        request.waypoints = []  # Liste vide
        
        self.get_logger().info("🗑️  Clearing all waypoints...")
        
        try:
            future = self.set_waypoints_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() and future.result().success:
                self.get_logger().info("✅ All waypoints cleared")
                return True
            else:
                self.get_logger().error("❌ Failed to clear waypoints")
                return False
        except Exception as e:
            self.get_logger().error(f"❌ Exception: {e}")
            return False

def main():
    if len(sys.argv) < 2:
        print_usage()
        return
    
    command = sys.argv[1].lower()
    
    rclpy.init()
    node = ManualTestNode()
    
    try:
        if command == "goto_local":
            if len(sys.argv) != 5:
                print("❌ Usage: goto_local <x> <y> <z>")
                return
            x, y, z = sys.argv[2], sys.argv[3], sys.argv[4]
            node.goto_local(x, y, z)
            
        elif command == "goto_gps":
            if len(sys.argv) != 5:
                print("❌ Usage: goto_gps <lat> <lon> <alt>")
                return
            lat, lon, alt = sys.argv[2], sys.argv[3], sys.argv[4]
            node.goto_gps(lat, lon, alt)
            
        elif command == "add_waypoint":
            if len(sys.argv) != 5:
                print("❌ Usage: add_waypoint <x> <y> <z>")
                return
            x, y, z = sys.argv[2], sys.argv[3], sys.argv[4]
            node.add_waypoint(x, y, z)
            
        elif command == "list_waypoints":
            node.list_waypoints()
            
        elif command == "clear_waypoints":
            node.clear_waypoints()
            
        else:
            print(f"❌ Unknown command: {command}")
            print_usage()
            
    except KeyboardInterrupt:
        node.get_logger().info("Test interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Test failed: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
