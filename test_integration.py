#!/usr/bin/env python3
"""
Script de test d'intégration pour les nœuds drone_interface et drone_navigation
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from drone_navigation.srv import SetPosition, SetWaypoints, GetWaypoints
from drone_navigation.msg import Waypoint
from geometry_msgs.msg import Point
import time

class IntegrationTest(Node):
    def __init__(self):
        super().__init__('integration_test')
        
        # Clients de service
        self.safety_client = self.create_client(Trigger, '/drone/safety_check')
        self.arm_client = self.create_client(Trigger, '/drone/arm')
        self.set_position_client = self.create_client(SetPosition, '/navigation/set_position')
        self.set_waypoints_client = self.create_client(SetWaypoints, '/navigation/set_waypoints')
        self.get_waypoints_client = self.create_client(GetWaypoints, '/navigation/get_waypoints')
        self.hold_position_client = self.create_client(Trigger, '/navigation/hold_position')
        self.return_home_client = self.create_client(Trigger, '/navigation/return_home')
        
        self.get_logger().info("🧪 Test d'intégration initialisé")
    
    def wait_for_services(self):
        """Attendre que tous les services soient disponibles"""
        services = [
            ('/drone/safety_check', self.safety_client),
            ('/drone/arm', self.arm_client),
            ('/navigation/set_position', self.set_position_client),
            ('/navigation/set_waypoints', self.set_waypoints_client),
            ('/navigation/get_waypoints', self.get_waypoints_client),
            ('/navigation/hold_position', self.hold_position_client),
            ('/navigation/return_home', self.return_home_client)
        ]
        
        for service_name, client in services:
            self.get_logger().info(f"⏳ Attente du service {service_name}")
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"⏳ Service {service_name} non disponible...")
        
        self.get_logger().info("✅ Tous les services sont disponibles")
    
    def test_safety_check(self):
        """Test de vérification de sécurité"""
        self.get_logger().info("🔍 Test de vérification de sécurité")
        
        request = Trigger.Request()
        future = self.safety_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.success:
            self.get_logger().info(f"✅ Sécurité OK: {response.message}")
        else:
            self.get_logger().error(f"❌ Problème de sécurité: {response.message}")
        
        return response.success
    
    def test_navigation_single_position(self):
        """Test de navigation vers une position unique"""
        self.get_logger().info("🧭 Test de navigation vers position unique")
        
        # D'abord, remettre en position home
        self.hold_position()
        time.sleep(1)
        
        request = SetPosition.Request()
        request.position = Point(x=2.0, y=2.0, z=3.0)
        request.yaw = 0.0
        
        future = self.set_position_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.success:
            self.get_logger().info(f"✅ Navigation démarrée: {response.message}")
        else:
            self.get_logger().error(f"❌ Échec navigation: {response.message}")
        
        return response.success
    
    def test_navigation_multiple_waypoints(self):
        """Test de navigation avec plusieurs waypoints"""
        self.get_logger().info("🗺️ Test de navigation multi-waypoints")
        
        # D'abord, arrêter toute navigation en cours
        self.hold_position()
        time.sleep(1)
        
        # Créer des waypoints
        waypoints = []
        
        # Waypoint 1
        wp1 = Waypoint()
        wp1.position = Point(x=1.0, y=1.0, z=3.0)
        wp1.yaw = 0.0
        wp1.tolerance = 0.5
        wp1.yaw_tolerance = 0.1
        waypoints.append(wp1)
        
        # Waypoint 2
        wp2 = Waypoint()
        wp2.position = Point(x=3.0, y=1.0, z=4.0)
        wp2.yaw = 1.57
        wp2.tolerance = 0.5
        wp2.yaw_tolerance = 0.1
        waypoints.append(wp2)
        
        # Waypoint 3
        wp3 = Waypoint()
        wp3.position = Point(x=3.0, y=3.0, z=3.0)
        wp3.yaw = 3.14
        wp3.tolerance = 0.5
        wp3.yaw_tolerance = 0.1
        waypoints.append(wp3)
        
        request = SetWaypoints.Request()
        request.waypoints = waypoints
        
        future = self.set_waypoints_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.success:
            self.get_logger().info(f"✅ Waypoints définis: {response.message}")
        else:
            self.get_logger().error(f"❌ Échec waypoints: {response.message}")
        
        return response.success
    
    def test_get_waypoints(self):
        """Test de récupération des waypoints"""
        self.get_logger().info("📋 Test de récupération des waypoints")
        
        request = GetWaypoints.Request()
        future = self.get_waypoints_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.success:
            self.get_logger().info(f"✅ Waypoints récupérés: {len(response.waypoints)} waypoints")
            for i, wp in enumerate(response.waypoints):
                self.get_logger().info(f"  WP{i+1}: ({wp.position.x:.1f}, {wp.position.y:.1f}, {wp.position.z:.1f})")
        else:
            self.get_logger().error(f"❌ Échec récupération: {response.message}")
        
        return response.success
    
    def hold_position(self):
        """Maintenir la position actuelle"""
        request = Trigger.Request()
        future = self.hold_position_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.success:
            self.get_logger().info("✅ Position maintenue")
        else:
            self.get_logger().warn(f"⚠️ Échec maintien position: {response.message}")
    
    def test_return_home(self):
        """Test de retour à la maison"""
        self.get_logger().info("🏠 Test de retour à la maison")
        
        request = Trigger.Request()
        future = self.return_home_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.success:
            self.get_logger().info(f"✅ Retour maison: {response.message}")
        else:
            self.get_logger().error(f"❌ Échec retour: {response.message}")
        
        return response.success
    
    def run_all_tests(self):
        """Exécuter tous les tests"""
        self.get_logger().info("🚀 Démarrage des tests d'intégration")
        
        # Attendre les services
        self.wait_for_services()
        
        tests = [
            ("Safety Check", self.test_safety_check),
            ("Navigation Position", self.test_navigation_single_position),
            ("Hold Position", lambda: (self.hold_position(), True)[1]),
            ("Get Waypoints", self.test_get_waypoints),
            ("Multi Waypoints", self.test_navigation_multiple_waypoints),
            ("Get Waypoints", self.test_get_waypoints),
            ("Return Home", self.test_return_home),
        ]
        
        results = []
        for test_name, test_func in tests:
            self.get_logger().info(f"\n{'='*50}")
            self.get_logger().info(f"🧪 Test: {test_name}")
            self.get_logger().info(f"{'='*50}")
            
            try:
                result = test_func()
                results.append((test_name, result))
                time.sleep(2)  # Attente entre les tests
            except Exception as e:
                self.get_logger().error(f"❌ Erreur dans {test_name}: {str(e)}")
                results.append((test_name, False))
        
        # Résultats finaux
        self.get_logger().info(f"\n{'='*50}")
        self.get_logger().info("📊 RÉSULTATS FINAUX")
        self.get_logger().info(f"{'='*50}")
        
        passed = 0
        for test_name, result in results:
            status = "✅ PASS" if result else "❌ FAIL"
            self.get_logger().info(f"{status} - {test_name}")
            if result:
                passed += 1
        
        self.get_logger().info(f"\n🎯 Score: {passed}/{len(results)} tests réussis")
        
        if passed == len(results):
            self.get_logger().info("🎉 Tous les tests sont passés avec succès !")
        else:
            self.get_logger().warning(f"⚠️ {len(results) - passed} test(s) ont échoué")


def main():
    rclpy.init()
    test_node = IntegrationTest()
    
    try:
        test_node.run_all_tests()
    except KeyboardInterrupt:
        test_node.get_logger().info("🛑 Tests interrompus")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
