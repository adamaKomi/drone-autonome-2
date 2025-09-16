#!/usr/bin/env python3
"""
test_altitudes.py - Script pour tester la gestion des altitudes
"""

import rclpy
from rclpy.node import Node
from drone_msgs.srv import GotoPosition, GotoLocal

class AltitudeTestNode(Node):
    def __init__(self):
        super().__init__('altitude_test_node')
        
        # Attendre que les services soient disponibles
        self.get_logger().info("🧪 En attente des services de navigation...")
        
        self.gps_client = self.create_client(GotoPosition, '/drone_nav/goto_position')
        self.local_client = self.create_client(GotoLocal, '/drone_nav/goto_local')
        
        # Attendre les services
        self.gps_client.wait_for_service(timeout_sec=10.0)
        self.local_client.wait_for_service(timeout_sec=10.0)
        
        self.get_logger().info("✅ Services disponibles !")
    
    def test_gps_altitude_relative(self, alt_relative):
        """Test avec altitude relative (mode recommandé)"""
        self.get_logger().info(f"🧪 Test GPS - Altitude RELATIVE: {alt_relative}m")
        
        request = GotoPosition.Request()
        # Casablanca, légèrement déplacé
        request.latitude = 33.70673081
        request.longitude = -7.35041262
        request.altitude = alt_relative  # Sera interprétée comme relative
        request.tolerance = 2.0
        
        future = self.gps_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            result = future.result()
            self.get_logger().info(f"✅ GPS: {result.message}")
        else:
            self.get_logger().error("❌ Échec service GPS")
    
    def test_gps_altitude_absolute(self, alt_amsl):
        """Test avec altitude absolue (AMSL)"""
        self.get_logger().info(f"🧪 Test GPS - Altitude ABSOLUE: {alt_amsl}m AMSL")
        
        request = GotoPosition.Request()
        # Casablanca, légèrement déplacé
        request.latitude = 33.70673081
        request.longitude = -7.35041262
        request.altitude = alt_amsl  # Sera interprétée comme absolue
        request.tolerance = 2.0
        
        future = self.gps_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            result = future.result()
            self.get_logger().info(f"✅ GPS: {result.message}")
        else:
            self.get_logger().error("❌ Échec service GPS")
    
    def test_local_altitude(self, z_local):
        """Test avec altitude locale"""
        self.get_logger().info(f"🧪 Test LOCAL - Altitude: {z_local}m")
        
        request = GotoLocal.Request()
        request.x = 10.0  # 10m vers l'est
        request.y = 10.0  # 10m vers le nord
        request.z = z_local
        request.yaw_angle = 0.0
        
        future = self.local_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            result = future.result()
            self.get_logger().info(f"✅ LOCAL: {result.message}")
        else:
            self.get_logger().error("❌ Échec service LOCAL")

def main():
    rclpy.init()
    node = AltitudeTestNode()
    
    try:
        print("\n=== Tests d'Altitude - Drone Navigation ===")
        print("Ce script teste la gestion des altitudes pour éviter l'atterrissage")
        
        # Tests GPS avec différentes altitudes
        print("\n--- Tests GPS ---")
        node.test_gps_altitude_relative(50.0)   # 50m au-dessus du terrain (sûr)
        node.test_gps_altitude_relative(10.0)   # 10m au-dessus du terrain (sûr)
        node.test_gps_altitude_relative(1.0)    # 1m au-dessus du terrain (limite)
        
        node.test_gps_altitude_absolute(150.0)  # 150m AMSL (sûr pour Casablanca)
        node.test_gps_altitude_absolute(48.0)   # 48m AMSL (DANGEREUX - sous le terrain!)
        
        # Tests locaux
        print("\n--- Tests Locaux ---")
        node.test_local_altitude(5.0)   # 5m local (sûr)
        node.test_local_altitude(0.3)   # 0.3m local (trop bas - sera ajusté)
        node.test_local_altitude(150.0) # 150m local (trop haut - sera ajusté)
        
        print("\n✅ Tests terminés ! Vérifiez les logs pour voir les ajustements d'altitude.")
        
    except KeyboardInterrupt:
        node.get_logger().info("Test interrompu par l'utilisateur")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()