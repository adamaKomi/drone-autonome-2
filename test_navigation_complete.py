#!/usr/bin/env python3
"""
Script de test pour la navigation avec armement et décollage
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from drone_navigation.srv import SetPosition
from geometry_msgs.msg import Point
import time

class NavigationTestWithArm(Node):
    def __init__(self):
        super().__init__('navigation_test_with_arm')
        
        # Clients pour MAVROS
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        
        # Clients pour navigation
        self.prepare_arm_client = self.create_client(Trigger, '/navigation/prepare_arm')
        self.set_position_client = self.create_client(SetPosition, '/navigation/set_position')
        self.hold_position_client = self.create_client(Trigger, '/navigation/hold_position')
        
        self.get_logger().info("🧪 Test de navigation avec armement initialisé")
    
    def wait_for_services(self):
        """Attendre que tous les services soient disponibles"""
        services = [
            ('/mavros/cmd/arming', self.arm_client),
            ('/mavros/set_mode', self.mode_client),
            ('/mavros/cmd/takeoff', self.takeoff_client),
            ('/navigation/prepare_arm', self.prepare_arm_client),
            ('/navigation/set_position', self.set_position_client),
            ('/navigation/hold_position', self.hold_position_client)
        ]
        
        for service_name, client in services:
            self.get_logger().info(f"⏳ Attente du service {service_name}")
            while not client.wait_for_service(timeout_sec=1.0):
                self.get_logger().info(f"⏳ Service {service_name} non disponible...")
        
        self.get_logger().info("✅ Tous les services sont disponibles")
    
    def set_mode(self, mode):
        """Changer le mode de vol"""
        request = SetMode.Request()
        request.custom_mode = mode
        
        future = self.mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.mode_sent:
            self.get_logger().info(f"✅ Mode changé vers {mode}")
            return True
        else:
            self.get_logger().error(f"❌ Échec changement mode vers {mode}")
            return False
    
    def prepare_arm(self):
        """Préparer l'armement"""
        self.get_logger().info("📡 Préparation pour l'armement...")
        
        request = Trigger.Request()
        future = self.prepare_arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.success:
            self.get_logger().info(f"✅ {response.message}")
            return True
        else:
            self.get_logger().error(f"❌ {response.message}")
            return False
    
    def arm_drone(self):
        """Armer le drone"""
        self.get_logger().info("🔓 Armement du drone...")
        
        request = CommandBool.Request()
        request.value = True
        
        future = self.arm_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.success:
            self.get_logger().info("✅ Drone armé")
            return True
        else:
            self.get_logger().error(f"❌ Échec armement: {response.result}")
            return False
    
    def takeoff(self, altitude=5.0):
        """Décollage"""
        self.get_logger().info(f"🚁 Décollage à {altitude}m...")
        
        request = CommandTOL.Request()
        request.altitude = altitude
        
        future = self.takeoff_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.success:
            self.get_logger().info("✅ Décollage initié")
            return True
        else:
            self.get_logger().error(f"❌ Échec décollage: {response.result}")
            return False
    
    def navigate_to_position(self, x, y, z, yaw=0.0):
        """Navigation vers une position"""
        self.get_logger().info(f"🧭 Navigation vers ({x}, {y}, {z})")
        
        request = SetPosition.Request()
        request.position = Point(x=x, y=y, z=z)
        request.yaw = yaw
        
        future = self.set_position_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        response = future.result()
        if response.success:
            self.get_logger().info(f"✅ {response.message}")
            return True
        else:
            self.get_logger().error(f"❌ {response.message}")
            return False
    
    def run_complete_test(self):
        """Exécuter le test complet"""
        self.get_logger().info("🚀 Démarrage du test complet de navigation")
        
        # Attendre les services
        self.wait_for_services()
        
        # Séquence de test
        try:
            # 1. Changer en mode GUIDED
            self.get_logger().info("\n📋 Étape 1: Mode GUIDED")
            if not self.set_mode("GUIDED"):
                return False
            time.sleep(2)
            
            # 2. Préparer l'armement
            self.get_logger().info("\n📋 Étape 2: Préparation armement")
            if not self.prepare_arm():
                return False
            time.sleep(2)
            
            # 3. Armer le drone
            self.get_logger().info("\n📋 Étape 3: Armement")
            if not self.arm_drone():
                return False
            time.sleep(2)
            
            # 4. Décollage
            self.get_logger().info("\n📋 Étape 4: Décollage")
            if not self.takeoff(5.0):
                return False
            time.sleep(5)  # Attendre le décollage
            
            # 5. Navigation vers différentes positions
            positions = [
                (5.0, 0.0, 10.0),
                (10.0, 5.0, 8.0),
                (0.0, 10.0, 6.0),
                (0.0, 0.0, 5.0)  # Retour proche de l'origine
            ]
            
            for i, (x, y, z) in enumerate(positions):
                self.get_logger().info(f"\n📋 Étape {5+i}: Navigation vers position {i+1}")
                if not self.navigate_to_position(x, y, z):
                    return False
                time.sleep(10)  # Attendre la navigation
            
            self.get_logger().info("\n🎉 Test complet réussi !")
            return True
            
        except Exception as e:
            self.get_logger().error(f"❌ Erreur pendant le test: {str(e)}")
            return False


def main():
    rclpy.init()
    test_node = NavigationTestWithArm()
    
    try:
        test_node.run_complete_test()
    except KeyboardInterrupt:
        test_node.get_logger().info("🛑 Test interrompu")
    finally:
        test_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
