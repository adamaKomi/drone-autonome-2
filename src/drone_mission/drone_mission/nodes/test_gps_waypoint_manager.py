#!/usr/bin/env python3
"""
test_gps_waypoint_manager.py - Script de test pour le gestionnaire de waypoints GPS
"""

import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Bool
from drone_msgs.msg import MissionStatus, WaypointList


class GPSWaypointTester(Node):
    def __init__(self):
        super().__init__('gps_waypoint_tester')
        
        # Clients de services
        self.start_mission_client = self.create_client(
            Bool, '/drone_mission/start_gps_mission'
        )
        self.pause_mission_client = self.create_client(
            Bool, '/drone_mission/pause_gps_mission'
        )
        self.resume_mission_client = self.create_client(
            Bool, '/drone_mission/resume_gps_mission'
        )
        self.cancel_mission_client = self.create_client(
            Bool, '/drone_mission/cancel_gps_mission'
        )
        
        # Souscriptions pour surveiller l'état
        self.create_subscription(
            MissionStatus, '/drone_mission/gps_mission_status',
            self.mission_status_callback, 10
        )
        self.create_subscription(
            WaypointList, '/drone_mission/gps_waypoint_list',
            self.waypoint_list_callback, 10
        )
        
        # Publisher pour simuler la sécurité
        self.safety_pub = self.create_publisher(
            Bool, '/drone_nav/safe_to_navigate', 10
        )
        
        self.mission_status = None
        self.waypoint_list = None
        
        self.get_logger().info("GPS Waypoint Tester initialized")

    def mission_status_callback(self, msg):
        self.mission_status = msg
        self.get_logger().info(
            f"Mission Status: {msg.status}, "
            f"Waypoint: {msg.current_waypoint}/{msg.total_waypoints}, "
            f"Progress: {msg.progress:.2f}"
        )

    def waypoint_list_callback(self, msg):
        self.waypoint_list = msg
        self.get_logger().info(f"Waypoint List: {msg.count} waypoints loaded")

    def set_safety(self, safe):
        """Active ou désactive la sécurité"""
        msg = Bool()
        msg.data = safe
        self.safety_pub.publish(msg)
        self.get_logger().info(f"Safety set to: {safe}")

    def start_mission(self):
        """Démarre la mission GPS"""
        if not self.start_mission_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service start_gps_mission non disponible")
            return False
        
        request = Bool.Request()
        request.data = True
        
        future = self.start_mission_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            result = future.result().data
            self.get_logger().info(f"Start mission result: {result}")
            return result
        return False

    def pause_mission(self):
        """Met en pause la mission GPS"""
        if not self.pause_mission_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service pause_gps_mission non disponible")
            return False
        
        request = Bool.Request()
        request.data = True
        
        future = self.pause_mission_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            result = future.result().data
            self.get_logger().info(f"Pause mission result: {result}")
            return result
        return False

    def resume_mission(self):
        """Reprend la mission GPS"""
        if not self.resume_mission_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service resume_gps_mission non disponible")
            return False
        
        request = Bool.Request()
        request.data = True
        
        future = self.resume_mission_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            result = future.result().data
            self.get_logger().info(f"Resume mission result: {result}")
            return result
        return False

    def cancel_mission(self):
        """Annule la mission GPS"""
        if not self.cancel_mission_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service cancel_gps_mission non disponible")
            return False
        
        request = Bool.Request()
        request.data = True
        
        future = self.cancel_mission_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            result = future.result().data
            self.get_logger().info(f"Cancel mission result: {result}")
            return result
        return False

    def run_test_sequence(self):
        """Exécute une séquence de test complète"""
        self.get_logger().info("=== DÉBUT DU TEST GPS WAYPOINT MANAGER ===")
        
        # 1. Activer la sécurité
        self.get_logger().info("1. Activation de la sécurité...")
        self.set_safety(True)
        time.sleep(2)
        
        # 2. Démarrer la mission
        self.get_logger().info("2. Démarrage de la mission...")
        if not self.start_mission():
            self.get_logger().error("Échec du démarrage de la mission")
            return
        
        # 3. Laisser la mission tourner
        self.get_logger().info("3. Mission en cours... (10 secondes)")
        time.sleep(10)
        
        # 4. Mettre en pause
        self.get_logger().info("4. Mise en pause de la mission...")
        self.pause_mission()
        time.sleep(3)
        
        # 5. Reprendre
        self.get_logger().info("5. Reprise de la mission...")
        self.resume_mission()
        time.sleep(5)
        
        # 6. Test de sécurité
        self.get_logger().info("6. Test de sécurité - désactivation...")
        self.set_safety(False)
        time.sleep(3)
        
        self.get_logger().info("7. Réactivation de la sécurité...")
        self.set_safety(True)
        time.sleep(3)
        
        # 7. Annuler la mission
        self.get_logger().info("8. Annulation de la mission...")
        self.cancel_mission()
        
        self.get_logger().info("=== FIN DU TEST ===")


def main():
    rclpy.init()
    tester = GPSWaypointTester()
    
    try:
        # Attendre un peu pour les connexions
        time.sleep(2)
        
        # Exécuter les tests
        tester.run_test_sequence()
        
        # Continuer à écouter les messages
        tester.get_logger().info("Surveillance continue... (Ctrl+C pour arrêter)")
        rclpy.spin(tester)
        
    except KeyboardInterrupt:
        tester.get_logger().info("Test interrompu par l'utilisateur")
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
