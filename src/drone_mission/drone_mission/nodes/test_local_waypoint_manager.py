#!/usr/bin/env python3
"""
test_local_waypoint_manager.py - Script de test pour le gestionnaire de waypoints locaux
"""

import rclpy
from rclpy.node import Node
import time
from std_msgs.msg import Bool
from drone_msgs.msg import MissionStatus, WaypointList


class LocalWaypointTester(Node):
    def __init__(self):
        super().__init__('local_waypoint_tester')
        
        # Clients de services
        self.start_mission_client = self.create_client(
            Bool, '/drone_mission/start_local_mission'
        )
        self.pause_mission_client = self.create_client(
            Bool, '/drone_mission/pause_local_mission'
        )
        self.resume_mission_client = self.create_client(
            Bool, '/drone_mission/resume_local_mission'
        )
        self.cancel_mission_client = self.create_client(
            Bool, '/drone_mission/cancel_local_mission'
        )
        self.skip_waypoint_client = self.create_client(
            Bool, '/drone_mission/skip_local_waypoint'
        )
        
        # Souscriptions pour surveiller l'état
        self.create_subscription(
            MissionStatus, '/drone_mission/local_mission_status',
            self.mission_status_callback, 10
        )
        self.create_subscription(
            WaypointList, '/drone_mission/local_waypoint_list',
            self.waypoint_list_callback, 10
        )
        
        # Publisher pour simuler la sécurité
        self.safety_pub = self.create_publisher(
            Bool, '/drone_nav/safe_to_navigate', 10
        )
        
        self.mission_status = None
        self.waypoint_list = None
        
        self.get_logger().info("Local Waypoint Tester initialized")

    def mission_status_callback(self, msg):
        self.mission_status = msg
        self.get_logger().info(
            f"Mission Status: {msg.status}, "
            f"Waypoint: {msg.current_waypoint}/{msg.total_waypoints}, "
            f"Progress: {msg.progress:.2f}"
        )

    def waypoint_list_callback(self, msg):
        self.waypoint_list = msg
        if msg.count > 0:
            self.get_logger().info(f"Waypoint List: {msg.count} waypoints loaded")
            for i, wp in enumerate(msg.waypoints):
                self.get_logger().info(
                    f"  WP{i}: ({wp.position.x:.1f}, {wp.position.y:.1f}, {wp.position.z:.1f}) "
                    f"type={wp.wp_type}, yaw={wp.yaw:.2f}"
                )

    def set_safety(self, safe):
        """Active ou désactive la sécurité"""
        msg = Bool()
        msg.data = safe
        self.safety_pub.publish(msg)
        self.get_logger().info(f"Safety set to: {safe}")

    def start_mission(self):
        """Démarre la mission locale"""
        if not self.start_mission_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service start_local_mission non disponible")
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
        """Met en pause la mission locale"""
        if not self.pause_mission_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service pause_local_mission non disponible")
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
        """Reprend la mission locale"""
        if not self.resume_mission_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service resume_local_mission non disponible")
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
        """Annule la mission locale"""
        if not self.cancel_mission_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service cancel_local_mission non disponible")
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

    def skip_waypoint(self):
        """Passe au waypoint suivant"""
        if not self.skip_waypoint_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service skip_local_waypoint non disponible")
            return False
        
        request = Bool.Request()
        request.data = True
        
        future = self.skip_waypoint_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result():
            result = future.result().data
            self.get_logger().info(f"Skip waypoint result: {result}")
            return result
        return False

    def run_test_sequence(self):
        """Exécute une séquence de test complète"""
        self.get_logger().info("=== DÉBUT DU TEST LOCAL WAYPOINT MANAGER ===")
        
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
        self.get_logger().info("3. Mission en cours... (8 secondes)")
        time.sleep(8)
        
        # 4. Test skip waypoint
        self.get_logger().info("4. Test skip waypoint...")
        self.skip_waypoint()
        time.sleep(3)
        
        # 5. Mettre en pause
        self.get_logger().info("5. Mise en pause de la mission...")
        self.pause_mission()
        time.sleep(3)
        
        # 6. Reprendre
        self.get_logger().info("6. Reprise de la mission...")
        self.resume_mission()
        time.sleep(5)
        
        # 7. Test de sécurité
        self.get_logger().info("7. Test de sécurité - désactivation...")
        self.set_safety(False)
        time.sleep(3)
        
        self.get_logger().info("8. Réactivation de la sécurité...")
        self.set_safety(True)
        time.sleep(3)
        
        # 8. Annuler la mission
        self.get_logger().info("9. Annulation de la mission...")
        self.cancel_mission()
        
        self.get_logger().info("=== FIN DU TEST ===")

    def run_interactive_test(self):
        """Test interactif avec commandes utilisateur"""
        self.get_logger().info("=== TEST INTERACTIF ===")
        self.get_logger().info("Commandes disponibles:")
        self.get_logger().info("  s - start mission")
        self.get_logger().info("  p - pause mission")
        self.get_logger().info("  r - resume mission")
        self.get_logger().info("  c - cancel mission")
        self.get_logger().info("  k - skip waypoint")
        self.get_logger().info("  t - toggle safety")
        self.get_logger().info("  q - quit")
        
        safety_state = True
        self.set_safety(safety_state)
        
        while True:
            try:
                cmd = input("\nCommande: ").strip().lower()
                
                if cmd == 's':
                    self.start_mission()
                elif cmd == 'p':
                    self.pause_mission()
                elif cmd == 'r':
                    self.resume_mission()
                elif cmd == 'c':
                    self.cancel_mission()
                elif cmd == 'k':
                    self.skip_waypoint()
                elif cmd == 't':
                    safety_state = not safety_state
                    self.set_safety(safety_state)
                elif cmd == 'q':
                    break
                else:
                    self.get_logger().info("Commande non reconnue")
                
                # Permettre le traitement des messages
                rclpy.spin_once(self, timeout_sec=0.1)
                
            except KeyboardInterrupt:
                break


def main():
    rclpy.init()
    tester = LocalWaypointTester()
    
    try:
        # Attendre un peu pour les connexions
        time.sleep(2)
        
        # Choix du mode de test
        import sys
        if len(sys.argv) > 1 and sys.argv[1] == "--interactive":
            tester.run_interactive_test()
        else:
            # Exécuter les tests automatiques
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
