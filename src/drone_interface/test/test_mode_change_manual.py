#!/usr/bin/env python3
"""
Script de test manuel pour le changement de mode du drone

Ce script permet de tester manuellement la fonctionnalité de changement de mode
en interagissant avec le service /drone/set_flight_mode.

Usage:
    python3 test_mode_change_manual.py
"""

import rclpy
from rclpy.node import Node
from drone_msgs.srv import SetFlightMode
import sys

class ModeChangeTestClient(Node):
    """Client de test pour le changement de mode"""
    
    def __init__(self):
        super().__init__('mode_change_test_client')
        self.client = self.create_client(SetFlightMode, '/drone/set_flight_mode')
        
        # Attendre que le service soit disponible
        self.get_logger().info("Attente du service /drone/set_flight_mode...")
        if not self.client.wait_for_service(timeout_sec=30.0):
            self.get_logger().error("Service non disponible après 30 secondes!")
            sys.exit(1)
        
        self.get_logger().info("Service disponible!")
    
    def test_mode_change(self, mode: str):
        """Teste le changement vers un mode spécifique"""
        self.get_logger().info(f"Test changement vers mode: {mode}")
        
        # Créer la requête
        request = SetFlightMode.Request()
        request.flight_mode = mode
        request.sub_mode = ''
        request.max_speed = 10.0
        request.max_altitude = 100.0
        request.max_distance = 1000.0
        request.enable_obstacle_avoidance = True
        request.enable_geofence = True
        request.enable_return_to_launch = True
        request.failsafe_altitude = 20.0
        request.operator_id = 'manual_test'
        request.override_restrictions = False
        request.custom_parameters = []
        
        # Appeler le service
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f"✅ SUCCÈS: {response.message}")
                return True
            else:
                self.get_logger().error(f"❌ ÉCHEC: {response.message}")
                return False
        else:
            self.get_logger().error("❌ ÉCHEC: Timeout lors de l'appel au service")
            return False

def main():
    """Fonction principale de test"""
    rclpy.init()
    
    try:
        test_client = ModeChangeTestClient()
        
        # Tests systématiques
        modes_to_test = [
            'GUIDED',
            'STABILIZE', 
            'ALTITUDE_HOLD',
            'POSITION_HOLD',
            'LOITER',
            'INVALID_MODE',  # Test d'un mode invalide
        ]
        
        print("\n" + "="*60)
        print("TESTS DE CHANGEMENT DE MODE")
        print("="*60)
        
        success_count = 0
        total_tests = len(modes_to_test)
        
        for i, mode in enumerate(modes_to_test, 1):
            print(f"\n[{i}/{total_tests}] Test du mode: {mode}")
            print("-" * 40)
            
            if test_client.test_mode_change(mode):
                if mode != 'INVALID_MODE':  # Mode invalide doit échouer
                    success_count += 1
            elif mode == 'INVALID_MODE':  # Mode invalide doit échouer
                success_count += 1
                test_client.get_logger().info("✅ Mode invalide correctement rejeté")
            
            # Pause entre les tests
            rclpy.spin_once(test_client, timeout_sec=1.0)
        
        print("\n" + "="*60)
        print(f"RÉSULTATS: {success_count}/{total_tests} tests réussis")
        print("="*60)
        
        # Test interactif
        print("\nMode test interactif (tapez 'quit' pour sortir)")
        while True:
            try:
                mode = input("\nEntrez un mode de vol (GUIDED, STABILIZE, etc.): ").strip().upper()
                if mode.lower() == 'quit':
                    break
                if mode:
                    test_client.test_mode_change(mode)
                    rclpy.spin_once(test_client, timeout_sec=1.0)
            except KeyboardInterrupt:
                break
        
    except Exception as e:
        print(f"Erreur: {e}")
    finally:
        test_client.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
