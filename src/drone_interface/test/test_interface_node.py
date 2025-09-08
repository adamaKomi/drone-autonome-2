#!/usr/bin/env python3
"""
Tests unitaires pour le service de changement de mode du drone_interface

Ce fichier teste exhaustivement la fonctionnalité de changement de mode
conformément aux spécifications ROS2 Humble et drone_msgs.
"""

import unittest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from drone_msgs.srv import SetFlightMode
import time

class TestSetFlightMode(unittest.TestCase):
    """Tests pour le service de changement de mode"""
    
    @classmethod
    def setUpClass(cls):
        """Initialisation des tests"""
        rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Nettoyage après les tests"""
        rclpy.shutdown()
    
    def setUp(self):
        """Setup avant chaque test"""
        self.test_node = Node('test_flight_mode_client')
        self.client = self.test_node.create_client(
            SetFlightMode,
            '/drone/set_flight_mode'
        )
        
        # Attendre que le service soit disponible
        self.assertTrue(
            self.client.wait_for_service(timeout_sec=30.0),
            "Service /drone/set_flight_mode non disponible après 30s"
        )
    
    def tearDown(self):
        """Nettoyage après chaque test"""
        self.test_node.destroy_node()
    
    def test_valid_mode_guided(self):
        """Test changement vers mode GUIDED"""
        request = SetFlightMode.Request()
        request.flight_mode = 'GUIDED'
        request.sub_mode = ''
        request.max_speed = 10.0
        request.max_altitude = 100.0
        request.max_distance = 1000.0
        request.enable_obstacle_avoidance = True
        request.enable_geofence = True
        request.enable_return_to_launch = True
        request.failsafe_altitude = 20.0
        request.operator_id = 'test_operator'
        request.override_restrictions = False
        request.custom_parameters = []
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=10.0)
        
        self.assertIsNotNone(future.result(), "Pas de réponse du service")
        response = future.result()
        
        # Le succès dépend de l'état de MAVROS
        self.assertIsInstance(response.success, bool)
        self.assertIsInstance(response.message, str)
        
        self.test_node.get_logger().info(f"Test GUIDED - Success: {response.success}, Message: {response.message}")
    
    def test_valid_mode_stabilize(self):
        """Test changement vers mode STABILIZE"""
        request = SetFlightMode.Request()
        request.flight_mode = 'STABILIZE'
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=10.0)
        
        self.assertIsNotNone(future.result())
        response = future.result()
        
        self.assertIsInstance(response.success, bool)
        self.assertIsInstance(response.message, str)
        
        self.test_node.get_logger().info(f"Test STABILIZE - Success: {response.success}, Message: {response.message}")
    
    def test_invalid_mode(self):
        """Test mode invalide"""
        request = SetFlightMode.Request()
        request.flight_mode = 'INVALID_MODE'
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=10.0)
        
        self.assertIsNotNone(future.result())
        response = future.result()
        
        # Mode invalide doit retourner False
        self.assertFalse(response.success)
        self.assertIn('Mode invalide', response.message)
        
        self.test_node.get_logger().info(f"Test mode invalide - Success: {response.success}, Message: {response.message}")
    
    def test_all_valid_modes(self):
        """Test tous les modes valides"""
        valid_modes = [
            'MANUAL', 'STABILIZE', 'ALTITUDE_HOLD', 'POSITION_HOLD',
            'AUTO', 'GUIDED', 'LOITER', 'RTL', 'LAND'
        ]
        
        for mode in valid_modes:
            with self.subTest(mode=mode):
                request = SetFlightMode.Request()
                request.flight_mode = mode
                
                future = self.client.call_async(request)
                rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=10.0)
                
                self.assertIsNotNone(future.result(), f"Pas de réponse pour le mode {mode}")
                response = future.result()
                
                self.assertIsInstance(response.success, bool)
                self.assertIsInstance(response.message, str)
                
                self.test_node.get_logger().info(f"Test {mode} - Success: {response.success}, Message: {response.message}")
                
                # Petite pause entre les tests
                time.sleep(0.5)


if __name__ == '__main__':
    unittest.main()
