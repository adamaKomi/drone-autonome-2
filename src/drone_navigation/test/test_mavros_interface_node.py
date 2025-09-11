#!/usr/bin/env python3
"""
Script de test complet pour MAVROS Interface Node
Auteur: Adama Komi
Date: 2025-09-11

Ce script effectue des tests complets sur le nœud MAVROS Interface :
- Tests unitaires
- Tests d'intégration
- Tests de performance
- Tests de sécurité
"""

import unittest
import rclpy
import time
import threading
import json
from unittest.mock import Mock, patch, MagicMock

# Messages et services
from geometry_msgs.msg import PoseStamped, TwistStamped, Point, Vector3
from sensor_msgs.msg import NavSatFix, BatteryState
from std_msgs.msg import String
from mavros_msgs.msg import State, Altitude
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_srvs.srv import Trigger

# Nœud à tester
import sys
import os
# Ajouter le chemin vers le nœud
node_path = os.path.join(os.path.dirname(os.path.dirname(os.path.abspath(__file__))), 'drone_navigation', 'nodes')
sys.path.append(node_path)
from mavros_interface_node import MavrosInterfaceNode, DroneStatus, FlightMode


class TestMavrosInterfaceNode(unittest.TestCase):
    """Tests unitaires pour MavrosInterfaceNode"""
    
    @classmethod
    def setUpClass(cls):
        """Initialisation des tests"""
        if not rclpy.ok():
            rclpy.init()
    
    @classmethod
    def tearDownClass(cls):
        """Nettoyage après les tests"""
        if rclpy.ok():
            rclpy.shutdown()
    
    def setUp(self):
        """Préparation de chaque test"""
        self.node = MavrosInterfaceNode()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        
        # Thread pour exécuter le nœud
        self.executor_thread = threading.Thread(target=self._run_executor)
        self.executor_thread.daemon = True
        self.executor_thread.start()
        
        # Attendre l'initialisation
        time.sleep(0.5)
    
    def tearDown(self):
        """Nettoyage après chaque test"""
        try:
            self.node.destroy_node()
        except:
            pass
    
    def _run_executor(self):
        """Exécute l'executor dans un thread séparé"""
        try:
            self.executor.spin()
        except:
            pass
    
    def test_node_initialization(self):
        """Test de l'initialisation du nœud"""
        self.assertIsNotNone(self.node)
        self.assertEqual(self.node.get_name(), 'mavros_interface_node')
        self.assertEqual(self.node.get_namespace(), '/drone_nav')
        self.assertIsInstance(self.node.drone_status, DroneStatus)
    
    def test_drone_status_structure(self):
        """Test de la structure DroneStatus"""
        status = DroneStatus()
        
        # Vérifier les valeurs par défaut
        self.assertFalse(status.connected)
        self.assertFalse(status.armed)
        self.assertEqual(status.mode, "UNKNOWN")
        self.assertIsNotNone(status.position)
        self.assertIsNotNone(status.velocity)
        self.assertEqual(status.altitude, 0.0)
    
    def test_flight_modes_validation(self):
        """Test de la validation des modes de vol"""
        # Modes valides
        self.assertTrue(self.node._is_valid_flight_mode("GUIDED"))
        self.assertTrue(self.node._is_valid_flight_mode("STABILIZE"))
        self.assertTrue(self.node._is_valid_flight_mode("AUTO"))
        
        # Modes invalides
        self.assertFalse(self.node._is_valid_flight_mode("INVALID_MODE"))
        self.assertFalse(self.node._is_valid_flight_mode(""))
        self.assertFalse(self.node._is_valid_flight_mode(None))
    
    def test_quaternion_to_yaw_conversion(self):
        """Test de la conversion quaternion vers yaw"""
        from geometry_msgs.msg import Quaternion
        
        # Quaternion identité (0 radians)
        q = Quaternion()
        q.w = 1.0
        q.x = 0.0
        q.y = 0.0
        q.z = 0.0
        
        yaw = self.node._quaternion_to_yaw(q)
        self.assertAlmostEqual(yaw, 0.0, places=3)
        
        # Quaternion pour 90 degrés (π/2 radians)
        q.w = 0.707
        q.z = 0.707
        yaw = self.node._quaternion_to_yaw(q)
        self.assertAlmostEqual(yaw, 1.5708, places=3)  # π/2 ≈ 1.5708
    
    def test_battery_percentage_estimation(self):
        """Test de l'estimation du pourcentage de batterie"""
        # Tension maximale
        percentage = self.node._estimate_battery_percentage(16.8)
        self.assertAlmostEqual(percentage, 100.0, places=1)
        
        # Tension minimale
        percentage = self.node._estimate_battery_percentage(12.8)
        self.assertAlmostEqual(percentage, 0.0, places=1)
        
        # Tension nominale (environ 50%)
        percentage = self.node._estimate_battery_percentage(14.8)
        self.assertGreater(percentage, 40.0)
        self.assertLess(percentage, 60.0)
    
    def test_state_callback(self):
        """Test du callback d'état MAVROS"""
        # Créer un message d'état
        state_msg = State()
        state_msg.connected = True
        state_msg.armed = True
        state_msg.guided = True
        state_msg.mode = "GUIDED"
        state_msg.system_status = 4
        
        # Appeler le callback
        initial_time = time.time()
        self.node._state_callback(state_msg)
        
        # Vérifier que l'état a été mis à jour
        status = self.node.get_drone_status()
        self.assertTrue(status.connected)
        self.assertTrue(status.armed)
        self.assertTrue(status.guided)
        self.assertEqual(status.mode, "GUIDED")
        self.assertEqual(status.system_status, 4)
        self.assertGreaterEqual(status.last_heartbeat, initial_time)
    
    def test_position_callback(self):
        """Test du callback de position"""
        from geometry_msgs.msg import Quaternion
        
        # Créer un message de position
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = 10.0
        pose_msg.pose.position.y = 20.0
        pose_msg.pose.position.z = 30.0
        pose_msg.pose.orientation.w = 1.0
        
        # Appeler le callback
        initial_time = time.time()
        self.node._local_pose_callback(pose_msg)
        
        # Vérifier que la position a été mise à jour
        status = self.node.get_drone_status()
        self.assertAlmostEqual(status.position.x, 10.0)
        self.assertAlmostEqual(status.position.y, 20.0)
        self.assertAlmostEqual(status.position.z, 30.0)
        self.assertGreaterEqual(status.last_position_update, initial_time)
    
    def test_velocity_callback(self):
        """Test du callback de vitesse"""
        # Créer un message de vitesse
        vel_msg = TwistStamped()
        vel_msg.twist.linear.x = 5.0
        vel_msg.twist.linear.y = 3.0
        vel_msg.twist.linear.z = 1.0
        
        # Appeler le callback
        self.node._local_velocity_callback(vel_msg)
        
        # Vérifier que la vitesse a été mise à jour
        status = self.node.get_drone_status()
        self.assertAlmostEqual(status.velocity.x, 5.0)
        self.assertAlmostEqual(status.velocity.y, 3.0)
        self.assertAlmostEqual(status.velocity.z, 1.0)
    
    def test_battery_callback(self):
        """Test du callback de batterie"""
        # Créer un message de batterie
        battery_msg = BatteryState()
        battery_msg.voltage = 15.6
        battery_msg.percentage = 0.75  # 75%
        
        # Appeler le callback
        self.node._battery_callback(battery_msg)
        
        # Vérifier que la batterie a été mise à jour
        status = self.node.get_drone_status()
        self.assertAlmostEqual(status.battery_voltage, 15.6)
        self.assertAlmostEqual(status.battery_percentage, 75.0)
    
    def test_gps_callback(self):
        """Test du callback GPS"""
        # Créer un message GPS
        gps_msg = NavSatFix()
        gps_msg.status.status = 1  # Fix disponible
        gps_msg.status.service = 8  # 8 satellites
        
        # Appeler le callback
        self.node._global_position_callback(gps_msg)
        
        # Vérifier que le GPS a été mis à jour
        status = self.node.get_drone_status()
        self.assertTrue(status.gps_fix)
        self.assertEqual(status.satellites, 8)
    
    def test_safety_status_evaluation(self):
        """Test de l'évaluation du statut de sécurité"""
        # Configuration normale
        with self.node.status_lock:
            self.node.drone_status.connected = True
            self.node.drone_status.gps_fix = True
            self.node.drone_status.satellites = 8
            self.node.drone_status.battery_percentage = 50.0
            self.node.drone_status.altitude = 20.0
        
        safety = self.node._evaluate_safety_status()
        self.assertEqual(safety['level'], 'NORMAL')
        self.assertEqual(len(safety['errors']), 0)
        
        # Configuration avec avertissements
        with self.node.status_lock:
            self.node.drone_status.battery_percentage = 15.0  # Batterie faible
            self.node.drone_status.satellites = 4  # Peu de satellites
        
        safety = self.node._evaluate_safety_status()
        self.assertEqual(safety['level'], 'WARNING')
        self.assertGreater(len(safety['warnings']), 0)
        
        # Configuration critique
        with self.node.status_lock:
            self.node.drone_status.connected = False
            self.node.drone_status.battery_percentage = 5.0  # Batterie critique
        
        safety = self.node._evaluate_safety_status()
        self.assertEqual(safety['level'], 'CRITICAL')
        self.assertGreater(len(safety['errors']), 0)


class TestMavrosInterfaceIntegration(unittest.TestCase):
    """Tests d'intégration pour MavrosInterfaceNode"""
    
    @classmethod
    def setUpClass(cls):
        """Initialisation des tests d'intégration"""
        if not rclpy.ok():
            rclpy.init()
    
    def setUp(self):
        """Préparation de chaque test d'intégration"""
        self.node = MavrosInterfaceNode()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        
        self.executor_thread = threading.Thread(target=self._run_executor)
        self.executor_thread.daemon = True
        self.executor_thread.start()
        time.sleep(0.5)
    
    def tearDown(self):
        """Nettoyage après chaque test d'intégration"""
        try:
            self.node.destroy_node()
        except:
            pass
    
    def _run_executor(self):
        """Exécute l'executor dans un thread séparé"""
        try:
            self.executor.spin()
        except:
            pass
    
    def test_service_get_status(self):
        """Test du service get_drone_status"""
        # Créer un client pour le service
        client = self.node.create_client(Trigger, '/drone_nav/get_drone_status')
        
        # Attendre que le service soit disponible
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        
        # Appeler le service
        request = Trigger.Request()
        future = client.call_async(request)
        
        # Attendre la réponse
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        # Vérifier la réponse
        self.assertIsNotNone(future.result())
        response = future.result()
        self.assertTrue(response.success)
        
        # Vérifier que le message contient du JSON valide
        status_data = json.loads(response.message)
        self.assertIn('connected', status_data)
        self.assertIn('armed', status_data)
        self.assertIn('mode', status_data)
    
    def test_service_get_position(self):
        """Test du service get_current_position"""
        # Simuler une position
        with self.node.status_lock:
            self.node.drone_status.position.x = 100.0
            self.node.drone_status.position.y = 200.0
            self.node.drone_status.position.z = 50.0
            self.node.drone_status.heading = 1.57  # 90 degrés
            self.node.drone_status.altitude = 50.0
        
        # Créer un client pour le service
        client = self.node.create_client(Trigger, '/drone_nav/get_current_position')
        
        # Attendre que le service soit disponible
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        
        # Appeler le service
        request = Trigger.Request()
        future = client.call_async(request)
        
        # Attendre la réponse
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        # Vérifier la réponse
        self.assertIsNotNone(future.result())
        response = future.result()
        self.assertTrue(response.success)
        
        # Vérifier les données de position
        position_data = json.loads(response.message)
        self.assertAlmostEqual(position_data['position']['x'], 100.0)
        self.assertAlmostEqual(position_data['position']['y'], 200.0)
        self.assertAlmostEqual(position_data['position']['z'], 50.0)
    
    def test_service_diagnostics(self):
        """Test du service get_diagnostics"""
        # Créer un client pour le service
        client = self.node.create_client(Trigger, '/drone_nav/get_diagnostics')
        
        # Attendre que le service soit disponible
        self.assertTrue(client.wait_for_service(timeout_sec=5.0))
        
        # Appeler le service
        request = Trigger.Request()
        future = client.call_async(request)
        
        # Attendre la réponse
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        # Vérifier la réponse
        self.assertIsNotNone(future.result())
        response = future.result()
        self.assertTrue(response.success)
        
        # Vérifier les données de diagnostic
        diagnostics = json.loads(response.message)
        self.assertIn('node_name', diagnostics)
        self.assertIn('message_counters', diagnostics)
        self.assertIn('parameters', diagnostics)
        self.assertEqual(diagnostics['node_name'], 'mavros_interface_node')


class TestMavrosInterfacePerformance(unittest.TestCase):
    """Tests de performance pour MavrosInterfaceNode"""
    
    @classmethod
    def setUpClass(cls):
        """Initialisation des tests de performance"""
        if not rclpy.ok():
            rclpy.init()
    
    def setUp(self):
        """Préparation de chaque test de performance"""
        self.node = MavrosInterfaceNode()
        self.executor = rclpy.executors.SingleThreadedExecutor()
        self.executor.add_node(self.node)
        
        self.executor_thread = threading.Thread(target=self._run_executor)
        self.executor_thread.daemon = True
        self.executor_thread.start()
        time.sleep(0.5)
    
    def tearDown(self):
        """Nettoyage après chaque test de performance"""
        try:
            self.node.destroy_node()
        except:
            pass
    
    def _run_executor(self):
        """Exécute l'executor dans un thread séparé"""
        try:
            self.executor.spin()
        except:
            pass
    
    def test_callback_performance(self):
        """Test des performances des callbacks"""
        from geometry_msgs.msg import Quaternion
        
        # Préparer les messages
        state_msg = State()
        state_msg.connected = True
        state_msg.armed = True
        state_msg.mode = "GUIDED"
        
        pose_msg = PoseStamped()
        pose_msg.pose.position.x = 10.0
        pose_msg.pose.position.y = 20.0
        pose_msg.pose.position.z = 30.0
        pose_msg.pose.orientation.w = 1.0
        
        # Mesurer le temps d'exécution
        iterations = 1000
        
        # Test callback état
        start_time = time.time()
        for _ in range(iterations):
            self.node._state_callback(state_msg)
        state_time = time.time() - start_time
        
        # Test callback position
        start_time = time.time()
        for _ in range(iterations):
            self.node._local_pose_callback(pose_msg)
        position_time = time.time() - start_time
        
        # Vérifier les performances (moins de 1ms par callback en moyenne)
        avg_state_time = (state_time / iterations) * 1000  # en millisecondes
        avg_position_time = (position_time / iterations) * 1000
        
        self.assertLess(avg_state_time, 1.0, f"Callback état trop lent: {avg_state_time:.3f}ms")
        self.assertLess(avg_position_time, 1.0, f"Callback position trop lent: {avg_position_time:.3f}ms")
        
        print(f"Performance callbacks:")
        print(f"  - État: {avg_state_time:.3f}ms par callback")
        print(f"  - Position: {avg_position_time:.3f}ms par callback")
    
    def test_concurrent_access(self):
        """Test d'accès concurrent au statut"""
        def worker_read():
            """Worker pour lecture du statut"""
            for _ in range(100):
                status = self.node.get_drone_status()
                self.assertIsNotNone(status)
                time.sleep(0.001)
        
        def worker_write():
            """Worker pour écriture du statut"""
            for i in range(100):
                state_msg = State()
                state_msg.connected = True
                state_msg.mode = f"MODE_{i}"
                self.node._state_callback(state_msg)
                time.sleep(0.001)
        
        # Lancer plusieurs threads
        threads = []
        for _ in range(3):
            t1 = threading.Thread(target=worker_read)
            t2 = threading.Thread(target=worker_write)
            threads.extend([t1, t2])
        
        start_time = time.time()
        for t in threads:
            t.start()
        
        for t in threads:
            t.join()
        
        execution_time = time.time() - start_time
        self.assertLess(execution_time, 5.0, "Test de concurrence trop lent")
        
        print(f"Test concurrence terminé en {execution_time:.2f}s")


def run_tests():
    """Exécute tous les tests"""
    print("=== Tests MAVROS Interface Node ===\n")
    
    # Tests unitaires
    print("1. Tests unitaires...")
    suite1 = unittest.TestLoader().loadTestsFromTestCase(TestMavrosInterfaceNode)
    runner1 = unittest.TextTestRunner(verbosity=2)
    result1 = runner1.run(suite1)
    
    print("\n" + "="*50 + "\n")
    
    # Tests d'intégration
    print("2. Tests d'intégration...")
    suite2 = unittest.TestLoader().loadTestsFromTestCase(TestMavrosInterfaceIntegration)
    runner2 = unittest.TextTestRunner(verbosity=2)
    result2 = runner2.run(suite2)
    
    print("\n" + "="*50 + "\n")
    
    # Tests de performance
    print("3. Tests de performance...")
    suite3 = unittest.TestLoader().loadTestsFromTestCase(TestMavrosInterfacePerformance)
    runner3 = unittest.TextTestRunner(verbosity=2)
    result3 = runner3.run(suite3)
    
    # Résumé
    total_tests = result1.testsRun + result2.testsRun + result3.testsRun
    total_failures = len(result1.failures) + len(result2.failures) + len(result3.failures)
    total_errors = len(result1.errors) + len(result2.errors) + len(result3.errors)
    
    print("\n" + "="*50)
    print("RÉSUMÉ DES TESTS")
    print("="*50)
    print(f"Total tests: {total_tests}")
    print(f"Succès: {total_tests - total_failures - total_errors}")
    print(f"Échecs: {total_failures}")
    print(f"Erreurs: {total_errors}")
    
    if total_failures + total_errors == 0:
        print("\n✅ TOUS LES TESTS SONT RÉUSSIS!")
        return True
    else:
        print("\n❌ CERTAINS TESTS ONT ÉCHOUÉ!")
        return False


if __name__ == '__main__':
    success = run_tests()
    exit(0 if success else 1)
