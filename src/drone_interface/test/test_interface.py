#!/usr/bin/env python3
"""
=============================================================================
TESTS POUR DRONE INTERFACE - Tests unitaires et d'intégration
=============================================================================
Auteur: Adama Komi
Date: 2025-09-03

Description:
    Tests complets pour le module drone_interface.
    Couvre les fonctionnalités principales et les cas d'erreur.
=============================================================================
"""

import unittest
import time
import threading
from unittest.mock import Mock, MagicMock, patch

# Imports ROS2 pour les tests
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from sensor_msgs.msg import BatteryState, NavSatFix
from std_srvs.srv import Trigger

# Imports des modules à tester
from drone_interface.safety_manager import SafetyManager, SafetyLevel
from drone_interface.state_manager import StateManager, DroneState, DroneStatusData
from drone_interface.health_monitor import HealthMonitor
from drone_interface.interface_node import DroneInterface


class TestSafetyManager(unittest.TestCase):
    """Tests pour le SafetyManager"""
    
    def setUp(self):
        """Configuration des tests"""
        self.logger = Mock()
        self.safety_manager = SafetyManager(self.logger)
        
    def test_init(self):
        """Test de l'initialisation"""
        self.assertIsNotNone(self.safety_manager)
        self.assertEqual(self.safety_manager.min_battery_voltage, 10.5)
        
    def test_check_pre_arm_conditions_success(self):
        """Test des conditions pré-armement - succès"""
        status = DroneStatusData()
        status.connected = True
        status.gps_fix = True
        status.gps_satellites = 8
        status.gps_hdop = 1.5
        status.battery_voltage = 11.5
        status.guided = True
        status.last_heartbeat = time.time()
        
        can_arm, errors, warnings = self.safety_manager.check_pre_arm_conditions(status)
        
        self.assertTrue(can_arm)
        self.assertEqual(len(errors), 0)
        
    def test_check_pre_arm_conditions_failure(self):
        """Test des conditions pré-armement - échec"""
        status = DroneStatusData()
        status.connected = False  # Pas connecté
        status.gps_fix = False   # Pas de GPS
        status.battery_voltage = 9.0  # Batterie faible
        
        can_arm, errors, warnings = self.safety_manager.check_pre_arm_conditions(status)
        
        self.assertFalse(can_arm)
        self.assertGreater(len(errors), 0)
        
    def test_check_flight_safety_normal(self):
        """Test de sécurité en vol - normal"""
        status = DroneStatusData()
        status.armed = True
        status.connected = True
        status.gps_fix = True
        status.battery_percentage = 50.0
        status.position = (0, 0, 30)  # 30m altitude
        status.last_heartbeat = time.time()
        
        safety_level, errors, warnings = self.safety_manager.check_flight_safety(status)
        
        self.assertEqual(safety_level, SafetyLevel.SAFE)
        self.assertEqual(len(errors), 0)
        
    def test_check_flight_safety_critical_battery(self):
        """Test de sécurité en vol - batterie critique"""
        status = DroneStatusData()
        status.armed = True
        status.battery_percentage = 5.0  # Batterie critique
        status.last_heartbeat = time.time()
        
        safety_level, errors, warnings = self.safety_manager.check_flight_safety(status)
        
        self.assertEqual(safety_level, SafetyLevel.CRITICAL)
        self.assertGreater(len(errors), 0)
        
    def test_update_battery_trend(self):
        """Test de mise à jour de tendance batterie"""
        # Ajout de plusieurs points
        for i in range(5):
            voltage = 11.0 - i * 0.1
            percentage = 100.0 - i * 5.0
            self.safety_manager.update_battery_trend(voltage, percentage)
            
        self.assertEqual(len(self.safety_manager._battery_history), 5)
        
        # Test de nettoyage (simule un historique ancien)
        old_time = time.time() - 400  # 400 secondes ago
        self.safety_manager._battery_history.insert(0, (old_time, 12.0, 100.0))
        
        # Nouvelle mise à jour doit nettoyer l'ancien
        self.safety_manager.update_battery_trend(10.5, 80.0)
        
        # Vérifier que l'ancien point a été supprimé
        for timestamp, _, _ in self.safety_manager._battery_history:
            self.assertGreater(timestamp, time.time() - 300)


class TestStateManager(unittest.TestCase):
    """Tests pour le StateManager"""
    
    def setUp(self):
        """Configuration des tests"""
        self.logger = Mock()
        self.safety_manager = Mock()
        self.state_manager = StateManager(self.logger, self.safety_manager)
        
    def test_init(self):
        """Test de l'initialisation"""
        self.assertIsNotNone(self.state_manager)
        self.assertEqual(self.state_manager.status.state, DroneState.UNKNOWN)
        
    def test_update_mavros_state(self):
        """Test de mise à jour état MAVROS"""
        msg = State()
        msg.connected = True
        msg.armed = False
        msg.guided = True
        msg.mode = "GUIDED"
        
        self.state_manager.update_mavros_state(msg)
        
        self.assertTrue(self.state_manager.status.connected)
        self.assertFalse(self.state_manager.status.armed)
        self.assertTrue(self.state_manager.status.guided)
        self.assertEqual(self.state_manager.status.mode, "GUIDED")
        self.assertEqual(self.state_manager.status.state, DroneState.CONNECTED)
        
    def test_update_position(self):
        """Test de mise à jour position"""
        msg = PoseStamped()
        msg.pose.position.x = 10.0
        msg.pose.position.y = 20.0
        msg.pose.position.z = 5.0
        msg.pose.orientation.w = 1.0  # quaternion valide
        
        self.state_manager.update_position(msg)
        
        self.assertEqual(self.state_manager.status.position, (10.0, 20.0, 5.0))
        
    def test_update_battery(self):
        """Test de mise à jour batterie"""
        msg = BatteryState()
        msg.voltage = 11.1
        msg.current = 5.5
        msg.percentage = 0.75  # 75%
        
        self.state_manager.update_battery(msg)
        
        self.assertEqual(self.state_manager.status.battery_voltage, 11.1)
        self.assertEqual(self.state_manager.status.battery_current, 5.5)
        self.assertEqual(self.state_manager.status.battery_percentage, 75.0)
        
    def test_set_home_position(self):
        """Test de définition position home"""
        self.state_manager.set_home_position(0.0, 0.0, 0.0)
        
        self.assertEqual(self.state_manager._home_position, (0.0, 0.0, 0.0))
        
    def test_get_status(self):
        """Test de récupération du statut"""
        # Configuration du safety manager mock
        self.safety_manager.check_flight_safety.return_value = (SafetyLevel.SAFE, [], [])
        
        status = self.state_manager.get_status()
        
        self.assertIsInstance(status, DroneStatusData)
        self.safety_manager.check_flight_safety.assert_called_once()
        
    def test_validate_pose_message_valid(self):
        """Test de validation message pose valide"""
        msg = PoseStamped()
        msg.pose.position.x = 1.0
        msg.pose.position.y = 2.0
        msg.pose.position.z = 3.0
        msg.pose.orientation.w = 1.0
        
        is_valid = self.state_manager._validate_pose_message(msg)
        self.assertTrue(is_valid)
        
    def test_validate_pose_message_invalid(self):
        """Test de validation message pose invalide"""
        msg = PoseStamped()
        msg.pose.position.x = float('nan')  # Valeur invalide
        msg.pose.position.y = 2.0
        msg.pose.position.z = 3.0
        msg.pose.orientation.w = 1.0
        
        is_valid = self.state_manager._validate_pose_message(msg)
        self.assertFalse(is_valid)


class TestHealthMonitor(unittest.TestCase):
    """Tests pour le HealthMonitor"""
    
    def setUp(self):
        """Configuration des tests"""
        self.logger = Mock()
        self.health_monitor = HealthMonitor(self.logger)
        
    def test_init(self):
        """Test de l'initialisation"""
        self.assertIsNotNone(self.health_monitor)
        self.assertFalse(self.health_monitor.health.mavros_connection)
        
    def test_check_mavros_services(self):
        """Test de vérification services MAVROS"""
        # Mock d'un nœud avec services
        node = Mock()
        service1 = Mock()
        service1.service_is_ready.return_value = True
        service2 = Mock()
        service2.service_is_ready.return_value = True
        
        node.arm_client = service1
        node.mode_client = service2
        node.takeoff_client = None  # Test avec service manquant
        
        result = self.health_monitor.check_mavros_services(node)
        
        # Devrait retourner True car les services valides sont prêts
        self.assertTrue(result)
        
    def test_update_health(self):
        """Test de mise à jour santé"""
        status = DroneStatusData()
        status.connected = True
        status.safety_messages = []
        
        self.health_monitor.update_health(status)
        
        self.assertTrue(self.health_monitor.health.mavros_connection)
        self.assertEqual(self.health_monitor.health.error_count, 0)
        
    def test_add_warning(self):
        """Test d'ajout d'avertissement"""
        warning_msg = "Test warning"
        
        self.health_monitor.add_warning(warning_msg)
        
        self.assertIn(warning_msg, self.health_monitor.health.warnings)
        
    def test_clear_warnings(self):
        """Test de nettoyage des avertissements"""
        self.health_monitor.add_warning("Warning 1")
        self.health_monitor.add_warning("Warning 2")
        
        self.assertEqual(len(self.health_monitor.health.warnings), 2)
        
        self.health_monitor.clear_warnings()
        
        self.assertEqual(len(self.health_monitor.health.warnings), 0)
        
    def test_is_healthy(self):
        """Test de vérification santé globale"""
        # Configuration d'un état sain
        self.health_monitor.health.mavros_connection = True
        self.health_monitor.health.services_ready = True
        self.health_monitor.health.error_count = 0
        self.health_monitor.health.warnings = []
        
        self.assertTrue(self.health_monitor.is_healthy())
        
        # Test avec erreurs
        self.health_monitor.health.error_count = 10
        
        self.assertFalse(self.health_monitor.is_healthy())


class TestDroneInterfaceIntegration(unittest.TestCase):
    """Tests d'intégration pour DroneInterface"""
    
    @classmethod
    def setUpClass(cls):
        """Configuration une fois pour tous les tests"""
        rclpy.init()
        
    @classmethod
    def tearDownClass(cls):
        """Nettoyage après tous les tests"""
        rclpy.shutdown()
        
    def setUp(self):
        """Configuration pour chaque test"""
        # Note: Ces tests nécessitent un environnement ROS2 complet
        # En pratique, on utiliserait des mocks pour la plupart
        pass
        
    def test_node_creation(self):
        """Test de création du nœud (test basique)"""
        # Ce test vérifie juste que le constructeur ne lève pas d'exception
        try:
            node = DroneInterface()
            node.destroy_node()
            success = True
        except Exception:
            success = False
            
        self.assertTrue(success)


class TestThreadSafety(unittest.TestCase):
    """Tests de sécurité des threads"""
    
    def setUp(self):
        """Configuration des tests"""
        self.logger = Mock()
        self.safety_manager = SafetyManager(self.logger)
        self.state_manager = StateManager(self.logger, self.safety_manager)
        
    def test_concurrent_battery_updates(self):
        """Test de mises à jour batterie concurrentes"""
        def update_battery():
            for i in range(10):
                self.safety_manager.update_battery_trend(11.0 - i * 0.01, 100.0 - i)
                time.sleep(0.001)
                
        # Lancement de plusieurs threads
        threads = []
        for _ in range(3):
            thread = threading.Thread(target=update_battery)
            threads.append(thread)
            thread.start()
            
        # Attente de la fin de tous les threads
        for thread in threads:
            thread.join()
            
        # Vérification que les données sont cohérentes
        self.assertGreater(len(self.safety_manager._battery_history), 0)
        
    def test_concurrent_status_access(self):
        """Test d'accès concurrent au statut"""
        # Mock du safety manager
        self.state_manager.safety_manager.check_flight_safety.return_value = (
            SafetyLevel.SAFE, [], []
        )
        
        def read_status():
            for _ in range(10):
                status = self.state_manager.get_status()
                self.assertIsInstance(status, DroneStatusData)
                time.sleep(0.001)
                
        def update_status():
            for i in range(10):
                msg = State()
                msg.connected = True
                msg.armed = i % 2 == 0
                msg.mode = f"MODE_{i}"
                self.state_manager.update_mavros_state(msg)
                time.sleep(0.001)
                
        # Lancement de threads lecteurs et écrivains
        threads = []
        
        # Threads lecteurs
        for _ in range(2):
            thread = threading.Thread(target=read_status)
            threads.append(thread)
            thread.start()
            
        # Thread écrivain
        thread = threading.Thread(target=update_status)
        threads.append(thread)
        thread.start()
        
        # Attente de la fin
        for thread in threads:
            thread.join()
            
        # Le test passe s'il n'y a pas de deadlock ou d'exception


def run_tests():
    """Lance tous les tests"""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Ajouter toutes les classes de test
    test_classes = [
        TestSafetyManager,
        TestStateManager,
        TestHealthMonitor,
        TestDroneInterfaceIntegration,
        TestThreadSafety
    ]
    
    for test_class in test_classes:
        tests = loader.loadTestsFromTestCase(test_class)
        suite.addTests(tests)
        
    # Lancement des tests
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_tests()
    exit(0 if success else 1)
