#!/usr/bin/env python3
"""
=============================================================================
TESTS UNITAIRES POUR DRONE INTERFACE
=============================================================================
Auteur: Assistant IA
Date: 2025-08-29
Version: 1.0.0

Description:
    Tests unitaires pour valider le fonctionnement de chaque module
    du système de contrôle de drone ArduPilot.
    
Utilisation:
    python3 test_drone_interface.py
    
    ou avec ROS2:
    ros2 run drone_interface test_drone_interface
=============================================================================
"""

import unittest
import time
import math
from unittest.mock import Mock, MagicMock, patch

# Import des modules à tester
import sys
import os
sys.path.append(os.path.dirname(__file__))

from interface_node import (
    DroneState, FlightMode, DroneStatus, SafetyManager,
    StateManager, ArmingManager, ModeManager, PositionController,
    TakeoffLandingManager, DroneInterface
)

# Messages ROS2 mockés
class MockState:
    def __init__(self, connected=True, armed=False, guided=False, mode="STABILIZE"):
        self.connected = connected
        self.armed = armed
        self.guided = guided
        self.mode = mode

class MockPoseStamped:
    def __init__(self, x=0.0, y=0.0, z=0.0):
        self.pose = Mock()
        self.pose.position = Mock()
        self.pose.position.x = x
        self.pose.position.y = y
        self.pose.position.z = z

class MockTwistStamped:
    def __init__(self, vx=0.0, vy=0.0, vz=0.0):
        self.twist = Mock()
        self.twist.linear = Mock()
        self.twist.linear.x = vx
        self.twist.linear.y = vy
        self.twist.linear.z = vz

class MockBatteryState:
    def __init__(self, voltage=12.0, percentage=0.8):
        self.voltage = voltage
        self.percentage = percentage

class MockNavSatFix:
    def __init__(self, status=0):
        self.status = Mock()
        self.status.status = status


class TestDroneState(unittest.TestCase):
    """Tests pour l'énumération DroneState"""
    
    def test_drone_state_values(self):
        """Test des valeurs de l'énumération DroneState"""
        self.assertEqual(DroneState.UNKNOWN.value, "UNKNOWN")
        self.assertEqual(DroneState.CONNECTED.value, "CONNECTED")
        self.assertEqual(DroneState.ARMED.value, "ARMED")
        self.assertEqual(DroneState.FLYING.value, "FLYING")
        self.assertEqual(DroneState.LANDING.value, "LANDING")
        self.assertEqual(DroneState.EMERGENCY.value, "EMERGENCY")


class TestFlightMode(unittest.TestCase):
    """Tests pour l'énumération FlightMode"""
    
    def test_flight_mode_values(self):
        """Test des valeurs de l'énumération FlightMode"""
        self.assertEqual(FlightMode.STABILIZE.value, "STABILIZE")
        self.assertEqual(FlightMode.GUIDED.value, "GUIDED")
        self.assertEqual(FlightMode.AUTO.value, "AUTO")
        self.assertEqual(FlightMode.RTL.value, "RTL")
        self.assertEqual(FlightMode.LAND.value, "LAND")


class TestDroneStatus(unittest.TestCase):
    """Tests pour la structure DroneStatus"""
    
    def test_default_values(self):
        """Test des valeurs par défaut de DroneStatus"""
        status = DroneStatus()
        self.assertEqual(status.state, DroneState.UNKNOWN)
        self.assertEqual(status.mode, "UNKNOWN")
        self.assertFalse(status.armed)
        self.assertFalse(status.connected)
        self.assertFalse(status.guided)
        self.assertEqual(status.position, (0.0, 0.0, 0.0))
        self.assertEqual(status.velocity, (0.0, 0.0, 0.0))
        self.assertEqual(status.battery_voltage, 0.0)
        self.assertEqual(status.battery_percentage, 0.0)
        self.assertFalse(status.gps_fix)
        self.assertEqual(status.gps_satellites, 0)
        self.assertEqual(status.last_update, 0.0)


class TestSafetyManager(unittest.TestCase):
    """Tests pour le SafetyManager"""
    
    def setUp(self):
        """Configuration pour chaque test"""
        self.logger = Mock()
        self.safety_manager = SafetyManager(self.logger)
    
    def test_check_arm_conditions_not_connected(self):
        """Test: vérification armement - drone non connecté"""
        status = DroneStatus()
        status.connected = False
        
        can_arm, reason = self.safety_manager.check_arm_conditions(status)
        
        self.assertFalse(can_arm)
        self.assertIn("not connected", reason)
    
    def test_check_arm_conditions_no_gps(self):
        """Test: vérification armement - pas de GPS"""
        status = DroneStatus()
        status.connected = True
        status.gps_fix = False
        
        can_arm, reason = self.safety_manager.check_arm_conditions(status)
        
        self.assertFalse(can_arm)
        self.assertIn("GPS fix not available", reason)
    
    def test_check_arm_conditions_insufficient_satellites(self):
        """Test: vérification armement - satellites insuffisants"""
        status = DroneStatus()
        status.connected = True
        status.gps_fix = True
        status.gps_satellites = 3  # Moins que le minimum requis (6)
        
        can_arm, reason = self.safety_manager.check_arm_conditions(status)
        
        self.assertFalse(can_arm)
        self.assertIn("Insufficient GPS satellites", reason)
    
    def test_check_arm_conditions_low_battery(self):
        """Test: vérification armement - batterie faible"""
        status = DroneStatus()
        status.connected = True
        status.gps_fix = True
        status.gps_satellites = 8
        status.battery_voltage = 9.0  # Moins que le minimum requis (10.5V)
        
        can_arm, reason = self.safety_manager.check_arm_conditions(status)
        
        self.assertFalse(can_arm)
        self.assertIn("Battery voltage too low", reason)
    
    def test_check_arm_conditions_success(self):
        """Test: vérification armement - toutes conditions OK"""
        status = DroneStatus()
        status.connected = True
        status.gps_fix = True
        status.gps_satellites = 8
        status.battery_voltage = 12.0
        
        can_arm, reason = self.safety_manager.check_arm_conditions(status)
        
        self.assertTrue(can_arm)
        self.assertIn("All safety checks passed", reason)
    
    def test_check_flight_safety_altitude_too_high(self):
        """Test: vérification sécurité vol - altitude trop élevée"""
        status = DroneStatus()
        status.position = (0.0, 0.0, 150.0)  # Plus que le maximum (120m)
        
        is_safe, warning = self.safety_manager.check_flight_safety(status)
        
        self.assertFalse(is_safe)
        self.assertIn("Altitude too high", warning)
    
    def test_check_flight_safety_outside_geofence(self):
        """Test: vérification sécurité vol - hors géofence"""
        status = DroneStatus()
        status.position = (150.0, 0.0, 10.0)  # Plus loin que le rayon (100m)
        
        is_safe, warning = self.safety_manager.check_flight_safety(status)
        
        self.assertFalse(is_safe)
        self.assertIn("Outside geofence", warning)
    
    def test_check_flight_safety_critical_battery(self):
        """Test: vérification sécurité vol - batterie critique"""
        status = DroneStatus()
        status.position = (10.0, 10.0, 50.0)
        status.battery_percentage = 15.0  # Moins que 20%
        
        is_safe, warning = self.safety_manager.check_flight_safety(status)
        
        self.assertFalse(is_safe)
        self.assertIn("Critical battery level", warning)
    
    def test_check_flight_safety_success(self):
        """Test: vérification sécurité vol - tout OK"""
        status = DroneStatus()
        status.position = (10.0, 10.0, 50.0)
        status.battery_percentage = 80.0
        
        is_safe, warning = self.safety_manager.check_flight_safety(status)
        
        self.assertTrue(is_safe)
        self.assertEqual(warning, "Flight safety OK")


class TestStateManager(unittest.TestCase):
    """Tests pour le StateManager"""
    
    def setUp(self):
        """Configuration pour chaque test"""
        self.logger = Mock()
        self.state_manager = StateManager(self.logger)
    
    def test_update_mavros_state_disconnected(self):
        """Test: mise à jour état - déconnecté"""
        mock_state = MockState(connected=False)
        
        self.state_manager.update_mavros_state(mock_state)
        
        status = self.state_manager.get_status()
        self.assertFalse(status.connected)
        self.assertEqual(status.state, DroneState.DISCONNECTED)
    
    def test_update_mavros_state_connected_disarmed(self):
        """Test: mise à jour état - connecté mais désarmé"""
        mock_state = MockState(connected=True, armed=False)
        
        self.state_manager.update_mavros_state(mock_state)
        
        status = self.state_manager.get_status()
        self.assertTrue(status.connected)
        self.assertFalse(status.armed)
        self.assertEqual(status.state, DroneState.CONNECTED)
    
    def test_update_mavros_state_armed(self):
        """Test: mise à jour état - armé"""
        mock_state = MockState(connected=True, armed=True, mode="GUIDED")
        
        self.state_manager.update_mavros_state(mock_state)
        
        status = self.state_manager.get_status()
        self.assertTrue(status.connected)
        self.assertTrue(status.armed)
        self.assertEqual(status.mode, "GUIDED")
    
    def test_update_mavros_state_landing(self):
        """Test: mise à jour état - atterrissage"""
        mock_state = MockState(connected=True, armed=True, mode="LAND")
        
        self.state_manager.update_mavros_state(mock_state)
        
        status = self.state_manager.get_status()
        self.assertEqual(status.state, DroneState.LANDING)
    
    def test_update_position(self):
        """Test: mise à jour position"""
        mock_pose = MockPoseStamped(x=1.0, y=2.0, z=3.0)
        
        self.state_manager.update_position(mock_pose)
        
        status = self.state_manager.get_status()
        self.assertEqual(status.position, (1.0, 2.0, 3.0))
    
    def test_update_velocity(self):
        """Test: mise à jour vélocité"""
        mock_twist = MockTwistStamped(vx=0.5, vy=1.0, vz=0.2)
        
        self.state_manager.update_velocity(mock_twist)
        
        status = self.state_manager.get_status()
        self.assertEqual(status.velocity, (0.5, 1.0, 0.2))
    
    def test_update_battery(self):
        """Test: mise à jour batterie"""
        mock_battery = MockBatteryState(voltage=11.5, percentage=0.75)
        
        self.state_manager.update_battery(mock_battery)
        
        status = self.state_manager.get_status()
        self.assertEqual(status.battery_voltage, 11.5)
        self.assertEqual(status.battery_percentage, 75.0)
    
    def test_update_gps(self):
        """Test: mise à jour GPS"""
        mock_gps = MockNavSatFix(status=1)  # Fix disponible
        
        self.state_manager.update_gps(mock_gps)
        
        status = self.state_manager.get_status()
        self.assertTrue(status.gps_fix)
    
    def test_state_callback(self):
        """Test: callback de changement d'état"""
        callback_called = False
        callback_status = None
        
        def test_callback(status):
            nonlocal callback_called, callback_status
            callback_called = True
            callback_status = status
        
        self.state_manager.add_state_callback(test_callback)
        
        mock_state = MockState(connected=True, armed=True)
        self.state_manager.update_mavros_state(mock_state)
        
        self.assertTrue(callback_called)
        self.assertIsNotNone(callback_status)
        self.assertTrue(callback_status.armed)


class TestPositionController(unittest.TestCase):
    """Tests pour le PositionController"""
    
    def setUp(self):
        """Configuration pour chaque test"""
        self.mock_node = Mock()
        self.mock_state_manager = Mock()
        self.logger = Mock()
        
        # Mock du publisher
        self.mock_publisher = Mock()
        self.mock_node.create_publisher.return_value = self.mock_publisher
        
        # Mock du timer
        self.mock_timer = Mock()
        self.mock_node.create_timer.return_value = self.mock_timer
        
        # Mock du clock
        self.mock_clock = Mock()
        self.mock_node.get_clock.return_value = self.mock_clock
        self.mock_clock.now.return_value.to_msg.return_value = Mock()
        
        self.position_controller = PositionController(self.mock_node, self.mock_state_manager)
    
    def test_set_position(self):
        """Test: définition d'une position cible"""
        result = self.position_controller.set_position(1.0, 2.0, 3.0, math.pi/4)
        
        self.assertTrue(result)
        self.assertIsNotNone(self.position_controller._current_setpoint)
        
        setpoint = self.position_controller._current_setpoint
        self.assertEqual(setpoint.pose.position.x, 1.0)
        self.assertEqual(setpoint.pose.position.y, 2.0)
        self.assertEqual(setpoint.pose.position.z, 3.0)
    
    def test_hold_position(self):
        """Test: maintien de position"""
        # Mock du status actuel
        mock_status = DroneStatus()
        mock_status.position = (5.0, 6.0, 7.0)
        self.mock_state_manager.get_status.return_value = mock_status
        
        result = self.position_controller.hold_position()
        
        self.assertTrue(result)
        
        setpoint = self.position_controller._current_setpoint
        self.assertEqual(setpoint.pose.position.x, 5.0)
        self.assertEqual(setpoint.pose.position.y, 6.0)
        self.assertEqual(setpoint.pose.position.z, 7.0)
    
    def test_get_position_error(self):
        """Test: calcul d'erreur de position"""
        # Définir un setpoint
        self.position_controller.set_position(10.0, 20.0, 30.0)
        
        # Mock de la position actuelle
        mock_status = DroneStatus()
        mock_status.position = (8.0, 18.0, 28.0)
        self.mock_state_manager.get_status.return_value = mock_status
        
        error_x, error_y, error_z = self.position_controller.get_position_error()
        
        self.assertEqual(error_x, 2.0)  # 10.0 - 8.0
        self.assertEqual(error_y, 2.0)  # 20.0 - 18.0
        self.assertEqual(error_z, 2.0)  # 30.0 - 28.0
    
    def test_stop_setpoint_publishing(self):
        """Test: arrêt de publication de setpoints"""
        # Démarrer la publication
        self.position_controller.set_position(1.0, 2.0, 3.0)
        
        # Arrêter la publication
        self.position_controller.stop_setpoint_publishing()
        
        self.assertFalse(self.position_controller._setpoint_active)
        self.mock_timer.cancel.assert_called_once()


class TestIntegration(unittest.TestCase):
    """Tests d'intégration pour vérifier l'interaction entre modules"""
    
    def setUp(self):
        """Configuration pour les tests d'intégration"""
        self.logger = Mock()
        
        # Création des modules avec des mocks
        self.state_manager = StateManager(self.logger)
        self.safety_manager = SafetyManager(self.logger)
    
    def test_safety_state_integration(self):
        """Test: intégration SafetyManager et StateManager"""
        # Configurer un état sûr
        status = DroneStatus()
        status.connected = True
        status.gps_fix = True
        status.gps_satellites = 8
        status.battery_voltage = 12.0
        status.position = (10.0, 10.0, 50.0)
        status.battery_percentage = 80.0
        
        # Vérifier les conditions d'armement
        can_arm, arm_reason = self.safety_manager.check_arm_conditions(status)
        self.assertTrue(can_arm)
        
        # Vérifier la sécurité en vol
        is_safe, flight_reason = self.safety_manager.check_flight_safety(status)
        self.assertTrue(is_safe)
    
    def test_state_change_workflow(self):
        """Test: workflow de changement d'état"""
        # État initial: déconnecté
        mock_state = MockState(connected=False)
        self.state_manager.update_mavros_state(mock_state)
        
        status = self.state_manager.get_status()
        self.assertEqual(status.state, DroneState.DISCONNECTED)
        
        # Connexion
        mock_state = MockState(connected=True, armed=False)
        self.state_manager.update_mavros_state(mock_state)
        
        status = self.state_manager.get_status()
        self.assertEqual(status.state, DroneState.CONNECTED)
        
        # Armement
        mock_state = MockState(connected=True, armed=True, mode="GUIDED")
        self.state_manager.update_mavros_state(mock_state)
        
        status = self.state_manager.get_status()
        self.assertTrue(status.armed)
        self.assertEqual(status.mode, "GUIDED")


class TestPerformance(unittest.TestCase):
    """Tests de performance pour vérifier l'efficacité du système"""
    
    def test_state_update_performance(self):
        """Test: performance des mises à jour d'état"""
        logger = Mock()
        state_manager = StateManager(logger)
        
        # Mesurer le temps pour 1000 mises à jour
        start_time = time.time()
        
        for i in range(1000):
            mock_state = MockState(connected=True, armed=(i % 2 == 0))
            state_manager.update_mavros_state(mock_state)
        
        end_time = time.time()
        duration = end_time - start_time
        
        # La mise à jour de 1000 états ne devrait pas prendre plus de 1 seconde
        self.assertLess(duration, 1.0, f"Mises à jour d'état trop lentes: {duration:.3f}s")
        
        # Vérifier que les données sont cohérentes
        status = state_manager.get_status()
        self.assertTrue(status.connected)
    
    def test_safety_check_performance(self):
        """Test: performance des vérifications de sécurité"""
        logger = Mock()
        safety_manager = SafetyManager(logger)
        
        status = DroneStatus()
        status.connected = True
        status.gps_fix = True
        status.gps_satellites = 8
        status.battery_voltage = 12.0
        status.position = (10.0, 10.0, 50.0)
        status.battery_percentage = 80.0
        
        # Mesurer le temps pour 1000 vérifications
        start_time = time.time()
        
        for i in range(1000):
            safety_manager.check_arm_conditions(status)
            safety_manager.check_flight_safety(status)
        
        end_time = time.time()
        duration = end_time - start_time
        
        # 1000 vérifications de sécurité ne devraient pas prendre plus de 0.5 seconde
        self.assertLess(duration, 0.5, f"Vérifications de sécurité trop lentes: {duration:.3f}s")


def run_all_tests():
    """
    Lance tous les tests avec un rapport détaillé
    """
    print("=" * 60)
    print("🧪 LANCEMENT DES TESTS UNITAIRES DRONE INTERFACE")
    print("=" * 60)
    
    # Créer la suite de tests
    test_suite = unittest.TestSuite()
    
    # Ajouter toutes les classes de tests
    test_classes = [
        TestDroneState, TestFlightMode, TestDroneStatus,
        TestSafetyManager, TestStateManager, TestPositionController,
        TestIntegration, TestPerformance
    ]
    
    for test_class in test_classes:
        tests = unittest.TestLoader().loadTestsFromTestCase(test_class)
        test_suite.addTests(tests)
    
    # Lancer les tests avec un runner détaillé
    runner = unittest.TextTestRunner(
        verbosity=2,
        descriptions=True,
        failfast=False
    )
    
    print(f"\n🚀 Lancement de {test_suite.countTestCases()} tests...\n")
    
    result = runner.run(test_suite)
    
    # Rapport final
    print("\n" + "=" * 60)
    print("📊 RAPPORT FINAL DES TESTS")
    print("=" * 60)
    print(f"✅ Tests réussis: {result.testsRun - len(result.failures) - len(result.errors)}")
    print(f"❌ Tests échoués: {len(result.failures)}")
    print(f"💥 Erreurs: {len(result.errors)}")
    print(f"⏭️ Tests ignorés: {len(result.skipped) if hasattr(result, 'skipped') else 0}")
    print(f"📈 Taux de réussite: {((result.testsRun - len(result.failures) - len(result.errors)) / result.testsRun * 100):.1f}%")
    
    if result.failures:
        print("\n❌ ÉCHECS:")
        for test, traceback in result.failures:
            print(f"  - {test}: {traceback.split('AssertionError: ')[-1].split('\\n')[0]}")
    
    if result.errors:
        print("\n💥 ERREURS:")
        for test, traceback in result.errors:
            print(f"  - {test}: {traceback.split('\\n')[-2]}")
    
    print("=" * 60)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_all_tests()
    exit(0 if success else 1)
