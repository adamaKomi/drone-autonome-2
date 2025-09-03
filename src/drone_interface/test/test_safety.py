#!/usr/bin/env python3
"""
=============================================================================
TESTS DE SÉCURITÉ - Tests spécifiques pour le système de sécurité
=============================================================================
Auteur: Adama Komi
Date: 2025-09-03

Description:
    Tests spécialisés pour valider le comportement du système de sécurité
    dans différents scenarios critiques.
=============================================================================
"""

import unittest
import time
from unittest.mock import Mock

from drone_interface.safety_manager import SafetyManager, SafetyLevel
from drone_interface.state_manager import DroneStatusData, DroneState


class TestSafetyScenarios(unittest.TestCase):
    """Tests de scénarios de sécurité spécifiques"""
    
    def setUp(self):
        """Configuration des tests"""
        self.logger = Mock()
        self.safety_manager = SafetyManager(self.logger)
        
    def create_nominal_status(self):
        """Crée un statut nominal pour les tests"""
        status = DroneStatusData()
        status.connected = True
        status.armed = True
        status.guided = True
        status.gps_fix = True
        status.gps_satellites = 8
        status.gps_hdop = 1.5
        status.battery_voltage = 11.1
        status.battery_percentage = 50.0
        status.position = (0.0, 0.0, 10.0)  # 10m altitude
        status.last_heartbeat = time.time()
        return status
        
    def test_scenario_takeoff_safety(self):
        """Scénario: Vérifications avant décollage"""
        status = self.create_nominal_status()
        status.armed = False  # Pas encore armé
        
        # Test avec conditions nominales
        can_arm, errors, warnings = self.safety_manager.check_pre_arm_conditions(status)
        self.assertTrue(can_arm, "Le drone devrait pouvoir être armé en conditions nominales")
        
        # Test avec GPS insuffisant
        status.gps_satellites = 4
        can_arm, errors, warnings = self.safety_manager.check_pre_arm_conditions(status)
        self.assertFalse(can_arm, "Le drone ne devrait pas pouvoir être armé avec peu de satellites")
        self.assertIn("satellites", " ".join(errors).lower())
        
    def test_scenario_low_battery_progression(self):
        """Scénario: Progression de décharge batterie"""
        status = self.create_nominal_status()
        
        # Test batterie normale (50%)
        status.battery_percentage = 50.0
        safety_level, errors, warnings = self.safety_manager.check_flight_safety(status)
        self.assertEqual(safety_level, SafetyLevel.SAFE)
        
        # Test batterie en avertissement (25%)
        status.battery_percentage = 25.0
        safety_level, errors, warnings = self.safety_manager.check_flight_safety(status)
        self.assertEqual(safety_level, SafetyLevel.WARNING)
        self.assertGreater(len(warnings), 0)
        
        # Test batterie critique (8%)
        status.battery_percentage = 8.0
        safety_level, errors, warnings = self.safety_manager.check_flight_safety(status)
        self.assertEqual(safety_level, SafetyLevel.CRITICAL)
        self.assertGreater(len(errors), 0)
        
    def test_scenario_gps_loss_in_flight(self):
        """Scénario: Perte GPS en vol"""
        status = self.create_nominal_status()
        
        # Vol normal avec GPS
        safety_level, errors, warnings = self.safety_manager.check_flight_safety(status)
        self.assertEqual(safety_level, SafetyLevel.SAFE)
        
        # Perte GPS en vol
        status.gps_fix = False
        safety_level, errors, warnings = self.safety_manager.check_flight_safety(status)
        self.assertEqual(safety_level, SafetyLevel.EMERGENCY)
        self.assertIn("gps", " ".join(errors).lower())
        
    def test_scenario_altitude_violation(self):
        """Scénario: Violation d'altitude"""
        status = self.create_nominal_status()
        
        # Altitude normale
        status.position = (0.0, 0.0, 50.0)
        safety_level, errors, warnings = self.safety_manager.check_flight_safety(status)
        self.assertEqual(safety_level, SafetyLevel.SAFE)
        
        # Altitude excessive
        status.position = (0.0, 0.0, 150.0)  # Au-dessus de la limite
        safety_level, errors, warnings = self.safety_manager.check_flight_safety(status)
        self.assertEqual(safety_level, SafetyLevel.CRITICAL)
        self.assertIn("altitude", " ".join(errors).lower())
        
    def test_scenario_communication_loss(self):
        """Scénario: Perte de communication"""
        status = self.create_nominal_status()
        
        # Communication normale
        status.last_heartbeat = time.time()
        safety_level, errors, warnings = self.safety_manager.check_flight_safety(status)
        self.assertEqual(safety_level, SafetyLevel.SAFE)
        
        # Perte de communication (ancienne)
        status.last_heartbeat = time.time() - 10.0  # 10 secondes ago
        safety_level, errors, warnings = self.safety_manager.check_flight_safety(status)
        self.assertEqual(safety_level, SafetyLevel.EMERGENCY)
        self.assertIn("connexion", " ".join(errors).lower())
        
    def test_scenario_multiple_failures(self):
        """Scénario: Pannes multiples"""
        status = self.create_nominal_status()
        
        # Combinaison de problèmes
        status.battery_percentage = 8.0  # Batterie critique
        status.gps_fix = False  # Perte GPS
        status.position = (0.0, 0.0, 140.0)  # Altitude excessive
        
        safety_level, errors, warnings = self.safety_manager.check_flight_safety(status)
        
        # Devrait être au niveau d'urgence
        self.assertEqual(safety_level, SafetyLevel.EMERGENCY)
        
        # Devrait y avoir plusieurs erreurs
        self.assertGreaterEqual(len(errors), 3)
        
    def test_scenario_pre_arm_with_warnings(self):
        """Scénario: Pré-armement avec avertissements"""
        status = self.create_nominal_status()
        status.armed = False
        
        # Conditions qui génèrent des avertissements mais permettent l'armement
        status.gps_satellites = 6  # Minimum acceptable
        status.gps_hdop = 2.5  # Légèrement élevé
        
        can_arm, errors, warnings = self.safety_manager.check_pre_arm_conditions(status)
        
        self.assertTrue(can_arm, "Devrait pouvoir armer avec des avertissements")
        self.assertEqual(len(errors), 0, "Ne devrait pas y avoir d'erreurs")
        self.assertGreater(len(warnings), 0, "Devrait y avoir des avertissements")
        
    def test_scenario_battery_trend_analysis(self):
        """Scénario: Analyse de tendance de batterie"""
        # Simulation d'une décharge normale
        timestamps = []
        voltages = []
        percentages = []
        
        base_time = time.time()
        for i in range(10):
            timestamp = base_time + i * 30  # Toutes les 30 secondes
            voltage = 11.1 - i * 0.05  # Décharge lente
            percentage = 100.0 - i * 5.0
            
            timestamps.append(timestamp)
            voltages.append(voltage)
            percentages.append(percentage)
            
            self.safety_manager.update_battery_trend(voltage, percentage)
            
        # Vérifier que l'historique est maintenu
        self.assertGreater(len(self.safety_manager._battery_history), 5)
        
        # Vérifier que les anciens points sont supprimés
        # Simuler un point très ancien
        old_time = base_time - 400  # Plus de 5 minutes
        self.safety_manager._battery_history.insert(0, (old_time, 12.0, 100.0))
        
        # Nouvelle mise à jour devrait nettoyer
        self.safety_manager.update_battery_trend(10.5, 75.0)
        
        # Vérifier que l'ancien point a été supprimé
        for timestamp, _, _ in self.safety_manager._battery_history:
            self.assertGreater(timestamp, base_time - 300)
            
    def test_scenario_rapid_battery_discharge(self):
        """Scénario: Décharge rapide de batterie"""
        base_time = time.time()
        
        # Simulation d'une décharge rapide
        for i in range(5):
            timestamp = base_time + i * 10  # Toutes les 10 secondes
            voltage = 11.1 - i * 0.3  # Décharge rapide
            percentage = 100.0 - i * 15.0  # Décharge très rapide
            
            self.safety_manager.update_battery_trend(voltage, percentage)
            
        # Obtenir l'analyse de tendance
        trend = self.safety_manager.get_battery_trend()
        
        # Devrait détecter une décharge rapide
        self.assertIn("rapide", trend.lower())
        
    def test_scenario_safety_manager_limits_configuration(self):
        """Scénario: Configuration des limites de sécurité"""
        # Test de modification des limites
        original_min_voltage = self.safety_manager.min_battery_voltage
        
        # Modification des limites
        self.safety_manager.min_battery_voltage = 11.0
        self.safety_manager.min_battery_percentage = 30.0
        
        status = self.create_nominal_status()
        status.armed = False
        status.battery_voltage = 10.8  # Sous la nouvelle limite
        status.battery_percentage = 25.0  # Sous la nouvelle limite
        
        can_arm, errors, warnings = self.safety_manager.check_pre_arm_conditions(status)
        
        self.assertFalse(can_arm, "Ne devrait pas pouvoir armer avec les nouvelles limites")
        
        # Restaurer les limites originales
        self.safety_manager.min_battery_voltage = original_min_voltage


class TestSafetyCriticalPaths(unittest.TestCase):
    """Tests des chemins critiques de sécurité"""
    
    def setUp(self):
        """Configuration des tests"""
        self.logger = Mock()
        self.safety_manager = SafetyManager(self.logger)
        
    def test_thread_safety_battery_updates(self):
        """Test de sécurité thread pour les mises à jour batterie"""
        import threading
        
        def update_battery_worker():
            for i in range(50):
                voltage = 11.0 + (i % 10) * 0.01
                percentage = 50.0 + (i % 20) * 2.0
                self.safety_manager.update_battery_trend(voltage, percentage)
                time.sleep(0.001)
                
        # Lancement de plusieurs threads
        threads = []
        for _ in range(3):
            thread = threading.Thread(target=update_battery_worker)
            threads.append(thread)
            thread.start()
            
        # Attente de completion
        for thread in threads:
            thread.join()
            
        # Vérification que les données sont cohérentes
        history_len = len(self.safety_manager._battery_history)
        self.assertGreater(history_len, 0)
        self.assertLessEqual(history_len, 150)  # Maximum attendu
        
    def test_extreme_values_handling(self):
        """Test de gestion des valeurs extrêmes"""
        status = DroneStatusData()
        
        # Test avec valeurs extrêmes
        status.connected = True
        status.armed = True
        status.battery_percentage = -10.0  # Valeur impossible
        status.gps_hdop = float('inf')  # Valeur infinie
        status.position = (float('nan'), 0.0, -1000.0)  # Valeurs problématiques
        status.last_heartbeat = time.time() - 1000000  # Très ancien
        
        # Le système ne devrait pas planter
        try:
            safety_level, errors, warnings = self.safety_manager.check_flight_safety(status)
            success = True
        except Exception:
            success = False
            
        self.assertTrue(success, "Le système devrait gérer les valeurs extrêmes sans planter")
        
    def test_memory_management_battery_history(self):
        """Test de gestion mémoire pour l'historique batterie"""
        # Ajouter beaucoup de points pour tester le nettoyage
        base_time = time.time()
        
        # Ajouter des points sur une longue période
        for i in range(1000):
            timestamp = base_time - (1000 - i) * 60  # Étalé sur 1000 minutes
            voltage = 11.0 + (i % 10) * 0.01
            percentage = 50.0 + (i % 20) * 2.0
            
            self.safety_manager._battery_history.append((timestamp, voltage, percentage))
            
        # Déclencher le nettoyage
        self.safety_manager.update_battery_trend(11.1, 75.0)
        
        # Vérifier que l'historique a été nettoyé (garde seulement 5 minutes)
        for timestamp, _, _ in self.safety_manager._battery_history:
            self.assertGreater(timestamp, base_time - 300)
            
        # L'historique ne devrait pas être trop long
        self.assertLess(len(self.safety_manager._battery_history), 100)


def run_safety_tests():
    """Lance tous les tests de sécurité"""
    loader = unittest.TestLoader()
    suite = unittest.TestSuite()
    
    # Ajouter les classes de test
    test_classes = [
        TestSafetyScenarios,
        TestSafetyCriticalPaths
    ]
    
    for test_class in test_classes:
        tests = loader.loadTestsFromTestCase(test_class)
        suite.addTests(tests)
        
    # Lancement avec verbosité élevée pour les tests de sécurité
    runner = unittest.TextTestRunner(verbosity=2)
    result = runner.run(suite)
    
    return result.wasSuccessful()


if __name__ == '__main__':
    success = run_safety_tests()
    exit(0 if success else 1)
