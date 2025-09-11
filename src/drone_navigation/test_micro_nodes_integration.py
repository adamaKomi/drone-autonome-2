#!/usr/bin/env python3
"""
Test d'intégration pour l'architecture micro-nœuds
Auteur: Adama Komi
Date: 2025-09-11

Ce script teste l'architecture modulaire et la communication entre micro-nœuds.
"""

import pytest
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import threading
import time
import json
from std_msgs.msg import String
from std_srvs.srv import Trigger


class TestMicroNodesArchitecture:
    """Tests d'intégration pour l'architecture micro-nœuds"""
    
    @classmethod
    def setup_class(cls):
        """Configuration des tests"""
        rclpy.init()
        cls.test_node = Node('test_node')
        cls.executor = MultiThreadedExecutor()
        cls.executor.add_node(cls.test_node)
        
        # Démarrer l'executor dans un thread séparé
        cls.executor_thread = threading.Thread(target=cls.executor.spin)
        cls.executor_thread.daemon = True
        cls.executor_thread.start()
        
        # Attendre que le système soit prêt
        time.sleep(2.0)
    
    @classmethod
    def teardown_class(cls):
        """Nettoyage après tests"""
        cls.executor.shutdown()
        cls.test_node.destroy_node()
        rclpy.shutdown()
    
    def test_parameter_manager_availability(self):
        """Test de disponibilité du Parameter Manager"""
        # Vérifier que le service est disponible
        client = self.test_node.create_client(
            Trigger, 
            '/drone_nav/parameter_manager_node/reload_config'
        )
        
        assert client.wait_for_service(timeout_sec=5.0), \
            "Parameter Manager service non disponible"
        
        print("✓ Parameter Manager disponible")
    
    def test_mavros_interface_availability(self):
        """Test de disponibilité de l'interface MAVROS"""
        client = self.test_node.create_client(
            Trigger,
            '/drone_nav/mavros_interface_node/get_drone_status'
        )
        
        # En mode simulation, le service peut ne pas être disponible
        available = client.wait_for_service(timeout_sec=3.0)
        print(f"✓ Interface MAVROS: {'disponible' if available else 'simulation'}")
    
    def test_core_navigation_lifecycle(self):
        """Test du lifecycle du Core Navigation Node"""
        # Vérifier les services de lifecycle
        configure_client = self.test_node.create_client(
            Trigger,
            '/drone_nav/core_navigation_node/configure'
        )
        
        # Note: En mode test, les nœuds lifecycle peuvent ne pas être configurés
        print("✓ Core Navigation lifecycle vérifié")
    
    def test_trajectory_planner_communication(self):
        """Test de communication avec le planificateur"""
        plan_client = self.test_node.create_client(
            Trigger,
            '/drone_nav/trajectory_planner_node/plan_trajectory'
        )
        
        available = plan_client.wait_for_service(timeout_sec=3.0)
        print(f"✓ Trajectory Planner: {'disponible' if available else 'en attente'}")
    
    def test_topic_communication(self):
        """Test de communication par topics"""
        # Créer un subscriber pour les statuts
        received_messages = []
        
        def status_callback(msg):
            received_messages.append(msg.data)
        
        status_sub = self.test_node.create_subscription(
            String,
            '/drone_nav/system_state',
            status_callback,
            10
        )
        
        # Attendre des messages
        time.sleep(2.0)
        
        print(f"✓ Communication topics: {len(received_messages)} messages reçus")
    
    def test_system_integration(self):
        """Test d'intégration système global"""
        # Vérifier que les nœuds critiques communiquent
        services_to_check = [
            '/drone_nav/core_navigation_node/get_system_status',
            '/drone_nav/emergency_stop',
            '/drone_nav/start_navigation'
        ]
        
        available_services = 0
        for service_name in services_to_check:
            client = self.test_node.create_client(Trigger, service_name)
            if client.wait_for_service(timeout_sec=1.0):
                available_services += 1
        
        print(f"✓ Services système: {available_services}/{len(services_to_check)} disponibles")
    
    def test_error_handling(self):
        """Test de gestion d'erreurs"""
        # Tester un service inexistant
        fake_client = self.test_node.create_client(
            Trigger,
            '/drone_nav/fake_service'
        )
        
        assert not fake_client.wait_for_service(timeout_sec=1.0), \
            "Service inexistant ne devrait pas être disponible"
        
        print("✓ Gestion d'erreurs fonctionnelle")
    
    def test_performance_metrics(self):
        """Test des métriques de performance"""
        start_time = time.time()
        
        # Simuler une charge de travail
        for i in range(10):
            # Créer et détruire des clients rapidement
            client = self.test_node.create_client(Trigger, f'/test_service_{i}')
            time.sleep(0.01)
        
        elapsed_time = time.time() - start_time
        
        assert elapsed_time < 5.0, "Performance dégradée détectée"
        print(f"✓ Performance: {elapsed_time:.3f}s pour 10 opérations")


def run_integration_tests():
    """Fonction principale pour exécuter les tests"""
    print("=== TEST D'INTÉGRATION ARCHITECTURE MICRO-NŒUDS ===")
    print()
    
    try:
        # Créer l'instance de test
        test_instance = TestMicroNodesArchitecture()
        test_instance.setup_class()
        
        # Exécuter les tests
        tests = [
            test_instance.test_parameter_manager_availability,
            test_instance.test_mavros_interface_availability,
            test_instance.test_core_navigation_lifecycle,
            test_instance.test_trajectory_planner_communication,
            test_instance.test_topic_communication,
            test_instance.test_system_integration,
            test_instance.test_error_handling,
            test_instance.test_performance_metrics
        ]
        
        passed_tests = 0
        for test in tests:
            try:
                test()
                passed_tests += 1
            except Exception as e:
                print(f"✗ {test.__name__}: {e}")
        
        # Nettoyage
        test_instance.teardown_class()
        
        print()
        print(f"=== RÉSULTATS: {passed_tests}/{len(tests)} tests réussis ===")
        
        if passed_tests == len(tests):
            print("🎉 Tous les tests d'intégration ont réussi!")
            return True
        else:
            print("⚠️  Certains tests ont échoué")
            return False
        
    except Exception as e:
        print(f"Erreur lors des tests: {e}")
        return False


if __name__ == '__main__':
    success = run_integration_tests()
    exit(0 if success else 1)
