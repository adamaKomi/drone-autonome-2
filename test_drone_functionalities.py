#!/usr/bin/env python3
"""
=============================================================================
SCRIPT DE TEST COMPLET - DroneInterface v3.0
=============================================================================
Auteur: Adama Komi
Date: 2025-09-06

Description:
    Script de test automatisé pour toutes les fonctionnalités du système
    drone_interface. Teste l'armement, le désarmement, les modes, la sécurité,
    et tous les services disponibles.
=============================================================================
"""

import rclpy
import time
import sys
from rclpy.node import Node
from drone_msgs.srv import ArmDrone, DisarmDrone, SetFlightMode, SafetyCheck, EmergencyStop
from std_srvs.srv import Trigger
from drone_msgs.msg import DroneStatus
from std_msgs.msg import String


class DroneTestSuite(Node):
    """Suite de tests pour le DroneInterface"""
    
    def __init__(self):
        super().__init__('drone_test_suite')
        self.logger = self.get_logger()
        
        # Clients des services
        self.arm_client = self.create_client(ArmDrone, '/drone/arm')
        self.disarm_client = self.create_client(DisarmDrone, '/drone/disarm')
        self.mode_client = self.create_client(SetFlightMode, '/drone/set_mode')
        self.safety_client = self.create_client(SafetyCheck, '/drone/safety_check')
        self.health_client = self.create_client(Trigger, '/drone/health_check')
        self.emergency_client = self.create_client(EmergencyStop, '/drone/emergency_stop')
        
        # Subscription au statut
        self.status_sub = self.create_subscription(
            DroneStatus, '/drone/status', 
            self.status_callback, 10
        )
        
        self.current_status = None
        
    def status_callback(self, msg):
        """Callback pour recevoir le statut du drone"""
        self.current_status = msg
        
    def wait_for_services(self, timeout=10.0):
        """Attend que tous les services soient disponibles"""
        self.logger.info("🔍 Attente des services...")
        
        services = [
            (self.arm_client, "arm"),
            (self.disarm_client, "disarm"),
            (self.mode_client, "set_mode"),
            (self.safety_client, "safety_check"),
            (self.health_client, "health_check"),
            (self.emergency_client, "emergency_stop")
        ]
        
        for client, name in services:
            if not client.wait_for_service(timeout_sec=timeout):
                self.logger.error(f"❌ Service {name} non disponible")
                return False
                
        self.logger.info("✅ Tous les services sont disponibles")
        return True
        
    def test_health_check(self):
        """Test de vérification de santé"""
        self.logger.info("\n� TEST: Health Check")
        
        request = Trigger.Request()
        
        future = self.health_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result():
            response = future.result()
            self.logger.info(f"✅ Health: {response.message}")
            return response.success
        else:
            self.logger.error("❌ Health check timeout")
            return False
            
    def test_safety_check(self):
        """Test de vérification de sécurité"""
        self.logger.info("\n�️ TEST: Safety Check")
        
        request = SafetyCheck.Request()
        request.check_type = "PREFLIGHT"
        request.check_all_systems = True
        
        future = self.safety_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result():
            response = future.result()
            self.logger.info(f"✅ Safety: {response.message}")
            self.logger.info(f"   Score: {response.safety_score:.2f}")
            if response.warnings:
                self.logger.warning(f"   Warnings: {response.warnings}")
            return response.success
        else:
            self.logger.error("❌ Safety check timeout")
            return False
            
    def test_mode_change(self, mode):
        """Test de changement de mode"""
        self.logger.info(f"\n🎯 TEST: Changement de mode vers {mode}")
        
        request = SetFlightMode.Request()
        request.flight_mode = mode
        request.sub_mode = ""
        request.max_speed = 10.0
        request.max_altitude = 100.0
        request.max_distance = 1000.0
        request.enable_obstacle_avoidance = True
        request.enable_geofence = True
        request.enable_return_to_launch = True
        request.failsafe_altitude = 20.0
        request.operator_id = "test_operator"
        request.override_restrictions = False
        request.custom_parameters = []
        
        future = self.mode_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result():
            response = future.result()
            self.logger.info(f"✅ Mode {mode}: {response.message}")
            return response.success
        else:
            self.logger.error(f"❌ Mode change {mode} timeout")
            return False
            
    def test_arm_disarm_cycle(self):
        """Test du cycle armement/désarmement"""
        self.logger.info("\n🔫 TEST: Cycle Armement/Désarmement")
        
        # Test d'armement
        arm_request = ArmDrone.Request()
        arm_request.arm_mode = "NORMAL"
        arm_request.force_arm = False
        arm_request.skip_preflight = False
        
        self.logger.info("   Tentative d'armement...")
        future = self.arm_client.call_async(arm_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result():
            arm_response = future.result()
            self.logger.info(f"   Armement: {arm_response.message}")
            
            if arm_response.success:
                self.logger.info("✅ Drone armé avec succès")
                
                # Attendre un peu
                time.sleep(2)
                
                # Test de désarmement
                disarm_request = DisarmDrone.Request()
                disarm_request.force_disarm = False
                
                self.logger.info("   Tentative de désarmement...")
                future = self.disarm_client.call_async(disarm_request)
                rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
                
                if future.result():
                    disarm_response = future.result()
                    self.logger.info(f"   Désarmement: {disarm_response.message}")
                    return disarm_response.success
                else:
                    self.logger.error("❌ Désarmement timeout")
                    return False
            else:
                self.logger.warning(f"⚠️ Armement refusé: {arm_response.message}")
                if arm_response.errors:
                    for error in arm_response.errors:
                        self.logger.warning(f"     Error: {error}")
                return False
        else:
            self.logger.error("❌ Armement timeout")
            return False
            
    def test_status_monitoring(self):
        """Test de surveillance du statut"""
        self.logger.info("\n� TEST: Monitoring du statut")
        
        # Attendre un message de statut
        timeout = 5.0
        start_time = time.time()
        
        while self.current_status is None and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if self.current_status:
            status = self.current_status
            self.logger.info("✅ Statut reçu:")
            self.logger.info(f"   Connecté: {status.is_connected}")
            self.logger.info(f"   Armé: {status.is_armed}")
            self.logger.info(f"   En vol: {status.is_flying}")
            self.logger.info(f"   Sain: {status.is_healthy}")
            self.logger.info(f"   Mode: {status.flight_mode}")
            self.logger.info(f"   Batterie: {status.battery.percentage_remaining:.1f}%")
            self.logger.info(f"   Position: x={status.position.x:.2f}, y={status.position.y:.2f}, z={status.position.z:.2f}")
            return True
        else:
            self.logger.error("❌ Aucun statut reçu")
            return False
            
    def run_all_tests(self):
        """Exécute tous les tests"""
        self.logger.info("� DÉBUT DES TESTS DRONE INTERFACE v3.0\n")
        
        tests = [
            ("Services disponibles", self.wait_for_services),
            ("Monitoring statut", self.test_status_monitoring),
            ("Vérification santé", self.test_health_check),
            ("Vérification sécurité", self.test_safety_check),
            ("Mode GUIDED", lambda: self.test_mode_change("GUIDED")),
            ("Cycle Arm/Disarm", self.test_arm_disarm_cycle),
            ("Mode STABILIZE", lambda: self.test_mode_change("STABILIZE")),
        ]
        
        results = []
        
        for test_name, test_func in tests:
            try:
                result = test_func()
                results.append((test_name, result))
                if result:
                    self.logger.info(f"✅ {test_name}: RÉUSSI")
                else:
                    self.logger.error(f"❌ {test_name}: ÉCHOUÉ")
            except Exception as e:
                self.logger.error(f"❌ {test_name}: ERREUR - {e}")
                results.append((test_name, False))
                
            time.sleep(1)  # Pause entre les tests
            
        # Résumé
        self.logger.info("\n📋 RÉSUMÉ DES TESTS:")
        successful = sum(1 for _, result in results if result)
        total = len(results)
        
        for test_name, result in results:
            status = "✅ RÉUSSI" if result else "❌ ÉCHOUÉ"
            self.logger.info(f"   {test_name}: {status}")
            
        self.logger.info(f"\n🎯 SCORE: {successful}/{total} tests réussis ({100*successful/total:.1f}%)")
        
        return successful == total


def main():
    """Point d'entrée principal"""
    rclpy.init()
    
    try:
        test_suite = DroneTestSuite()
        
        # Attendre un peu que le système soit prêt
        time.sleep(2)
        
        # Exécuter les tests
        success = test_suite.run_all_tests()
        
        if success:
            test_suite.get_logger().info("\n🎉 TOUS LES TESTS SONT RÉUSSIS!")
            sys.exit(0)
        else:
            test_suite.get_logger().error("\n💥 CERTAINS TESTS ONT ÉCHOUÉ!")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n🛑 Tests interrompus par l'utilisateur")
        sys.exit(1)
    except Exception as e:
        print(f"\n❌ Erreur critique: {e}")
        sys.exit(1)
    finally:
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == '__main__':
    main()


if __name__ == '__main__':
    main()
