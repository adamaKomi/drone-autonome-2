#!/usr/bin/env python3
"""
Test d'intégration avec simulation MAVROS
Auteur: Adama Komi
Date: 2025-09-11

Ce script teste l'intégration complète avec une simulation MAVROS
"""

import rclpy
import time
import json
import threading
from rclpy.node import Node

# Messages et services
from geometry_msgs.msg import PoseStamped, TwistStamped
from sensor_msgs.msg import NavSatFix, BatteryState
from std_msgs.msg import String
from mavros_msgs.msg import State, Altitude
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
from std_srvs.srv import Trigger


class MAVROSSimulator(Node):
    """Simulateur MAVROS pour les tests"""
    
    def __init__(self):
        super().__init__('mavros_simulator')
        
        # Publishers MAVROS simulés
        self.state_pub = self.create_publisher(
            State, '/mavros/state', 10
        )
        self.local_pose_pub = self.create_publisher(
            PoseStamped, '/mavros/local_position/pose', 10
        )
        self.velocity_pub = self.create_publisher(
            TwistStamped, '/mavros/local_position/velocity_local', 10
        )
        self.gps_pub = self.create_publisher(
            NavSatFix, '/mavros/global_position/global', 10
        )
        self.altitude_pub = self.create_publisher(
            Altitude, '/mavros/altitude', 10
        )
        self.battery_pub = self.create_publisher(
            BatteryState, '/mavros/battery', 10
        )
        
        # Services MAVROS simulés
        self.arming_srv = self.create_service(
            CommandBool, '/mavros/cmd/arming', self.handle_arming
        )
        self.set_mode_srv = self.create_service(
            SetMode, '/mavros/set_mode', self.handle_set_mode
        )
        self.takeoff_srv = self.create_service(
            CommandTOL, '/mavros/cmd/takeoff', self.handle_takeoff
        )
        self.land_srv = self.create_service(
            CommandTOL, '/mavros/cmd/land', self.handle_land
        )
        
        # État simulé
        self.simulated_state = {
            'connected': True,
            'armed': False,
            'guided': False,
            'mode': 'STABILIZE',
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'battery': 85.0,
            'altitude': 0.0,
            'gps_fix': True
        }
        
        # Timer pour publier les données
        self.pub_timer = self.create_timer(0.1, self.publish_data)
        
        self.get_logger().info("Simulateur MAVROS initialisé")
    
    def publish_data(self):
        """Publie les données simulées"""
        # État
        state_msg = State()
        state_msg.connected = self.simulated_state['connected']
        state_msg.armed = self.simulated_state['armed']
        state_msg.guided = self.simulated_state['guided']
        state_msg.mode = self.simulated_state['mode']
        state_msg.system_status = 4 if self.simulated_state['connected'] else 0
        self.state_pub.publish(state_msg)
        
        # Position
        pose_msg = PoseStamped()
        pose_msg.header.stamp = self.get_clock().now().to_msg()
        pose_msg.header.frame_id = "map"
        pose_msg.pose.position.x = self.simulated_state['position']['x']
        pose_msg.pose.position.y = self.simulated_state['position']['y']
        pose_msg.pose.position.z = self.simulated_state['position']['z']
        pose_msg.pose.orientation.w = 1.0
        self.local_pose_pub.publish(pose_msg)
        
        # Vitesse
        vel_msg = TwistStamped()
        vel_msg.header.stamp = self.get_clock().now().to_msg()
        vel_msg.twist.linear.x = self.simulated_state['velocity']['x']
        vel_msg.twist.linear.y = self.simulated_state['velocity']['y']
        vel_msg.twist.linear.z = self.simulated_state['velocity']['z']
        self.velocity_pub.publish(vel_msg)
        
        # GPS
        gps_msg = NavSatFix()
        gps_msg.header.stamp = self.get_clock().now().to_msg()
        gps_msg.status.status = 1 if self.simulated_state['gps_fix'] else -1
        gps_msg.status.service = 8  # 8 satellites
        gps_msg.latitude = 45.0
        gps_msg.longitude = 2.0
        gps_msg.altitude = self.simulated_state['altitude']
        self.gps_pub.publish(gps_msg)
        
        # Altitude
        alt_msg = Altitude()
        alt_msg.header.stamp = self.get_clock().now().to_msg()
        alt_msg.relative = self.simulated_state['altitude']
        alt_msg.amsl = self.simulated_state['altitude'] + 300.0
        self.altitude_pub.publish(alt_msg)
        
        # Batterie
        battery_msg = BatteryState()
        battery_msg.voltage = 15.6
        battery_msg.percentage = self.simulated_state['battery'] / 100.0
        self.battery_pub.publish(battery_msg)
    
    def handle_arming(self, request, response):
        """Gestionnaire d'armement simulé"""
        self.simulated_state['armed'] = request.value
        response.success = True
        response.result = 0
        
        action = "armé" if request.value else "désarmé"
        self.get_logger().info(f"Drone simulé {action}")
        return response
    
    def handle_set_mode(self, request, response):
        """Gestionnaire de changement de mode simulé"""
        valid_modes = ['STABILIZE', 'GUIDED', 'AUTO', 'LOITER', 'RTL', 'LAND']
        
        if request.custom_mode in valid_modes:
            self.simulated_state['mode'] = request.custom_mode
            self.simulated_state['guided'] = (request.custom_mode == 'GUIDED')
            response.mode_sent = True
            self.get_logger().info(f"Mode changé vers: {request.custom_mode}")
        else:
            response.mode_sent = False
            self.get_logger().warning(f"Mode invalide: {request.custom_mode}")
        
        return response
    
    def handle_takeoff(self, request, response):
        """Gestionnaire de décollage simulé"""
        if self.simulated_state['armed'] and self.simulated_state['guided']:
            # Simulation du décollage
            target_altitude = request.altitude
            self.simulated_state['altitude'] = target_altitude
            self.simulated_state['position']['z'] = target_altitude
            
            response.success = True
            self.get_logger().info(f"Décollage simulé à {target_altitude}m")
        else:
            response.success = False
            self.get_logger().warning("Décollage impossible: drone non armé ou non en mode GUIDED")
        
        return response
    
    def handle_land(self, request, response):
        """Gestionnaire d'atterrissage simulé"""
        # Simulation de l'atterrissage
        self.simulated_state['altitude'] = 0.0
        self.simulated_state['position']['z'] = 0.0
        self.simulated_state['velocity'] = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        
        response.success = True
        self.get_logger().info("Atterrissage simulé")
        return response
    
    def simulate_flight_pattern(self):
        """Simule un pattern de vol"""
        # Vol en carré de 10m x 10m
        waypoints = [
            {'x': 10.0, 'y': 0.0, 'z': 5.0},
            {'x': 10.0, 'y': 10.0, 'z': 5.0},
            {'x': 0.0, 'y': 10.0, 'z': 5.0},
            {'x': 0.0, 'y': 0.0, 'z': 5.0}
        ]
        
        for wp in waypoints:
            self.simulated_state['position'] = wp
            self.get_logger().info(f"Waypoint atteint: {wp}")
            time.sleep(2.0)


class IntegrationTester(Node):
    """Testeur d'intégration"""
    
    def __init__(self):
        super().__init__('integration_tester')
        
        # Clients pour les services du nœud MAVROS Interface
        self.status_client = self.create_client(
            Trigger, '/drone_nav/get_drone_status'
        )
        self.position_client = self.create_client(
            Trigger, '/drone_nav/get_current_position'
        )
        self.diagnostics_client = self.create_client(
            Trigger, '/drone_nav/get_diagnostics'
        )
        self.arm_client = self.create_client(
            CommandBool, '/drone_nav/arm_disarm'
        )
        self.mode_client = self.create_client(
            SetMode, '/drone_nav/set_mode'
        )
        self.takeoff_client = self.create_client(
            CommandTOL, '/drone_nav/takeoff'
        )
        self.land_client = self.create_client(
            CommandTOL, '/drone_nav/land'
        )
        
        # Subscribers pour monitoring
        self.status_sub = self.create_subscription(
            String, '/drone_nav/drone_status',
            self.status_callback, 10
        )
        self.safety_sub = self.create_subscription(
            String, '/drone_nav/safety_status',
            self.safety_callback, 10
        )
        
        # Compteurs de messages
        self.status_count = 0
        self.safety_count = 0
        
        self.get_logger().info("Testeur d'intégration initialisé")
    
    def status_callback(self, msg):
        """Callback pour le statut du drone"""
        self.status_count += 1
        
        try:
            status_data = json.loads(msg.data)
            self.get_logger().debug(
                f"Statut reçu #{self.status_count}: "
                f"connected={status_data.get('connected')}, "
                f"mode={status_data.get('mode')}"
            )
        except Exception as e:
            self.get_logger().error(f"Erreur parsing statut: {e}")
    
    def safety_callback(self, msg):
        """Callback pour le statut de sécurité"""
        self.safety_count += 1
        
        try:
            safety_data = json.loads(msg.data)
            level = safety_data.get('level', 'UNKNOWN')
            if level != 'NORMAL':
                self.get_logger().info(f"Statut sécurité: {level}")
        except Exception as e:
            self.get_logger().error(f"Erreur parsing sécurité: {e}")
    
    def wait_for_services(self, timeout=10.0):
        """Attend que tous les services soient disponibles"""
        services = [
            (self.status_client, "get_drone_status"),
            (self.position_client, "get_current_position"),
            (self.diagnostics_client, "get_diagnostics"),
            (self.arm_client, "arm_disarm"),
            (self.mode_client, "set_mode"),
            (self.takeoff_client, "takeoff"),
            (self.land_client, "land")
        ]
        
        for client, name in services:
            if not client.wait_for_service(timeout_sec=timeout):
                self.get_logger().error(f"Service {name} non disponible")
                return False
            else:
                self.get_logger().info(f"Service {name} disponible")
        
        return True
    
    def test_status_service(self):
        """Test du service de statut"""
        self.get_logger().info("=== Test Service Statut ===")
        
        request = Trigger.Request()
        future = self.status_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result():
            response = future.result()
            if response.success:
                status_data = json.loads(response.message)
                self.get_logger().info(f"✅ Statut reçu: {status_data['mode']}")
                return True
            else:
                self.get_logger().error("❌ Service statut a échoué")
        else:
            self.get_logger().error("❌ Timeout service statut")
        
        return False
    
    def test_position_service(self):
        """Test du service de position"""
        self.get_logger().info("=== Test Service Position ===")
        
        request = Trigger.Request()
        future = self.position_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result():
            response = future.result()
            if response.success:
                position_data = json.loads(response.message)
                pos = position_data['position']
                self.get_logger().info(
                    f"✅ Position reçue: x={pos['x']:.1f}, y={pos['y']:.1f}, z={pos['z']:.1f}"
                )
                return True
            else:
                self.get_logger().error("❌ Service position a échoué")
        else:
            self.get_logger().error("❌ Timeout service position")
        
        return False
    
    def test_diagnostics_service(self):
        """Test du service de diagnostics"""
        self.get_logger().info("=== Test Service Diagnostics ===")
        
        request = Trigger.Request()
        future = self.diagnostics_client.call_async(request)
        
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result():
            response = future.result()
            if response.success:
                diag_data = json.loads(response.message)
                self.get_logger().info(f"✅ Diagnostics reçus: {diag_data['node_name']}")
                return True
            else:
                self.get_logger().error("❌ Service diagnostics a échoué")
        else:
            self.get_logger().error("❌ Timeout service diagnostics")
        
        return False
    
    def test_flight_sequence(self):
        """Test d'une séquence de vol complète"""
        self.get_logger().info("=== Test Séquence de Vol ===")
        
        success_count = 0
        total_tests = 5
        
        # 1. Changement de mode GUIDED
        self.get_logger().info("1. Changement vers mode GUIDED...")
        mode_request = SetMode.Request()
        mode_request.custom_mode = "GUIDED"
        
        future = self.mode_client.call_async(mode_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            self.get_logger().info("✅ Mode GUIDED activé")
            success_count += 1
        else:
            self.get_logger().error("❌ Échec changement de mode")
        
        time.sleep(1.0)
        
        # 2. Armement
        self.get_logger().info("2. Armement du drone...")
        arm_request = CommandBool.Request()
        arm_request.value = True
        
        future = self.arm_client.call_async(arm_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() and future.result().success:
            self.get_logger().info("✅ Drone armé")
            success_count += 1
        else:
            self.get_logger().error("❌ Échec armement")
        
        time.sleep(1.0)
        
        # 3. Décollage
        self.get_logger().info("3. Décollage...")
        takeoff_request = CommandTOL.Request()
        takeoff_request.altitude = 5.0
        
        future = self.takeoff_client.call_async(takeoff_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() and future.result().success:
            self.get_logger().info("✅ Décollage réussi")
            success_count += 1
        else:
            self.get_logger().error("❌ Échec décollage")
        
        time.sleep(2.0)
        
        # 4. Vérification de l'altitude
        self.get_logger().info("4. Vérification de l'altitude...")
        if self.test_position_service():
            success_count += 1
        
        time.sleep(1.0)
        
        # 5. Atterrissage
        self.get_logger().info("5. Atterrissage...")
        land_request = CommandTOL.Request()
        
        future = self.land_client.call_async(land_request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
        
        if future.result() and future.result().success:
            self.get_logger().info("✅ Atterrissage réussi")
            success_count += 1
        else:
            self.get_logger().error("❌ Échec atterrissage")
        
        self.get_logger().info(f"Séquence terminée: {success_count}/{total_tests} succès")
        return success_count == total_tests
    
    def run_all_tests(self):
        """Exécute tous les tests d'intégration"""
        self.get_logger().info("Début des tests d'intégration MAVROS")
        
        # Attendre les services
        if not self.wait_for_services():
            return False
        
        # Attendre la stabilisation
        self.get_logger().info("Attente stabilisation (5s)...")
        time.sleep(5.0)
        
        results = []
        
        # Tests des services
        results.append(self.test_status_service())
        results.append(self.test_position_service())
        results.append(self.test_diagnostics_service())
        
        # Test de séquence de vol
        results.append(self.test_flight_sequence())
        
        # Résultats
        success_count = sum(results)
        total_count = len(results)
        
        self.get_logger().info("="*50)
        self.get_logger().info("RÉSULTATS TESTS D'INTÉGRATION")
        self.get_logger().info("="*50)
        self.get_logger().info(f"Succès: {success_count}/{total_count}")
        self.get_logger().info(f"Messages statut reçus: {self.status_count}")
        self.get_logger().info(f"Messages sécurité reçus: {self.safety_count}")
        
        if success_count == total_count:
            self.get_logger().info("✅ TOUS LES TESTS D'INTÉGRATION RÉUSSIS!")
            return True
        else:
            self.get_logger().info("❌ CERTAINS TESTS D'INTÉGRATION ONT ÉCHOUÉ!")
            return False


def main():
    """Point d'entrée principal"""
    rclpy.init()
    
    # Créer les nœuds
    simulator = MAVROSSimulator()
    tester = IntegrationTester()
    
    # Executor
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(simulator)
    executor.add_node(tester)
    
    # Thread pour l'executor
    executor_thread = threading.Thread(target=executor.spin)
    executor_thread.daemon = True
    executor_thread.start()
    
    try:
        # Attendre un peu pour l'initialisation
        time.sleep(2.0)
        
        # Exécuter les tests
        success = tester.run_all_tests()
        
        # Attendre un peu pour voir les derniers messages
        time.sleep(2.0)
        
    except KeyboardInterrupt:
        print("\nInterruption utilisateur")
        success = False
    
    finally:
        # Nettoyage
        simulator.destroy_node()
        tester.destroy_node()
        rclpy.shutdown()
    
    return success


if __name__ == '__main__':
    success = main()
    exit(0 if success else 1)
