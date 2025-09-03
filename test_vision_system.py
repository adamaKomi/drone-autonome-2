#!/usr/bin/env python3
"""
=============================================================================
TEST - Système de vision et collecte de données
=============================================================================
Script de test pour valider le fonctionnement des nouveaux nœuds:
- Test de détection de fleurs
- Test de collecte de données
- Simulation d'une mission de pollinisation

Usage:
    python3 test_vision_system.py
=============================================================================
"""

import time
import json
import asyncio
import subprocess
from typing import List, Dict, Any

# ROS2 imports
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool, Trigger


class VisionSystemTester(Node):
    """Testeur pour le système de vision"""
    
    def __init__(self):
        super().__init__('vision_system_tester')
        
        self.logger = self.get_logger()
        self.logger.info("🧪 Initialisation du testeur de vision...")
        
        # Clients de service
        self.vision_client = self.create_client(SetBool, '/vision/set_detection')
        self.collector_client = self.create_client(SetBool, '/data_collector/set_collection')
        self.save_client = self.create_client(Trigger, '/data_collector/save_data')
        
        # Souscripteurs pour monitoring
        self.flowers_sub = self.create_subscription(
            String,
            '/vision/flowers_detected',
            self._flowers_callback,
            10
        )
        
        self.status_sub = self.create_subscription(
            String,
            '/data_collector/status',
            self._status_callback,
            10
        )
        
        # État du test
        self.flowers_detected = []
        self.test_results = {}
        
        self.logger.info("✅ Testeur initialisé!")
        
    def _flowers_callback(self, msg: String):
        """Callback pour les fleurs détectées"""
        try:
            data = eval(msg.data)  # Simple parsing
            if data.get("count", 0) > 0:
                self.flowers_detected.extend(data.get("flowers", []))
                self.logger.info(f"🌸 {data['count']} fleur(s) détectée(s)")
        except Exception as e:
            self.logger.error(f"❌ Erreur parsing fleurs: {e}")
            
    def _status_callback(self, msg: String):
        """Callback pour le statut du collecteur"""
        self.logger.info(f"📊 Statut collecteur: {msg.data}")
        
    async def wait_for_services(self):
        """Attend que les services soient disponibles"""
        self.logger.info("⏳ Attente des services...")
        
        services = [
            (self.vision_client, '/vision/set_detection'),
            (self.collector_client, '/data_collector/set_collection'),
            (self.save_client, '/data_collector/save_data')
        ]
        
        for client, service_name in services:
            while not client.wait_for_service(timeout_sec=1.0):
                self.logger.info(f"⏳ Attente du service {service_name}...")
                
        self.logger.info("✅ Tous les services sont disponibles!")
        
    async def test_vision_activation(self):
        """Test d'activation de la détection"""
        self.logger.info("🔍 Test activation détection de vision...")
        
        # Activer la détection
        request = SetBool.Request()
        request.data = True
        
        future = self.vision_client.call_async(request)
        await asyncio.sleep(0.1)  # Petit délai
        
        response = await asyncio.wrap_future(future)
        
        if response.success:
            self.logger.info("✅ Détection activée avec succès!")
            self.test_results['vision_activation'] = True
        else:
            self.logger.error("❌ Échec activation détection")
            self.test_results['vision_activation'] = False
            
        return response.success
        
    async def test_data_collection(self):
        """Test de collecte de données"""
        self.logger.info("📊 Test activation collecte de données...")
        
        # Activer la collecte
        request = SetBool.Request()
        request.data = True
        
        future = self.collector_client.call_async(request)
        await asyncio.sleep(0.1)
        
        response = await asyncio.wrap_future(future)
        
        if response.success:
            self.logger.info("✅ Collecte activée avec succès!")
            self.test_results['data_collection'] = True
        else:
            self.logger.error("❌ Échec activation collecte")
            self.test_results['data_collection'] = False
            
        return response.success
        
    async def simulate_detection_period(self, duration: float = 10.0):
        """Simule une période de détection"""
        self.logger.info(f"⏱️ Simulation détection pendant {duration}s...")
        
        start_count = len(self.flowers_detected)
        start_time = time.time()
        
        # Attendre pendant la durée spécifiée
        while time.time() - start_time < duration:
            await asyncio.sleep(0.5)
            
        end_count = len(self.flowers_detected)
        detections_count = end_count - start_count
        
        self.logger.info(f"🌸 {detections_count} détections pendant la simulation")
        self.test_results['detections_count'] = detections_count
        
        return detections_count
        
    async def test_data_save(self):
        """Test de sauvegarde des données"""
        self.logger.info("💾 Test sauvegarde des données...")
        
        request = Trigger.Request()
        future = self.save_client.call_async(request)
        await asyncio.sleep(0.1)
        
        response = await asyncio.wrap_future(future)
        
        if response.success:
            self.logger.info(f"✅ Sauvegarde réussie: {response.message}")
            self.test_results['data_save'] = True
        else:
            self.logger.error(f"❌ Échec sauvegarde: {response.message}")
            self.test_results['data_save'] = False
            
        return response.success
        
    async def run_complete_test(self):
        """Lance le test complet"""
        self.logger.info("🚀 Début du test complet du système de vision")
        self.logger.info("=" * 60)
        
        try:
            # 1. Attendre les services
            await self.wait_for_services()
            await asyncio.sleep(2)
            
            # 2. Activer la vision
            await self.test_vision_activation()
            await asyncio.sleep(2)
            
            # 3. Activer la collecte
            await self.test_data_collection()
            await asyncio.sleep(2)
            
            # 4. Simuler une période de détection
            await self.simulate_detection_period(15.0)
            await asyncio.sleep(2)
            
            # 5. Sauvegarder les données
            await self.test_data_save()
            
            # 6. Résultats
            self._print_test_results()
            
        except Exception as e:
            self.logger.error(f"❌ Erreur pendant le test: {e}")
            
        self.logger.info("🏁 Test terminé!")
        
    def _print_test_results(self):
        """Affiche les résultats des tests"""
        self.logger.info("=" * 60)
        self.logger.info("📊 RÉSULTATS DES TESTS")
        self.logger.info("=" * 60)
        
        for test_name, result in self.test_results.items():
            status = "✅ RÉUSSI" if result else "❌ ÉCHEC"
            self.logger.info(f"{test_name}: {status}")
            
        self.logger.info("")
        self.logger.info(f"🌸 Total fleurs détectées: {len(self.flowers_detected)}")
        
        if self.flowers_detected:
            colors = {}
            for flower in self.flowers_detected:
                color = flower.get('color', 'unknown')
                colors[color] = colors.get(color, 0) + 1
                
            self.logger.info("🎨 Répartition par couleur:")
            for color, count in colors.items():
                self.logger.info(f"  • {color}: {count}")
                
        # Score global
        success_count = sum(1 for result in self.test_results.values() if result)
        total_tests = len(self.test_results)
        score = (success_count / total_tests * 100) if total_tests > 0 else 0
        
        self.logger.info("")
        self.logger.info(f"🎯 Score global: {score:.1f}% ({success_count}/{total_tests})")
        self.logger.info("=" * 60)


async def main():
    """Fonction principale"""
    rclpy.init()
    
    try:
        tester = VisionSystemTester()
        
        # Lancer le test dans une tâche séparée
        test_task = asyncio.create_task(tester.run_complete_test())
        
        # Faire tourner le nœud ROS2 en parallèle
        while not test_task.done():
            rclpy.spin_once(tester, timeout_sec=0.1)
            await asyncio.sleep(0.01)
            
        # Attendre que le test se termine
        await test_task
        
    except KeyboardInterrupt:
        print("\n🛑 Test interrompu par l'utilisateur")
    except Exception as e:
        print(f"❌ Erreur: {e}")
    finally:
        try:
            if 'tester' in locals():
                tester.destroy_node()
        except:
            pass
        rclpy.shutdown()


def check_nodes_running():
    """Vérifie si les nœuds nécessaires sont en cours d'exécution"""
    print("🔍 Vérification des nœuds actifs...")
    
    try:
        result = subprocess.run(['ros2', 'node', 'list'], 
                              capture_output=True, text=True, timeout=5)
        
        active_nodes = result.stdout.strip().split('\n') if result.stdout else []
        
        required_nodes = ['/drone_vision', '/drone_data_collector']
        missing_nodes = []
        
        for required in required_nodes:
            if required not in active_nodes:
                missing_nodes.append(required)
                
        if missing_nodes:
            print("❌ Nœuds manquants:")
            for node in missing_nodes:
                print(f"  • {node}")
            print("\n💡 Lancez d'abord:")
            print("ros2 launch drone_vision vision_system.launch.py")
            return False
        else:
            print("✅ Tous les nœuds requis sont actifs!")
            return True
            
    except Exception as e:
        print(f"❌ Erreur vérification nœuds: {e}")
        return False


if __name__ == '__main__':
    print("🧪 TESTEUR SYSTÈME DE VISION DRONE")
    print("=" * 50)
    
    # Vérifier que les nœuds sont lancés
    if not check_nodes_running():
        print("\n⚠️ Lancez d'abord le système de vision avant de lancer ce test")
        exit(1)
        
    print("\n🚀 Lancement du test...")
    asyncio.run(main())
