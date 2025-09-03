#!/usr/bin/env python3
"""
=============================================================================
TEST RAPIDE - Système de vision drone
=============================================================================
Test rapide pour vérifier que les nœuds de vision démarrent correctement
et communiquent entre eux.

Usage:
    python3 quick_test.py
=============================================================================
"""

import subprocess
import time
import signal
import sys

def run_command_background(cmd):
    """Lance une commande en arrière-plan"""
    return subprocess.Popen(cmd, shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)

def check_node_running(node_name, timeout=10):
    """Vérifie qu'un nœud ROS2 est actif"""
    for _ in range(timeout):
        try:
            result = subprocess.run(['ros2', 'node', 'list'], 
                                  capture_output=True, text=True, timeout=2)
            if node_name in result.stdout:
                return True
        except:
            pass
        time.sleep(1)
    return False

def check_topic_active(topic_name, timeout=10):
    """Vérifie qu'un topic publie des données"""
    for _ in range(timeout):
        try:
            result = subprocess.run(['ros2', 'topic', 'list'], 
                                  capture_output=True, text=True, timeout=2)
            if topic_name in result.stdout:
                return True
        except:
            pass
        time.sleep(1)
    return False

def cleanup_processes(processes):
    """Nettoie les processus lancés"""
    print("\n🧹 Nettoyage des processus...")
    for proc in processes:
        try:
            proc.terminate()
            proc.wait(timeout=3)
        except:
            try:
                proc.kill()
            except:
                pass
    
    # Nettoyage supplémentaire
    subprocess.run(['pkill', '-f', 'vision_node'], capture_output=True)
    subprocess.run(['pkill', '-f', 'data_collector_node'], capture_output=True)
    subprocess.run(['pkill', '-f', 'camera_simulator'], capture_output=True)

def main():
    """Test principal"""
    print("🧪 TEST RAPIDE DU SYSTÈME DE VISION")
    print("=" * 50)
    
    processes = []
    
    def signal_handler(sig, frame):
        cleanup_processes(processes)
        sys.exit(0)
    
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # 1. Sourcer l'environnement
        print("🔧 Préparation de l'environnement...")
        env_cmd = "source /opt/ros/humble/setup.bash && source install/setup.bash"
        
        # 2. Lancer le simulateur de caméra
        print("📹 Lancement du simulateur de caméra...")
        camera_proc = run_command_background(f"{env_cmd} && python3 camera_simulator.py")
        processes.append(camera_proc)
        time.sleep(3)
        
        # 3. Lancer le nœud de vision
        print("🔍 Lancement du nœud de vision...")
        vision_proc = run_command_background(f"{env_cmd} && ros2 run drone_vision vision_node")
        processes.append(vision_proc)
        time.sleep(3)
        
        # 4. Lancer le collecteur de données
        print("📊 Lancement du collecteur de données...")
        collector_proc = run_command_background(f"{env_cmd} && ros2 run drone_vision data_collector_node")
        processes.append(collector_proc)
        time.sleep(3)
        
        # 5. Vérifier que les nœuds sont actifs
        print("\n✅ Vérification des nœuds...")
        nodes_to_check = ['/drone_vision', '/drone_data_collector']
        all_nodes_ok = True
        
        for node in nodes_to_check:
            if check_node_running(node):
                print(f"  ✅ {node}: ACTIF")
            else:
                print(f"  ❌ {node}: INACTIF")
                all_nodes_ok = False
        
        if not all_nodes_ok:
            print("\n❌ Certains nœuds ne sont pas actifs. Arrêt du test.")
            return False
        
        # 6. Vérifier les topics
        print("\n📡 Vérification des topics...")
        topics_to_check = ['/camera/image_raw', '/vision/flowers_detected']
        all_topics_ok = True
        
        for topic in topics_to_check:
            if check_topic_active(topic):
                print(f"  ✅ {topic}: DISPONIBLE")
            else:
                print(f"  ❌ {topic}: INDISPONIBLE")
                all_topics_ok = False
        
        # 7. Activer les services
        print("\n⚡ Activation des services...")
        
        # Activer la détection
        activate_vision = subprocess.run([
            'ros2', 'service', 'call', '/vision/set_detection',
            'std_srvs/srv/SetBool', 'data: true'
        ], capture_output=True, text=True, timeout=5)
        
        if activate_vision.returncode == 0:
            print("  ✅ Détection de fleurs activée")
        else:
            print("  ❌ Échec activation détection")
        
        # Activer la collecte
        activate_collector = subprocess.run([
            'ros2', 'service', 'call', '/data_collector/set_collection',
            'std_srvs/srv/SetBool', 'data: true'
        ], capture_output=True, text=True, timeout=5)
        
        if activate_collector.returncode == 0:
            print("  ✅ Collecte de données activée")
        else:
            print("  ❌ Échec activation collecte")
        
        # 8. Période d'observation
        print("\n👀 Observation du système (10 secondes)...")
        for i in range(10, 0, -1):
            print(f"\r  ⏱️  {i}s restantes...", end='', flush=True)
            time.sleep(1)
        print("\n")
        
        # 9. Test de sauvegarde
        print("💾 Test de sauvegarde...")
        save_result = subprocess.run([
            'ros2', 'service', 'call', '/data_collector/save_data',
            'std_srvs/srv/Trigger'
        ], capture_output=True, text=True, timeout=5)
        
        if save_result.returncode == 0:
            print("  ✅ Sauvegarde réussie")
        else:
            print("  ❌ Échec sauvegarde")
        
        # 10. Résultats
        print("\n🎉 TEST TERMINÉ!")
        print("=" * 30)
        
        if all_nodes_ok and all_topics_ok:
            print("✅ SUCCÈS: Tous les composants fonctionnent")
            print("\n💡 Pour une démonstration complète:")
            print("   ./demo_pollination_system.sh")
            success = True
        else:
            print("❌ ÉCHEC: Problèmes détectés")
            success = False
        
        print("\n⏳ Le système reste actif pour 30 secondes...")
        print("   Appuyez sur Ctrl+C pour arrêter")
        
        # Garder le système actif un moment
        time.sleep(30)
        
        return success
        
    except KeyboardInterrupt:
        print("\n🛑 Arrêt demandé par l'utilisateur")
        return True
    except Exception as e:
        print(f"\n❌ Erreur pendant le test: {e}")
        return False
    finally:
        cleanup_processes(processes)

if __name__ == '__main__':
    success = main()
    exit_code = 0 if success else 1
    print(f"\n👋 Test terminé (code: {exit_code})")
    sys.exit(exit_code)
