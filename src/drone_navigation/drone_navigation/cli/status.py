#!/usr/bin/env python3
"""
Drone Status CLI Tool
Affiche le statut actuel du système de navigation du drone
"""

import rclpy
from rclpy.node import Node
import sys

def main():
    """Point d'entrée principal pour drone-status"""
    
    rclpy.init()
    
    try:
        # Créer un nœud temporaire pour les appels de service
        node = Node('drone_status_temp')
        
        # Attendre un peu pour la découverte des nœuds
        import time
        time.sleep(1.0)
        
        print("🚁 STATUT DU DRONE DE NAVIGATION")
        print("=" * 40)
        
        # Vérifier si le parameter manager est actif
        node_names = node.get_node_names()
        param_manager_active = '/drone_nav/parameter_manager_node' in node_names
        
        print(f"📡 Parameter Manager: {'✅ ACTIF' if param_manager_active else '❌ INACTIF'}")
        print(f"🔍 NŒUDS DÉTECTÉS ({len(node_names)}): {node_names}")
        
        if param_manager_active:
            # Récupérer quelques paramètres clés
            try:
                # Créer un client pour les paramètres
                from rcl_interfaces.srv import GetParameters
                client = node.create_client(GetParameters, '/drone_nav/parameter_manager_node/get_parameters')
                
                if client.wait_for_service(timeout_sec=2.0):
                    request = GetParameters.Request()
                    request.names = [
                        'navigation.max_velocity',
                        'safety.min_altitude',
                        'safety.max_altitude',
                        'algorithms.default_planner'
                    ]
                    
                    future = client.call_async(request)
                    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
                    
                    if future.result():
                        response = future.result()
                        values = response.values
                        
                        print("\n📊 PARAMÈTRES ACTIFS:")
                        print(f"  🚀 Vitesse max: {values[0].double_value} m/s")
                        print(f"  ⬇️ Altitude min: {values[1].double_value} m")
                        print(f"  ⬆️ Altitude max: {values[2].double_value} m") 
                        print(f"  🗺️ Planificateur: {values[3].string_value}")
                    else:
                        print("❌ Erreur lors de la récupération des paramètres")
                else:
                    print("❌ Service de paramètres non disponible")
                    
            except Exception as e:
                print(f"❌ Erreur: {e}")
        
        # Vérifier les autres nœuds
        print(f"\n🔍 NŒUDS DÉTECTÉS ({len(node_names)}):")
        for node_name in sorted(node_names):
            if 'drone_nav' in node_name:
                print(f"  ✅ {node_name}")
        
        print("\n🎯 Utilisez 'ros2 param list /drone_nav/parameter_manager_node' pour voir tous les paramètres")
        
    except Exception as e:
        print(f"❌ Erreur: {e}")
        return 1
    finally:
        try:
            node.destroy_node()
            rclpy.shutdown()
        except:
            pass
    
    return 0

if __name__ == '__main__':
    sys.exit(main())
