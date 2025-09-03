#!/usr/bin/env python3
"""
=============================================================================
SAFETY CHECK CLI TOOL - Outil de vérification sécurité pour drone autonome
=============================================================================
Auteur: Adama Komi
Date: 2025-09-03

Description:
    Outil en ligne de commande pour effectuer une vérification de sécurité
    complète avant le vol.
=============================================================================
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys


class SafetyCheckTool(Node):
    """Outil CLI pour vérification de sécurité"""
    
    def __init__(self):
        super().__init__('safety_check_tool')
        
        # Client pour le service de sécurité
        self.safety_client = self.create_client(Trigger, '/drone/safety_check')
        
    def check_safety(self):
        """Effectue une vérification de sécurité"""
        print("🔒 Vérification de sécurité en cours...")
        
        if not self.safety_client.wait_for_service(timeout_sec=5.0):
            print("❌ Service de sécurité non disponible")
            print("   Vérifiez que le nœud drone_interface est actif")
            return False
            
        # Appel du service
        request = Trigger.Request()
        
        try:
            future = self.safety_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                response = future.result()
                
                print("\n" + "="*50)
                print("🔒 RAPPORT DE SÉCURITÉ")
                print("="*50)
                
                if response.success:
                    print("✅ SÉCURITÉ: VALIDÉE")
                    print("   Le drone est prêt pour l'armement")
                else:
                    print("❌ SÉCURITÉ: ÉCHEC")
                    print("   Problèmes détectés:")
                    
                print(f"\n📝 Détails: {response.message}")
                print("="*50)
                
                return response.success
            else:
                print("❌ Pas de réponse du service de sécurité")
                return False
                
        except Exception as e:
            print(f"❌ Erreur lors de la vérification: {e}")
            return False


def main():
    """Point d'entrée principal"""
    rclpy.init()
    
    try:
        tool = SafetyCheckTool()
        
        success = tool.check_safety()
        
        if success:
            print("\n🚀 Drone prêt pour l'armement")
            return 0
        else:
            print("\n⛔ Drone NON prêt - Résolvez les problèmes avant de continuer")
            return 1
            
    except KeyboardInterrupt:
        print("\n🛑 Arrêt demandé")
        return 0
    except Exception as e:
        print(f"❌ Erreur: {e}")
        return 1
    finally:
        if 'tool' in locals():
            tool.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
