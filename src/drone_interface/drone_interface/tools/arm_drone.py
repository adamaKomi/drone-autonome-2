#!/usr/bin/env python3
"""
=============================================================================
ARM DRONE CLI TOOL - Outil d'armement pour drone autonome
=============================================================================
Auteur: Adama Komi
Date: 2025-09-03

Description:
    Outil en ligne de commande pour armer le drone de façon sécurisée
    avec vérifications préalables.
=============================================================================
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import sys


class ArmDroneTool(Node):
    """Outil CLI pour armement sécurisé du drone"""
    
    def __init__(self):
        super().__init__('arm_drone_tool')
        
        # Clients pour les services
        self.safety_client = self.create_client(Trigger, '/drone/safety_check')
        self.arm_client = self.create_client(Trigger, '/drone/arm')
        
    def arm_drone(self, force: bool = False):
        """Arme le drone avec vérifications préalables"""
        print("🚀 Procédure d'armement du drone")
        print("="*40)
        
        # Vérification des services
        if not self._wait_for_services():
            return False
            
        # Vérification de sécurité sauf si forcé
        if not force:
            print("\n1️⃣ Vérification de sécurité...")
            if not self._check_safety():
                print("❌ Échec de la vérification de sécurité")
                print("   Utilisez --force pour ignorer (non recommandé)")
                return False
            print("✅ Sécurité validée")
        else:
            print("⚠️  Vérification de sécurité ignorée (mode force)")
            
        # Armement
        print("\n2️⃣ Armement en cours...")
        return self._arm_drone()
        
    def _wait_for_services(self):
        """Attend la disponibilité des services"""
        print("⏳ Vérification des services...")
        
        services = [
            (self.safety_client, "safety_check"),
            (self.arm_client, "arm")
        ]
        
        for client, name in services:
            if not client.wait_for_service(timeout_sec=5.0):
                print(f"❌ Service '{name}' non disponible")
                return False
                
        print("✅ Tous les services sont disponibles")
        return True
        
    def _check_safety(self):
        """Effectue la vérification de sécurité"""
        try:
            request = Trigger.Request()
            future = self.safety_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                response = future.result()
                if not response.success:
                    print(f"   Problèmes: {response.message}")
                return response.success
            else:
                print("❌ Pas de réponse du service de sécurité")
                return False
                
        except Exception as e:
            print(f"❌ Erreur lors de la vérification: {e}")
            return False
            
    def _arm_drone(self):
        """Effectue l'armement du drone"""
        try:
            request = Trigger.Request()
            future = self.arm_client.call_async(request)
            rclpy.spin_until_future_complete(self, future, timeout_sec=15.0)
            
            if future.result() is not None:
                response = future.result()
                
                if response.success:
                    print("✅ DRONE ARMÉ AVEC SUCCÈS")
                    print("🚁 Le drone est maintenant prêt pour le décollage")
                else:
                    print("❌ ÉCHEC DE L'ARMEMENT")
                    print(f"   Raison: {response.message}")
                    
                return response.success
            else:
                print("❌ Pas de réponse du service d'armement")
                return False
                
        except Exception as e:
            print(f"❌ Erreur lors de l'armement: {e}")
            return False


def main():
    """Point d'entrée principal"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Outil d\'armement sécurisé du drone')
    parser.add_argument('--force', action='store_true', 
                       help='Force l\'armement sans vérifications de sécurité')
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        tool = ArmDroneTool()
        
        success = tool.arm_drone(force=args.force)
        
        print("\n" + "="*40)
        if success:
            print("🎉 Armement réussi")
            return 0
        else:
            print("💥 Armement échoué")
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
