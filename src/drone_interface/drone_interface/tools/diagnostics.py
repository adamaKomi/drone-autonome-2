#!/usr/bin/env python3
"""
=============================================================================
DIAGNOSTICS CLI TOOL - Outil de diagnostic pour drone autonome
=============================================================================
Auteur: Adama Komi
Date: 2025-09-03

Description:
    Outil en ligne de commande pour effectuer des diagnostics système
    sur l'interface drone.
=============================================================================
"""

import rclpy
from rclpy.node import Node
from diagnostic_msgs.msg import DiagnosticArray
import sys


class DiagnosticsTool(Node):
    """Outil CLI pour diagnostics système"""
    
    def __init__(self):
        super().__init__('diagnostics_tool')
        self.diagnostics_received = False
        
        # Souscription aux diagnostics
        self.sub = self.create_subscription(
            DiagnosticArray,
            '/diagnostics',
            self.diagnostics_callback,
            10
        )
        
    def diagnostics_callback(self, msg):
        """Callback pour afficher les diagnostics"""
        self.diagnostics_received = True
        
        print("\n" + "="*60)
        print("📊 DIAGNOSTICS SYSTÈME DRONE")
        print("="*60)
        
        for status in msg.status:
            # Code couleur selon le niveau
            if status.level == 0:  # OK
                level_str = "✅ OK"
            elif status.level == 1:  # WARN
                level_str = "⚠️  WARN"
            elif status.level == 2:  # ERROR
                level_str = "❌ ERROR"
            else:
                level_str = f"❓ LEVEL_{status.level}"
                
            print(f"\n🔧 {status.name}")
            print(f"   État: {level_str}")
            print(f"   Message: {status.message}")
            
            if status.values:
                print("   Détails:")
                for kv in status.values:
                    print(f"     {kv.key}: {kv.value}")
        
        print("\n" + "="*60)


def main():
    """Point d'entrée principal"""
    rclpy.init()
    
    try:
        tool = DiagnosticsTool()
        
        print("🔍 Attente des diagnostics système...")
        print("   (Appuyez sur Ctrl+C pour quitter)")
        
        # Attendre les diagnostics pendant 10 secondes
        import time
        start_time = time.time()
        
        while not tool.diagnostics_received and time.time() - start_time < 10.0:
            rclpy.spin_once(tool, timeout_sec=0.1)
            
        if not tool.diagnostics_received:
            print("❌ Aucun diagnostic reçu dans les 10 secondes")
            print("   Vérifiez que le nœud drone_interface est actif")
            return 1
            
        return 0
        
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
