#!/usr/bin/env python3
"""
=============================================================================
STATUS CLI TOOL - Outil d'affichage du statut pour drone autonome
=============================================================================
Auteur: Adama Komi
Date: 2025-09-03

Description:
    Outil en ligne de commande pour afficher l'état détaillé du drone
    en temps réel ou en mode instantané.
=============================================================================
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import sys
import time


class StatusTool(Node):
    """Outil CLI pour affichage du statut drone"""
    
    def __init__(self):
        super().__init__('status_tool')
        self.status_received = False
        self.latest_status = None
        
        # Souscription au statut
        self.sub = self.create_subscription(
            String,
            '/drone/status',
            self.status_callback,
            10
        )
        
    def status_callback(self, msg):
        """Callback pour recevoir le statut"""
        self.status_received = True
        try:
            self.latest_status = json.loads(msg.data)
        except json.JSONDecodeError:
            self.latest_status = {"raw_message": msg.data}
            
    def display_status_once(self):
        """Affiche le statut une fois"""
        print("📡 Récupération du statut drone...")
        
        # Attendre le statut pendant 15 secondes
        start_time = time.time()
        timeout = 15.0
        while not self.status_received and time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            
        if not self.status_received:
            print(f"❌ Aucun statut reçu dans les {timeout} secondes")
            print("   Vérifiez que le nœud drone_interface est actif")
            return False
            
        self._print_status()
        return True
        
    def display_status_continuous(self):
        """Affiche le statut en continu"""
        print("📡 Affichage du statut en temps réel...")
        print("   (Appuyez sur Ctrl+C pour quitter)")
        
        try:
            while True:
                rclpy.spin_once(self, timeout_sec=0.1)
                
                if self.status_received:
                    # Nettoyer l'écran
                    print("\033[2J\033[H")  # Clear screen
                    self._print_status()
                    self.status_received = False  # Reset pour la prochaine mise à jour
                    
                time.sleep(0.5)  # Rafraîchissement à 2Hz
                
        except KeyboardInterrupt:
            print("\n🛑 Arrêt de l'affichage")
            
    def _print_status(self):
        """Affiche le statut formaté"""
        if not self.latest_status:
            print("❌ Aucun statut disponible")
            return
            
        print("\n" + "="*60)
        print("🚁 STATUT DRONE - " + time.strftime("%H:%M:%S"))
        print("="*60)
        
        # Statut principal
        state = self.latest_status.get('state', 'UNKNOWN')
        mode = self.latest_status.get('mode', 'UNKNOWN')
        armed = self.latest_status.get('armed', False)
        connected = self.latest_status.get('connected', False)
        
        # Couleurs selon l'état
        state_icon = "🔴" if not connected else ("🟢" if armed else "🟡")
        
        print(f"{state_icon} État: {state} | Mode: {mode}")
        print(f"🔗 Connecté: {'✅' if connected else '❌'} | Armé: {'✅' if armed else '❌'}")
        
        # Position
        if 'position' in self.latest_status:
            pos = self.latest_status['position']
            if isinstance(pos, list) and len(pos) >= 3:
                print(f"📍 Position: X={pos[0]:.2f}m, Y={pos[1]:.2f}m, Z={pos[2]:.2f}m")
                
        # Vitesse
        if 'velocity' in self.latest_status:
            vel = self.latest_status['velocity']
            if isinstance(vel, list) and len(vel) >= 3:
                speed = (vel[0]**2 + vel[1]**2)**0.5
                print(f"💨 Vitesse: {speed:.2f}m/s | Montée: {vel[2]:.2f}m/s")
                
        # Batterie
        battery_v = self.latest_status.get('battery_voltage', 0)
        battery_p = self.latest_status.get('battery_percentage', 0)
        if battery_v > 0:
            battery_icon = "🔋" if battery_p > 50 else ("🪫" if battery_p > 20 else "⚠️")
            print(f"{battery_icon} Batterie: {battery_p:.1f}% ({battery_v:.1f}V)")
            
        # GPS
        gps_fix = self.latest_status.get('gps_fix', False)
        gps_sats = self.latest_status.get('gps_satellites', 0)
        gps_hdop = self.latest_status.get('gps_hdop', 99.99)
        
        gps_icon = "🛰️" if gps_fix else "📡"
        print(f"{gps_icon} GPS: {'Fix' if gps_fix else 'No Fix'} | Sats: {gps_sats} | HDOP: {gps_hdop:.2f}")
        
        # Sécurité
        safety_level = self.latest_status.get('safety_level', 0)
        safety_messages = self.latest_status.get('safety_messages', [])
        
        if safety_level == 0:
            safety_icon = "🟢"
            safety_text = "SAFE"
        elif safety_level == 1:
            safety_icon = "🟡"
            safety_text = "WARNING"
        elif safety_level == 2:
            safety_icon = "🟠"
            safety_text = "CRITICAL"
        else:
            safety_icon = "🔴"
            safety_text = "EMERGENCY"
            
        print(f"{safety_icon} Sécurité: {safety_text}")
        
        if safety_messages:
            print("⚠️  Messages:")
            for msg in safety_messages[:3]:  # Limite à 3 messages
                print(f"   • {msg}")
                
        print("="*60)


def main():
    """Point d'entrée principal"""
    import argparse
    
    parser = argparse.ArgumentParser(description='Outil d\'affichage du statut drone')
    parser.add_argument('--continuous', '-c', action='store_true',
                       help='Affichage continu en temps réel')
    args = parser.parse_args()
    
    rclpy.init()
    
    try:
        tool = StatusTool()
        
        if args.continuous:
            tool.display_status_continuous()
        else:
            success = tool.display_status_once()
            return 0 if success else 1
            
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
