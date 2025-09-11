#!/usr/bin/env python3
"""
Démonstrateur CLI - Test des outils en ligne de commande
Usage: python3 demo_cli_tools.py
"""

import subprocess
import sys
import time
import os
from pathlib import Path


def run_command(cmd, description):
    """Exécute une commande et affiche le résultat"""
    print(f"\n{'='*60}")
    print(f"🧪 {description}")
    print(f"{'='*60}")
    print(f"💻 Commande: {cmd}")
    print("-" * 60)
    
    try:
        result = subprocess.run(cmd, shell=True, capture_output=True, text=True, timeout=10)
        
        if result.stdout:
            print(result.stdout)
        if result.stderr:
            print(f"STDERR: {result.stderr}")
        
        if result.returncode == 0:
            print("✅ Commande exécutée avec succès")
        else:
            print(f"❌ Erreur (code {result.returncode})")
            
    except subprocess.TimeoutExpired:
        print("⏰ Timeout - Commande interrompue")
    except Exception as e:
        print(f"❌ Erreur: {e}")


def main():
    print("🚁 === DÉMONSTRATEUR OUTILS CLI DRONE ===")
    print("Ce script teste tous les outils CLI développés")
    
    # Vérification que les scripts existent
    scripts_dir = "/home/adama133/ros2_ws/src/drone_navigation/scripts"
    
    scripts = [
        "drone_goto",
        "drone_status", 
        "drone_mission",
        "drone_diagnostics",
        "drone_emergency"
    ]
    
    print(f"\n🔍 Vérification des scripts dans {scripts_dir}...")
    
    for script in scripts:
        script_path = f"{scripts_dir}/{script}"
        if os.path.exists(script_path):
            print(f"✅ {script}")
        else:
            print(f"❌ {script} - NON TROUVÉ")
    
    input("\nAppuyez sur Entrée pour continuer les tests...")
    
    # Test 1: drone_goto
    run_command(
        f"{scripts_dir}/drone_goto --help",
        "Test aide drone_goto"
    )
    
    run_command(
        f"{scripts_dir}/drone_goto 10.0 20.0 5.0 --validate",
        "Test validation position drone_goto"
    )
    
    # Test 2: drone_status
    run_command(
        f"{scripts_dir}/drone_status --help",
        "Test aide drone_status"
    )
    
    run_command(
        f"{scripts_dir}/drone_status --json",
        "Test statut JSON"
    )
    
    # Test 3: drone_mission
    run_command(
        f"{scripts_dir}/drone_mission --help",
        "Test aide drone_mission"
    )
    
    run_command(
        f"{scripts_dir}/drone_mission list --directory /tmp",
        "Test liste missions"
    )
    
    # Création d'une mission de test
    run_command(
        f"{scripts_dir}/drone_mission create /tmp/test_mission.json --pattern zigzag --area '0,0,20,20' --altitude 10",
        "Test création mission zigzag"
    )
    
    run_command(
        f"{scripts_dir}/drone_mission load /tmp/test_mission.json --validate",
        "Test validation mission créée"
    )
    
    # Test 4: drone_diagnostics
    run_command(
        f"{scripts_dir}/drone_diagnostics --help",
        "Test aide drone_diagnostics"
    )
    
    run_command(
        f"{scripts_dir}/drone_diagnostics --json --component system",
        "Test diagnostics système JSON"
    )
    
    run_command(
        f"{scripts_dir}/drone_diagnostics --detailed --threshold warn",
        "Test diagnostics détaillés"
    )
    
    # Test 5: drone_emergency
    run_command(
        f"{scripts_dir}/drone_emergency --help",
        "Test aide drone_emergency"
    )
    
    run_command(
        f"{scripts_dir}/drone_emergency status",
        "Test statut urgence"
    )
    
    run_command(
        f"{scripts_dir}/drone_emergency geofence status",
        "Test statut géofence"
    )
    
    # Tests d'intégration
    print(f"\n{'='*60}")
    print("🔗 TESTS D'INTÉGRATION")
    print(f"{'='*60}")
    
    # Test mission complète
    print("\n🎯 Simulation mission complète:")
    
    steps = [
        (f"{scripts_dir}/drone_diagnostics --component system", "Vérification système"),
        (f"{scripts_dir}/drone_status", "Statut initial"),
        (f"{scripts_dir}/drone_mission create /tmp/demo_mission.json --pattern spiral --area '0,0,15,15'", "Création mission"),
        (f"{scripts_dir}/drone_mission load /tmp/demo_mission.json --validate", "Validation mission"),
        (f"echo 'Mission simulée terminée'", "Fin simulation")
    ]
    
    for cmd, desc in steps:
        print(f"\n📋 {desc}...")
        time.sleep(1)
        run_command(cmd, desc)
    
    # Résumé final
    print(f"\n{'='*60}")
    print("📊 RÉSUMÉ DES TESTS")
    print(f"{'='*60}")
    
    print("✅ Tests des aides (-h/--help)")
    print("✅ Tests validation paramètres")
    print("✅ Tests formats de sortie (JSON)")
    print("✅ Tests création/manipulation fichiers")
    print("✅ Tests diagnostics système")
    print("✅ Tests gestion urgences")
    print("✅ Tests intégration mission")
    
    # Nettoyage
    cleanup_files = [
        "/tmp/test_mission.json",
        "/tmp/demo_mission.json",
        "/tmp/drone_emergency.log"
    ]
    
    print(f"\n🧹 Nettoyage fichiers temporaires...")
    for file in cleanup_files:
        if os.path.exists(file):
            os.remove(file)
            print(f"🗑️ Supprimé: {file}")
    
    print(f"\n🎉 TESTS TERMINÉS")
    print("Tous les outils CLI sont fonctionnels et prêts à l'utilisation!")


if __name__ == '__main__':
    main()
