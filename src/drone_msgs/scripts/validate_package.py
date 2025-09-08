#!/usr/bin/env python3
"""
Script de configuration et validation pour le package drone_msgs
"""

import os
import sys
import subprocess
import json
from pathlib import Path


class DroneMessageSetup:
    """Classe pour configurer et valider le package drone_msgs"""
    
    def __init__(self):
        self.package_path = Path(__file__).parent.parent  # Remonter au niveau du package
        self.msg_path = self.package_path / "msg"
        self.srv_path = self.package_path / "srv"
        self.action_path = self.package_path / "action"
        
    def validate_structure(self):
        """Valide la structure du package"""
        print("🔍 Validation de la structure du package...")
        
        required_dirs = [self.msg_path, self.srv_path, self.action_path]
        for dir_path in required_dirs:
            if not dir_path.exists():
                print(f"❌ Répertoire manquant: {dir_path}")
                return False
            print(f"✅ Répertoire trouvé: {dir_path}")
        
        return True
    
    def count_files(self):
        """Compte les fichiers de messages, services et actions"""
        print("\n📊 Comptage des fichiers...")
        
        msg_files = list(self.msg_path.glob("*.msg"))
        srv_files = list(self.srv_path.glob("*.srv"))
        action_files = list(self.action_path.glob("*.action"))
        
        print(f"📝 Messages: {len(msg_files)}/23")
        print(f"🔧 Services: {len(srv_files)}/21")
        print(f"⚡ Actions: {len(action_files)}/9")
        
        # Liste des fichiers attendus
        expected_messages = [
            "DroneStatus", "DroneState", "FlightMode", "SafetyStatus", "BatteryStatus",
            "Position3D", "Orientation", "Velocity3D", "NavigationCommand", "Waypoint",
            "Trajectory", "FlowerDetection", "FlowerClassification", "FlowerTarget",
            "PollinationResult", "MissionStatus", "MissionProgress", "MissionWaypoint",
            "ZoneDefinition", "EnvironmentData", "PerformanceMetrics", "SystemAlert",
            "DiagnosticInfo"
        ]
        
        expected_services = [
            "ArmDrone", "DisarmDrone", "SetFlightMode", "Takeoff", "Land", "ReturnToLaunch",
            "EmergencyStop", "LoadMission", "StartMission", "StopMission", "PauseMission",
            "SetWaypoint", "SetGeofence", "CalibrateSensors", "SystemDiagnostic",
            "UpdateParameters", "GetSystemStatus", "ConfigureCamera", "DetectFlowers",
            "ExecutePollination", "SaveData"
        ]
        
        expected_actions = [
            "ExecuteMission", "NavigateToPosition", "FollowTrajectory", "SearchAndPollinate",
            "MonitorZone", "CollectData", "MapArea", "PerformMaintenance", "EmergencyLanding"
        ]
        
        # Vérification des fichiers manquants
        missing_messages = [name for name in expected_messages 
                          if not (self.msg_path / f"{name}.msg").exists()]
        missing_services = [name for name in expected_services 
                          if not (self.srv_path / f"{name}.srv").exists()]
        missing_actions = [name for name in expected_actions 
                         if not (self.action_path / f"{name}.action").exists()]
        
        if missing_messages:
            print(f"❌ Messages manquants: {missing_messages}")
        if missing_services:
            print(f"❌ Services manquants: {missing_services}")
        if missing_actions:
            print(f"❌ Actions manquantes: {missing_actions}")
        
        all_complete = not (missing_messages or missing_services or missing_actions)
        if all_complete:
            print("✅ Tous les fichiers requis sont présents")
        
        return all_complete
    
    def validate_syntax(self):
        """Valide la syntaxe des fichiers de définition"""
        print("\n🔍 Validation de la syntaxe...")
        
        valid = True
        
        # Validation basique des fichiers .msg
        for msg_file in self.msg_path.glob("*.msg"):
            try:
                with open(msg_file, 'r') as f:
                    content = f.read()
                    if not content.strip():
                        print(f"❌ Fichier vide: {msg_file.name}")
                        valid = False
                    elif "std_msgs/Header header" not in content:
                        print(f"⚠️ Header manquant dans: {msg_file.name}")
            except Exception as e:
                print(f"❌ Erreur lecture {msg_file.name}: {e}")
                valid = False
        
        # Validation basique des fichiers .srv
        for srv_file in self.srv_path.glob("*.srv"):
            try:
                with open(srv_file, 'r') as f:
                    content = f.read()
                    if "---" not in content:
                        print(f"❌ Séparateur '---' manquant dans: {srv_file.name}")
                        valid = False
            except Exception as e:
                print(f"❌ Erreur lecture {srv_file.name}: {e}")
                valid = False
        
        # Validation basique des fichiers .action
        for action_file in self.action_path.glob("*.action"):
            try:
                with open(action_file, 'r') as f:
                    content = f.read()
                    if content.count("---") != 2:
                        print(f"❌ Structure action incorrecte dans: {action_file.name}")
                        valid = False
            except Exception as e:
                print(f"❌ Erreur lecture {action_file.name}: {e}")
                valid = False
        
        if valid:
            print("✅ Syntaxe validée pour tous les fichiers")
        
        return valid
    
    def check_dependencies(self):
        """Vérifie les dépendances dans package.xml"""
        print("\n📦 Vérification des dépendances...")
        
        package_xml = self.package_path / "package.xml"
        if not package_xml.exists():
            print("❌ package.xml non trouvé")
            return False
        
        with open(package_xml, 'r') as f:
            content = f.read()
        
        required_deps = [
            "std_msgs", "geometry_msgs", "sensor_msgs", 
            "builtin_interfaces", "diagnostic_msgs", "nav_msgs"
        ]
        
        missing_deps = []
        for dep in required_deps:
            if dep not in content:
                missing_deps.append(dep)
        
        if missing_deps:
            print(f"❌ Dépendances manquantes: {missing_deps}")
            return False
        
        print("✅ Toutes les dépendances sont présentes")
        return True
    
    def generate_summary(self):
        """Génère un résumé du package"""
        print("\n📋 Génération du résumé...")
        
        summary = {
            "package_name": "drone_msgs",
            "version": "1.0.0",
            "description": "Messages, services et actions pour drone de pollinisation",
            "messages": len(list(self.msg_path.glob("*.msg"))),
            "services": len(list(self.srv_path.glob("*.srv"))),
            "actions": len(list(self.action_path.glob("*.action"))),
            "files": {
                "messages": [f.stem for f in self.msg_path.glob("*.msg")],
                "services": [f.stem for f in self.srv_path.glob("*.srv")],
                "actions": [f.stem for f in self.action_path.glob("*.action")]
            }
        }
        
        summary_file = self.package_path / "package_summary.json"
        with open(summary_file, 'w') as f:
            json.dump(summary, f, indent=2)
        
        print(f"✅ Résumé généré: {summary_file}")
        return summary
    
    def run_validation(self):
        """Execute la validation complète"""
        print("🚀 Validation du package drone_msgs\n")
        
        checks = [
            ("Structure", self.validate_structure),
            ("Fichiers", self.count_files),
            ("Syntaxe", self.validate_syntax),
            ("Dépendances", self.check_dependencies)
        ]
        
        results = []
        for name, check_func in checks:
            try:
                result = check_func()
                results.append((name, result))
            except Exception as e:
                print(f"❌ Erreur dans {name}: {e}")
                results.append((name, False))
        
        # Résumé final
        print("\n" + "="*50)
        print("📊 RÉSUMÉ DE LA VALIDATION")
        print("="*50)
        
        all_passed = True
        for name, result in results:
            status = "✅ PASS" if result else "❌ FAIL"
            print(f"{name:15} : {status}")
            if not result:
                all_passed = False
        
        print("="*50)
        if all_passed:
            print("🎉 VALIDATION RÉUSSIE - Package prêt pour compilation")
            self.generate_summary()
        else:
            print("⚠️ VALIDATION ÉCHOUÉE - Corrections nécessaires")
        
        return all_passed


if __name__ == "__main__":
    setup = DroneMessageSetup()
    success = setup.run_validation()
    sys.exit(0 if success else 1)
