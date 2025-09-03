#!/usr/bin/env python3
"""
=============================================================================
SETUP ENVIRONMENT - Script de configuration de l'environnement drone
=============================================================================
Auteur: Adama Komi
Date: 2025-09-03

Description:
    Script pour configurer automatiquement l'environnement de développement
    et de déploiement pour le système drone autonome.
=============================================================================
"""

import os
import sys
import subprocess
import argparse
import yaml
from pathlib import Path


class EnvironmentSetup:
    """Gestionnaire de configuration de l'environnement"""
    
    def __init__(self):
        self.script_dir = Path(__file__).parent
        self.package_dir = self.script_dir.parent
        self.workspace_dir = self.package_dir.parent.parent.parent
        
    def setup_development_environment(self):
        """Configure l'environnement de développement"""
        print("🔧 Configuration de l'environnement de développement...")
        
        # 1. Vérification des dépendances Python
        self._check_python_dependencies()
        
        # 2. Configuration des hooks Git (si dans un repo)
        self._setup_git_hooks()
        
        # 3. Configuration des alias ROS2
        self._setup_ros2_aliases()
        
        # 4. Création des répertoires de logs
        self._create_log_directories()
        
        print("✅ Environnement de développement configuré")
        
    def setup_production_environment(self):
        """Configure l'environnement de production"""
        print("🚀 Configuration de l'environnement de production...")
        
        # 1. Installation des dépendances système
        self._install_system_dependencies()
        
        # 2. Configuration des services systemd
        self._setup_systemd_services()
        
        # 3. Configuration de la surveillance
        self._setup_monitoring()
        
        # 4. Configuration des sauvegardes
        self._setup_backup()
        
        print("✅ Environnement de production configuré")
        
    def _check_python_dependencies(self):
        """Vérifie les dépendances Python"""
        print("  📦 Vérification des dépendances Python...")
        
        required_packages = [
            'rclpy',
            'mavros_msgs',
            'geometry_msgs',
            'sensor_msgs',
            'std_msgs',
            'diagnostic_msgs',
            'psutil',
            'pyyaml'
        ]
        
        missing_packages = []
        
        for package in required_packages:
            try:
                __import__(package)
                print(f"    ✅ {package}")
            except ImportError:
                print(f"    ❌ {package} (manquant)")
                missing_packages.append(package)
                
        if missing_packages:
            print(f"  ⚠️  Packages manquants: {', '.join(missing_packages)}")
            print("     Installez-les avec: pip install <package>")
        else:
            print("  ✅ Toutes les dépendances Python sont satisfaites")
            
    def _setup_git_hooks(self):
        """Configure les hooks Git"""
        print("  🔧 Configuration des hooks Git...")
        
        git_dir = self.workspace_dir / '.git'
        if not git_dir.exists():
            print("    ℹ️  Pas de repository Git détecté")
            return
            
        hooks_dir = git_dir / 'hooks'
        hooks_dir.mkdir(exist_ok=True)
        
        # Hook pre-commit pour vérifications
        pre_commit_hook = hooks_dir / 'pre-commit'
        hook_content = '''#!/bin/bash
# Pre-commit hook pour drone_interface

echo "🔍 Vérifications pré-commit..."

# Vérification de la syntaxe Python
python3 -m py_compile src/drone_interface/drone_interface/*.py
if [ $? -ne 0 ]; then
    echo "❌ Erreurs de syntaxe Python détectées"
    exit 1
fi

# Vérification du formatage (si black est installé)
if command -v black &> /dev/null; then
    black --check --diff src/drone_interface/
    if [ $? -ne 0 ]; then
        echo "⚠️  Code non formaté - exécutez: black src/drone_interface/"
        # Ne pas bloquer le commit, juste avertir
    fi
fi

echo "✅ Vérifications pré-commit réussies"
'''
        
        pre_commit_hook.write_text(hook_content)
        pre_commit_hook.chmod(0o755)
        print("    ✅ Hook pre-commit configuré")
        
    def _setup_ros2_aliases(self):
        """Configure des alias ROS2 utiles"""
        print("  🎯 Configuration des alias ROS2...")
        
        bashrc_path = Path.home() / '.bashrc'
        
        aliases = [
            "# Alias drone_interface",
            "alias drone_build='cd ~/ros2_ws && colcon build --packages-select drone_interface'",
            "alias drone_test='cd ~/ros2_ws && colcon test --packages-select drone_interface'",
            "alias drone_status='ros2 run drone_interface status'",
            "alias drone_safety='ros2 run drone_interface safety_check'", 
            "alias drone_arm='ros2 run drone_interface arm_drone'",
            "alias drone_diag='ros2 run drone_interface diagnostics'",
            "alias drone_launch='ros2 launch drone_interface drone_interface_launch.py'",
            ""
        ]
        
        # Vérifier si les alias existent déjà
        if bashrc_path.exists():
            content = bashrc_path.read_text()
            if "# Alias drone_interface" in content:
                print("    ℹ️  Alias déjà configurés")
                return
                
        # Ajouter les alias
        with open(bashrc_path, 'a') as f:
            f.write('\n' + '\n'.join(aliases))
            
        print("    ✅ Alias ROS2 ajoutés au .bashrc")
        print("    ℹ️  Exécutez 'source ~/.bashrc' pour les activer")
        
    def _create_log_directories(self):
        """Crée les répertoires de logs"""
        print("  📁 Création des répertoires de logs...")
        
        log_dirs = [
            '/tmp/drone_logs',
            '/tmp/drone_logs/interface',
            '/tmp/drone_logs/safety',
            '/tmp/drone_logs/diagnostics'
        ]
        
        for log_dir in log_dirs:
            Path(log_dir).mkdir(parents=True, exist_ok=True)
            print(f"    ✅ {log_dir}")
            
    def _install_system_dependencies(self):
        """Installe les dépendances système"""
        print("  📦 Installation des dépendances système...")
        
        # Liste des packages système nécessaires
        packages = [
            'ros-humble-mavros',
            'ros-humble-mavros-extras',
            'python3-psutil',
            'htop',
            'iotop'
        ]
        
        for package in packages:
            try:
                result = subprocess.run(['dpkg', '-l', package], 
                                      capture_output=True, text=True)
                if result.returncode == 0:
                    print(f"    ✅ {package} (déjà installé)")
                else:
                    print(f"    📦 Installation de {package}...")
                    subprocess.run(['sudo', 'apt', 'install', '-y', package], 
                                 check=True)
                    print(f"    ✅ {package} installé")
            except subprocess.CalledProcessError:
                print(f"    ❌ Échec de l'installation de {package}")
                
    def _setup_systemd_services(self):
        """Configure les services systemd"""
        print("  ⚙️  Configuration des services systemd...")
        
        service_content = '''[Unit]
Description=Drone Interface Service
After=network.target
StartLimitIntervalSec=0

[Service]
Type=simple
Restart=always
RestartSec=1
User=ubuntu
Environment=ROS_DOMAIN_ID=0
WorkingDirectory=/home/ubuntu/ros2_ws
ExecStart=/bin/bash -c "source /opt/ros/humble/setup.bash && source install/setup.bash && ros2 launch drone_interface drone_interface_launch.py"

[Install]
WantedBy=multi-user.target
'''
        
        service_file = Path('/etc/systemd/system/drone-interface.service')
        
        try:
            service_file.write_text(service_content)
            subprocess.run(['sudo', 'systemctl', 'daemon-reload'], check=True)
            print("    ✅ Service drone-interface configuré")
        except PermissionError:
            print("    ⚠️  Permissions insuffisantes pour configurer systemd")
            print("       Exécutez ce script avec sudo pour la configuration complète")
            
    def _setup_monitoring(self):
        """Configure la surveillance système"""
        print("  📊 Configuration de la surveillance...")
        
        # Script de surveillance simple
        monitor_script = Path('/usr/local/bin/drone-monitor.sh')
        
        script_content = '''#!/bin/bash
# Script de surveillance drone

LOG_FILE="/var/log/drone-monitor.log"

while true; do
    TIMESTAMP=$(date '+%Y-%m-%d %H:%M:%S')
    
    # Vérification du processus
    if ! pgrep -f "drone_interface" > /dev/null; then
        echo "[$TIMESTAMP] ALERT: drone_interface process not running" >> $LOG_FILE
        # Redémarrage automatique
        systemctl restart drone-interface
    fi
    
    # Vérification de la mémoire
    MEM_USAGE=$(ps -o pid,pcpu,pmem,comm -C python3 | grep drone | awk '{print $3}')
    if [ "$MEM_USAGE" -gt "50" ]; then
        echo "[$TIMESTAMP] WARNING: High memory usage: $MEM_USAGE%" >> $LOG_FILE
    fi
    
    sleep 30
done
'''
        
        try:
            monitor_script.write_text(script_content)
            monitor_script.chmod(0o755)
            print("    ✅ Script de surveillance configuré")
        except PermissionError:
            print("    ⚠️  Permissions insuffisantes pour la surveillance")
            
    def _setup_backup(self):
        """Configure les sauvegardes"""
        print("  💾 Configuration des sauvegardes...")
        
        backup_script = Path('/usr/local/bin/drone-backup.sh')
        
        script_content = '''#!/bin/bash
# Script de sauvegarde configuration drone

BACKUP_DIR="/var/backups/drone"
DATE=$(date '+%Y%m%d_%H%M%S')

mkdir -p $BACKUP_DIR

# Sauvegarde de la configuration
tar -czf "$BACKUP_DIR/config_$DATE.tar.gz" /home/ubuntu/ros2_ws/src/drone_interface/config/

# Sauvegarde des logs (derniers 7 jours)
find /tmp/drone_logs -name "*.log" -mtime -7 -exec tar -czf "$BACKUP_DIR/logs_$DATE.tar.gz" {} +

# Nettoyage des anciennes sauvegardes (garde 30 jours)
find $BACKUP_DIR -name "*.tar.gz" -mtime +30 -delete

echo "Backup completed: $DATE"
'''
        
        try:
            backup_script.write_text(script_content)
            backup_script.chmod(0o755)
            print("    ✅ Script de sauvegarde configuré")
        except PermissionError:
            print("    ⚠️  Permissions insuffisantes pour la sauvegarde")


def main():
    """Point d'entrée principal"""
    parser = argparse.ArgumentParser(description='Configuration environnement drone')
    parser.add_argument('--dev', action='store_true', 
                       help='Configuration environnement développement')
    parser.add_argument('--prod', action='store_true',
                       help='Configuration environnement production')
    parser.add_argument('--all', action='store_true',
                       help='Configuration complète')
    
    args = parser.parse_args()
    
    if not any([args.dev, args.prod, args.all]):
        print("❌ Spécifiez --dev, --prod ou --all")
        return 1
        
    setup = EnvironmentSetup()
    
    try:
        if args.dev or args.all:
            setup.setup_development_environment()
            
        if args.prod or args.all:
            setup.setup_production_environment()
            
        print("\n🎉 Configuration terminée avec succès!")
        return 0
        
    except Exception as e:
        print(f"\n❌ Erreur lors de la configuration: {e}")
        return 1


if __name__ == '__main__':
    sys.exit(main())
