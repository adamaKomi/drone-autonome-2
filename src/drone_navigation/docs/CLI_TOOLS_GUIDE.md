# CLI Tools - Guide d'Utilisation des Outils en Ligne de Commande

## Vue d'ensemble

Le package `drone_navigation` fournit une suite complète d'outils en ligne de commande pour le contrôle et la supervision des drones. Ces outils permettent une interaction directe avec le système de navigation sans interface graphique.

## 🛠️ Outils Disponibles

### 1. `drone_goto` - Navigation vers Position

**Usage:** `drone_goto <x> <y> <z> [options]`

**Fonctionnalités:**
- Navigation vers coordonnées spécifiques
- Validation des paramètres de vol
- Mode simulation pour tests
- Logs détaillés des commandes

**Exemples:**
```bash
# Navigation basique
drone_goto 10.0 20.0 5.0

# Navigation avec validation uniquement
drone_goto 10.0 20.0 5.0 --validate

# Navigation avec vitesse spécifique
drone_goto 10.0 20.0 5.0 --speed 2.5

# Mode simulation
drone_goto 10.0 20.0 5.0 --simulate
```

### 2. `drone_status` - Statut Système

**Usage:** `drone_status [options]`

**Fonctionnalités:**
- Affichage statut complet du drone
- Format JSON pour intégration
- Mode surveillance continue
- Statut détaillé par composant

**Exemples:**
```bash
# Statut standard
drone_status

# Statut en format JSON
drone_status --json

# Statut détaillé
drone_status --detailed

# Mode surveillance (actualisation toutes les 2s)
drone_status --watch 2

# Statut spécifique
drone_status --component battery
```

### 3. `drone_mission` - Gestion des Missions

**Usage:** `drone_mission <command> [options]`

**Commandes disponibles:**
- `create` - Créer nouvelle mission
- `load` - Charger mission existante
- `start` - Démarrer mission
- `stop` - Arrêter mission
- `status` - Statut mission actuelle
- `list` - Lister missions disponibles

**Exemples:**
```bash
# Créer mission zigzag
drone_mission create ma_mission.json --pattern zigzag --area "0,0,20,20" --altitude 5.0

# Créer mission spirale
drone_mission create spiral.json --pattern spiral --area "0,0,15,15" --spacing 3.0

# Lister missions disponibles
drone_mission list --directory ./missions

# Charger et valider mission
drone_mission load ma_mission.json --validate

# Démarrer mission avec confirmation
drone_mission start --confirm

# Statut mission en cours
drone_mission status
```

### 4. `drone_diagnostics` - Diagnostics Système

**Usage:** `drone_diagnostics [options]`

**Fonctionnalités:**
- Diagnostics complets système
- Surveillance par composant
- Détection automatique problèmes
- Export des résultats
- Mode surveillance continue

**Exemples:**
```bash
# Diagnostics complets
drone_diagnostics

# Diagnostics détaillés
drone_diagnostics --detailed

# Diagnostics spécifique
drone_diagnostics --component mavros

# Format JSON
drone_diagnostics --json

# Mode surveillance
drone_diagnostics --watch 5

# Export résultats
drone_diagnostics --export diag_report.json

# Filtrage par seuil
drone_diagnostics --threshold error
```

### 5. `drone_emergency` - Gestion des Urgences

**Usage:** `drone_emergency <command> [options]`

**Commandes d'urgence:**
- `land` - Atterrissage d'urgence
- `rtl` - Return To Launch
- `hold` - Maintenir position
- `kill` - Arrêt moteurs (DANGER!)
- `geofence` - Actions géofence
- `status` - Statut urgence
- `override` - Override sécurité

**Exemples:**
```bash
# Atterrissage d'urgence standard
drone_emergency land

# Atterrissage immédiat
drone_emergency land --immediate

# Return to Launch
drone_emergency rtl --altitude 20

# Maintenir position 30 secondes
drone_emergency hold --duration 30

# Statut systèmes urgence
drone_emergency status

# Actions géofence
drone_emergency geofence status
drone_emergency geofence breach
```

## 🔧 CLI Bridge Node

Le nœud `cli_bridge_node.py` fournit l'interface entre les outils CLI et le système ROS2. Il expose tous les services nécessaires pour l'interaction avec les micro-nœuds.

**Fonctionnalités:**
- Interface unifiée pour tous les outils CLI
- Gestion des services ROS2
- Validation des commandes
- Logs centralisés
- Mode direct et service

## 🚀 Lancement Rapide

### 1. Démarrage du Système Complet

```bash
# Lancement orchestré complet
ros2 launch drone_navigation navigation_complete.launch.py

# Lancement minimal pour tests
ros2 launch drone_navigation navigation_test.launch.py
```

### 2. Test des Outils CLI

```bash
# Script de démonstration
python3 demo_cli_tools.py

# Tests individuels
drone_status
drone_diagnostics
drone_mission list
```

### 3. Mission Typique

```bash
# 1. Vérification système
drone_diagnostics --detailed

# 2. Statut initial
drone_status

# 3. Création mission
drone_mission create ma_mission.json --pattern zigzag --area "0,0,50,50"

# 4. Validation mission
drone_mission load ma_mission.json --validate

# 5. Démarrage mission
drone_mission start

# 6. Surveillance
drone_status --watch 3

# 7. Arrêt si nécessaire
drone_emergency land
```

## 📋 Codes de Retour

Tous les outils CLI suivent les conventions standard Unix:
- `0`: Succès
- `1`: Erreur générale
- `2`: Erreur paramètres
- `130`: Interruption utilisateur (Ctrl+C)

## 🔐 Sécurité

### Commandes Sensibles
- `drone_emergency kill` nécessite confirmation explicite
- Override sécurité nécessite code d'authentification
- Validation automatique des paramètres de vol

### Logs de Sécurité
- Toutes les commandes d'urgence sont loggées
- Horodatage et traçabilité complète
- Export possible pour audit

## 🛠️ Développement et Extension

### Ajout de Nouvelles Commandes

1. **Étendre CLI Bridge:**
```python
# Dans cli_bridge_node.py
def handle_new_command(self, request, response):
    # Logique de traitement
    return response
```

2. **Créer Script CLI:**
```python
#!/usr/bin/env python3
def main():
    # Interface utilisateur
    # Appel services ROS2
    pass
```

3. **Rendre Exécutable:**
```bash
chmod +x nouveau_script
```

### Tests et Validation

```bash
# Test avec démonstrateur
python3 demo_cli_tools.py

# Tests unitaires
ros2 run drone_navigation test_cli_tools.py

# Validation dans SITL
drone_diagnostics --component mavros
```

## 📚 Intégration avec Autres Systèmes

### Scripts Bash
```bash
#!/bin/bash
# Mission automatisée
drone_status --json | jq '.battery.remaining' > /tmp/battery_level
if [ $(cat /tmp/battery_level) -lt 20 ]; then
    drone_emergency land
fi
```

### Python Scripts
```python
import subprocess
import json

result = subprocess.run(['drone_status', '--json'], capture_output=True, text=True)
status = json.loads(result.stdout)
print(f"Battery: {status['battery']['remaining']}%")
```

### Monitoring Systems
```bash
# Supervision continue
drone_diagnostics --watch 10 --export /var/log/drone_diag.json &

# Alerte automatique
drone_status --json | jq '.system.cpu_usage' | 
  awk '{ if ($1 > 80) system("drone_emergency hold --duration 60") }'
```

## 🎯 Cas d'Usage Avancés

### 1. Pipeline de Mission Complète
```bash
#!/bin/bash
set -e

echo "🚁 Démarrage pipeline mission"

# Vérifications préalables
drone_diagnostics --threshold error || exit 1
drone_status | grep -q "READY" || exit 1

# Création mission adaptative
WEATHER=$(curl -s "api.weather.com/current" | jq '.wind_speed')
if [ $WEATHER -gt 10 ]; then
    SPACING=3.0
else
    SPACING=2.0
fi

drone_mission create auto_mission.json --spacing $SPACING

# Exécution avec surveillance
drone_mission load auto_mission.json
drone_mission start &
MISSION_PID=$!

# Surveillance continue
while kill -0 $MISSION_PID 2>/dev/null; do
    drone_status --json | jq '.battery.remaining' | 
      awk '{ if ($1 < 25) { system("drone_emergency rtl"); exit } }'
    sleep 5
done

echo "✅ Mission terminée"
```

### 2. Système de Surveillance Distribute
```bash
# Station sol 1: Surveillance système
drone_diagnostics --watch 5 --export /shared/diag.json &

# Station sol 2: Surveillance mission
while true; do
    drone_mission status | tee /shared/mission_status.log
    sleep 10
done &

# Station sol 3: Surveillance urgence
drone_emergency status | grep -q "EMERGENCY" && 
  echo "ALERT" > /shared/emergency_alert.flag
```

## 📖 Référence Complète

### Variables d'Environnement
- `DRONE_CLI_TIMEOUT`: Timeout commandes (défaut: 30s)
- `DRONE_CLI_LOG_LEVEL`: Niveau logs (info/warn/error)
- `DRONE_CLI_CONFIG_PATH`: Chemin configuration personnalisée

### Fichiers de Configuration
- `/config/drone_config.yaml`: Configuration principale
- `~/.drone_cli_config.yaml`: Configuration utilisateur
- `/tmp/drone_emergency.log`: Logs urgences

### Formats de Sortie JSON

**drone_status --json:**
```json
{
  "timestamp": "2024-01-15T10:30:00Z",
  "system": {"status": "OK", "cpu_usage": 45.2},
  "battery": {"remaining": 85, "voltage": 12.6},
  "position": {"lat": 45.764, "lon": 4.836, "alt": 50.0},
  "navigation": {"mode": "AUTO", "mission_active": true}
}
```

**drone_diagnostics --json:**
```json
{
  "timestamp": "2024-01-15T10:30:00Z",
  "system": {"status": "OK", "issues": []},
  "mavros": {"status": "CONNECTED", "flight_mode": "GUIDED"},
  "sensors": {"gps": {"satellites": 12, "hdop": 1.2}}
}
```

Cette documentation complète permet une utilisation efficace de tous les outils CLI du système de navigation drone. Les exemples pratiques et cas d'usage avancés facilitent l'intégration dans des workflows opérationnels complexes.
