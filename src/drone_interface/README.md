# Drone Interface Package

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.8+-green.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Interface ROS2 robuste et modulaire pour le contrôle de drones ArduPilot via MAVROS avec gestion avancée de la sécurité et surveillance système.

## 🚀 Caractéristiques

- **Architecture modulaire** : Séparation claire des responsabilités (Safety, State, Health)
- **Lifecycle management** : Gestion complète des états ROS2 Lifecycle
- **Sécurité avancée** : Vérifications pré-vol, monitoring continu, gestion d'urgence
- **Threading sécurisé** : Gestion thread-safe avec timeouts appropriés
- **Diagnostics intégrés** : Surveillance système et reporting détaillé
- **Outils CLI** : Suite d'outils en ligne de commande pour opérations courantes
- **Configuration flexible** : Paramètres YAML pour adaptation aux besoins

## 📁 Structure du Package

```
drone_interface/
├── drone_interface/                 # Package Python principal
│   ├── __init__.py
│   ├── interface_node.py           # Nœud principal
│   ├── safety_manager.py           # Gestionnaire de sécurité
│   ├── state_manager.py            # Gestionnaire d'état
│   ├── health_monitor.py           # Moniteur de santé
│   └── tools/                      # Outils CLI
│       ├── __init__.py
│       ├── diagnostics.py
│       ├── safety_check.py
│       ├── arm_drone.py
│       └── status.py
├── launch/                         # Fichiers de lancement
│   └── drone_interface_launch.py
├── config/                         # Fichiers de configuration
│   ├── default_params.yaml
│   └── safety_limits.yaml
├── test/                           # Tests
│   ├── test_interface.py
│   └── test_safety.py
├── scripts/                        # Scripts utilitaires
│   └── setup_environment.py
├── resource/                       # Ressources du package
│   └── drone_interface
├── setup.py                       # Configuration Python
├── package.xml                    # Métadonnées ROS2
└── README.md                      # Documentation
```

## 🛠️ Installation

### Prérequis

- ROS2 Humble
- Python 3.8+
- MAVROS
- ArduPilot SITL (pour simulation)

### Installation des dépendances

```bash
# Dépendances ROS2
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# Dépendances Python
pip install psutil pyyaml
```

### Compilation

```bash
cd ~/ros2_ws
colcon build --packages-select drone_interface
source install/setup.bash
```

## 🚁 Utilisation

### 1. Démarrage du système

```bash
# Terminal 1: Lancer MAVROS
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555

# Terminal 2: Lancer l'interface drone
ros2 launch drone_interface drone_interface_launch.py

# Terminal 3: Configurer et activer le nœud
ros2 lifecycle set /drone_interface configure
ros2 lifecycle set /drone_interface activate
```

### 2. Utilisation des outils CLI

```bash
# Vérification de statut
ros2 run drone_interface drone_status

# Vérification de sécurité
ros2 run drone_interface drone_safety_check

# Armement sécurisé
ros2 run drone_interface drone_arm

# Diagnostics système
ros2 run drone_interface drone_diagnostics

# Statut en temps réel
ros2 run drone_interface drone_status --continuous
```

### 3. Services disponibles

| Service | Type | Description |
|---------|------|-------------|
| `/drone/arm` | `std_srvs/Trigger` | Armement sécurisé |
| `/drone/disarm` | `std_srvs/Trigger` | Désarmement |
| `/drone/set_mode` | `mavros_msgs/SetMode` | Changement de mode |
| `/drone/safety_check` | `std_srvs/Trigger` | Vérification sécurité |
| `/drone/health_check` | `std_srvs/Trigger` | État de santé |
| `/drone/emergency_stop` | `std_srvs/Trigger` | Arrêt d'urgence |

### 4. Topics publiés

| Topic | Type | Description |
|-------|------|-------------|
| `/drone/status` | `std_msgs/String` | État détaillé du drone (JSON) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Diagnostics système |
| `/drone/safety_status` | `std_msgs/String` | Événements de sécurité |

## ⚙️ Configuration

### Paramètres de sécurité

Éditez `config/safety_limits.yaml` pour ajuster les limites :

```yaml
safety:
  min_battery_voltage: 10.5        # Tension minimale (V)
  min_battery_percentage: 20.0     # Pourcentage minimal (%)
  max_altitude: 120.0              # Altitude maximale (m)
  min_gps_satellites: 6            # Satellites GPS minimaux
```

### Paramètres de performance

Configurez `config/default_params.yaml` :

```yaml
performance:
  status_rate: 10.0                # Fréquence publication statut (Hz)
  health_check_rate: 1.0           # Fréquence vérifications santé (Hz)
  diagnostics_rate: 0.5            # Fréquence diagnostics (Hz)
```

## 🔒 Système de Sécurité

### Vérifications pré-armement

- Connexion MAVROS établie
- Fix GPS avec qualité suffisante
- Niveau de batterie acceptable
- Mode GUIDED activé
- Communication récente avec l'autopilot

### Monitoring en vol

- Surveillance batterie avec prédiction de tendance
- Détection perte GPS
- Contrôle altitude maximale
- Surveillance communication
- Détection pannes multiples

### Actions d'urgence

- Désarmement automatique en cas critique
- Atterrissage d'urgence batterie faible
- Return-to-Launch (RTL) en cas de perte communication

## 🧪 Tests

```bash
# Tests unitaires
cd ~/ros2_ws
colcon test --packages-select drone_interface

# Tests de sécurité spécifiques
python3 src/drone_interface/test/test_safety.py

# Tests d'intégration
python3 src/drone_interface/test/test_interface.py
```

## 📊 Monitoring et Diagnostics

### Surveillance système

```bash
# Surveillance continue
ros2 run drone_interface drone_diagnostics

# Vérification ponctuelle
ros2 service call /drone/health_check std_srvs/srv/Trigger
```

### Logs

Les logs sont disponibles dans `/tmp/drone_logs/` avec séparation par composant :

- `interface/` : Logs du nœud principal
- `safety/` : Logs de sécurité
- `diagnostics/` : Logs de diagnostic

## 🛠️ Développement

### Configuration environnement

```bash
# Configuration développement
python3 scripts/setup_environment.py --dev

# Configuration production
sudo python3 scripts/setup_environment.py --prod
```

### Architecture

Le package suit une architecture modulaire :

- **SafetyManager** : Gestion des vérifications de sécurité
- **StateManager** : Maintien de l'état du drone
- **HealthMonitor** : Surveillance de la santé système
- **DroneInterface** : Nœud principal avec lifecycle management

## 🚨 Dépannage

### Problèmes courants

**Service MAVROS non disponible**
```bash
# Vérifier MAVROS
ros2 topic list | grep mavros
ros2 service list | grep mavros
```

**Échec d'armement**
```bash
# Vérifier sécurité
ros2 run drone_interface drone_safety_check
```

**Perte de communication**
```bash
# Vérifier connexion
ros2 topic echo /mavros/state
```

## 📝 Changelog

### Version 3.0.0
- Architecture modulaire complète
- Système de sécurité avancé
- Outils CLI intégrés
- Lifecycle management
- Tests complets

## 👥 Auteurs

- **Adama Komi** - *Développement initial* - [adamaKomi](https://github.com/adamaKomi)
- **Détection de Tendances** : Analyse prédictive de l'état de la batterie
- **Niveaux de Sécurité** : Classification des risques (SAFE/WARNING/CRITICAL/EMERGENCY)

### Outils CLI Intégrés
- **drone_status** : Affichage de l'état du drone en temps réel
- **drone_safety_check** : Vérification complète de sécurité
- **drone_arm** : Armement sécurisé avec vérifications
- **drone_diagnostics** : Diagnostics système détaillés

## 📁 Structure du Package

```
drone_interface/
├── drone_interface/                 # Package Python principal
│   ├── __init__.py
│   ├── interface_node.py           # Nœud principal
│   ├── safety_manager.py           # Gestionnaire de sécurité
│   ├── state_manager.py            # Gestionnaire d'état
│   ├── health_monitor.py           # Moniteur de santé
│   └── tools/                      # Outils CLI
│       ├── __init__.py
│       ├── diagnostics.py
│       ├── safety_check.py
│       ├── arm_drone.py
│       └── status.py
├── launch/                         # Fichiers de lancement
│   └── drone_interface_launch.py
├── config/                         # Fichiers de configuration
│   ├── default_params.yaml
│   └── safety_limits.yaml
├── test/                           # Tests
│   ├── test_interface.py
│   └── test_safety.py
├── scripts/                        # Scripts utilitaires
│   └── setup_environment.py
├── resource/                       # Ressources du package
│   └── drone_interface            # Fichier marqueur (vide)
├── setup.py                       # Configuration Python
├── package.xml                    # Métadonnées ROS2
└── README.md                      # Documentation
```

## 🛠️ Installation

### Prérequis

- ROS2 Humble
- Python 3.8+
- MAVROS
- ArduPilot SITL ou drone physique

### Installation des Dépendances

```bash
# Dépendances système
sudo apt update
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# Dépendances Python
pip install psutil pyyaml
```

### Compilation

```bash
cd ~/ros2_ws
colcon build --packages-select drone_interface
source install/setup.bash
```

### Configuration Automatique

```bash
# Configuration environnement de développement
ros2 run drone_interface setup_environment --dev

# Configuration environnement de production
ros2 run drone_interface setup_environment --prod
```

## 🚁 Utilisation

### Démarrage du Système

1. **Lancement de MAVROS** :
```bash
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555
```

2. **Lancement de l'Interface Drone** :
```bash
ros2 launch drone_interface drone_interface_launch.py
```

3. **Configuration du Nœud** :
```bash
ros2 lifecycle set /drone_interface configure
ros2 lifecycle set /drone_interface activate
```

### Utilisation des Outils CLI

#### Vérification de l'État
```bash
# Affichage unique
ros2 run drone_interface drone_status

# Affichage continu
ros2 run drone_interface drone_status --continuous
```

#### Vérification de Sécurité
```bash
ros2 run drone_interface drone_safety_check
```

#### Armement Sécurisé
```bash
# Armement avec vérifications
ros2 run drone_interface drone_arm

# Armement forcé (non recommandé)
ros2 run drone_interface drone_arm --force
```

#### Diagnostics Système
```bash
ros2 run drone_interface drone_diagnostics
```

### Services Disponibles

| Service | Type | Description |
|---------|------|-------------|
| `/drone/arm` | `std_srvs/Trigger` | Armement sécurisé |
| `/drone/disarm` | `std_srvs/Trigger` | Désarmement |
| `/drone/set_mode` | `mavros_msgs/SetMode` | Changement de mode |
| `/drone/safety_check` | `std_srvs/Trigger` | Vérification sécurité |
| `/drone/health_check` | `std_srvs/Trigger` | État de santé |
| `/drone/emergency_stop` | `std_srvs/Trigger` | Arrêt d'urgence |

### Topics Publiés

| Topic | Type | Description |
|-------|------|-------------|
| `/drone/status` | `std_msgs/String` | État détaillé (JSON) |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | Diagnostics système |
| `/drone/safety_status` | `std_msgs/String` | Événements sécurité |

## ⚙️ Configuration

### Paramètres Principaux

Le fichier `config/default_params.yaml` contient tous les paramètres configurables :

```yaml
# Sécurité
safety:
  min_battery_voltage: 10.5
  min_battery_percentage: 20.0
  max_altitude: 120.0
  min_gps_satellites: 6

# Communication
communication:
  heartbeat_timeout: 5.0
  service_timeout: 30.0
  retry_attempts: 3

# Performance
performance:
  status_rate: 10.0
  health_check_rate: 1.0
  diagnostics_rate: 0.5
```

### Limites de Sécurité

Le fichier `config/safety_limits.yaml` définit les limites critiques :

```yaml
critical_limits:
  battery:
    voltage_min: 9.5
    percentage_min: 10.0
  altitude:
    max_absolute: 150.0
  navigation:
    max_speed_horizontal: 20.0
```

## 🧪 Tests

### Exécution des Tests

```bash
# Tests unitaires
python3 -m pytest test/test_interface.py -v

# Tests de sécurité
python3 -m pytest test/test_safety.py -v

# Tests avec couverture
python3 -m pytest --cov=drone_interface test/ --cov-report=html
```

### Tests d'Intégration

```bash
# Test complet avec MAVROS
ros2 launch drone_interface test_integration.launch.py
```

## 📊 Monitoring et Diagnostics

### Surveillance en Temps Réel

```bash
# Monitoring continu
watch -n 1 "ros2 topic echo /drone/status --once"

# Diagnostics ROS2
ros2 topic echo /diagnostics
```

### Logs et Debugging

```bash
# Logs détaillés
ros2 launch drone_interface drone_interface_launch.py log_level:=DEBUG

# Visualisation des logs
tail -f /tmp/drone_logs/interface/drone_interface.log
```

## 🔧 Développement

### Architecture

Le package suit une architecture modulaire avec séparation des responsabilités :

- **SafetyManager** : Logique de sécurité et vérifications
- **StateManager** : Gestion de l'état du drone
- **HealthMonitor** : Surveillance de la santé système
- **DroneInterface** : Nœud principal avec lifecycle management

### Contribution

1. Fork le repository
2. Créer une branche feature (`git checkout -b feature/nouvelle-fonctionnalite`)
3. Commiter les changements (`git commit -am 'Ajout nouvelle fonctionnalité'`)
4. Pousser vers la branche (`git push origin feature/nouvelle-fonctionnalite`)
5. Créer une Pull Request

### Standards de Code

- **PEP 8** : Respect des conventions Python
- **Type Hints** : Utilisation obligatoire des annotations de type
- **Documentation** : Docstrings pour toutes les fonctions publiques
- **Tests** : Couverture minimale de 80%

## 🚨 Sécurité

### Vérifications Automatiques

Le système effectue automatiquement :
- Validation de la connexion MAVROS
- Vérification du niveau de batterie
- Contrôle de la qualité GPS
- Surveillance de l'altitude
- Détection de perte de communication

### Procédures d'Urgence

En cas de problème critique :
1. **Arrêt d'urgence** : `ros2 service call /drone/emergency_stop std_srvs/srv/Trigger`
2. **Désarmement forcé** : Le système peut désarmer automatiquement
3. **Mode RTL** : Retour automatique à la base

## 📚 Documentation Supplémentaire

- [Guide de Déploiement](docs/deployment.md)
- [Configuration Avancée](docs/advanced_config.md)
- [Troubleshooting](docs/troubleshooting.md)
- [API Reference](docs/api_reference.md)

## 📝 Changelog

### Version 3.0.0 (2025-09-03)
- Refactorisation complète avec architecture modulaire
- Ajout du système de sécurité avancé
- Implémentation des outils CLI
- Lifecycle management complet
- Tests exhaustifs

### Version 2.x
- Voir [CHANGELOG.md](CHANGELOG.md) pour l'historique complet

## 📄 License

Ce projet est sous licence MIT. Voir le fichier [LICENSE](LICENSE) pour plus de détails.

## 👥 Équipe

- **Adama Komi** - Développeur principal - [GitHub](https://github.com/adamaKomi)

## 🆘 Support

- **Issues** : [GitHub Issues](https://github.com/adamaKomi/drone-autonome-2/issues)
- **Discussions** : [GitHub Discussions](https://github.com/adamaKomi/drone-autonome-2/discussions)
- **Email** : adama.komi@example.com

---

⚡ **Développé avec ❤️ pour la communauté des drones autonomes**