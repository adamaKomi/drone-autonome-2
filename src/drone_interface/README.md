# Drone Interface - Système de Contrôle ArduPilot

## 🚁 Description

Ce package ROS2 fournit une interface complète et modulaire pour contrôler un drone ArduPilot via MAVROS. Il est conçu pour fonctionner avec le simulateur SITL (Software In The Loop) d'ArduPilot et peut être adapté pour un drone physique.

## 🏗️ Architecture

### Modules Principaux

1. **StateManager** - Gestion de l'état du drone
2. **SafetyManager** - Vérifications de sécurité
3. **ArmingManager** - Armement/désarmement
4. **ModeManager** - Gestion des modes de vol
5. **PositionController** - Contrôle de position
6. **TakeoffLandingManager** - Décollage/atterrissage
7. **MissionExecutor** - Exécution de missions

### Structure des Fichiers

```
drone_interface/
├── interface_node.py          # Nœud principal
├── config.py                  # Configuration centralisée
├── drone_examples.py          # Exemples et missions
└── README.md                  # Documentation
```

## 🚀 Installation et Configuration

### Prérequis

1. **ROS2 Humble/Iron/Rolling**
2. **MAVROS** installé et configuré
3. **ArduPilot SITL** pour la simulation

```bash
# Installer MAVROS
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# Installer les dépendances GeographicLib
sudo /opt/ros/humble/lib/mavros/install_geographiclib_datasets.sh

# Installer ArduPilot SITL
git clone --recurse-submodules https://github.com/ArduPilot/ardupilot.git
cd ardupilot
./Tools/environment_install/install-prereqs-ubuntu.sh -y
. ~/.profile
```

### Installation du Package

```bash
# Aller dans votre workspace ROS2
cd ~/ros2_ws/src

# Le package est déjà présent dans drone_interface/
# Compiler le workspace
cd ~/ros2_ws
colcon build --packages-select drone_interface

# Sourcer l'environnement
source install/setup.bash
```

## 🎮 Utilisation

### 1. Démarrage du Simulateur SITL

```bash
# Terminal 1: Démarrer ArduPilot SITL
sim_vehicle.py -v ArduCopter --console --map --out=127.0.0.1:14550
```

### 2. Démarrage de MAVROS

```bash
# Terminal 2: Démarrer MAVROS
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555
```

### 3. Lancement du Nœud d'Interface

```bash
# Terminal 3: Lancer l'interface drone
ros2 run drone_interface interface_node
```

### 4. Commandes de Base

```bash
# Vérifier l'état
ros2 param set /drone_interface action "STATUS"

# Armer le drone
ros2 param set /drone_interface action "ARM"

# Passer en mode GUIDED
ros2 param set /drone_interface action "GUIDED"

# Décoller à 3m
ros2 param set /drone_interface altitude 3.0
ros2 param set /drone_interface action "TAKEOFF"

# Aller à une position
ros2 param set /drone_interface x 5.0
ros2 param set /drone_interface y 3.0
ros2 param set /drone_interface z 2.0
ros2 param set /drone_interface action "POSITION"

# Maintenir la position actuelle
ros2 param set /drone_interface action "HOLD"

# Atterrir
ros2 param set /drone_interface action "LAND"

# Désarmer
ros2 param set /drone_interface action "DISARM"
```

### 5. Commandes Avancées

```bash
# Forcer l'armement (ignore les vérifications de sécurité)
ros2 param set /drone_interface action "FORCE_ARM"

# Changer de mode de vol
ros2 param set /drone_interface mode "LOITER"
ros2 param set /drone_interface action "MODE"

# Procédure d'urgence (RTL ou LAND)
ros2 param set /drone_interface action "EMERGENCY"

# Arrêter les setpoints de position
ros2 param set /drone_interface action "STOP_SETPOINTS"
```

## 🎯 Missions Prédéfinies

### Lancer des Missions d'Exemple

```bash
# Lister les missions disponibles
ros2 run drone_interface drone_examples --list-missions

# Exécuter une mission simple
ros2 run drone_interface drone_examples --mission basic_flight

# Exécuter un motif carré
ros2 run drone_interface drone_examples --mission square_pattern

# Mission en triangle
ros2 run drone_interface drone_examples --mission triangle_pattern

# Vol en cercle
ros2 run drone_interface drone_examples --mission circle_pattern
```

### Missions Disponibles

| Mission | Description | Altitude | Durée |
|---------|-------------|----------|-------|
| `basic_flight` | Vol stationnaire simple | 3m | 1min |
| `square_pattern` | Motif carré 10x10m | 3m | 2min |
| `triangle_pattern` | Triangle équilatéral | 4m | 2min |
| `circle_pattern` | Cercle de 8m de rayon | 5m | 3min |
| `altitude_test` | Test de montée/descente | 2-8m | 3min |
| `perimeter_inspection` | Inspection périmètre 20x15m | 6m | 10min |
| `speed_test` | Test de déplacement rapide | 4m | 5min |

## ⚙️ Configuration

### Profils de Configuration

Le système supporte plusieurs profils de configuration :

```bash
# Configuration par défaut
ros2 run drone_interface interface_node

# Configuration pour simulation
DRONE_CONFIG_PROFILE=simulation ros2 run drone_interface interface_node

# Configuration conservatrice (sécurité maximale)
DRONE_CONFIG_PROFILE=conservative ros2 run drone_interface interface_node

# Configuration agressive (performance maximale)
DRONE_CONFIG_PROFILE=aggressive ros2 run drone_interface interface_node
```

### Variables d'Environnement

```bash
# Personnaliser la configuration via variables d'environnement
export DRONE_MIN_BATTERY_VOLTAGE=11.0
export DRONE_MAX_ALTITUDE=50.0
export DRONE_GEOFENCE_RADIUS=100.0
export DRONE_TAKEOFF_ALTITUDE=3.0
export DRONE_DISABLE_SAFETY=false
export MAVROS_NAMESPACE=mavros

# Puis lancer le nœud
ros2 run drone_interface interface_node
```

## 🧪 Tests

### Lancer les Tests Unitaires



### Tests de Performance

Les tests incluent :
- ✅ Tests des énumérations et structures de données
- ✅ Tests du SafetyManager
- ✅ Tests du StateManager
- ✅ Tests du PositionController
- ✅ Tests d'intégration entre modules
- ✅ Tests de performance

## 🛡️ Sécurité

### Vérifications Automatiques

Le système effectue automatiquement :
- ✅ Vérification de la connexion MAVROS
- ✅ Vérification du fix GPS et nombre de satellites
- ✅ Vérification du niveau de batterie
- ✅ Surveillance de l'altitude maximum
- ✅ Géofence de sécurité
- ✅ Surveillance continue en vol

### Paramètres de Sécurité par Défaut

```python
min_battery_voltage = 10.5V      # Voltage minimum
min_gps_satellites = 6           # Satellites minimum
max_altitude = 120.0m            # Altitude maximum
geofence_radius = 100.0m         # Rayon de sécurité
critical_battery = 20%           # Seuil batterie critique
```

### Procédure d'Urgence

En cas de problème, le système peut :
1. Passer automatiquement en mode RTL (Return to Launch)
2. Si RTL échoue, passer en mode LAND
3. Désarmer le drone si nécessaire

## 📊 Surveillance et Monitoring

### Informations Affichées

Le système affiche périodiquement :
- 📡 État de connexion MAVROS
- 🔧 État d'armement
- 🎮 Mode de vol actuel
- 📍 Position (x, y, z)
- 🔋 Voltage et pourcentage batterie
- 🛰️ État GPS et satellites

### Topics ROS2 Utilisés

```bash
# Surveillance (subscribers)
/mavros/state                           # État du drone
/mavros/local_position/pose             # Position locale
/mavros/local_position/velocity_local   # Vélocité
/mavros/battery                         # État batterie
/mavros/global_position/global          # Position GPS

# Contrôle (publishers)
/mavros/setpoint_position/local         # Setpoints de position

# Services utilisés
/mavros/cmd/arming                      # Armement/désarmement
/mavros/set_mode                        # Changement de mode
/mavros/cmd/takeoff                     # Décollage
/mavros/cmd/land                        # Atterrissage
```

## 🔧 Modes de Vol Supportés

| Mode | Description | Usage |
|------|-------------|-------|
| `STABILIZE` | Stabilisation manuelle | Contrôle RC |
| `GUIDED` | Mode guidé | Contrôle autonome |
| `AUTO` | Mission automatique | Waypoints préprogrammés |
| `LOITER` | Vol stationnaire | Maintien position GPS |
| `RTL` | Retour au lancement | Urgence/fin de mission |
| `LAND` | Atterrissage auto | Atterrissage contrôlé |
| `ALT_HOLD` | Maintien altitude | Vol manuel avec altitude fixe |
| `POSHOLD` | Maintien position | Vol manuel avec position fixe |

## 🚨 Résolution des Problèmes

### Problèmes Courants

#### 1. "Service /mavros/cmd/arming non disponible"
```bash
# Vérifier que MAVROS fonctionne
ros2 topic echo /mavros/state

# Redémarrer MAVROS si nécessaire
```

#### 2. "GPS fix not available"
```bash
# En simulation, attendre quelques secondes
# Vérifier les paramètres SITL si problème persiste
```

#### 3. "Drone not connected to flight controller"
```bash
# Vérifier la connexion SITL
# Vérifier l'URL de connexion MAVROS
```

#### 4. Armement refusé
```bash
# Vérifier l'état avec STATUS
ros2 param set /drone_interface action "STATUS"

# Forcer l'armement si nécessaire (simulation uniquement)
ros2 param set /drone_interface action "FORCE_ARM"
```

### Logs Utiles

```bash
# Voir les logs du nœud
ros2 run drone_interface interface_node

# Voir l'état MAVROS
ros2 topic echo /mavros/state

# Voir la position
ros2 topic echo /mavros/local_position/pose
```

## 🔬 Développement

### Structure du Code

```
interface_node.py:
├── DroneInterface          # Nœud principal
├── StateManager           # Gestion d'état
├── SafetyManager          # Sécurité
├── ArmingManager          # Armement
├── ModeManager            # Modes de vol
├── PositionController     # Contrôle position
└── TakeoffLandingManager  # Décollage/atterrissage
```

### Ajouter une Nouvelle Fonctionnalité

1. Créer un nouveau manager dans `interface_node.py`
2. L'initialiser dans `DroneInterface.__init__()`
3. Ajouter les commandes dans `_handle_action_command()`

### Bonnes Pratiques

- ✅ Toutes les opérations ont des timeouts
- ✅ Gestion d'erreurs complète
- ✅ Logs détaillés pour le débogage
- ✅ Vérifications de sécurité systématiques
- ✅ Architecture modulaire et testable
- ✅ Configuration centralisée

## 📚 API Reference

### Commandes Principales

| Commande | Paramètres | Description |
|----------|------------|-------------|
| `ARM` | - | Arme le drone avec vérifications |
| `DISARM` | - | Désarme le drone |
| `TAKEOFF` | `altitude` | Décollage automatisé |
| `LAND` | - | Atterrissage automatisé |
| `POSITION` | `x, y, z, yaw` | Va à une position |
| `HOLD` | - | Maintient la position actuelle |
| `MODE` | `mode` | Change le mode de vol |
| `STATUS` | - | Affiche l'état détaillé |
| `EMERGENCY` | - | Procédure d'urgence |

### Exemples d'Utilisation en Python

```python
# Importer le module
from drone_interface.interface_node import DroneInterface

# Créer une instance
drone = DroneInterface()

# Utiliser les managers directement
success, msg = drone.arming_manager.arm()
success, msg = drone.mode_manager.set_mode("GUIDED")
success = drone.position_controller.set_position(5.0, 3.0, 2.0)
```

## 📞 Support

### Documentation Officielle

- [ArduPilot SITL](https://ardupilot.org/dev/docs/sitl-simulator-software-in-the-loop.html)
- [MAVROS](http://wiki.ros.org/mavros)
- [MAVLink](https://mavlink.io/en/)

### Contribution

1. Fork le projet
2. Créer une branche feature
3. Commiter les changements
4. Pousser vers la branche
5. Créer une Pull Request

## 📄 Licence

Ce projet est sous licence MIT. Voir le fichier LICENSE pour plus de détails.

---

🚁 **Happy Flying!** 🚁
