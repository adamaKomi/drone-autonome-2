# Architecture Modulaire Drone - ROS2

## 🚁 Vue d'ensemble

Cette implémentation fournit une architecture modulaire pour le contrôle de drones ArduPilot avec ROS2. Le système est divisé en **nœuds spécialisés** qui communiquent entre eux pour réaliser des missions complexes.

## 🏗️ Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    ARCHITECTURE MODULAIRE                   │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────────┐ │
│  │  DRONE_MISSION  │  │ DRONE_INTERFACE │  │DRONE_NAVIGATION │ │
│  │    (Mission)    │◄─┤   (Central)     ├─►│   (Position)    │ │
│  └─────────────────┘  └─────────────────┘  └─────────────────┘ │
│           │                     │                     │        │
│           ▼                     ▼                     ▼        │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │                     MAVROS                          │ │
│  └─────────────────────────────────────────────────────────────┘ │
│                              │                                  │
│                              ▼                                  │
│  ┌─────────────────────────────────────────────────────────────┐ │
│  │                  ArduPilot SITL                     │ │
│  └─────────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
```

## 📦 Composants

### 1. drone_interface (Nœud Central)
**Responsabilités :**
- Interface directe avec MAVROS
- Armement/Désarmement sécurisé
- Changement de modes de vol
- Publication de l'état du drone
- Vérifications de sécurité globales

**Services fournis :**
- `/drone/control` - Contrôle général (ARM, DISARM, SET_MODE, etc.)
- `/drone/safety_check` - Vérifications de sécurité

**Topics publiés :**
- `/drone/status` - État général du drone
- `/drone/armed` - État d'armement
- `/drone/flight_mode` - Mode de vol actuel
- `/drone/battery_percentage` - Niveau de batterie
- `/drone/safety_alerts` - Alertes de sécurité

### 2. drone_navigation (Navigation & Position)
**Responsabilités :**
- Navigation autonome vers waypoints
- Contrôle de position PID
- Gestion des trajectoires
- Évitement d'obstacles (futur)

**Services fournis :**
- `/drone/navigation/goto` - Aller à une position
- `/drone/navigation/add_waypoint` - Ajouter un waypoint
- `/drone/navigation/control` - Contrôle navigation (START, STOP, etc.)

**Topics publiés :**
- `/drone/navigation/status` - État de navigation
- `/drone/navigation/planned_path` - Trajectoire prévue
- `/mavros/setpoint_raw/local` - Commandes de position

### 3. drone_mission (Missions & Orchestration)
**Responsabilités :**
- Planification de missions complexes
- Exécution séquentielle de tâches
- Gestion des conditions et événements
- Sauvegarde/Chargement de missions

**Services fournis :**
- `/drone/mission/load` - Charger une mission
- `/drone/mission/create` - Créer une mission
- `/drone/mission/control` - Contrôle mission (START, PAUSE, STOP)

**Topics publiés :**
- `/drone/mission/status` - État de la mission
- `/drone/mission/progress` - Progression de la mission

## 🚀 Installation et Compilation

### 1. Prérequis
```bash
# ROS2 Humble
sudo apt install ros-humble-desktop-full

# MAVROS
sudo apt install ros-humble-mavros ros-humble-mavros-extras

# ArduPilot SITL (optionnel pour simulation)
pip install MAVProxy
```

### 2. Compilation
```bash
cd ~/ros2_ws
colcon build --packages-select drone_interface drone_navigation drone_mission
source install/setup.bash
```

## 🎮 Utilisation

### 1. Démarrage manuel des nœuds

Terminal 1 - Interface central :
```bash
ros2 run drone_interface interface_node
```

Terminal 2 - Navigation :
```bash
ros2 run drone_navigation navigation_node
```

Terminal 3 - Mission :
```bash
ros2 run drone_mission mission_node
```

### 2. Démarrage automatisé
```bash
# Utiliser le script de test
./test_modular_architecture.sh start

# Vérifier le statut
./test_modular_architecture.sh status

# Tester les communications
./test_modular_architecture.sh test

# Arrêter tous les nœuds
./test_modular_architecture.sh stop
```

### 3. Commandes de base

**Contrôle du drone :**
```bash
# Armer le drone
ros2 service call /drone/control std_msgs/srv/SetString "{data: 'ARM'}"

# Changer en mode GUIDED
ros2 service call /drone/control std_msgs/srv/SetString "{data: 'SET_MODE:GUIDED'}"

# Décoller à 3m
ros2 service call /drone/control std_msgs/srv/SetString "{data: 'TAKEOFF:3.0'}"
```

**Navigation :**
```bash
# Aller à une position (x, y, z, yaw)
ros2 service call /drone/navigation/goto std_msgs/srv/SetString "{data: '5.0:5.0:3.0:1.57'}"

# Démarrer la navigation
ros2 service call /drone/navigation/control std_msgs/srv/SetString "{data: 'START'}"
```

**Missions :**
```bash
# Charger une mission
ros2 service call /drone/mission/load std_msgs/srv/SetString "{data: 'test_mission'}"

# Démarrer la mission
ros2 service call /drone/mission/control std_msgs/srv/SetString "{data: 'START'}"
```

## 📋 Missions

### Format de Mission (JSON)
```json
{
  "id": "mission-001",
  "name": "test_mission",
  "description": "Mission de test",
  "tasks": [
    {
      "id": "task-001",
      "type": "ARM",
      "description": "Armer le drone",
      "parameters": {},
      "timeout": 30.0,
      "required": true
    },
    {
      "id": "task-002",
      "type": "TAKEOFF",
      "description": "Décoller à 3m",
      "parameters": {"altitude": 3.0},
      "timeout": 60.0,
      "required": true
    }
  ]
}
```

### Types de Tâches Supportées
- `ARM` / `DISARM` - Armement/Désarmement
- `TAKEOFF` / `LAND` - Décollage/Atterrissage  
- `GOTO` / `WAYPOINT` - Navigation
- `SET_MODE` - Changement de mode
- `WAIT` - Attente temporisée
- `RTL` - Retour au point de départ
- `SURVEY` / `ORBIT` - Missions spécialisées (futur)

## 🔧 Configuration

### Paramètres de Sécurité (interface_node)
```python
min_battery_voltage = 10.5      # Voltage minimum
min_battery_percentage = 20.0   # Pourcentage minimum  
min_gps_satellites = 6          # Satellites GPS minimum
max_altitude = 120.0            # Altitude maximum (m)
geofence_radius = 100.0         # Rayon géofence (m)
```

### Paramètres de Navigation (navigation_node)
```python
max_velocity = 2.0              # Vitesse maximale (m/s)
position_tolerance = 0.5        # Tolérance position (m)
pid_gains = {                   # Gains PID
    'kp': [1.5, 1.5, 2.0],
    'ki': [0.1, 0.1, 0.1], 
    'kd': [0.3, 0.3, 0.5]
}
```

## 🔍 Surveillance et Debugging

### Topics de monitoring
```bash
# État général
ros2 topic echo /drone/status

# Navigation
ros2 topic echo /drone/navigation/status

# Mission
ros2 topic echo /drone/mission/status

# Alertes de sécurité
ros2 topic echo /drone/safety_alerts
```

### Logs
- Les logs sont disponibles dans `~/ros2_ws/logs/test_*/`
- Chaque nœud a son propre fichier de log

### Visualisation
```bash
# Liste des nœuds actifs
ros2 node list

# Liste des topics
ros2 topic list

# Liste des services  
ros2 service list

# Graphique des nœuds
rqt_graph
```

## 🧪 Tests

### Tests unitaires
```bash
# Tester l'architecture
./test_modular_architecture.sh test

# Tester les services
./test_modular_architecture.sh services
```

### Tests d'intégration avec SITL
```bash
# 1. Démarrer ArduPilot SITL
sim_vehicle.py -v ArduCopter --map --console

# 2. Démarrer MAVROS  
ros2 launch mavros apm.launch fcu_url:=tcp://127.0.0.1:5760

# 3. Démarrer l'architecture
./test_modular_architecture.sh start

# 4. Charger et exécuter une mission
ros2 service call /drone/mission/load std_msgs/srv/SetString "{data: 'test_mission'}"
ros2 service call /drone/mission/control std_msgs/srv/SetString "{data: 'START'}"
```

## 🔮 Développements Futurs

### Fonctionnalités prévues
- **drone_vision** - Traitement d'image et vision
- **Messages personnalisés** - Remplacement des String par des messages structurés
- **Interface web** - Dashboard de contrôle
- **Planificateur de trajectoire** - Optimisation de chemins
- **Évitement d'obstacles** - Navigation intelligente
- **Multi-drone** - Coordination d'essaims

### Améliorations techniques
- Passage en C++ pour la performance
- Tests automatisés avec pytest
- Documentation auto-générée
- Intégration CI/CD
- Métriques de performance

## 📞 Support

### Structure des fichiers
```
ros2_ws/
├── src/
│   ├── drone_interface/          # Nœud central
│   ├── drone_navigation/         # Navigation  
│   ├── drone_mission/           # Missions
│   └── drone_vision/            # Vision (futur)
├── test_modular_architecture.sh  # Script de test
└── README.md                    # Cette documentation
```

### Commandes utiles
```bash
# Redémarrage complet
./test_modular_architecture.sh stop
./test_modular_architecture.sh start

# Debugging d'un nœud
ros2 run drone_interface interface_node --log-level debug

# Vérification des dépendances
rosdep check --from-paths src --ignore-src
```

---

## 🎯 Résumé

Cette architecture modulaire offre :
- ✅ **Séparation des responsabilités** - Chaque nœud a un rôle spécifique
- ✅ **Facilité de maintenance** - Code modulaire et testable  
- ✅ **Évolutivité** - Ajout facile de nouvelles fonctionnalités
- ✅ **Robustesse** - Isolation des pannes entre modules
- ✅ **Réutilisabilité** - Composants interchangeables

L'architecture est prête pour des missions autonomes complexes tout en restant simple à comprendre et maintenir.
