# 🌸 Système Drone Autonome de Pollinisation

## Vue d'ensemble

Ce projet implémente un système drone autonome pour la pollinisation de fleurs, basé sur ROS2 Humble. Le système combine la navigation autonome, la détection visuelle de fleurs et la collecte de données pour créer une solution complète de pollinisation automatisée.

## 🏗️ Architecture du Système

### Composants Principaux

1. **🚁 drone_interface** - Interface de contrôle du drone (MAVROS)
2. **🗺️ drone_navigation** - Navigation autonome et évitement d'obstacles
3. **🎯 drone_mission** - Orchestration des missions et tâches
4. **🔍 drone_vision** - Détection visuelle de fleurs
5. **📊 drone_data_collector** - Collecte et sauvegarde des données

### Nouveaux Composants (Système de Vision)

#### drone_vision Package
- **vision_node.py** - Détection de fleurs par couleur et forme
- **data_collector_node.py** - Collecte de données GPS, images et métriques

## 🌸 Fonctionnalités de Pollinisation

### Types de Tâches Missions
- `DETECT_FLOWERS` - Scanner et détecter des fleurs
- `APPROACH_FLOWER` - Approche précise d'une fleur détectée
- `POLLINATE` - Simulation du processus de pollinisation
- `COLLECT_DATA` - Collecte de données scientifiques
- `SCAN_AREA` - Balayage systématique d'une zone

### Détection de Fleurs
- Détection par couleur (rouge, jaune, rose, orange)
- Localisation précise dans l'image
- Calcul de confiance basé sur la forme
- Tracking temporel des détections

### Collecte de Données
- Enregistrement GPS continu
- Capture d'images aux points d'intérêt
- Métadonnées de mission complètes
- Export JSON/CSV pour analyse

## 🚀 Installation et Configuration

### Prérequis
```bash
# ROS2 Humble
sudo apt install ros-humble-desktop

# Dépendances vision
sudo apt install python3-opencv
pip3 install cv-bridge numpy

# MAVROS (pour l'interface drone)
sudo apt install ros-humble-mavros ros-humble-mavros-extras
```

### Compilation
```bash
cd ~/ros2_ws
colcon build --packages-select drone_vision drone_mission drone_navigation drone_interface
source install/setup.bash
```

## 🎮 Utilisation

### 1. Démonstration Complète (Recommandé)
```bash
cd ~/ros2_ws
./demo_pollination_system.sh
```

Cette démonstration lance automatiquement :
- Simulateur de caméra avec fleurs
- Système de vision complet
- Collecte de données
- Tests automatiques

### 2. Lancement Manuel

#### Système de Vision Seul
```bash
# Terminal 1: Lancer le système de vision
ros2 launch drone_vision vision_system.launch.py

# Terminal 2: Simulateur de caméra (optionnel)
python3 camera_simulator.py

# Terminal 3: Activer la détection
ros2 service call /vision/set_detection std_srvs/srv/SetBool "data: true"
ros2 service call /data_collector/set_collection std_srvs/srv/SetBool "data: true"
```

#### Système Complet avec Navigation
```bash
# Lance tout le système (MAVROS + Navigation + Mission + Vision)
ros2 launch drone_vision complete_system.launch.py
```

### 3. Mission de Pollinisation
```bash
# Charger et exécuter une mission de pollinisation
ros2 run drone_mission mission_node --ros-args -p mission_file:=pollination_mission.json
```

## 📡 Topics et Services

### Topics Principaux
- `/vision/flowers_detected` - Fleurs détectées (JSON)
- `/vision/debug_image` - Images avec annotations
- `/vision/target_flower_position` - Position de la fleur cible
- `/data_collector/status` - Statut de collecte
- `/camera/image_raw` - Flux vidéo caméra

### Services Disponibles
- `/vision/set_detection` - Activer/désactiver détection
- `/vision/get_closest_flower` - Obtenir la fleur la plus proche
- `/data_collector/set_collection` - Contrôler collecte de données
- `/data_collector/save_data` - Forcer sauvegarde
- `/data_collector/capture_image` - Capturer image manuelle

## 📊 Données Collectées

### Structure des Données

#### Fichiers GPS (`gps_data_*.csv`)
```csv
timestamp,latitude,longitude,altitude,accuracy,mission_phase
1693123456.789,45.1234,-73.5678,120.5,2.1,scan_area
```

#### Détections de Fleurs (`flowers_*.json`)
```json
[
  {
    "id": "red_320_240_1693123456789",
    "timestamp": 1693123456.789,
    "gps_lat": 45.1234,
    "gps_lon": -73.5678,
    "gps_alt": 120.5,
    "image_x": 320.0,
    "image_y": 240.0,
    "radius": 25.0,
    "color": "red",
    "confidence": 0.85,
    "image_filename": "flower_red_320_240_1693123456789.jpg",
    "mission_phase": "detection"
  }
]
```

#### Métriques de Mission (`metrics_*.json`)
```json
{
  "mission_id": "abc123ef",
  "start_time": 1693123400.0,
  "end_time": 1693124000.0,
  "total_distance": 1250.75,
  "flowers_detected": 15,
  "images_captured": 23,
  "areas_scanned": 3,
  "mission_success": true
}
```

## 🧪 Tests et Validation

### Test Automatique
```bash
cd ~/ros2_ws
python3 test_vision_system.py
```

### Visualisation en Temps Réel
```bash
# Voir les images avec détections
ros2 run rqt_image_view rqt_image_view /vision/debug_image

# Monitorer les détections
ros2 topic echo /vision/flowers_detected

# Voir les métriques
ros2 topic echo /data_collector/status
```

## 🎯 Missions d'Exemple

### Mission Simple de Pollinisation
```json
{
  "mission_id": "pollination_basic",
  "description": "Mission de pollinisation de base",
  "tasks": [
    {"type": "ARM", "params": {}},
    {"type": "TAKEOFF", "params": {"altitude": 10}},
    {"type": "SCAN_AREA", "params": {"area": "zone_1"}},
    {"type": "DETECT_FLOWERS", "params": {"colors": ["red", "yellow"]}},
    {"type": "APPROACH_FLOWER", "params": {"precision": 0.5}},
    {"type": "POLLINATE", "params": {"duration": 5}},
    {"type": "COLLECT_DATA", "params": {"capture_image": true}},
    {"type": "LAND", "params": {}},
    {"type": "DISARM", "params": {}}
  ]
}
```

## 🔧 Configuration

### Paramètres de Détection
```python
# Dans vision_node.py
color_ranges = {
    'red': [(np.array([0, 50, 50]), np.array([10, 255, 255]))],
    'yellow': [(np.array([20, 50, 50]), np.array([30, 255, 255]))],
    # ... autres couleurs
}
min_radius = 10
max_radius = 100
min_confidence = 0.5
```

### Paramètres de Collecte
```python
# Dans data_collector_node.py
data_dir = "~/ros2_ws/mission_data"
gps_frequency = 1.0  # Hz
save_interval = 30.0  # secondes
```

## 🐛 Dépannage

### Problèmes Courants

1. **Pas d'image caméra**
   ```bash
   # Vérifier le topic
   ros2 topic list | grep camera
   # Lancer le simulateur
   python3 camera_simulator.py
   ```

2. **Détection ne fonctionne pas**
   ```bash
   # Vérifier l'activation
   ros2 service call /vision/set_detection std_srvs/srv/SetBool "data: true"
   ```

3. **Données non sauvegardées**
   ```bash
   # Vérifier les permissions
   ls -la ~/ros2_ws/mission_data/
   # Forcer la sauvegarde
   ros2 service call /data_collector/save_data std_srvs/srv/Trigger
   ```

### Logs de Debug
```bash
# Logs détaillés du système de vision
ros2 run drone_vision vision_node --ros-args --log-level DEBUG

# Logs du collecteur de données
ros2 run drone_vision data_collector_node --ros-args --log-level DEBUG
```

## 📈 Métriques de Performance

### Objectifs de Performance
- **Détection**: >80% de précision sur fleurs colorées
- **Latence**: <100ms pour traitement d'image
- **Collecte**: 1Hz pour données GPS, images à la demande
- **Sauvegarde**: Toutes les 30s automatiquement

### Monitoring
```bash
# Performance du système
ros2 topic hz /camera/image_raw
ros2 topic hz /vision/flowers_detected

# Utilisation ressources
htop
```

## 🤝 Contribution

### Structure du Code
```
src/drone_vision/
├── drone_vision/
│   ├── __init__.py
│   ├── vision_node.py          # Détection de fleurs
│   └── data_collector_node.py  # Collecte de données
├── launch/
│   ├── vision_system.launch.py # Système vision
│   └── complete_system.launch.py # Système complet
├── package.xml
└── setup.py
```

### Standards de Code
- Python 3.8+
- PEP 8 pour le style
- Documentation docstring complète
- Tests unitaires requis

## 📚 Ressources

### Documentation ROS2
- [ROS2 Humble Documentation](https://docs.ros.org/en/humble/)
- [MAVROS Documentation](http://wiki.ros.org/mavros)
- [OpenCV Python](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)

### Articles Scientifiques
- "Autonomous Pollination Systems Using Computer Vision"
- "ROS2-based Agricultural Robotics"

## 📄 Licence

MIT License - Voir le fichier LICENSE pour plus de détails.

## 👥 Équipe

- **Navigation**: Système de navigation autonome
- **Vision**: Détection et localisation de fleurs
- **Data**: Collecte et analyse de données
- **Mission**: Orchestration et planification

---

🌸 **Projet Drone Autonome de Pollinisation** - Contribution à l'agriculture durable 🌸
