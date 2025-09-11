# Drone Navigation Package

[![ROS2](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![Python](https://img.shields.io/badge/Python-3.8+-green.svg)](https://www.python.org/)
[![License](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

Système de navigation autonome avancé pour drones de pollinisation avec planification de trajectoires, évitement d'obstacles, et patterns de couverture intelligents.

## 🚀 Caractéristiques

- **Navigation Autonome Avancée** : Algorithmes A*, RRT*, Dijkstra avec optimisation génétique
- **Contrôle de Précision** : PID multi-axes avec anti-windup et mode approche précise (±0.5m)
- **Sécurité Intégrée** : Geofences, évitement d'obstacles par champs de potentiel, arrêt d'urgence
- **Patterns de Couverture** : Zigzag, spiral, adaptatif pour missions de pollinisation optimisées
- **Lifecycle Management** : Gestion complète des états ROS2 avec threading sécurisé
- **Outils CLI Complets** : Suite de 5 outils en ligne de commande pour toutes les opérations
- **Intégration MAVROS** : Compatible ArduPilot SITL et drones physiques
- **Diagnostics Avancés** : Surveillance temps réel, tests automatisés et métriques de performance

## 📁 Structure du Package

```
drone_navigation/
├── drone_navigation/                 # Package Python principal
│   ├── __init__.py
│   ├── navigation_node.py           # Nœud principal de navigation
│   ├── trajectory_planner.py        # Planificateur de trajectoires
│   ├── position_controller.py       # Contrôleur de position PID
│   ├── path_optimizer.py            # Optimiseur de chemins génétique
│   ├── obstacle_avoidance.py        # Évitement d'obstacles
│   ├── geofence_manager.py          # Gestionnaire de geofences
│   ├── coverage_patterns.py         # Patterns de couverture
│   └── tools/                       # Outils CLI
│       ├── __init__.py
│       ├── goto_position.py         # Navigation vers position
│       ├── plan_mission.py          # Planification de missions
│       ├── nav_status.py            # Surveillance temps réel
│       ├── nav_diagnostics.py       # Diagnostics complets
│       └── test_waypoints.py        # Tests de navigation
├── launch/                          # Fichiers de lancement
│   └── navigation_launch.py
├── config/                          # Fichiers de configuration
│   ├── navigation_params.yaml       # Paramètres principaux
│   ├── pid_tuning.yaml             # Réglages PID
│   ├── geofence_zones.yaml         # Zones de vol autorisées
│   └── coverage_patterns.yaml      # Configuration des patterns
├── test/                           # Tests unitaires
│   ├── test_navigation.py
│   ├── test_trajectory.py
│   ├── test_controllers.py
│   └── test_patterns.py
├── scripts/                        # Scripts utilitaires
│   ├── calibrate_navigation.py
│   └── tune_pid.py
├── resource/                       # Ressources du package
│   └── drone_navigation
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
- Shapely (géométrie)
- NumPy, SciPy (calculs scientifiques)

### Installation des Dépendances

```bash
# Dépendances ROS2
sudo apt update
sudo apt install ros-humble-mavros ros-humble-mavros-extras
sudo apt install ros-humble-geometry-msgs ros-humble-sensor-msgs

# Dépendances Python
pip install shapely numpy scipy psutil pyyaml
```

### Compilation

```bash
cd ~/ros2_ws
colcon build --packages-select drone_navigation
source install/setup.bash
```

### Configuration Automatique

```bash
# Calibration des paramètres de navigation
ros2 run drone_navigation calibrate_navigation

# Réglage automatique des PID
ros2 run drone_navigation tune_pid
```

## 🚁 Utilisation

### Démarrage du Système

1. **Lancement Complet (recommandé)** :
```bash
# Démarrage orchestré avec MAVROS, SITL et diagnostics
ros2 launch drone_navigation navigation_launch.py

# Avec configuration personnalisée
ros2 launch drone_navigation navigation_launch.py \
    simulation_mode:=true \
    mavros_enabled:=true \
    auto_start:=false \
    log_level:=info
```

2. **Démarrage Manuel** :
```bash
# Terminal 1: MAVROS
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555

# Terminal 2: Navigation
ros2 run drone_navigation navigation_node

# Terminal 3: Configuration
ros2 lifecycle set /drone_nav/navigation_node configure
ros2 lifecycle set /drone_nav/navigation_node activate
```

### Utilisation des Outils CLI

#### Navigation vers Position

```bash
# Navigation GPS
ros2 run drone_navigation goto_position --gps 34.0522 -6.8332 15.0

# Navigation locale avec waypoints
ros2 run drone_navigation goto_position --local 10.0 5.0 12.0 \
    --waypoints "[(5,2.5,10), (7.5,3.5,11)]" --speed 3.0

# Navigation avec timeout et tolérance
ros2 run drone_navigation goto_position --local 20.0 0.0 10.0 \
    --timeout 60 --tolerance 0.3 --verbose
```

#### Planification de Missions

```bash
# Couverture de zone avec pattern zigzag
ros2 run drone_navigation plan_mission --area field_polygon.json \
    --pattern zigzag --altitude 15 --overlap 25 --save mission.json

# Couverture spirale autour d'un point
ros2 run drone_navigation plan_mission --coverage spiral \
    --center "34.0522,-6.8332" --radius 50 --spacing 5

# Mission avec waypoints prédéfinis
ros2 run drone_navigation plan_mission --waypoints mission_points.json \
    --save planned_mission.json

# Pattern adaptatif pour pollinisation
ros2 run drone_navigation plan_mission --area orchard.json \
    --pattern adaptive --altitude 12 --speed 4 --verbose
```

#### Surveillance Temps Réel

```bash
# Surveillance continue
ros2 run drone_navigation nav_status --continuous --rate 2.0

# Surveillance avec timeout et export
ros2 run drone_navigation nav_status --continuous --timeout 300 \
    --export flight_log.json

# Affichage unique détaillé
ros2 run drone_navigation nav_status --verbose
```

#### Diagnostics Complets

```bash
# Diagnostic complet du système
ros2 run drone_navigation nav_diagnostics --verbose

# Test spécifique
ros2 run drone_navigation nav_diagnostics --test connectivity

# Surveillance continue avec export
ros2 run drone_navigation nav_diagnostics --monitor --interval 120 \
    --export diagnostics_report.json

# Diagnostic silencieux pour scripts
ros2 run drone_navigation nav_diagnostics --quiet --export status.json
```

#### Tests de Navigation

```bash
# Test basique (pattern carré)
ros2 run drone_navigation test_waypoints --basic --size 20 --verbose

# Tests de patterns complexes
ros2 run drone_navigation test_waypoints --pattern spiral --size 30
ros2 run drone_navigation test_waypoints --pattern figure8 --size 25

# Test de précision d'atterrissage
ros2 run drone_navigation test_waypoints --precision --accuracy 0.3

# Test de stress avec nombreux waypoints
ros2 run drone_navigation test_waypoints --stress --waypoints 100 --size 50

# Test depuis fichier de waypoints
ros2 run drone_navigation test_waypoints --file test_mission.json

# Suite complète de tests avec export
ros2 run drone_navigation test_waypoints --all --export test_results.json
```

### Services Disponibles

| Service | Type | Description |
|---------|------|-------------|
| `/drone_nav/goto_position` | `geometry_msgs/Point` | Navigation vers position |
| `/drone_nav/start_mission` | `std_srvs/Trigger` | Démarrage de mission |
| `/drone_nav/stop_mission` | `std_srvs/Trigger` | Arrêt de mission |
| `/drone_nav/emergency_stop` | `std_srvs/Trigger` | Arrêt d'urgence |
| `/drone_nav/get_status` | `std_srvs/Trigger` | État de navigation |
| `/drone_nav/health_check` | `std_srvs/Trigger` | Vérification santé |
| `/drone_nav/plan_path` | `nav_msgs/Path` | Planification de chemin |
| `/drone_nav/set_geofence` | `geometry_msgs/Polygon` | Configuration geofence |

### Topics Publiés

| Topic | Type | Description |
|-------|------|-------------|
| `/drone_nav/status` | `std_msgs/String` | État navigation (JSON) |
| `/drone_nav/metrics` | `std_msgs/String` | Métriques performance |
| `/drone_nav/path` | `nav_msgs/Path` | Trajectoire planifiée |
| `/drone_nav/obstacles` | `sensor_msgs/PointCloud2` | Obstacles détectés |
| `/drone_nav/geofence_status` | `std_msgs/Bool` | Statut geofence |

## ⚙️ Configuration

### Paramètres Principaux

Le fichier `config/navigation_params.yaml` contient la configuration principale :

```yaml
# Navigation
navigation:
  max_velocity: 10.0              # Vitesse maximale (m/s)
  max_acceleration: 3.0           # Accélération max (m/s²)
  planning_frequency: 10.0        # Fréquence planification (Hz)
  control_frequency: 50.0         # Fréquence contrôle (Hz)
  
# Sécurité
safety:
  min_altitude: 2.0               # Altitude minimale (m)
  max_altitude: 120.0             # Altitude maximale (m)
  safety_distance: 5.0            # Distance sécurité obstacles (m)
  emergency_descent_rate: 2.0     # Vitesse descente urgence (m/s)

# Algorithmes
algorithms:
  default_planner: "astar"        # Planificateur par défaut
  optimization_enabled: true      # Optimisation génétique
  obstacle_avoidance: "potential_field"  # Méthode évitement
```

### Réglages PID

Configuration fine des contrôleurs dans `config/pid_tuning.yaml` :

```yaml
pid_controllers:
  position_x:
    kp: 1.2
    ki: 0.1
    kd: 0.05
    max_output: 5.0
    
  position_y:
    kp: 1.2
    ki: 0.1
    kd: 0.05
    max_output: 5.0
    
  position_z:
    kp: 2.0
    ki: 0.2
    kd: 0.08
    max_output: 3.0
```

### Geofences

Définition des zones de vol dans `config/geofence_zones.yaml` :

```yaml
geofences:
  primary_zone:
    type: "inclusion"
    vertices:
      - {lat: 34.0520, lon: -6.8330}
      - {lat: 34.0525, lon: -6.8330}
      - {lat: 34.0525, lon: -6.8335}
      - {lat: 34.0520, lon: -6.8335}
    
  no_fly_zone:
    type: "exclusion"
    vertices:
      - {lat: 34.0522, lon: -6.8332}
      - {lat: 34.0523, lon: -6.8332}
      - {lat: 34.0523, lon: -6.8333}
      - {lat: 34.0522, lon: -6.8333}
```

### Patterns de Couverture

Configuration des motifs dans `config/coverage_patterns.yaml` :

```yaml
coverage_patterns:
  zigzag:
    default_spacing: 3.0
    overlap_percentage: 20.0
    turn_radius: 2.0
    
  spiral:
    initial_radius: 5.0
    spacing: 2.5
    max_turns: 5
    
  adaptive:
    flower_density_threshold: 0.7
    coverage_resolution: 1.0
    priority_zones_weight: 2.0
```

## 🧪 Tests

### Tests Unitaires

```bash
# Tests individuels des composants
python3 -m pytest test/test_navigation.py -v
python3 -m pytest test/test_trajectory.py -v
python3 -m pytest test/test_controllers.py -v
python3 -m pytest test/test_patterns.py -v

# Suite complète avec couverture
python3 -m pytest test/ --cov=drone_navigation --cov-report=html

# Tests de performance
python3 -m pytest test/ --benchmark-only
```

### Tests d'Intégration

```bash
# Test complet du système de navigation
ros2 launch drone_navigation test_integration.launch.py

# Tests avec simulation SITL
ros2 launch drone_navigation navigation_launch.py simulation_mode:=true \
    auto_start:=true log_level:=debug
```

### Tests de Validation

```bash
# Validation des algorithmes de planification
python3 test/validate_planners.py

# Tests de précision des contrôleurs
python3 test/validate_controllers.py

# Validation des patterns de couverture
python3 test/validate_patterns.py
```

## 📊 Monitoring et Diagnostics

### Surveillance en Temps Réel

```bash
# Dashboard de navigation
ros2 run drone_navigation nav_status --continuous --rate 5.0

# Métriques de performance
watch -n 2 "ros2 topic echo /drone_nav/metrics --once"

# Visualisation des trajectoires
ros2 run rviz2 rviz2 -d config/navigation_viz.rviz
```

### Logs et Debugging

```bash
# Logs détaillés
ros2 launch drone_navigation navigation_launch.py log_level:=debug

# Analyse des performances
ros2 run drone_navigation analyze_performance --log-file /tmp/nav_logs/

# Profiling du système
python3 -m cProfile -o navigation_profile.prof \
    $(ros2 pkg prefix drone_navigation)/lib/drone_navigation/navigation_node
```

### Métriques de Performance

Le système collecte automatiquement :
- **Erreur de trajectoire** : Précision du suivi de chemin
- **Temps de planification** : Performance des algorithmes
- **Fréquence de contrôle** : Stabilité temps réel
- **Efficacité de couverture** : Optimisation des patterns
- **Utilisation CPU/Mémoire** : Ressources système

## 🔧 Développement

### Architecture

Le package suit une architecture modulaire avec séparation des responsabilités :

- **NavigationNode** : Nœud principal avec lifecycle management
- **TrajectoryPlanner** : Algorithmes de planification (A*, RRT*, Dijkstra)
- **PositionController** : Contrôleurs PID multi-axes avec anti-windup
- **PathOptimizer** : Optimisation génétique multi-objectifs
- **ObstacleAvoidance** : Évitement par champs de potentiel
- **GeofenceManager** : Gestion des zones de vol avec Shapely
- **CoveragePatterns** : Génération de motifs de couverture

### Algorithmes Implémentés

#### Planification de Trajectoires
- **A*** : Recherche optimale avec heuristique
- **RRT*** : Échantillonnage pour espaces complexes
- **Dijkstra** : Chemins optimaux guarantis
- **Bézier** : Lissage des trajectoires
- **Dubins** : Contraintes de virage

#### Optimisation
- **Algorithme Génétique** : Multi-objectifs (temps, distance, énergie)
- **Recuit Simulé** : Optimisation locale de fallback
- **Gradient Descent** : Fine-tuning des paramètres

#### Contrôle
- **PID Multi-axes** : Contrôle précis X, Y, Z
- **Anti-windup** : Prévention de la saturation
- **Mode Précision** : Approche finale ±0.5m
- **Limitation Vitesse/Accélération** : Sécurité dynamique

### Contribution

1. Fork le repository
2. Créer une branche feature (`git checkout -b feature/nouveau-algorithme`)
3. Implémenter avec tests (`pytest test/test_nouveau.py`)
4. Documenter (`sphinx-build docs/`)
5. Commiter (`git commit -am 'Ajout algorithme de navigation'`)
6. Pousser (`git push origin feature/nouveau-algorithme`)
7. Créer une Pull Request

### Standards de Code

- **PEP 8** : Respect des conventions Python
- **Type Hints** : Annotations obligatoires
- **Documentation** : Docstrings NumPy style
- **Tests** : Couverture minimale 85%
- **Performance** : Profiling pour fonctions critiques

## 🚨 Sécurité et Robustesse

### Systèmes de Sécurité

#### Geofences Multi-niveaux
- **Zones d'inclusion** : Périmètres de vol autorisés
- **Zones d'exclusion** : No-fly zones avec buffer
- **Zones d'alerte** : Warnings préventifs
- **Validation temps réel** : Vérification continue

#### Évitement d'Obstacles
- **Champs de Potentiel** : Répulsion dynamique
- **Détection Prédictive** : Anticipation des collisions
- **Manœuvres d'Évitement** : Trajectoires de secours
- **Escalade de Sécurité** : Actions progressives

#### Gestion d'Urgence
- **Arrêt d'Urgence** : Immobilisation immédiate
- **Atterrissage Forcé** : Descente contrôlée
- **Return-to-Home** : Retour automatique sécurisé
- **Mode Dégradé** : Fonctionnement minimal

### Validation et Tests

#### Tests de Sécurité
```bash
# Tests de geofence
ros2 run drone_navigation test_geofence_violation

# Tests d'évitement d'obstacles
ros2 run drone_navigation test_obstacle_avoidance

# Tests de situations d'urgence
ros2 run drone_navigation test_emergency_scenarios
```

#### Validation Formelle
- **Model Checking** : Vérification des propriétés de sécurité
- **Simulation Monte Carlo** : Tests statistiques robustesse
- **Analyse de Sensibilité** : Impact des paramètres
- **Tests d'Endurance** : Stabilité long terme

## 📚 Documentation Avancée

### Guides Spécialisés
- [Guide d'Optimisation des Trajectoires](docs/trajectory_optimization.md)
- [Configuration Avancée des PID](docs/pid_advanced_tuning.md)
- [Développement de Nouveaux Patterns](docs/custom_patterns.md)
- [Intégration de Capteurs](docs/sensor_integration.md)

### API Reference
- [Navigation Node API](docs/api/navigation_node.md)
- [Trajectory Planner API](docs/api/trajectory_planner.md)
- [Controllers API](docs/api/controllers.md)
- [Utilities API](docs/api/utilities.md)

### Exemples Pratiques
- [Mission de Pollinisation Complète](examples/pollination_mission.py)
- [Navigation en Intérieur](examples/indoor_navigation.py)
- [Suivi d'Objet Mobile](examples/object_tracking.py)
- [Cartographie Aérienne](examples/aerial_mapping.py)

## 🛠️ Outils Utilitaires

### Calibration et Réglage

```bash
# Calibration automatique des paramètres
ros2 run drone_navigation calibrate_navigation --auto

# Réglage interactif des PID
ros2 run drone_navigation tune_pid --interactive

# Validation de la configuration
ros2 run drone_navigation validate_config

# Optimisation des performances
ros2 run drone_navigation optimize_performance --duration 300
```

### Visualisation et Analyse

```bash
# Visualisation 3D des trajectoires
ros2 run drone_navigation trajectory_visualizer

# Analyse des logs de vol
ros2 run drone_navigation analyze_flight_logs --file flight_2025_09_05.log

# Génération de rapports
ros2 run drone_navigation generate_report --mission mission_001.json
```

## 🚨 Dépannage

### Problèmes Courants

**Navigation ne démarre pas**
```bash
# Vérifier les services MAVROS
ros2 service list | grep mavros

# Diagnostics complets
ros2 run drone_navigation nav_diagnostics --verbose

# Vérifier la configuration
ros2 param list | grep drone_nav
```

**Trajectoires imprécises**
```bash
# Recalibrer les PID
ros2 run drone_navigation tune_pid --reset

# Vérifier les limites de vitesse
ros2 param get /drone_nav/navigation_node max_velocity

# Analyser les erreurs de trajectoire
ros2 topic echo /drone_nav/metrics
```

**Problèmes de geofence**
```bash
# Valider la configuration des zones
ros2 run drone_navigation validate_geofence

# Tester les violations
ros2 run drone_navigation test_geofence_boundaries

# Visualiser les zones
ros2 run rviz2 rviz2 -d config/geofence_viz.rviz
```

**Évitement d'obstacles défaillant**
```bash
# Tester la détection
ros2 topic echo /drone_nav/obstacles

# Calibrer les champs de potentiel
ros2 run drone_navigation calibrate_obstacle_avoidance

# Vérifier les paramètres de sécurité
ros2 param get /drone_nav/navigation_node safety_distance
```

### Codes d'Erreur

| Code | Description | Solution |
|------|-------------|----------|
| NAV_001 | Position GPS invalide | Vérifier fix GPS et geofence |
| NAV_002 | Planification échouée | Réduire complexité ou changer algorithme |
| NAV_003 | Contrôleur PID instable | Recalibrer gains PID |
| NAV_004 | Obstacle non évitable | Replanifier trajectoire ou arrêt d'urgence |
| NAV_005 | Geofence violée | Retour en zone autorisée immédiat |

## 📈 Performances et Optimisation

### Métriques de Performance

Le système surveille automatiquement :
- **Latence de planification** : < 100ms pour A*, < 500ms pour RRT*
- **Précision de trajectoire** : Erreur RMS < 0.5m en mode précision
- **Fréquence de contrôle** : 50Hz maintenue avec jitter < 5%
- **Efficacité de couverture** : > 95% pour patterns zigzag/spiral
- **Utilisation CPU** : < 30% en navigation normale

### Optimisation

```bash
# Profiling des performances
ros2 run drone_navigation profile_navigation --duration 60

# Optimisation automatique des paramètres
ros2 run drone_navigation auto_optimize --target efficiency

# Réglage fin pour hardware spécifique
ros2 run drone_navigation hardware_tuning --platform odroid_xu4
```

## 📝 Changelog

### Version 1.0.0 (2025-09-05)
- **Navigation autonome avancée** : Algorithmes A*, RRT*, Dijkstra, optimisation génétique
- **Contrôle de précision** : PID multi-axes avec anti-windup, mode précision ±0.5m
- **Sécurité complète** : Geofences, évitement d'obstacles, arrêt d'urgence
- **Patterns de couverture** : Zigzag, spiral, adaptatif pour pollinisation
- **Outils CLI complets** : 5 outils avec interface utilisateur avancée
- **Lifecycle management** : Threading sécurisé, gestion d'états robuste
- **Intégration MAVROS** : Compatible ArduPilot SITL et drones physiques
- **Tests exhaustifs** : Validation unitaire, intégration, performance

### Roadmap v1.1
- Support multi-drones avec coordination
- Machine learning pour optimisation adaptative
- Capteurs LiDAR/RGB-D pour navigation 3D
- Interface web de monitoring temps réel

## 📄 License

Ce projet est sous licence MIT. Voir le fichier [LICENSE](LICENSE) pour plus de détails.

## 👥 Équipe

- **Adama Komi** - Développeur principal - [GitHub](https://github.com/adamaKomi)
- **Spécialisation** : Navigation autonome, algorithmes de planification, contrôle de précision

## 🏆 Reconnaissance

Merci aux contributeurs de la communauté ROS2 et aux équipes de :
- **MAVROS** : Interface robuste avec ArduPilot
- **ArduPilot** : Plateforme autopilot open-source
- **Shapely** : Géométrie computationnelle efficace

## 🆘 Support

- **Issues** : [GitHub Issues](https://github.com/adamaKomi/drone-autonome-2/issues)
- **Discussions** : [GitHub Discussions](https://github.com/adamaKomi/drone-autonome-2/discussions)
- **Documentation** : [Wiki](https://github.com/adamaKomi/drone-autonome-2/wiki)
- **Email** : adama.komi@example.com

---

⚡ **Développé avec ❤️ pour l'avenir de l'agriculture autonome et la pollinisation durable**
