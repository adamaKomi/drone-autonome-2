# 🧭 PROMPT IA - TRANSFORMATION PACKAGE DRONE_NAVIGATION

## 🎯 OBJECTIF
Transforme complètement le package `drone_navigation` en suivant EXACTEMENT la même structure, qualité et méthodologie que le package `drone_interface` fourni en exemple.

## 📋 CONTEXTE DU PROJET
Le package `drone_navigation` fait partie d'un **système de drone autonome pour la pollinisation de fleurs**. Il doit fournir des capacités de navigation avancées pour permettre au drone de :
- Naviguer de manière autonome vers des zones de pollinisation
- Planifier des trajectoires optimisées pour couvrir efficacement les zones de fleurs
- Effectuer des approches de précision pour la pollinisation (±0.5m)
- Éviter les obstacles et respecter les géobarrières
- S'intégrer parfaitement avec les autres composants du système

## 🏗️ ARCHITECTURE REQUISE

### Structure EXACTE à implémenter :
```
drone_navigation/
├── drone_navigation/                    # Package Python principal
│   ├── __init__.py
│   ├── navigation_node.py              # Nœud principal avec lifecycle ROS2
│   ├── trajectory_planner.py           # Planificateur de trajectoires avancé
│   ├── position_controller.py          # Contrôleur de position PID avancé
│   ├── path_optimizer.py               # Optimiseur de chemins intelligents
│   ├── obstacle_avoidance.py           # Système d'évitement d'obstacles
│   ├── geofence_manager.py             # Gestionnaire de géobarrières
│   ├── coverage_patterns.py            # Patterns de couverture (zigzag, spiral)
│   └── tools/                          # Outils CLI complets
│       ├── __init__.py
│       ├── goto_position.py            # Navigation CLI vers position
│       ├── plan_mission.py             # Planification mission CLI
│       ├── nav_status.py               # État navigation CLI
│       ├── nav_diagnostics.py          # Diagnostics navigation CLI
│       └── test_waypoints.py           # Test navigation CLI
├── launch/                             # Fichiers de lancement
│   └── navigation_launch.py            # Lancement complet navigation
├── config/                             # Configuration YAML
│   ├── navigation_params.yaml          # Paramètres principaux
│   ├── pid_tuning.yaml                 # Réglages PID
│   ├── geofence_zones.yaml             # Zones géographiques
│   └── coverage_patterns.yaml          # Patterns de couverture
├── test/                               # Tests complets
│   ├── test_navigation.py              # Tests navigation
│   ├── test_trajectory.py              # Tests trajectoires
│   ├── test_controllers.py             # Tests contrôleurs
│   └── test_patterns.py               # Tests patterns
├── scripts/                            # Scripts utilitaires
│   ├── calibrate_navigation.py         # Calibration navigation
│   └── tune_pid.py                     # Réglage PID automatique
├── resource/                           # Ressources package
│   └── drone_navigation
├── setup.py                           # Configuration Python
├── package.xml                        # Métadonnées ROS2
└── README.md                          # Documentation exhaustive
```

## 🔧 SPÉCIFICATIONS TECHNIQUES DÉTAILLÉES

### 1. NAVIGATION NODE (navigation_node.py)
**Caractéristiques obligatoires :**
- **Lifecycle management complet** (configure/activate/deactivate/cleanup)
- **Threading sécurisé** avec ReentrantCallbackGroup
- **Integration MAVROS** pour position targets et telemetry
- **Recovery automatique** en cas de perte de connexion
- **Performance temps réel** (<100ms response time)

**Services exposés (OBLIGATOIRES) :**
```python
/drone_nav/goto_position         # Navigation vers position GPS
/drone_nav/goto_local           # Navigation position locale
/drone_nav/plan_path            # Planification trajectoire
/drone_nav/set_waypoints        # Définir liste waypoints
/drone_nav/start_mission        # Démarrer mission navigation
/drone_nav/pause_mission        # Pause mission
/drone_nav/resume_mission       # Reprendre mission
/drone_nav/emergency_rtl        # Retour d'urgence
/drone_nav/set_geofence         # Configuration géobarrière
/drone_nav/clear_geofence       # Effacer géobarrière
/drone_nav/set_home             # Définir position home
```

**Topics publiés (OBLIGATOIRES) :**
```python
/drone_nav/status               # État navigation détaillé (JSON)
/drone_nav/trajectory           # Trajectoire planifiée actuelle
/drone_nav/waypoint_reached     # Événements waypoints atteints
/drone_nav/path_progress        # Progression du chemin
/drone_nav/obstacles_detected   # Obstacles détectés
/diagnostics                    # Diagnostics système
```

### 2. TRAJECTORY PLANNER (trajectory_planner.py)
**Algorithmes OBLIGATOIRES :**
- **A* pathfinding** pour navigation avec obstacles
- **RRT* (Rapidly-exploring Random Tree)** pour espaces complexes
- **Dijkstra** pour chemins optimaux
- **Bézier curves** pour lissage de trajectoires
- **Dubins paths** pour contraintes de virage

**Patterns de couverture OBLIGATOIRES :**
```python
class CoveragePatterns:
    def zigzag_pattern(self, area_polygon, altitude, overlap=20)
    def spiral_pattern(self, center_point, radius, altitude)
    def lawn_mower_pattern(self, area_polygon, stripe_width)
    def boustrophedon_pattern(self, area_polygon, turn_radius)
    def adaptive_pattern(self, area_polygon, flower_density_map)
```

### 3. POSITION CONTROLLER (position_controller.py)
**Contrôleurs PID OBLIGATOIRES :**
- **Position XYZ** avec anti-windup
- **Velocity control** avec rampes d'accélération
- **Attitude control** pour stabilisation
- **Precision approach** pour pollinisation (<0.5m accuracy)

**Modes de contrôle :**
```python
class ControlModes:
    POSITION_HOLD = "position_hold"
    GUIDED = "guided" 
    FOLLOW_TRAJECTORY = "follow_trajectory"
    PRECISION_APPROACH = "precision_approach"
    EMERGENCY_LAND = "emergency_land"
```

### 4. PATH OPTIMIZER (path_optimizer.py)
**Optimisations OBLIGATOIRES :**
- **Algorithme génétique** pour optimisation multi-objectifs
- **Minimisation temps/distance/énergie**
- **Contraintes dynamiques** (vitesse, accélération, batterie)
- **Replanification temps réel** selon conditions

### 5. OBSTACLE AVOIDANCE (obstacle_avoidance.py)
**Méthodes OBLIGATOIRES :**
- **Potential Fields** pour répulsion/attraction
- **Dynamic Window Approach (DWA)** pour navigation dynamique
- **3D obstacle mapping** avec altitude management
- **Emergency maneuvers** (stop, hover, escape)

### 6. GEOFENCE MANAGER (geofence_manager.py)
**Fonctionnalités OBLIGATOIRES :**
- **Zones multiples** inclusion/exclusion
- **Polygones complexes** avec validation géométrique
- **Actions automatiques** (warning, RTL, emergency stop)
- **Altitude constraints** min/max
- **Dynamic updates** runtime

### 7. OUTILS CLI COMPLETS (tools/)

#### goto_position.py
```bash
ros2 run drone_navigation goto_position --lat 34.0 --lon -6.8 --alt 50
ros2 run drone_navigation goto_position --x 10 --y 20 --z 15 --frame local
ros2 run drone_navigation goto_position --waypoint home
```

#### plan_mission.py
```bash
ros2 run drone_navigation plan_mission --area polygon.kml --pattern zigzag
ros2 run drone_navigation plan_mission --coverage spiral --center "34.0,-6.8"
ros2 run drone_navigation plan_mission --waypoints waypoints.json
```

#### nav_status.py
```bash
ros2 run drone_navigation nav_status
ros2 run drone_navigation nav_status --continuous
ros2 run drone_navigation nav_status --json
```

#### nav_diagnostics.py
```bash
ros2 run drone_navigation nav_diagnostics
ros2 run drone_navigation nav_diagnostics --full
ros2 run drone_navigation nav_diagnostics --performance
```

## 📊 CONFIGURATION YAML COMPLÈTE

### navigation_params.yaml
```yaml
navigation:
  position_control:
    pid_gains:
      position:
        p_xy: 1.0
        i_xy: 0.1
        d_xy: 0.05
        p_z: 1.5
        i_z: 0.2
        d_z: 0.1
      velocity:
        p_xy: 0.8
        i_xy: 0.05
        d_xy: 0.02
    limits:
      max_velocity_xy: 10.0        # m/s
      max_velocity_z: 5.0          # m/s
      max_acceleration: 5.0        # m/s²
      max_jerk: 10.0              # m/s³
  
  trajectory:
    planning:
      lookahead_distance: 10.0     # m
      path_resolution: 1.0         # m
      smoothing_factor: 0.8
      turn_radius_min: 2.0         # m
    optimization:
      algorithm: "genetic"
      population_size: 50
      generations: 100
      mutation_rate: 0.1
  
  coverage:
    patterns:
      zigzag:
        overlap_percentage: 20
        turn_radius: 5.0
      spiral:
        spacing: 5.0
        max_radius: 100.0
      lawn_mower:
        stripe_width: 10.0
        turn_style: "sharp"
  
  obstacles:
    detection:
      enabled: true
      safety_margin: 2.0           # m
      max_detection_range: 50.0    # m
    avoidance:
      method: "potential_fields"
      repulsion_strength: 1.0
      attraction_strength: 0.5
  
  geofence:
    enabled: true
    action: "RTL"                  # RTL, LAND, HOVER, STOP
    buffer_distance: 5.0           # m
    altitude_limits:
      min: 5.0                     # m
      max: 120.0                   # m
  
  safety:
    emergency:
      rtl_altitude: 50.0           # m
      emergency_descent_rate: 2.0   # m/s
      communication_timeout: 10.0  # s
    limits:
      max_tilt_angle: 45.0         # degrees
      min_battery_rtl: 25.0        # %
      max_wind_speed: 15.0         # m/s
  
  performance:
    update_rates:
      position_control: 50.0       # Hz
      trajectory_planning: 10.0    # Hz
      obstacle_detection: 20.0     # Hz
      status_publishing: 5.0       # Hz
```

## 🧪 TESTS OBLIGATOIRES

### test_navigation.py
```python
class TestNavigation(unittest.TestCase):
    def test_lifecycle_management(self)
    def test_position_accuracy(self)
    def test_waypoint_navigation(self)
    def test_emergency_procedures(self)
    def test_mavros_integration(self)
```

### test_trajectory.py
```python
class TestTrajectoryPlanner(unittest.TestCase):
    def test_path_planning_algorithms(self)
    def test_coverage_patterns(self)
    def test_trajectory_optimization(self)
    def test_real_time_replanning(self)
```

## 🎯 INTÉGRATION SYSTÈME

### Avec drone_interface :
- État drone et commandes de vol
- Mode changes et armement
- Telemetry et diagnostics

### Avec drone_vision :
- Positions des fleurs détectées
- Coordination pour approche précision
- Visual servoing pour pollinisation

### Avec drone_mission :
- Coordination des tâches de navigation
- Exécution de missions complexes
- Reporting de progression

### Avec MAVROS :
- Position targets (SET_POSITION_TARGET_LOCAL_NED)
- Flight modes (GUIDED, AUTO, RTL)
- Position feedback et telemetry

## 🚀 EXEMPLES D'UTILISATION

### Mission de pollinisation :
```python
# Planification mission pour pollinisation
mission = await nav_node.plan_pollination_mission(
    area_polygon=garden_area,
    flower_positions=detected_flowers,
    approach_altitude=2.0,
    precision_radius=0.5
)

# Exécution avec approche précision
for waypoint in mission.waypoints:
    result = await nav_node.goto_position_precise(
        waypoint, 
        precision_radius=0.5,
        approach_speed=1.0,
        stabilization_time=2.0
    )
    if result.success:
        await nav_node.trigger_pollination_action()
```

### Configuration géobarrière :
```python
# Configuration zone sécurisée
geofence = GeofenceZone(
    polygon=safety_area,
    min_altitude=10.0,
    max_altitude=100.0,
    action=GeofenceAction.RTL,
    buffer_distance=5.0
)
await nav_node.set_geofence(geofence)
```

## 📏 MÉTRIQUES DE PERFORMANCE

### Objectifs OBLIGATOIRES :
- **Précision navigation** : ±0.5m pour approche fleurs
- **Response time** : <100ms pour commandes position
- **Path optimization** : >95% efficiency vs ligne droite
- **Obstacle avoidance** : 100% collision prevention
- **Memory usage** : <500MB total
- **CPU usage** : <30% sur Raspberry Pi 4
- **Update frequency** : 50Hz pour contrôle position

## 🔒 CONTRAINTES CRITIQUES

1. **AUCUN CMakeLists.txt** - Package Python pur uniquement
2. **Même qualité que drone_interface** - Architecture, documentation, tests
3. **CLI tools fonctionnels** - Testables immédiatement
4. **Performance temps réel** - Pas de latence >100ms
5. **Thread safety** - Accès concurrents sécurisés
6. **Error recovery** - Récupération automatique
7. **MAVROS integration** - Compatible ArduPilot SITL
8. **Documentation complète** - README détaillé comme drone_interface

## 📚 DOCUMENTATION README.md

Le README.md doit être **aussi détaillé que drone_interface** avec :
- Description complète du package
- Instructions d'installation
- Exemples d'utilisation pratiques
- Configuration des paramètres
- Guide de développement
- Troubleshooting complet
- API reference complète

---

**IMPORTANT : Respecte RIGOUREUSEMENT cette structure et ces spécifications. Chaque fichier doit être complet, fonctionnel et documenté au même niveau que drone_interface.**
