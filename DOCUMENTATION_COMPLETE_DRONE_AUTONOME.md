# Documentation Exhaustive - Projet Drone Autonome de Pollinisation

## Table des Matières

1. [Vue d'ensemble du Projet](#vue-densemble-du-projet)
2. [Architecture Globale](#architecture-globale)
3. [Packages et Nœuds](#packages-et-nœuds)
4. [Services, Topics et Interfaces](#services-topics-et-interfaces)
5. [Workflow d'une Mission](#workflow-dune-mission)
6. [Guide de Développement](#guide-de-développement)
7. [Tests et Validation](#tests-et-validation)
8. [Intégration et Coordination](#intégration-et-coordination)
9. [Commandes et Utilisation](#commandes-et-utilisation)
10. [Dépannage](#dépannage)

---

## Vue d'ensemble du Projet

### Objectif Principal
Développer un système de drone autonome pour la pollinisation des cultures, capable de naviguer de manière autonome, éviter les obstacles, et effectuer des missions de pollinisation précises dans des zones agricoles définies.

### Fonctionnalités Clés
- **Navigation autonome** : Planification et suivi de trajectoires
- **Évitement d'obstacles** : Détection et contournement d'obstacles en temps réel
- **Géobarrières** : Respect des zones autorisées/interdites
- **Pollinisation précise** : Approche de précision pour cibler les fleurs
- **Sécurité** : Systèmes de sécurité redondants (RTL, atterrissage d'urgence)
- **Interface MAVROS** : Communication avec l'autopilote du drone

---

## Architecture Globale

### Diagramme de l'Architecture
```
┌─────────────────────────────────────────────────────────────────┐
│                         DRONE AUTONOME                          │
└─────────────────────────────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────────────────────────┐
│                         ROS2 HUMBLE                            │
└─────────────────────────────────────────────────────────────────┘
           │
           ▼
┌─────────────────┬─────────────────┬─────────────────┬────────────┐
│  drone_interface│ drone_navigation│  drone_mission  │drone_vision│
│                 │                 │                 │            │
│ • Interface     │ • Navigation    │ • Missions      │ • Vision   │
│   générale      │ • Contrôle      │ • Pollinisation │ • Détection│
│ • Sécurité      │ • Trajectoires  │ • Workflows     │ • Tracking │
│ • État drone    │ • Géobarrières  │ • Coordination  │ • Analyse  │
└─────────────────┴─────────────────┴─────────────────┴────────────┘
           │
           ▼
┌─────────────────────────────────────────────────────────────────┐
│                         MAVROS                                 │
│                    (Interface MAVLink)                         │
└─────────────────────────────────────────────────────────────────┘
           │
           ▼
┌─────────────────────────────────────────────────────────────────┐
│                     AUTOPILOTE                                 │
│                   (ArduCopter/PX4)                             │
└─────────────────────────────────────────────────────────────────┘
```

### Flux de Données
```
Mission Request → drone_mission → drone_navigation → drone_interface → MAVROS → Autopilote
                                      ↑                    ↓
                                 drone_vision         Télémétrie
                                      ↑                    ↓
                              Capteurs (caméra,      État drone
                              LiDAR, capteurs)
```

---

## Packages et Nœuds

### 1. Package `drone_msgs`
**Rôle** : Définit tous les messages et services personnalisés pour la communication entre les nœuds.

**Messages définis** :
- `Waypoint` : Point de navigation avec coordonnées GPS/locales
- `Trajectory` : Séquence de waypoints
- `MissionProgress` : Progression d'une mission
- `ZoneDefinition` : Définition de zones (géobarrières, pollinisation)
- `Velocity3D` : Vitesse 3D (x, y, z)
- `FlowerTarget` : Cible de pollinisation détectée

**Services définis** :
- `NavigateToWaypoint` : Navigation vers un point
- `SetTrajectory` : Définition d'une trajectoire
- `SetGeofence` : Configuration de géobarrières
- `StartMission` : Démarrage d'une mission
- `SetHome` : Définition du point de retour

### 2. Package `drone_interface`
**Rôle** : Interface principale avec le drone, gestion de la sécurité et de l'état général.

#### Nœud Principal : `drone_interface_node`
**Responsabilités** :
- Communication avec MAVROS
- Gestion des modes de vol (GUIDED, RTL, LAND, etc.)
- Armement/désarmement du drone
- Surveillance de la sécurité (batterie, communication, météo)
- Interface de haut niveau pour les autres packages

**Topics Publiés** :
- `/drone/status` (String) : État général du drone
- `/drone/safety_status` (String) : État de sécurité

**Services Fournis** :
- `/drone/arm` (Trigger) : Armement des moteurs
- `/drone/disarm` (Trigger) : Désarmement des moteurs
- `/drone/set_mode` (SetMode) : Changement de mode de vol
- `/drone/emergency_stop` (Trigger) : Arrêt d'urgence
- `/drone/health_check` (Trigger) : Vérification de santé
- `/drone/safety_check` (Trigger) : Vérification de sécurité

**Lifecycle States** :
- `unconfigured` → `inactive` → `active` → `inactive` → `finalized`

### 3. Package `drone_navigation`
**Rôle** : Navigation autonome, planification de trajectoires, contrôle de position.

#### Nœud Principal : `navigation_node`
**Responsabilités** :
- Planification de trajectoires optimisées
- Contrôle de position PID multi-axes
- Évitement d'obstacles en temps réel
- Gestion des géobarrières
- Génération de patterns de couverture
- Interface avec le contrôleur de position

**Topics Publiés** :
- `/drone_nav/navigation/status` (String) : État de navigation
- `/drone_nav/navigation/trajectory` (String) : Trajectoire active
- `/drone_nav/navigation/waypoint_reached` (String) : Waypoint atteint
- `/drone_nav/navigation/progress` (String) : Progression mission
- `/drone_nav/navigation/obstacles` (String) : Obstacles détectés
- `/mavros/setpoint_velocity/cmd_vel` (TwistStamped) : Commandes de vitesse

**Topics Souscrits** :
- `/mavros/local_position/pose` (PoseStamped) : Position locale
- `/mavros/state` (State) : État MAVROS

**Services Fournis** :
- `/drone_nav/navigation/goto_position` (Trigger) : Navigation vers position
- `/drone_nav/navigation/plan_path` (Trigger) : Planification de chemin
- `/drone_nav/navigation/start_mission` (Trigger) : Démarrage mission
- `/drone_nav/navigation/pause_mission` (Trigger) : Pause mission
- `/drone_nav/navigation/resume_mission` (Trigger) : Reprise mission
- `/drone_nav/navigation/stop_mission` (Trigger) : Arrêt mission
- `/drone_nav/navigation/emergency_rtl` (Trigger) : Retour d'urgence
- `/drone_nav/navigation/set_geofence` (Trigger) : Configuration géobarrière
- `/drone_nav/navigation/clear_geofence` (Trigger) : Suppression géobarrière

#### Modules de Navigation :

##### 1. `PositionController`
**Fonction** : Contrôle de position PID avancé
**Paramètres** : Gains PID pour X, Y, Z
**Méthodes clés** :
- `compute_velocity(position, target)` : Calcule vitesse nécessaire
- `is_waypoint_reached(position, target)` : Vérifie si cible atteinte
- `set_control_mode(mode)` : Change mode de contrôle
- `tune_gains(axis, gains)` : Ajuste gains PID

##### 2. `TrajectoryPlanner`
**Fonction** : Planification de trajectoires optimisées
**Algorithmes** : A*, RRT, Bézier
**Méthodes clés** :
- `plan_to_waypoint(target)` : Planifie vers un point
- `plan_trajectory(waypoints)` : Planifie trajectoire complète
- `replan_if_needed(obstacles)` : Replanifie si obstacles

##### 3. `PathOptimizer`
**Fonction** : Optimisation de chemins
**Algorithmes** : Génétique, plus proche voisin, 2-opt
**Méthodes clés** :
- `optimize_path(waypoints)` : Optimise ordre des points
- `smooth_path(waypoints)` : Lisse la trajectoire
- `reduce_waypoints(waypoints)` : Réduit points redondants

##### 4. `ObstacleAvoidance`
**Fonction** : Évitement d'obstacles en temps réel
**Méthodes** : Champs de potentiel, forces répulsives
**Méthodes clés** :
- `detect_obstacles()` : Détecte obstacles
- `avoid_obstacles(trajectory, obstacles)` : Modifie trajectoire
- `compute_avoidance_vector(obstacles)` : Calcule vecteur d'évitement

##### 5. `GeofenceManager`
**Fonction** : Gestion des géobarrières 3D
**Types** : Inclusion, exclusion, avertissement
**Méthodes clés** :
- `add_zone(zone)` : Ajoute zone
- `check_violation(position)` : Vérifie violations
- `is_position_safe(position)` : Teste sécurité position

##### 6. `CoveragePatterns`
**Fonction** : Génération de patterns de couverture
**Patterns** : Zigzag, spirale, grille, lawn mower
**Méthodes clés** :
- `generate_pattern(type, zone)` : Génère pattern
- `generate_zigzag_pattern(zone)` : Pattern zigzag
- `generate_spiral_pattern(zone)` : Pattern spirale

#### Outils CLI :
- `goto_position` : Navigation CLI vers position
- `plan_mission` : Planification mission CLI
- `nav_status` : État navigation CLI
- `nav_diagnostics` : Diagnostics navigation CLI
- `test_waypoints` : Test navigation CLI

### 4. Package `drone_mission`
**Rôle** : Gestion des missions de pollinisation, coordination des tâches.

#### Nœud Principal : `mission_node`
**Responsabilités** :
- Planification des missions de pollinisation
- Coordination entre navigation et vision
- Gestion des séquences de tâches
- Optimisation des parcours de pollinisation
- Interface utilisateur pour définition des missions

### 5. Package `drone_vision`
**Rôle** : Vision par ordinateur, détection et tracking des cibles.

#### Nœud Principal : `vision_node`
**Responsabilités** :
- Détection des fleurs et cibles de pollinisation
- Tracking d'objets en mouvement
- Estimation de pose des cibles
- Analyse de la qualité de pollinisation
- Interface avec caméras et capteurs visuels

### 6. Package `drone_system`
**Rôle** : Intégration système, monitoring global.

#### Nœud Principal : `system_monitor`
**Responsabilités** :
- Surveillance de l'état global du système
- Coordination entre tous les packages
- Gestion des alertes et erreurs
- Interface de supervision
- Logging et diagnostics système

---

## Services, Topics et Interfaces

### Topics de Communication

#### Topics de Navigation
```yaml
# Statut et progression
/drone_nav/navigation/status:
  type: std_msgs/String
  description: État actuel de la navigation (JSON)
  frequency: 5 Hz
  example: {"mission_active": true, "current_waypoint": 5}

/drone_nav/navigation/progress:
  type: std_msgs/String
  description: Progression de la mission
  frequency: 1 Hz
  example: {"completion": 45.5, "estimated_time": 120}

/drone_nav/navigation/waypoint_reached:
  type: std_msgs/String
  description: Notification qu'un waypoint est atteint
  frequency: On event
  example: "Waypoint 5 reached at coordinates x=10.5, y=15.2"

# Trajectoires et obstacles
/drone_nav/navigation/trajectory:
  type: std_msgs/String
  description: Trajectoire active
  frequency: 0.1 Hz
  example: {"waypoints": [...], "total_distance": 500.2}

/drone_nav/navigation/obstacles:
  type: std_msgs/String
  description: Obstacles détectés
  frequency: 10 Hz
  example: {"obstacles": [{"x": 10, "y": 5, "radius": 2}]}
```

#### Topics d'Interface Drone
```yaml
/drone/status:
  type: std_msgs/String
  description: État général du drone
  frequency: 5 Hz
  example: {"mode": "GUIDED", "armed": true, "battery": 85.5}

/drone/safety_status:
  type: std_msgs/String
  description: État de sécurité
  frequency: 1 Hz
  example: {"status": "OK", "warnings": [], "critical": []}
```

#### Topics MAVROS (Interface Autopilote)
```yaml
# Commandes
/mavros/setpoint_velocity/cmd_vel:
  type: geometry_msgs/TwistStamped
  description: Commandes de vitesse au drone
  frequency: 50 Hz (boucle de contrôle)
  
# Télémétrie
/mavros/local_position/pose:
  type: geometry_msgs/PoseStamped
  description: Position locale du drone
  frequency: 50 Hz

/mavros/state:
  type: mavros_msgs/State
  description: État MAVROS (mode, armé, connecté)
  frequency: 10 Hz
```

### Services de Navigation

#### Services de Base
```yaml
/drone_nav/navigation/goto_position:
  type: std_srvs/Trigger
  description: Navigation vers une position prédéfinie
  usage: "ros2 service call /drone_nav/navigation/goto_position std_srvs/srv/Trigger"
  
/drone_nav/navigation/plan_path:
  type: std_srvs/Trigger
  description: Planification d'un nouveau chemin
  usage: "ros2 service call /drone_nav/navigation/plan_path std_srvs/srv/Trigger"
```

#### Services de Mission
```yaml
/drone_nav/navigation/start_mission:
  type: std_srvs/Trigger
  description: Démarre une mission
  preconditions: ["drone armé", "mission définie", "sécurité OK"]
  
/drone_nav/navigation/pause_mission:
  type: std_srvs/Trigger
  description: Met en pause la mission active
  effect: "Maintien de position, arrêt progression"
  
/drone_nav/navigation/resume_mission:
  type: std_srvs/Trigger
  description: Reprend une mission en pause
  
/drone_nav/navigation/stop_mission:
  type: std_srvs/Trigger
  description: Arrête définitivement la mission
  effect: "Arrêt complet, retour manuel possible"
```

#### Services d'Urgence
```yaml
/drone_nav/navigation/emergency_rtl:
  type: std_srvs/Trigger
  description: Retour d'urgence à la base
  priority: "Critique - interrompt toute autre action"
  
/drone/emergency_stop:
  type: std_srvs/Trigger
  description: Arrêt d'urgence complet
  effect: "Arrêt moteurs ou atterrissage immédiat"
```

#### Services de Géobarrières
```yaml
/drone_nav/navigation/set_geofence:
  type: std_srvs/Trigger
  description: Configure une géobarrière
  
/drone_nav/navigation/clear_geofence:
  type: std_srvs/Trigger
  description: Supprime les géobarrières
```

#### Services d'Interface Drone
```yaml
/drone/arm:
  type: std_srvs/Trigger
  description: Arme les moteurs du drone
  preconditions: ["sécurité pré-vol OK", "mode approprié"]
  
/drone/disarm:
  type: std_srvs/Trigger
  description: Désarme les moteurs
  
/drone/set_mode:
  type: mavros_msgs/SetMode
  description: Change le mode de vol
  modes: ["GUIDED", "RTL", "LAND", "LOITER", "STABILIZE"]
  
/drone/health_check:
  type: std_srvs/Trigger
  description: Vérification de santé complète
  checks: ["batterie", "GPS", "IMU", "capteurs", "communication"]
  
/drone/safety_check:
  type: std_srvs/Trigger
  description: Vérification de sécurité pré-vol
  checks: ["météo", "géobarrières", "espace aérien", "équipement"]
```

---

## Workflow d'une Mission

### 1. Phase de Préparation

#### 1.1. Initialisation du Système
```bash
# Démarrage des nœuds principaux
ros2 launch drone_system full_system.launch.py

# Vérification de l'état des nœuds
ros2 lifecycle get /drone_interface
ros2 lifecycle get /drone_nav/navigation_node
ros2 lifecycle get /mission_node
ros2 lifecycle get /vision_node
```

#### 1.2. Configuration des Nœuds
```bash
# Configuration de l'interface drone
ros2 lifecycle set /drone_interface configure

# Configuration de la navigation
ros2 lifecycle set /drone_nav/navigation_node configure

# Configuration de la mission
ros2 lifecycle set /mission_node configure
```

#### 1.3. Vérifications de Sécurité
```bash
# Vérification de santé du drone
ros2 service call /drone/health_check std_srvs/srv/Trigger

# Vérification de sécurité pré-vol
ros2 service call /drone/safety_check std_srvs/srv/Trigger
```

### 2. Phase de Planification

#### 2.1. Définition de la Zone de Mission
```bash
# Configuration des géobarrières
ros2 service call /drone_nav/navigation/set_geofence std_srvs/srv/Trigger

# Définition de la zone de pollinisation
ros2 service call /mission/define_pollination_zone drone_msgs/srv/SetZone "{zone: {name: 'field_1', boundary: [...]}}"
```

#### 2.2. Génération du Pattern de Couverture
```python
# Via l'API Python ou service
pattern_type = "zigzag"
coverage_overlap = 20.0  # pourcentage
altitude = 15.0  # mètres

# Le système génère automatiquement les waypoints
waypoints = coverage_generator.generate_pattern(pattern_type, zone, overlap)
```

#### 2.3. Optimisation de la Trajectoire
```bash
# Optimisation automatique du chemin
ros2 service call /drone_nav/navigation/optimize_path std_srvs/srv/Trigger
```

### 3. Phase d'Exécution

#### 3.1. Activation du Système
```bash
# Activation de tous les nœuds
ros2 lifecycle set /drone_interface activate
ros2 lifecycle set /drone_nav/navigation_node activate
ros2 lifecycle set /mission_node activate
ros2 lifecycle set /vision_node activate
```

#### 3.2. Démarrage de la Mission
```bash
# Armement du drone
ros2 service call /drone/arm std_srvs/srv/Trigger

# Décollage
ros2 service call /drone/takeoff drone_msgs/srv/Takeoff "{altitude: 15.0}"

# Démarrage de la mission de navigation
ros2 service call /drone_nav/navigation/start_mission std_srvs/srv/Trigger

# Démarrage de la mission de pollinisation
ros2 service call /mission/start_pollination std_srvs/srv/Trigger
```

#### 3.3. Surveillance en Temps Réel
```bash
# Monitoring du statut
ros2 topic echo /drone_nav/navigation/status
ros2 topic echo /drone/status
ros2 topic echo /mission/progress

# Monitoring des obstacles
ros2 topic echo /drone_nav/navigation/obstacles

# Monitoring de la vision
ros2 topic echo /vision/detected_flowers
```

### 4. Boucle d'Exécution (50Hz)

#### 4.1. Navigation (navigation_node)
```python
def control_loop():
    # 1. Lecture de la position actuelle
    current_position = get_current_position()
    
    # 2. Vérification des géobarrières
    if geofence_manager.check_violation(current_position):
        trigger_emergency_rtl()
        return
    
    # 3. Détection d'obstacles
    obstacles = obstacle_avoidance.detect_obstacles()
    
    # 4. Évitement d'obstacles si nécessaire
    if obstacles:
        trajectory = obstacle_avoidance.avoid_obstacles(trajectory, obstacles)
    
    # 5. Contrôle de position
    if target_position:
        velocity = position_controller.compute_velocity(current_position, target_position)
        send_velocity_command(velocity)
        
        # 6. Vérification si waypoint atteint
        if position_controller.is_waypoint_reached(current_position, target_position):
            next_waypoint()
    
    # 7. Publication du statut
    publish_status()
```

#### 4.2. Vision (vision_node)
```python
def vision_loop():
    # 1. Acquisition d'image
    image = camera.capture()
    
    # 2. Détection de fleurs
    flowers = flower_detector.detect(image)
    
    # 3. Estimation de pose
    for flower in flowers:
        pose = pose_estimator.estimate_pose(flower)
        
        # 4. Publication des cibles détectées
        publish_flower_target(flower, pose)
    
    # 5. Feedback de pollinisation
    if pollination_in_progress:
        quality = assess_pollination_quality(image)
        publish_pollination_feedback(quality)
```

#### 4.3. Mission (mission_node)
```python
def mission_loop():
    # 1. Vérification de l'état de la mission
    if mission_active:
        # 2. Coordination navigation-vision
        if flower_detected():
            request_precision_approach()
        
        # 3. Gestion des séquences de pollinisation
        execute_pollination_sequence()
        
        # 4. Mise à jour de la progression
        update_mission_progress()
    
    # 5. Vérification des conditions d'arrêt
    check_mission_completion()
```

### 5. Événements Particuliers

#### 5.1. Détection d'Obstacle
```
1. ObstacleAvoidance.detect_obstacles() → obstacles détectés
2. Publication sur /drone_nav/navigation/obstacles
3. Modification de la trajectoire
4. Notification à mission_node
5. Replanification si nécessaire
```

#### 5.2. Violation de Géobarrière
```
1. GeofenceManager.check_violation() → violation détectée
2. Déclenchement automatique RTL
3. Arrêt de la mission en cours
4. Notification critique
5. Attente d'intervention manuelle
```

#### 5.3. Détection de Fleur
```
1. VisionNode détecte une fleur
2. Publication sur /vision/detected_flowers
3. MissionNode reçoit la notification
4. Demande d'approche de précision
5. NavigationNode passe en mode précision
6. Exécution de la séquence de pollinisation
```

#### 5.4. Batterie Faible
```
1. Surveillance continue de la batterie
2. Seuil critique atteint (ex: 20%)
3. Déclenchement automatique RTL
4. Notification d'urgence
5. Atterrissage de sécurité
```

### 6. Phase de Fin de Mission

#### 6.1. Fin Normale
```bash
# La mission se termine naturellement
# → Tous les waypoints atteints
# → Retour automatique au point de départ
# → Atterrissage
# → Désarmement
```

#### 6.2. Arrêt Manuel
```bash
# Arrêt demandé par l'opérateur
ros2 service call /drone_nav/navigation/stop_mission std_srvs/srv/Trigger
ros2 service call /mission/stop std_srvs/srv/Trigger
```

#### 6.3. Retour et Atterrissage
```bash
# Retour à la base
ros2 service call /drone_nav/navigation/return_to_home std_srvs/srv/Trigger

# Atterrissage
ros2 service call /drone/land std_srvs/srv/Trigger

# Désarmement
ros2 service call /drone/disarm std_srvs/srv/Trigger
```

---

## Guide de Développement

### Structure de Développement Recommandée

#### 1. Développement par Package
```
Phase 1: drone_msgs (Messages et services)
Phase 2: drone_interface (Interface de base)
Phase 3: drone_navigation (Navigation autonome)
Phase 4: drone_vision (Vision et détection)
Phase 5: drone_mission (Coordination et missions)
Phase 6: drone_system (Intégration globale)
```

#### 2. Développement par Nœud

##### A. drone_interface_node
```bash
# 1. Création de la structure
cd ~/ros2_ws/src/drone_interface/drone_interface/

# 2. Fichiers à créer
touch interface_node.py        # Nœud principal
touch safety_manager.py       # Gestionnaire de sécurité
touch mavros_interface.py     # Interface MAVROS
touch health_monitor.py       # Monitoring de santé

# 3. Implémentation progressive
# Étape 1: Communication MAVROS de base
# Étape 2: Gestion des modes de vol
# Étape 3: Système de sécurité
# Étape 4: Interface de haut niveau
```

##### B. navigation_node (déjà développé)
```bash
# Structure existante
drone_navigation/
├── navigation_node.py          ✅ Développé
├── position_controller.py      ✅ Développé
├── trajectory_planner.py       ✅ Développé
├── path_optimizer.py          ✅ Développé
├── obstacle_avoidance.py      ✅ Développé
├── geofence_manager.py        ✅ Développé
└── coverage_patterns.py       ✅ Développé
```

##### C. mission_node
```bash
# Fichiers à développer
touch mission_node.py           # Nœud principal
touch pollination_manager.py   # Gestionnaire pollinisation
touch mission_planner.py       # Planificateur de missions
touch coordination_manager.py  # Coordination des tâches
```

##### D. vision_node
```bash
# Fichiers à développer
touch vision_node.py           # Nœud principal
touch flower_detector.py       # Détecteur de fleurs
touch pose_estimator.py        # Estimateur de pose
touch tracking_manager.py      # Gestionnaire de tracking
```

### Étapes de Développement Détaillées

#### Étape 1: Finalisation des Messages (drone_msgs)

##### 1.1. Analyse des Besoins
```bash
# Révision des messages existants
cd ~/ros2_ws/src/drone_msgs/msg/

# Messages à créer/compléter
touch FlowerTarget.msg          # Cible de fleur détectée
touch PollinationStatus.msg     # État de pollinisation
touch MissionPlan.msg           # Plan de mission
touch SafetyAlert.msg           # Alerte de sécurité
```

##### 1.2. Définition des Messages
```yaml
# FlowerTarget.msg
Header header
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
float32 confidence
string flower_type
uint32 flower_id
bool already_pollinated

# PollinationStatus.msg
Header header
uint32 target_id
string status  # "approaching", "pollinating", "completed", "failed"
float32 quality_score
geometry_msgs/Point position
float64 timestamp

# MissionPlan.msg
Header header
string mission_id
string mission_type  # "pollination", "survey", "patrol"
ZoneDefinition coverage_zone
Waypoint[] waypoints
float32 estimated_duration
PollinationParameters pollination_params
```

##### 1.3. Services Avancés
```yaml
# StartPollinationMission.srv
MissionPlan mission_plan
---
bool success
string message
string mission_id
float32 estimated_completion_time

# UpdateMissionPlan.srv
string mission_id
MissionPlan updated_plan
---
bool success
string message

# GetMissionStatus.srv
string mission_id
---
bool success
MissionProgress progress
PollinationStatus[] completed_targets
```

#### Étape 2: Développement drone_interface

##### 2.1. Interface MAVROS (mavros_interface.py)
```python
class MavrosInterface:
    def __init__(self):
        # Clients MAVROS
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        
    def arm_motors(self) -> bool:
        """Arme les moteurs"""
        
    def disarm_motors(self) -> bool:
        """Désarme les moteurs"""
        
    def set_flight_mode(self, mode: str) -> bool:
        """Change le mode de vol"""
        
    def takeoff(self, altitude: float) -> bool:
        """Décollage à une altitude donnée"""
        
    def land(self) -> bool:
        """Atterrissage"""
        
    def return_to_launch(self) -> bool:
        """Retour au point de lancement"""
```

##### 2.2. Gestionnaire de Sécurité (safety_manager.py)
```python
class SafetyManager:
    def __init__(self):
        self.safety_checks = {
            'battery': self.check_battery,
            'gps': self.check_gps,
            'communication': self.check_communication,
            'weather': self.check_weather,
            'sensors': self.check_sensors
        }
        
    def perform_safety_check(self) -> SafetyStatus:
        """Effectue toutes les vérifications de sécurité"""
        
    def check_battery(self) -> bool:
        """Vérifie le niveau de batterie"""
        
    def check_gps(self) -> bool:
        """Vérifie la qualité GPS"""
        
    def monitor_flight_safety(self):
        """Surveillance continue pendant le vol"""
```

##### 2.3. Monitoring de Santé (health_monitor.py)
```python
class HealthMonitor:
    def __init__(self):
        self.health_parameters = {
            'cpu_usage': 0.0,
            'memory_usage': 0.0,
            'disk_usage': 0.0,
            'temperature': 0.0,
            'communication_quality': 0.0
        }
        
    def get_system_health(self) -> HealthStatus:
        """Retourne l'état de santé du système"""
        
    def diagnose_issues(self) -> List[str]:
        """Diagnostique les problèmes potentiels"""
```

#### Étape 3: Extension drone_navigation

##### 3.1. Outils CLI Complets
```bash
# Développement des outils CLI
cd ~/ros2_ws/src/drone_navigation/drone_navigation/tools/

# goto_position.py - Déjà développé ✅

# plan_mission.py
cat > plan_mission.py << 'EOF'
#!/usr/bin/env python3
"""Outil CLI pour planification de mission"""

class MissionPlannerTool:
    def plan_coverage_mission(self, zone_file: str, pattern: str):
        """Planifie une mission de couverture"""
        
    def plan_pollination_mission(self, targets_file: str):
        """Planifie une mission de pollinisation"""
        
    def optimize_mission(self, mission_file: str):
        """Optimise un plan de mission"""
EOF

# nav_status.py
cat > nav_status.py << 'EOF'
#!/usr/bin/env python3
"""Outil CLI pour état de navigation"""

class NavigationStatusTool:
    def get_navigation_status(self):
        """Affiche l'état de navigation"""
        
    def get_mission_progress(self):
        """Affiche la progression de mission"""
        
    def get_safety_status(self):
        """Affiche l'état de sécurité"""
EOF
```

##### 3.2. Amélioration des Modules
```python
# position_controller.py - Ajout de modes avancés
class PositionController:
    def set_precision_mode(self, enabled: bool):
        """Active/désactive le mode précision pour pollinisation"""
        
    def approach_target(self, target: FlowerTarget):
        """Approche précise d'une cible de pollinisation"""
        
    def maintain_hover(self, position: Point, duration: float):
        """Maintien de position stationnaire"""

# obstacle_avoidance.py - Intégration avec vision
class ObstacleAvoidance:
    def integrate_vision_obstacles(self, vision_obstacles):
        """Intègre les obstacles détectés par vision"""
        
    def dynamic_replanning(self, current_trajectory, obstacles):
        """Replanification dynamique en temps réel"""
```

#### Étape 4: Développement drone_vision

##### 4.1. Nœud Principal (vision_node.py)
```python
class VisionNode(LifecycleNode):
    def __init__(self):
        super().__init__('vision_node')
        
        # Modules de vision
        self.flower_detector = None
        self.pose_estimator = None
        self.tracking_manager = None
        
        # Publishers
        self.detected_flowers_pub = None
        self.pose_estimates_pub = None
        self.tracking_results_pub = None
        
        # Subscribers
        self.camera_sub = None
        self.depth_sub = None
        
    def on_configure(self):
        """Configuration du nœud vision"""
        
    def process_camera_frame(self, image_msg):
        """Traite une image de la caméra"""
        
    def detect_and_track_flowers(self, image):
        """Détecte et suit les fleurs"""
```

##### 4.2. Détecteur de Fleurs (flower_detector.py)
```python
class FlowerDetector:
    def __init__(self):
        # Modèle de détection (YOLO, CNN, etc.)
        self.detection_model = None
        
    def detect_flowers(self, image) -> List[FlowerTarget]:
        """Détecte les fleurs dans une image"""
        
    def classify_flower_type(self, flower_image) -> str:
        """Classifie le type de fleur"""
        
    def assess_pollination_need(self, flower) -> float:
        """Évalue le besoin de pollinisation"""
```

##### 4.3. Estimateur de Pose (pose_estimator.py)
```python
class PoseEstimator:
    def __init__(self):
        self.camera_matrix = None
        self.distortion_coeffs = None
        
    def estimate_flower_pose(self, flower_detection, depth_info):
        """Estime la pose 3D d'une fleur"""
        
    def compute_approach_vector(self, flower_pose):
        """Calcule le vecteur d'approche optimal"""
```

#### Étape 5: Développement drone_mission

##### 5.1. Nœud Principal (mission_node.py)
```python
class MissionNode(LifecycleNode):
    def __init__(self):
        super().__init__('mission_node')
        
        # Gestionnaires
        self.pollination_manager = None
        self.mission_planner = None
        self.coordination_manager = None
        
        # État de mission
        self.current_mission = None
        self.mission_progress = None
        
    def start_pollination_mission(self, mission_plan):
        """Démarre une mission de pollinisation"""
        
    def coordinate_navigation_vision(self):
        """Coordonne navigation et vision"""
```

##### 5.2. Gestionnaire de Pollinisation (pollination_manager.py)
```python
class PollinationManager:
    def __init__(self):
        self.pollination_sequences = {}
        self.active_targets = []
        
    def execute_pollination_sequence(self, target: FlowerTarget):
        """Exécute une séquence de pollinisation"""
        
    def monitor_pollination_quality(self):
        """Surveille la qualité de pollinisation"""
        
    def update_completion_status(self):
        """Met à jour l'état de completion"""
```

### Tests et Validation

#### Tests Unitaires
```bash
# Structure des tests
mkdir -p test/unit/
cd test/unit/

# Tests par module
test_position_controller.py    # Tests contrôleur PID
test_trajectory_planner.py     # Tests planification
test_obstacle_avoidance.py     # Tests évitement
test_geofence_manager.py       # Tests géobarrières
test_coverage_patterns.py      # Tests patterns
```

#### Tests d'Intégration
```bash
# Tests d'intégration
mkdir -p test/integration/
cd test/integration/

test_navigation_integration.py  # Test navigation complète
test_mission_integration.py     # Test mission complète
test_safety_integration.py      # Test systèmes de sécurité
```

#### Tests en Simulation
```bash
# Configuration Gazebo
mkdir -p simulation/worlds/
mkdir -p simulation/models/
mkdir -p simulation/launch/

# Monde de simulation
simulation/worlds/pollination_field.world

# Modèles
simulation/models/drone_pollinator/
simulation/models/flower_field/
simulation/models/obstacles/

# Lancement simulation
simulation/launch/simulation.launch.py
```

#### Tests Terrain
```bash
# Protocole de test terrain
1. Tests de sécurité pré-vol
2. Tests de communication
3. Tests de navigation simple
4. Tests d'évitement d'obstacles
5. Tests de précision
6. Tests de mission complète
```

---

## Commandes et Utilisation

### Commandes de Base

#### Démarrage du Système
```bash
# Terminal 1: Simulation ArduCopter
sim_vehicle.py -v ArduCopter --console --map --out=127.0.0.1:14550

# Terminal 2: MAVROS
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555

# Terminal 3: Interface Drone
ros2 run drone_interface interface_node

# Terminal 4: Navigation
ros2 run drone_navigation navigation_node

# Terminal 5: Mission
ros2 run drone_mission mission_node

# Terminal 6: Vision
ros2 run drone_vision vision_node
```

#### Configuration Lifecycle
```bash
# Configuration de tous les nœuds
ros2 lifecycle set /drone_interface configure
ros2 lifecycle set /drone_nav/navigation_node configure
ros2 lifecycle set /mission_node configure
ros2 lifecycle set /vision_node configure

# Activation
ros2 lifecycle set /drone_interface activate
ros2 lifecycle set /drone_nav/navigation_node activate
ros2 lifecycle set /mission_node activate
ros2 lifecycle set /vision_node activate
```

#### Missions Basiques
```bash
# Navigation simple
ros2 run drone_navigation goto_position --x 10 --y 10 --z 15

# Mission de couverture
ros2 run drone_navigation plan_mission --pattern zigzag --zone field1.yaml

# Statut navigation
ros2 run drone_navigation nav_status

# Diagnostics
ros2 run drone_navigation nav_diagnostics
```

### Monitoring et Surveillance

#### Topics de Surveillance
```bash
# Statut général
ros2 topic echo /drone/status
ros2 topic echo /drone_nav/navigation/status
ros2 topic echo /mission/status

# Progression
ros2 topic echo /drone_nav/navigation/progress
ros2 topic echo /mission/progress

# Sécurité
ros2 topic echo /drone/safety_status
ros2 topic echo /diagnostics

# Position et navigation
ros2 topic echo /mavros/local_position/pose
ros2 topic echo /mavros/state
```

#### Logs et Diagnostics
```bash
# Logs en temps réel
ros2 run rqt_console rqt_console

# Graphe des nœuds
ros2 run rqt_graph rqt_graph

# Monitoring des paramètres
ros2 run rqt_reconfigure rqt_reconfigure

# Plot des données
ros2 run rqt_plot rqt_plot
```

### Commandes d'Urgence

#### Arrêt d'Urgence
```bash
# Arrêt d'urgence complet
ros2 service call /drone/emergency_stop std_srvs/srv/Trigger

# Retour d'urgence
ros2 service call /drone_nav/navigation/emergency_rtl std_srvs/srv/Trigger

# Atterrissage d'urgence
ros2 service call /drone/land std_srvs/srv/Trigger
```

#### Arrêt Propre
```bash
# Arrêt de mission
ros2 service call /mission/stop std_srvs/srv/Trigger
ros2 service call /drone_nav/navigation/stop_mission std_srvs/srv/Trigger

# Désactivation des nœuds
ros2 lifecycle set /vision_node deactivate
ros2 lifecycle set /mission_node deactivate
ros2 lifecycle set /drone_nav/navigation_node deactivate
ros2 lifecycle set /drone_interface deactivate
```

---

## Intégration et Coordination

### Coordination entre Packages

#### 1. Interface Drone ↔ Navigation
```yaml
Communication:
  - drone_interface publie état drone
  - navigation_node souscrit à l'état
  - navigation_node envoie commandes via MAVROS
  - drone_interface surveille sécurité

Topics:
  /drone/status → /drone_nav/navigation_node
  /mavros/setpoint_velocity/cmd_vel ← /drone_nav/navigation_node
  
Services:
  /drone/emergency_stop ← /drone_nav/navigation_node
```

#### 2. Navigation ↔ Mission
```yaml
Communication:
  - mission_node planifie trajectoires
  - navigation_node exécute trajectoires
  - mission_node coordonne avec vision
  - navigation_node fournit statut

Topics:
  /mission/trajectory → /drone_nav/navigation_node
  /drone_nav/navigation/status → /mission_node
  /drone_nav/navigation/waypoint_reached → /mission_node

Services:
  /drone_nav/navigation/start_mission ← /mission_node
  /drone_nav/navigation/set_trajectory ← /mission_node
```

#### 3. Mission ↔ Vision
```yaml
Communication:
  - vision_node détecte cibles
  - mission_node gère séquences pollinisation
  - vision_node évalue qualité
  - mission_node optimise parcours

Topics:
  /vision/detected_flowers → /mission_node
  /vision/pose_estimates → /mission_node
  /mission/pollination_commands → /vision_node

Services:
  /vision/start_detection ← /mission_node
  /mission/update_targets ← /vision_node
```

### Séquences de Coordination

#### Séquence de Démarrage
```
1. drone_interface.configure()
2. drone_interface.activate()
3. Vérification santé système
4. navigation_node.configure()
5. navigation_node.activate()
6. mission_node.configure()
7. mission_node.activate()
8. vision_node.configure()
9. vision_node.activate()
10. Système prêt
```

#### Séquence de Mission
```
1. Mission définie par l'utilisateur
2. mission_node planifie trajectoire
3. navigation_node reçoit trajectoire
4. Armement et décollage
5. Démarrage navigation
6. vision_node active détection
7. Boucle coordination mission
8. Retour et atterrissage
```

#### Séquence d'Urgence
```
1. Détection d'urgence (n'importe quel nœud)
2. Propagation alerte d'urgence
3. Arrêt missions en cours
4. Activation mode sécurité
5. RTL ou atterrissage d'urgence
6. Désarmement
7. État sécurisé
```

### Architecture de Communication

#### Topics de Coordination Globale
```yaml
/system/global_status:
  type: std_msgs/String
  description: État global du système
  publishers: [system_monitor]
  subscribers: [tous les nœuds]

/system/emergency_alert:
  type: std_msgs/String
  description: Alertes d'urgence globales
  publishers: [tous les nœuds]
  subscribers: [tous les nœuds]

/system/coordination_commands:
  type: std_msgs/String
  description: Commandes de coordination
  publishers: [mission_node, system_monitor]
  subscribers: [navigation_node, vision_node]
```

#### Services de Coordination
```yaml
/system/global_emergency_stop:
  type: std_srvs/Trigger
  description: Arrêt d'urgence global
  
/system/reset_all_nodes:
  type: std_srvs/Trigger
  description: Reset de tous les nœuds
  
/system/get_system_status:
  type: system_msgs/GetSystemStatus
  description: État complet du système
```

---

## Configuration et Paramètres

### Fichiers de Configuration

#### navigation_params.yaml
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
      max_velocity_xy: 10.0
      max_velocity_z: 5.0
      max_acceleration: 5.0
      max_jerk: 10.0
  
  trajectory:
    planning:
      lookahead_distance: 5.0
      path_resolution: 0.5
      smoothing_factor: 0.5
      turn_radius_min: 2.0
    optimization:
      algorithm: "genetic"
      population_size: 50
      generations: 100
      mutation_rate: 0.1
  
  coverage:
    patterns:
      zigzag:
        overlap_percentage: 20.0
        turn_radius: 2.0
      spiral:
        spacing: 1.0
        max_radius: 50.0
      lawn_mower:
        stripe_width: 2.0
        turn_style: "sharp"
  
  obstacles:
    detection:
      enabled: true
      safety_margin: 2.0
      max_detection_range: 20.0
    avoidance:
      method: "potential_field"
      repulsion_strength: 1.0
      attraction_strength: 0.5
  
  geofence:
    enabled: true
    action: "rtl"
    buffer_distance: 5.0
    altitude_limits:
      min: 5.0
      max: 120.0
  
  safety:
    emergency:
      rtl_altitude: 20.0
      emergency_descent_rate: 2.0
      communication_timeout: 30.0
    limits:
      max_tilt_angle: 30.0
      min_battery_rtl: 20.0
      max_wind_speed: 15.0
  
  performance:
    update_rates:
      position_control: 50.0
      trajectory_planning: 10.0
      obstacle_detection: 20.0
      status_publishing: 5.0
```

#### mission_params.yaml
```yaml
mission:
  pollination:
    approach_altitude: 2.0
    hover_duration: 3.0
    pollination_duration: 5.0
    success_threshold: 0.8
    max_attempts: 3
  
  coverage:
    default_altitude: 15.0
    photo_overlap: 60.0
    speed_max: 5.0
    turn_radius: 3.0
  
  coordination:
    vision_timeout: 10.0
    navigation_timeout: 30.0
    retry_attempts: 3
    abort_threshold: 0.1
```

#### vision_params.yaml
```yaml
vision:
  camera:
    resolution: [1920, 1080]
    fps: 30
    fov: 60.0
  
  detection:
    confidence_threshold: 0.7
    nms_threshold: 0.4
    max_detections: 50
  
  tracking:
    max_disappeared: 30
    max_distance: 50
  
  pollination:
    quality_threshold: 0.8
    assessment_duration: 2.0
```

### Lancement Système Complet

#### Fichier de Lancement Principal
```python
# launch/full_system.launch.py
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    return LaunchDescription([
        # Arguments
        DeclareLaunchArgument('use_sim', default_value='true'),
        DeclareLaunchArgument('world_file', default_value='pollination_world.world'),
        
        # Nœuds principaux
        LifecycleNode(
            package='drone_interface',
            executable='interface_node',
            name='drone_interface',
            parameters=['/path/to/interface_params.yaml']
        ),
        
        LifecycleNode(
            package='drone_navigation',
            executable='navigation_node',
            name='navigation_node',
            namespace='drone_nav',
            parameters=['/path/to/navigation_params.yaml']
        ),
        
        LifecycleNode(
            package='drone_mission',
            executable='mission_node',
            name='mission_node',
            parameters=['/path/to/mission_params.yaml']
        ),
        
        LifecycleNode(
            package='drone_vision',
            executable='vision_node',
            name='vision_node',
            parameters=['/path/to/vision_params.yaml']
        ),
        
        # System monitor
        LifecycleNode(
            package='drone_system',
            executable='system_monitor',
            name='system_monitor'
        )
    ])
```

### Scripts de Démarrage

#### start_simulation.sh
```bash
#!/bin/bash
echo "Démarrage simulation drone autonome..."

# Démarrage ArduCopter SITL
gnome-terminal -- bash -c "sim_vehicle.py -v ArduCopter --console --map --out=127.0.0.1:14550; exec bash"

# Attente démarrage SITL
sleep 5

# Démarrage MAVROS
gnome-terminal -- bash -c "source ~/ros2_ws/install/setup.bash && ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555; exec bash"

# Attente MAVROS
sleep 3

# Démarrage système complet
gnome-terminal -- bash -c "source ~/ros2_ws/install/setup.bash && ros2 launch drone_system full_system.launch.py; exec bash"

echo "Système démarré. Configurez les nœuds avec configure_nodes.sh"
```

#### configure_nodes.sh
```bash
#!/bin/bash
echo "Configuration des nœuds lifecycle..."

# Attente que tous les nœuds soient prêts
sleep 5

# Configuration
ros2 lifecycle set /drone_interface configure
ros2 lifecycle set /drone_nav/navigation_node configure
ros2 lifecycle set /mission_node configure
ros2 lifecycle set /vision_node configure
ros2 lifecycle set /system_monitor configure

# Activation
ros2 lifecycle set /drone_interface activate
ros2 lifecycle set /drone_nav/navigation_node activate
ros2 lifecycle set /mission_node activate
ros2 lifecycle set /vision_node activate
ros2 lifecycle set /system_monitor activate

echo "Tous les nœuds sont actifs!"
```

#### stop_system.sh
```bash
#!/bin/bash
echo "Arrêt du système..."

# Arrêt des missions
ros2 service call /mission/stop std_srvs/srv/Trigger
ros2 service call /drone_nav/navigation/stop_mission std_srvs/srv/Trigger

# Atterrissage si en vol
ros2 service call /drone/land std_srvs/srv/Trigger

# Désactivation des nœuds
ros2 lifecycle set /vision_node deactivate
ros2 lifecycle set /mission_node deactivate
ros2 lifecycle set /drone_nav/navigation_node deactivate
ros2 lifecycle set /drone_interface deactivate
ros2 lifecycle set /system_monitor deactivate

echo "Système arrêté proprement."
```

---

## Dépannage et Debugging

### Problèmes Courants

#### 1. Nœud ne se configure pas
```bash
# Vérification logs
ros2 topic echo /rosout | grep ERROR

# État des paramètres
ros2 param list /drone_nav/navigation_node

# Reset du nœud
ros2 lifecycle set /drone_nav/navigation_node cleanup
ros2 lifecycle set /drone_nav/navigation_node configure
```

#### 2. Services non disponibles
```bash
# Vérification services actifs
ros2 service list | grep drone

# Test de service
ros2 service call /drone_nav/navigation/goto_position std_srvs/srv/Trigger
```

#### 3. Topics sans données
```bash
# Vérification topics
ros2 topic list
ros2 topic info /drone_nav/navigation/status
ros2 topic hz /mavros/local_position/pose
```

#### 4. MAVROS non connecté
```bash
# Vérification MAVROS
ros2 topic echo /mavros/state

# Reconnexion
ros2 service call /mavros/set_stream_rate mavros_msgs/srv/StreamRate "{stream_id: 0, message_rate: 10, on_off: true}"
```

### Outils de Debug

#### 1. Monitoring Complet
```bash
# Script de monitoring
#!/bin/bash
echo "=== Monitoring Système Drone ==="

echo "Nœuds actifs:"
ros2 node list | grep -E "(drone|nav|mission|vision)"

echo -e "\nÉtat lifecycle:"
ros2 lifecycle get /drone_interface
ros2 lifecycle get /drone_nav/navigation_node
ros2 lifecycle get /mission_node
ros2 lifecycle get /vision_node

echo -e "\nServices disponibles:"
ros2 service list | grep -E "(drone|nav|mission|vision)" | head -10

echo -e "\nTopics actifs:"
ros2 topic list | grep -E "(drone|nav|mission|vision)" | head -10

echo -e "\nStatut MAVROS:"
timeout 2s ros2 topic echo /mavros/state --once
```

#### 2. Logs Structurés
```bash
# Sauvegarde logs
ros2 bag record -a -o mission_logs_$(date +%Y%m%d_%H%M%S)

# Analyse logs
ros2 bag info mission_logs_XXXXXXXX
ros2 bag play mission_logs_XXXXXXXX
```

#### 3. Simulation de Problèmes
```bash
# Test évitement obstacles
ros2 topic pub /drone_nav/navigation/obstacles std_msgs/msg/String "{data: '{\"obstacles\": [{\"x\": 10, \"y\": 5, \"radius\": 2}]}'}"

# Test alerte sécurité
ros2 topic pub /system/emergency_alert std_msgs/msg/String "{data: 'EMERGENCY: Low battery'}"

# Test géobarrière
ros2 service call /drone_nav/navigation/set_geofence std_srvs/srv/Trigger
```

---

## Conclusion

Cette documentation exhaustive couvre tous les aspects du projet de drone autonome de pollinisation. Elle fournit :

1. **Architecture complète** : Structure des packages, nœuds et communication
2. **Interfaces détaillées** : Tous les services, topics et messages
3. **Workflow de mission** : Déroulement complet d'une mission
4. **Guide de développement** : Étapes pour implémenter chaque composant
5. **Commandes pratiques** : Utilisation opérationnelle du système
6. **Intégration système** : Coordination entre tous les composants

Le système est conçu pour être :
- **Modulaire** : Chaque package a une responsabilité claire
- **Robuste** : Multiples niveaux de sécurité et redondance
- **Extensible** : Architecture permettant l'ajout de nouvelles fonctionnalités
- **Maintenable** : Code structuré et bien documenté

Pour commencer le développement, il est recommandé de :
1. Finaliser `drone_msgs` avec tous les messages nécessaires
2. Compléter `drone_interface` pour l'interface MAVROS
3. Étendre `drone_navigation` avec les outils CLI
4. Développer `drone_vision` pour la détection
5. Implémenter `drone_mission` pour la coordination
6. Tester chaque composant individuellement puis ensemble

Cette approche progressive garantit un développement méthodique et une intégration réussie de tous les composants.
