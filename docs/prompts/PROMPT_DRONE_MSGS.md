# 📡 PROMPT IA - TRANSFORMATION PACKAGE DRONE_MSGS

## 🎯 OBJECTIF
Transforme complètement le package `drone_msgs` en suivant EXACTEMENT la même structure, qualité et méthodologie que le package `drone_interface` fourni en exemple.

## 📋 CONTEXTE DU PROJET
Le package `drone_msgs` est le **système de communication central** du drone de pollinisation. Il définit tous les messages, services et actions ROS2 pour la communication inter-packages :
- Messages standardisés pour tous les composants
- Services synchrones pour actions critiques
- Actions asynchrones pour tâches longues
- Types de données complexes pour navigation/vision/mission
- Interface unifiée pour communication externe
- Validation et sérialisation des données

## 🏗️ ARCHITECTURE REQUISE

### Structure EXACTE à implémenter :
```
drone_msgs/
├── msg/                                # Messages ROS2
│   ├── DroneStatus.msg                 # État général drone
│   ├── DroneState.msg                  # État détaillé système
│   ├── FlightMode.msg                  # Mode de vol actuel
│   ├── SafetyStatus.msg                # État sécurité
│   ├── BatteryStatus.msg               # État batterie détaillé
│   ├── Position3D.msg                  # Position 3D précise
│   ├── Orientation.msg                 # Orientation quaternion
│   ├── Velocity3D.msg                  # Vélocité 3D
│   ├── NavigationCommand.msg           # Commande navigation
│   ├── Waypoint.msg                    # Point navigation
│   ├── Trajectory.msg                  # Trajectoire complète
│   ├── FlowerDetection.msg             # Détection fleur vision
│   ├── FlowerClassification.msg        # Classification fleur
│   ├── FlowerTarget.msg                # Cible fleur complète
│   ├── PollinationResult.msg           # Résultat pollinisation
│   ├── MissionStatus.msg               # État mission
│   ├── MissionProgress.msg             # Progression mission
│   ├── MissionWaypoint.msg             # Point mission
│   ├── ZoneDefinition.msg              # Définition zone
│   ├── EnvironmentData.msg             # Données environnement
│   ├── PerformanceMetrics.msg          # Métriques performance
│   ├── SystemAlert.msg                 # Alertes système
│   └── DiagnosticInfo.msg              # Info diagnostique
├── srv/                                # Services ROS2
│   ├── ArmDrone.srv                    # Armement drone
│   ├── DisarmDrone.srv                 # Désarmement drone
│   ├── SetFlightMode.srv               # Changement mode vol
│   ├── TakeoffDrone.srv                # Décollage
│   ├── LandDrone.srv                   # Atterrissage
│   ├── ReturnToHome.srv                # Retour maison
│   ├── EmergencyStop.srv               # Arrêt urgence
│   ├── SafetyCheck.srv                 # Vérification sécurité
│   ├── NavigateToWaypoint.srv          # Navigation point
│   ├── SetTrajectory.srv               # Définir trajectoire
│   ├── DetectFlowers.srv               # Détection fleurs
│   ├── ClassifyFlower.srv              # Classification fleur
│   ├── TrackFlower.srv                 # Suivi fleur
│   ├── PollinateFlower.srv             # Pollinisation
│   ├── PlanMission.srv                 # Planification mission
│   ├── StartMission.srv                # Démarrage mission
│   ├── PauseMission.srv                # Pause mission
│   ├── AbortMission.srv                # Abandon mission
│   ├── OptimizeMission.srv             # Optimisation mission
│   ├── GetSystemStatus.srv             # État système
│   ├── ConfigureSystem.srv             # Configuration système
│   └── GenerateReport.srv              # Génération rapport
├── action/                             # Actions ROS2
│   ├── ExecuteMission.action           # Exécution mission complète
│   ├── NavigateToPosition.action       # Navigation position
│   ├── FollowTrajectory.action         # Suivi trajectoire
│   ├── ApproachFlower.action           # Approche fleur
│   ├── PollinationSequence.action      # Séquence pollinisation
│   ├── ZoneCoverage.action             # Couverture zone
│   ├── DataCollection.action           # Collecte données
│   ├── SystemCalibration.action        # Calibration système
│   └── EmergencyLanding.action         # Atterrissage urgence
├── cmake/                              # Configuration CMake
│   └── drone_msgs-extras.cmake
├── test/                               # Tests messages
│   ├── test_msg_compilation.py         # Test compilation
│   ├── test_msg_serialization.py       # Test sérialisation
│   ├── test_srv_interfaces.py          # Test services
│   └── test_action_interfaces.py       # Test actions
├── scripts/                            # Scripts utilitaires
│   ├── validate_messages.py            # Validation messages
│   ├── generate_docs.py                # Génération documentation
│   └── test_interfaces.py              # Test interfaces
├── docs/                               # Documentation
│   ├── MESSAGE_REFERENCE.md            # Référence messages
│   ├── SERVICE_REFERENCE.md            # Référence services
│   ├── ACTION_REFERENCE.md             # Référence actions
│   └── INTEGRATION_GUIDE.md            # Guide intégration
├── CMakeLists.txt                      # Configuration CMake
├── package.xml                         # Métadonnées ROS2
└── README.md                           # Documentation principale
```

## 🔧 MESSAGES DÉTAILLÉS

### Messages État Système

#### DroneStatus.msg
```msg
# État général du drone de pollinisation
std_msgs/Header header

# État général
bool is_armed
bool is_flying
bool is_connected
bool is_healthy

# Mode de vol
string flight_mode              # MANUAL, AUTO, GUIDED, RTL, LAND, etc.
string mission_state           # IDLE, PLANNING, EXECUTING, PAUSED, COMPLETED

# Position et orientation
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
geometry_msgs/Twist velocity

# Systèmes
BatteryStatus battery
SafetyStatus safety
PerformanceMetrics performance

# Statut modules
bool navigation_active
bool vision_active
bool mission_active
bool communication_active

# Alertes et erreurs
SystemAlert[] active_alerts
string[] error_messages
uint8 severity_level           # 0=INFO, 1=WARNING, 2=ERROR, 3=CRITICAL
```

#### FlowerDetection.msg
```msg
# Détection de fleur par le système vision
std_msgs/Header header

# Identification
uint32 detection_id
float64 timestamp

# Position dans l'image
geometry_msgs/Point2D image_position
geometry_msgs/Polygon2D bounding_box
geometry_msgs/Polygon2D segmentation_mask

# Position 3D estimée
geometry_msgs/Point world_position
float64 distance_estimate
float64 position_confidence

# Caractéristiques visuelles
string flower_color
float64 estimated_size          # diamètre en cm
float64 detection_confidence

# Métadonnées
string camera_id
geometry_msgs/Point camera_position
geometry_msgs/Quaternion camera_orientation
```

#### MissionStatus.msg
```msg
# État détaillé de la mission en cours
std_msgs/Header header

# Identification mission
string mission_id
string mission_name
string mission_type             # SINGLE_FLOWER, ZONE_COVERAGE, SURVEY

# État d'exécution
string status                   # PLANNING, STARTING, EXECUTING, PAUSED, COMPLETED, ABORTED
float64 progress_percentage     # 0.0 - 100.0
builtin_interfaces/Time start_time
builtin_interfaces/Time estimated_completion
builtin_interfaces/Time actual_completion

# Objectifs mission
uint32 total_waypoints
uint32 completed_waypoints
uint32 total_flowers_targeted
uint32 flowers_pollinated
uint32 flowers_failed

# Performance actuelle
float64 current_speed           # m/s
float64 average_speed           # m/s
float64 energy_consumed         # Wh
float64 estimated_energy_total  # Wh

# Waypoint actuel
MissionWaypoint current_waypoint
MissionWaypoint next_waypoint

# Données collectées
uint32 images_captured
uint32 data_points_collected
float64 area_covered            # m²

# Conditions
EnvironmentData environment
string[] active_constraints
string[] encountered_issues
```

### Messages Navigation

#### Waypoint.msg
```msg
# Point de navigation 3D avec métadonnées
std_msgs/Header header

# Position géographique
float64 latitude               # degrés
float64 longitude              # degrés
float64 altitude               # mètres AGL
float64 heading                # degrés (0-360)

# Position locale (optionnelle)
geometry_msgs/Point local_position

# Paramètres navigation
float64 acceptance_radius      # mètres
float64 approach_speed         # m/s
float64 hover_time            # secondes
string navigation_mode         # DIRECT, SMOOTH, PRECISION

# Actions à effectuer
string[] actions              # "CAPTURE_IMAGE", "DETECT_FLOWERS", "POLLINATE"
float64 action_timeout        # secondes

# Contraintes
float64 max_approach_speed
float64 min_altitude
float64 max_altitude
bool require_gps_lock

# Métadonnées
string waypoint_type          # NAVIGATION, FLOWER_TARGET, SAFETY, HOME
uint32 sequence_number
string description
```

#### Trajectory.msg
```msg
# Trajectoire complète avec timing
std_msgs/Header header

# Identification
string trajectory_id
string trajectory_type         # DIRECT, SMOOTH, OPTIMAL, EMERGENCY

# Points de trajectoire
Waypoint[] waypoints
builtin_interfaces/Duration[] segment_durations
float64[] segment_speeds

# Paramètres globaux
float64 total_distance         # mètres
builtin_interfaces/Duration total_duration
float64 max_speed             # m/s
float64 max_acceleration      # m/s²

# Contraintes appliquées
float64 safety_margin         # mètres
bool avoid_obstacles
bool maintain_altitude
string[] geofence_constraints

# Optimisation
string optimization_objective  # TIME, ENERGY, SAFETY, PRECISION
float64 optimization_score
```

### Messages Vision

#### FlowerClassification.msg
```msg
# Classification détaillée d'une fleur
std_msgs/Header header

# Référence à la détection
uint32 detection_id
FlowerDetection detection

# Classification espèce
string species_name
string species_scientific_name
float64 species_confidence
string[] possible_species      # alternatives avec confidences
float64[] species_confidences

# Classification couleur
string primary_color
string secondary_color
float64 color_confidence
uint8[] color_histogram        # histogramme HSV

# État de la fleur
string flower_state           # BUD, OPENING, FULL_BLOOM, WILTING
float64 maturity_score        # 0.0-1.0
float64 health_score         # 0.0-1.0
string[] health_issues        # "PEST_DAMAGE", "DISEASE", "DROUGHT"

# Caractéristiques physiques
float64 petal_count
float64 diameter_cm
float64 stem_height_cm
bool has_nectar_visible
bool has_pollen_visible

# Analyse qualité
float64 pollination_suitability # 0.0-1.0
float64 nectar_accessibility    # 0.0-1.0
string pollination_recommendation # "PROCEED", "SKIP", "WAIT"

# Métadonnées ML
string model_version
builtin_interfaces/Time inference_time
float64 processing_duration_ms
```

## 🔧 SERVICES DÉTAILLÉS

### Services Contrôle

#### ArmDrone.srv
```srv
# Demande d'armement du drone
---
# Requête
bool force_arm                 # Forcer armement même avec warnings
string safety_override_code    # Code override sécurité
---
# Réponse
bool success
bool is_armed
string message
string[] safety_checks_passed
string[] safety_warnings
float64 pre_arm_battery_level
builtin_interfaces/Time arm_timestamp
```

#### SetFlightMode.srv
```srv
# Changement de mode de vol
---
# Requête
string desired_mode            # MANUAL, AUTO, GUIDED, RTL, LAND
string reason                  # Raison du changement
bool emergency_override        # Override pour urgence
---
# Réponse
bool success
string current_mode
string previous_mode
string message
bool requires_operator_confirmation
builtin_interfaces/Time mode_change_time
```

### Services Navigation

#### NavigateToWaypoint.srv
```srv
# Navigation vers un waypoint spécifique
---
# Requête
Waypoint target_waypoint
float64 timeout_seconds
bool abort_on_obstacle
string navigation_mode         # DIRECT, SMOOTH, PRECISION
---
# Réponse
bool success
string result_status          # REACHED, TIMEOUT, ABORTED, FAILED
float64 final_distance_error  # mètres
float64 navigation_duration   # secondes
geometry_msgs/Point final_position
string[] warnings_encountered
```

### Services Vision

#### DetectFlowers.srv
```srv
# Détection de fleurs dans une image ou zone
---
# Requête
# Option 1: Image fournie
sensor_msgs/Image input_image
# Option 2: Capture depuis caméra
bool use_camera
string camera_id
# Option 3: Zone géographique
ZoneDefinition search_zone

# Paramètres détection
float64 confidence_threshold   # 0.0-1.0
uint32 max_detections
string[] flower_colors_filter  # Filtrer par couleurs
string[] species_filter        # Filtrer par espèces
---
# Réponse
bool success
uint32 detections_count
FlowerDetection[] detections
sensor_msgs/Image annotated_image
float64 processing_time_ms
string detection_method_used   # "YOLO", "COLOR", "HYBRID"
```

### Services Mission

#### PlanMission.srv
```srv
# Planification de mission automatique
---
# Requête
string mission_type           # SINGLE_FLOWER, ZONE_COVERAGE, SURVEY
# Pour mission fleur unique
FlowerTarget target_flower
# Pour couverture zone
ZoneDefinition coverage_zone
float64 coverage_spacing      # mètres entre points
# Pour survey
geometry_msgs/Polygon survey_area
string survey_pattern         # GRID, SPIRAL, RANDOM

# Contraintes mission
float64 max_duration_seconds
float64 max_energy_consumption
float64 min_battery_reserve
string[] weather_constraints

# Optimisation
string optimization_objective # TIME, ENERGY, COVERAGE
---
# Réponse
bool success
string mission_id
MissionStatus planned_mission
Trajectory planned_trajectory
float64 estimated_duration    # secondes
float64 estimated_energy      # Wh
uint32 total_waypoints
string[] warnings
string[] constraints_violated
```

## 🔧 ACTIONS DÉTAILLÉES

### Actions Mission

#### ExecuteMission.action
```action
# Exécution complète d'une mission
---
# Goal
string mission_id
bool monitor_progress         # Publier feedback détaillé
float64 progress_update_rate  # Hz
bool auto_optimize           # Réoptimiser en temps réel
bool collect_scientific_data # Collecter données scientifiques
---
# Result
bool success
string completion_status     # COMPLETED, ABORTED, FAILED, TIMEOUT
MissionStatus final_status
PerformanceMetrics performance
uint32 waypoints_completed
uint32 flowers_pollinated
float64 total_distance       # mètres
float64 total_duration       # secondes
float64 energy_consumed      # Wh
string[] issues_encountered
---
# Feedback
MissionStatus current_status
float64 progress_percentage
Waypoint current_waypoint
float64 estimated_remaining_time
float64 current_battery_level
EnvironmentData current_conditions
```

#### PollinationSequence.action
```action
# Séquence complète de pollinisation d'une fleur
---
# Goal
FlowerTarget target_flower
float64 approach_speed        # m/s
float64 precision_distance    # mètres de la fleur
float64 hover_duration       # secondes au-dessus
bool verify_success          # Vérifier succès pollinisation
bool collect_data           # Collecter données scientifiques
---
# Result
bool success
PollinationResult result
FlowerClassification final_classification
bool pollination_verified
sensor_msgs/Image[] captured_images
EnvironmentData conditions_during_pollination
---
# Feedback
string current_phase         # APPROACHING, POSITIONING, POLLINATING, VERIFYING
float64 distance_to_flower   # mètres
float64 positioning_accuracy # mètres
geometry_msgs/Point current_position
```

## 🔧 CONFIGURATION CMAKELIST

### CMakeLists.txt
```cmake
cmake_minimum_required(VERSION 3.8)
project(drone_msgs)

# Dépendances
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Génération interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/DroneStatus.msg"
  "msg/DroneState.msg"
  "msg/FlightMode.msg"
  "msg/SafetyStatus.msg"
  "msg/BatteryStatus.msg"
  "msg/Position3D.msg"
  "msg/Orientation.msg"
  "msg/Velocity3D.msg"
  "msg/NavigationCommand.msg"
  "msg/Waypoint.msg"
  "msg/Trajectory.msg"
  "msg/FlowerDetection.msg"
  "msg/FlowerClassification.msg"
  "msg/FlowerTarget.msg"
  "msg/PollinationResult.msg"
  "msg/MissionStatus.msg"
  "msg/MissionProgress.msg"
  "msg/MissionWaypoint.msg"
  "msg/ZoneDefinition.msg"
  "msg/EnvironmentData.msg"
  "msg/PerformanceMetrics.msg"
  "msg/SystemAlert.msg"
  "msg/DiagnosticInfo.msg"
  
  # Services
  "srv/ArmDrone.srv"
  "srv/DisarmDrone.srv"
  "srv/SetFlightMode.srv"
  "srv/TakeoffDrone.srv"
  "srv/LandDrone.srv"
  "srv/ReturnToHome.srv"
  "srv/EmergencyStop.srv"
  "srv/SafetyCheck.srv"
  "srv/NavigateToWaypoint.srv"
  "srv/SetTrajectory.srv"
  "srv/DetectFlowers.srv"
  "srv/ClassifyFlower.srv"
  "srv/TrackFlower.srv"
  "srv/PollinateFlower.srv"
  "srv/PlanMission.srv"
  "srv/StartMission.srv"
  "srv/PauseMission.srv"
  "srv/AbortMission.srv"
  "srv/OptimizeMission.srv"
  "srv/GetSystemStatus.srv"
  "srv/ConfigureSystem.srv"
  "srv/GenerateReport.srv"
  
  # Actions
  "action/ExecuteMission.action"
  "action/NavigateToPosition.action"
  "action/FollowTrajectory.action"
  "action/ApproachFlower.action"
  "action/PollinationSequence.action"
  "action/ZoneCoverage.action"
  "action/DataCollection.action"
  "action/SystemCalibration.action"
  "action/EmergencyLanding.action"
  
  DEPENDENCIES
  std_msgs
  geometry_msgs
  sensor_msgs
  builtin_interfaces
)

# Export dépendances
ament_export_dependencies(rosidl_default_runtime)

# Tests
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
  
  # Tests personnalisés
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_msg_compilation test/test_msg_compilation.py)
  ament_add_pytest_test(test_msg_serialization test/test_msg_serialization.py)
  ament_add_pytest_test(test_srv_interfaces test/test_srv_interfaces.py)
  ament_add_pytest_test(test_action_interfaces test/test_action_interfaces.py)
endif()

ament_package()
```

## 🧪 TESTS OBLIGATOIRES

### test_msg_compilation.py
```python
"""Tests de compilation des messages"""
import unittest
import rclpy
from drone_msgs.msg import *

class TestMessageCompilation(unittest.TestCase):
    def test_all_messages_importable(self):
        """Vérifier que tous les messages sont importables"""
        # Test tous les messages définis
        msg_types = [
            DroneStatus, DroneState, FlightMode, SafetyStatus,
            BatteryStatus, Position3D, Orientation, Velocity3D,
            NavigationCommand, Waypoint, Trajectory,
            FlowerDetection, FlowerClassification, FlowerTarget,
            PollinationResult, MissionStatus, MissionProgress,
            MissionWaypoint, ZoneDefinition, EnvironmentData,
            PerformanceMetrics, SystemAlert, DiagnosticInfo
        ]
        
        for msg_type in msg_types:
            with self.subTest(msg_type=msg_type):
                # Test création instance
                msg = msg_type()
                self.assertIsNotNone(msg)
                
                # Test sérialisation
                serialized = msg.serialize()
                self.assertIsInstance(serialized, bytes)
```

### test_srv_interfaces.py
```python
"""Tests des interfaces de services"""
import unittest
from drone_msgs.srv import *

class TestServiceInterfaces(unittest.TestCase):
    def test_service_request_response_structure(self):
        """Vérifier structure requête/réponse services"""
        services = [
            ArmDrone, DisarmDrone, SetFlightMode, TakeoffDrone,
            LandDrone, ReturnToHome, EmergencyStop, SafetyCheck,
            NavigateToWaypoint, SetTrajectory, DetectFlowers,
            ClassifyFlower, TrackFlower, PollinateFlower,
            PlanMission, StartMission, PauseMission, AbortMission,
            OptimizeMission, GetSystemStatus, ConfigureSystem,
            GenerateReport
        ]
        
        for srv_type in services:
            with self.subTest(service=srv_type):
                # Test création requête/réponse
                request = srv_type.Request()
                response = srv_type.Response()
                self.assertIsNotNone(request)
                self.assertIsNotNone(response)
```

## 📚 DOCUMENTATION README.md

Le README.md doit inclure :
- **Vue d'ensemble** du système de communication
- **Messages reference** avec tous les champs
- **Services reference** avec exemples utilisation
- **Actions reference** avec workflow
- **Integration examples** pour chaque package
- **Best practices** communication ROS2
- **Performance considerations** sérialisation
- **Troubleshooting** problèmes courants

## 🎯 MÉTRIQUES DE PERFORMANCE

### Objectifs OBLIGATOIRES :
- **Message size** : <1KB pour messages fréquents
- **Serialization time** : <1ms pour messages critiques
- **Network bandwidth** : <100KB/s pour télémétrie
- **Action feedback rate** : >1Hz pour missions longues
- **Service response time** : <100ms pour services critiques
- **Type safety** : 100% validation au compile-time

---

**IMPORTANT : Ce package est la fondation de communication - tous les autres packages en dépendent. La qualité et complétude sont CRITIQUES.**
