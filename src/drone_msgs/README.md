# Drone Messages Package

Package ROS2 contenant tous les messages, services et actions pour le système de drone de pollinisation autonome.

## Vue d'ensemble

Le package `drone_msgs` fournit l'interface de communication complète pour un drone de pollinisation autonome. Il contient :

- **23 Messages** : Définitions des structures de données
- **23 Services** : Interfaces de communication synchrone  
- **9 Actions** : Interfaces pour opérations longues avec feedback

## Messages

### Messages de Base du Drone
- `DroneStatus` : État général du drone
- `DroneState` : État détaillé du système
- `FlightMode` : Mode de vol et paramètres
- `SafetyStatus` : État des systèmes de sécurité
- `BatteryStatus` : État de la batterie

### Messages de Navigation
- `Position3D` : Position 3D précise
- `Orientation` : Orientation 3D complète
- `Velocity3D` : Vélocité 3D avec accélérations
- `NavigationCommand` : Commandes de navigation
- `Waypoint` : Points de navigation
- `Trajectory` : Trajectoires complètes

### Messages de Vision et Pollinisation
- `FlowerDetection` : Détection de fleurs
- `FlowerClassification` : Classification des fleurs
- `FlowerTarget` : Cibles de pollinisation
- `PollinationResult` : Résultats de pollinisation

### Messages de Mission
- `MissionStatus` : État des missions
- `MissionProgress` : Progression des missions
- `MissionWaypoint` : Waypoints de mission
- `ZoneDefinition` : Définition des zones

### Messages Système
- `EnvironmentData` : Données environnementales
- `PerformanceMetrics` : Métriques de performance
- `SystemAlert` : Alertes système
- `DiagnosticInfo` : Informations de diagnostic

## Services

### Services de Contrôle
- `ArmDrone` / `DisarmDrone` : Armement/désarmement
- `SetFlightMode` : Configuration mode de vol
- `Takeoff` / `Land` : Décollage/atterrissage
- `ReturnToLaunch` : Retour au point de lancement
- `EmergencyStop` : Arrêt d'urgence

### Services de Mission
- `LoadMission` : Chargement de mission
- `StartMission` / `StopMission` : Démarrage/arrêt mission
- `PauseMission` : Pause/reprise mission

### Services de Navigation
- `SetWaypoint` : Définition de waypoints
- `SetGeofence` : Configuration géobarrière

### Services de Configuration
- `CalibrateSensors` : Calibration capteurs
- `SystemDiagnostic` : Diagnostic système
- `UpdateParameters` : Mise à jour paramètres
- `ConfigureCamera` : Configuration caméra

### Services de Données
- `GetSystemStatus` : État système
- `DetectFlowers` : Détection de fleurs
- `ExecutePollination` : Exécution pollinisation
- `SaveData` : Sauvegarde données

## Actions

### Actions de Mission
- `ExecuteMission` : Exécution mission complète
- `SearchAndPollinate` : Recherche et pollinisation
- `MonitorZone` : Surveillance de zone

### Actions de Navigation
- `NavigateToPosition` : Navigation vers position
- `FollowTrajectory` : Suivi de trajectoire

### Actions de Données
- `CollectData` : Collecte de données
- `MapArea` : Cartographie de zone

### Actions de Maintenance
- `PerformMaintenance` : Maintenance préventive
- `EmergencyLanding` : Atterrissage d'urgence

## Structure du Package

```
drone_msgs/
├── CMakeLists.txt
├── package.xml
├── README.md
├── msg/                    # Messages
│   ├── DroneStatus.msg
│   ├── FlowerDetection.msg
│   └── ...
├── srv/                    # Services
│   ├── ArmDrone.srv
│   ├── DetectFlowers.srv
│   └── ...
├── action/                 # Actions
│   ├── ExecuteMission.action
│   ├── NavigateToPosition.action
│   └── ...
├── test/                   # Tests
│   ├── test_messages.py
│   ├── test_services.py
│   └── test_actions.py
└── docs/                   # Documentation
    ├── messages_reference.md
    ├── services_reference.md
    └── actions_reference.md
```

## Utilisation

### Import des Messages
```python
from drone_msgs.msg import DroneStatus, FlowerDetection, MissionStatus
```

### Utilisation des Services
```python
from drone_msgs.srv import ArmDrone, DetectFlowers
import rclpy
from rclpy.node import Node

class DroneClient(Node):
    def __init__(self):
        super().__init__('drone_client')
        self.arm_client = self.create_client(ArmDrone, 'arm_drone')
        
    def arm_drone(self):
        request = ArmDrone.Request()
        request.force_arm = False
        request.operator_id = "OPERATOR_001"
        future = self.arm_client.call_async(request)
        return future
```

### Utilisation des Actions
```python
from drone_msgs.action import ExecuteMission
from rclpy_action import ActionClient

class MissionExecutor(Node):
    def __init__(self):
        super().__init__('mission_executor')
        self.action_client = ActionClient(self, ExecuteMission, 'execute_mission')
        
    def execute_mission(self, mission_id):
        goal = ExecuteMission.Goal()
        goal.mission_id = mission_id
        goal.auto_takeoff = True
        return self.action_client.send_goal_async(goal)
```

## Compilation

```bash
cd ~/ros2_ws
colcon build --packages-select drone_msgs
source install/setup.bash
```

## Tests

Exécuter les tests unitaires :
```bash
cd ~/ros2_ws
colcon test --packages-select drone_msgs
colcon test-result --verbose
```

## Dépendances

- `std_msgs`
- `geometry_msgs`
- `sensor_msgs`
- `builtin_interfaces`
- `diagnostic_msgs`
- `nav_msgs`

## Compatibilité

- ROS2 Humble, Iron, Jazzy
- Python 3.8+
- C++14+

## Licence

Ce package est distribué sous licence Apache 2.0.

## Contribution

Pour contribuer à ce package :

1. Fork le repository
2. Créer une branche feature
3. Ajouter tests pour nouvelles fonctionnalités
4. Vérifier que tous les tests passent
5. Soumettre une pull request

## Support

Pour questions et support :
- Issues GitHub
- Documentation dans `/docs`
- Exemples dans `/test`
