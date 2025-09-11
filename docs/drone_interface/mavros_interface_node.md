# MAVROS Interface Node - Documentation Complète

## Vue d'ensemble

Le **MAVROS Interface Node** (`mavros_interface_node.py`) est un composant spécialisé du système de navigation autonome qui gère exclusivement la **collecte et distribution des données** depuis MAVROS et l'autopilot ArduPilot. Ce nœud abstrait les détails de MAVROS pour le reste du système et fournit une interface standardisée pour les données de télémétrie.

> **Note importante**: Ce nœud se concentre uniquement sur la collecte de données. Les commandes de vol (armement, changement de mode, décollage/atterrissage) sont gérées par les nœuds spécialisés du package `drone_interface`.

### Caractéristiques principales

- **Interface de lecture MAVROS** : Collecte exclusive des données de télémétrie
- **Abstraction des protocoles** : Normalisation des formats MAVROS vers ROS2 standard
- **Distribution temps réel** : Redistribution des données aux autres nœuds du système
- **Surveillance continue** : Monitoring de l'état du drone et de la connexion
- **Services d'information** : Accès aux données de statut et diagnostic

---

## Architecture et Design

### Position dans le système

```
┌─────────────────────┐    ┌─────────────────────┐    ┌─────────────────────┐
│  Navigation Nodes   │    │  MAVROS Interface   │    │     MAVROS         │
│  (mission, path,    │◄───│      Node           │◄───│   (MAVLink)        │
│   obstacle, etc.)   │    │   (Lecture seule)   │    │                     │
└─────────────────────┘    └─────────────────────┘    └─────────────────────┘
            │                         │                           ▲
            │              ┌─────────────────────┐                │
            └─────────────►│  drone_interface    │────────────────┘
                           │   (Commandes)       │
                           └─────────────────────┘
                           ┌─────────────────────┐
                           │    ArduPilot       │
                           │   (Autopilot)      │
                           └─────────────────────┘
```

### Responsabilités

1. **Collecte de données MAVROS**
   - Réception des données de télémétrie
   - Surveillance de l'état de connexion
   - Collecte des informations de vol

2. **Conversion et normalisation**
   - Conversion entre formats de messages MAVROS et ROS2 standard
   - Normalisation des interfaces pour le système de navigation
   - Publication d'odométrie pour la navigation

3. **Distribution de données**
   - Redistribution des données aux nœuds de navigation
   - Publication de statut système
   - Services d'information et diagnostic

4. **Monitoring et diagnostic**
   - Surveillance de la qualité de connexion
   - Évaluation des paramètres de sécurité
   - Diagnostic des performances du système

---

## Structure du code

### Classes et énumérations

#### `FlightMode` (Enum)
Énumération des modes de vol supportés par ArduPilot.

```python
class FlightMode(Enum):
    MANUAL = "MANUAL"
    STABILIZE = "STABILIZE"
    GUIDED = "GUIDED"
    AUTO = "AUTO"
    LOITER = "LOITER"
    RTL = "RTL"
    LAND = "LAND"
```

#### `VehicleState` (Enum)
États possibles du véhicule pour le diagnostic.

```python
class VehicleState(Enum):
    UNKNOWN = "UNKNOWN"
    DISCONNECTED = "DISCONNECTED"
    CONNECTED = "CONNECTED"
    ARMED = "ARMED"
    DISARMED = "DISARMED"
    FLYING = "FLYING"
    LANDED = "LANDED"
```

#### `DroneStatus` (Dataclass)
Structure de données centralisée pour l'état du drone.

```python
@dataclass
class DroneStatus:
    # État de connexion
    connected: bool = False
    armed: bool = False
    guided: bool = False
    mode: str = "UNKNOWN"
    system_status: int = 0
    
    # Position et orientation
    position: Optional[Point] = field(default_factory=lambda: Point())
    velocity: Optional[Vector3] = field(default_factory=lambda: Vector3())
    altitude: float = 0.0
    heading: float = 0.0
    
    # GPS et navigation
    gps_fix: bool = False
    satellites: int = 0
    
    # Énergie
    battery_voltage: float = 0.0
    battery_percentage: float = 0.0
    
    # Timestamps pour le diagnostic
    last_heartbeat: float = 0.0
    last_position_update: float = 0.0
```

### Classe principale : `MavrosInterfaceNode`

Hérite de `rclpy.node.Node` et implémente l'interface complète avec MAVROS.

---

## Configuration et paramètres

### Paramètres ROS2

Le nœud expose plusieurs paramètres configurables :

| Paramètre | Type | Défaut | Description |
|-----------|------|--------|-------------|
| `mavros_namespace` | string | `/mavros` | Namespace des topics MAVROS |
| `connection_timeout` | double | `5.0` | Timeout de connexion (secondes) |
| `command_timeout` | double | `10.0` | Timeout des commandes (secondes) |
| `status_frequency` | double | `1.0` | Fréquence de publication du statut (Hz) |
| `position_frequency` | double | `50.0` | Fréquence de publication de position (Hz) |
| `simulation_mode` | bool | `true` | Mode simulation activé |
| `local_frame` | string | `map` | Frame de référence local |
| `global_frame` | string | `earth` | Frame de référence global |

### Configuration QoS

Le nœud utilise des profils QoS adaptés aux différents types de données :

```python
# Pour les données de capteurs (tolérant aux pertes)
qos_sensor = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

# Pour les commandes critiques (fiabilité garantie)
qos_cmd = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=10
)

# Pour les données de statut (une seule valeur actuelle)
qos_status = QoSProfile(
    reliability=QoSReliabilityPolicy.RELIABLE,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1
)
```

---

## Interface ROS2

### Topics publiés

| Topic | Type de message | QoS | Fréquence | Description |
|-------|----------------|-----|-----------|-------------|
| `position` | `geometry_msgs/PoseStamped` | sensor | 50 Hz | Position et orientation actuelles |
| `velocity` | `geometry_msgs/TwistStamped` | sensor | 50 Hz | Vitesse linéaire et angulaire |
| `drone_status` | `std_msgs/String` | status | 1 Hz | Statut complet du drone (JSON) |
| `safety_status` | `std_msgs/String` | status | 1 Hz | Évaluation de sécurité (JSON) |
| `odom` | `nav_msgs/Odometry` | sensor | 50 Hz | Odométrie pour la navigation |

### Topics souscrits

#### Topics MAVROS (entrée)
| Topic MAVROS | Type de message | Description |
|--------------|----------------|-------------|
| `/mavros/state` | `mavros_msgs/State` | État de connexion et mode |
| `/mavros/extended_state` | `mavros_msgs/ExtendedState` | État étendu du véhicule |
| `/mavros/local_position/pose` | `geometry_msgs/PoseStamped` | Position locale |
| `/mavros/local_position/velocity_local` | `geometry_msgs/TwistStamped` | Vitesse locale |
| `/mavros/global_position/global` | `sensor_msgs/NavSatFix` | Position GPS |
| `/mavros/altitude` | `mavros_msgs/Altitude` | Altitude relative et absolue |
| `/mavros/battery` | `sensor_msgs/BatteryState` | État de la batterie |

#### Topics de commande (entrée)
| Topic | Type de message | Description |
|-------|----------------|-------------|
| `cmd_vel` | `geometry_msgs/TwistStamped` | Commandes de vitesse (transfert vers MAVROS) |
| `cmd_pose` | `geometry_msgs/PoseStamped` | Commandes de position (transfert vers MAVROS) |

### Topics publiés vers MAVROS (sortie)
| Topic MAVROS | Type de message | Description |
|--------------|----------------|-------------|
| `/mavros/setpoint_velocity/cmd_vel` | `geometry_msgs/TwistStamped` | Consignes de vitesse |
| `/mavros/setpoint_position/local` | `geometry_msgs/PoseStamped` | Consignes de position |

### Services fournis

| Service | Type | Description |
|---------|------|-------------|
| `get_drone_status` | `std_srvs/Trigger` | Obtenir le statut complet (JSON) |
| `get_current_position` | `std_srvs/Trigger` | Obtenir la position actuelle (JSON) |
| `get_diagnostics` | `std_srvs/Trigger` | Diagnostics du nœud (JSON) |

> **Note**: Les services de commandes (armement, mode, décollage/atterrissage) sont disponibles via le package `drone_interface`.

### Services MAVROS utilisés

Ce nœud n'utilise **aucun service MAVROS** car il se contente de collecter les données via les topics. Les services de commandes sont gérés par le package `drone_interface` :

- **Armement/Désarmement** : `drone_interface/arm_disarm_node`
- **Changement de mode** : `drone_interface/mode_node` 
- **Décollage/Atterrissage** : `drone_interface/takeoff_land_node`

---

## Fonctionnalités détaillées

### 1. Gestion de l'état et surveillance

#### Monitoring de connexion
- **Heartbeat MAVROS** : Vérification continue de la connexion (10 Hz)
- **Timeout de connexion** : Détection automatique des pertes de connexion
- **État de synchronisation** : Suivi de la cohérence des données

#### Collecte de télémétrie
- **Position et orientation** : Pose locale en temps réel
- **Vitesse** : Vitesses linéaires et angulaires
- **GPS** : Position globale et qualité du signal
- **Batterie** : Tension et pourcentage estimé
- **Altitude** : Altitude relative et absolue

#### Publication de statut
```python
def _publish_status(self):
    """Publie le statut complet toutes les secondes"""
    status_dict = {
        'timestamp': time.time(),
        'connected': self.drone_status.connected,
        'armed': self.drone_status.armed,
        'mode': self.drone_status.mode,
        'position': {...},
        'velocity': {...},
        'gps_fix': self.drone_status.gps_fix,
        'battery': {...}
    }
```

### 2. Distribution et transfert de données

#### Transfert des commandes de navigation
```python
def _cmd_vel_callback(self, msg: TwistStamped):
    """Transfert les commandes de vitesse vers MAVROS"""
    self.mavros_cmd_vel_pub.publish(msg)

def _cmd_pose_callback(self, msg: PoseStamped):
    """Transfert les commandes de position vers MAVROS"""
    self.mavros_cmd_pose_pub.publish(msg)
```

#### Publication de statut système
```python
def _publish_status(self):
    """Publie le statut complet toutes les secondes"""
    status_dict = {
        'timestamp': time.time(),
        'connected': self.drone_status.connected,
        'armed': self.drone_status.armed,
        'mode': self.drone_status.mode,
        'position': {...},
        'velocity': {...},
        'gps_fix': self.drone_status.gps_fix,
        'battery': {...}
    }
```

### 3. Conversion et abstraction des données

#### Conversion des messages
- **MAVROS → ROS2 standard** : Normalisation des formats
- **Timestamps** : Synchronisation des horodatages
- **Frames de référence** : Gestion cohérente des repères

#### Calculs de navigation
```python
def _quaternion_to_yaw(self, quaternion) -> float:
    """Conversion quaternion vers angle de lacet"""
    siny_cosp = 2 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
    cosy_cosp = 1 - 2 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
    return math.atan2(siny_cosp, cosy_cosp)
```

#### Publication d'odométrie
```python
def _publish_odometry(self, pose_msg: PoseStamped):
    """Génère l'odométrie pour la navigation"""
    odom_msg = Odometry()
    odom_msg.header.frame_id = self.get_parameter('local_frame').value
    odom_msg.child_frame_id = "base_link"
    # Combinaison pose + vitesse
```

### 4. Évaluation de sécurité

#### Système d'alertes multicouches
```python
def _evaluate_safety_status(self) -> Dict[str, Any]:
    """Évaluation complète de la sécurité"""
    warnings = []
    errors = []
    
    # Vérifications critiques (erreurs)
    if not self.drone_status.connected:
        errors.append("Drone non connecté")
    
    if self.drone_status.battery_percentage < 10:
        errors.append(f"Batterie critique: {self.drone_status.battery_percentage:.1f}%")
    
    # Vérifications importantes (avertissements)
    if not self.drone_status.gps_fix:
        warnings.append("GPS fix non disponible")
    
    # Classification du niveau de sécurité
    if errors:
        safety_level = "CRITICAL"
    elif warnings:
        safety_level = "WARNING"
    else:
        safety_level = "NORMAL"
```

#### Critères de sécurité surveillés
- **Connexion MAVROS** : État de la liaison de communication
- **Qualité GPS** : Fix disponible et nombre de satellites
- **Niveau de batterie** : Surveillance avec seuils d'alerte
- **Altitude opérationnelle** : Respect des limites réglementaires
- **Cohérence des données** : Validation des timestamps et fréquences

---

## Diagnostic et maintenance

### Système de diagnostic intégré

#### Compteurs de performance
```python
self.message_counters = {
    'state': 0,        # Messages d'état MAVROS
    'position': 0,     # Mises à jour de position
    'velocity': 0,     # Mises à jour de vitesse
    'battery': 0,      # Données de batterie
    'gps': 0          # Données GPS
}
```

#### Service de diagnostic
Le service `get_diagnostics` fournit :
- **État du nœud** : Initialisation, paramètres, configuration
- **Statistiques de performance** : Compteurs de messages, fréquences
- **État de connexion** : Derniers heartbeats, timeouts
- **Configuration active** : Paramètres et namespaces

### Journalisation et debugging

#### Niveaux de log structurés
- **INFO** : Événements normaux (connexion, changements de mode)
- **WARNING** : Problèmes non critiques (GPS faible, batterie basse)
- **ERROR** : Erreurs nécessitant attention (timeouts, commandes échouées)
- **DEBUG** : Informations détaillées pour le développement

#### Messages de log types
```python
# Connexion établie
self.get_logger().info("Connexion MAVROS établie")

# Changement de mode réussi
self.get_logger().info(f"Mode changé vers: {request.custom_mode}")

# Avertissement sécurité
self.get_logger().warning(f"Batterie faible: {self.drone_status.battery_percentage:.1f}%")

# Erreur critique
self.get_logger().error("Décollage impossible: drone non connecté")
```

---

## Utilisation pratique

### Démarrage du nœud

```bash
# Démarrage basique
ros2 run drone_navigation mavros_interface_node

# Avec paramètres personnalisés
ros2 run drone_navigation mavros_interface_node \
    --ros-args \
    -p mavros_namespace:=/my_mavros \
    -p simulation_mode:=false \
    -p connection_timeout:=10.0
```

### Intégration dans un launch file

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='drone_navigation',
            executable='mavros_interface_node',
            name='mavros_interface',
            namespace='drone_nav',
            parameters=[{
                'mavros_namespace': '/mavros',
                'simulation_mode': True,
                'connection_timeout': 5.0,
                'status_frequency': 2.0
            }],
            output='screen'
        )
    ])
```

### Surveillance du statut

```bash
# Écouter le statut du drone
ros2 topic echo /drone_nav/drone_status

# Écouter les alertes de sécurité
ros2 topic echo /drone_nav/safety_status

# Obtenir le statut via service
ros2 service call /drone_nav/get_drone_status std_srvs/srv/Trigger
```

### Commandes de base

```bash
# Obtenir le statut du drone
ros2 service call /drone_nav/get_drone_status std_srvs/srv/Trigger

# Obtenir la position actuelle
ros2 service call /drone_nav/get_current_position std_srvs/srv/Trigger

# Obtenir les diagnostics
ros2 service call /drone_nav/get_diagnostics std_srvs/srv/Trigger
```

> **Pour les commandes de vol, utiliser les nœuds du package drone_interface** :

```bash
# Armer le drone (via drone_interface)
ros2 topic pub /arm_control/arm_cmd std_msgs/msg/Bool "{data: true}" --once

# Changer le mode (via drone_interface)
ros2 topic pub /mode_command std_msgs/msg/String "{data: 'GUIDED'}" --once

# Décollage (via drone_interface)
ros2 topic pub /takeoff_command std_msgs/msg/Float64 "{data: 10.0}" --once
```

---

## Dépannage et résolution de problèmes

### Problèmes de connexion MAVROS

#### Symptômes
- `connected: false` dans le statut
- Messages "Perte de connexion MAVROS détectée"
- Pas de données de télémétrie

#### Solutions
1. **Vérifier MAVROS** :
   ```bash
   ros2 topic list | grep mavros
   ros2 topic echo /mavros/state
   ```

2. **Vérifier la configuration** :
   ```bash
   ros2 param get /drone_nav/mavros_interface_node mavros_namespace
   ```

3. **Redémarrer MAVROS** :
   ```bash
   ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555
   ```

### Problèmes de commandes

#### Symptômes
- Pas de réponse aux commandes de vol
- Messages d'erreur "Service non disponible"

#### Solutions
1. **Vérifier les nœuds drone_interface** :
   ```bash
   ros2 node list | grep -E "(arm_disarm|mode|takeoff_land)"
   ```

2. **Démarrer les nœuds manquants** :
   ```bash
   ros2 launch drone_interface interface.launch.py
   ```

3. **Vérifier les topics de commande** :
   ```bash
   ros2 topic list | grep -E "(arm_control|mode_command|takeoff_command)"
   ```

### Problèmes de GPS

#### Symptômes
- `gps_fix: false`
- `satellites: 0` ou nombre faible
- Avertissements "GPS fix non disponible"

#### Solutions
1. **En simulation** :
   - Vérifier que le simulateur fournit des données GPS
   - Attendre l'initialisation complète

2. **En conditions réelles** :
   - Vérifier l'antenne GPS
   - Attendre l'acquisition (peut prendre plusieurs minutes)
   - Vérifier l'environnement (pas d'obstacles)

### Problèmes de performance

#### Symptômes
- Fréquences de messages faibles
- Retards dans les réponses
- Timeouts fréquents

#### Optimisations
1. **Ajuster les paramètres QoS** :
   ```python
   # Réduire la profondeur des queues si nécessaire
   depth=5  # au lieu de 10
   ```

2. **Optimiser les fréquences** :
   ```bash
   ros2 param set /drone_nav/mavros_interface_node status_frequency 0.5
   ```

3. **Surveiller les performances** :
   ```bash
   ros2 service call /drone_nav/get_diagnostics std_srvs/srv/Trigger
   ```

---

## Intégration avec le système complet

### Interface avec autres nœuds

#### Nœuds drone_interface (commandes)
- **Entrée** : Topics de commande (`/arm_control/arm_cmd`, `/mode_command`, etc.)
- **Sortie** : Services MAVROS directs
- **Coordination** : Utilisation du statut via `drone_status`

#### Mission Manager
- **Entrée** : Statut du drone via `drone_status`
- **Sortie** : Commandes de mission via `cmd_pose` et `cmd_vel`
- **Données** : Position et vitesse via `position` et `velocity`

#### Path Planner
- **Entrée** : Position actuelle via `position`
- **Sortie** : Trajectoire via `cmd_pose`
- **Données** : Odométrie via `odom`

#### Obstacle Avoidance
- **Entrée** : Position et vitesse via `position` et `velocity`
- **Sortie** : Corrections via `cmd_vel`
- **Sécurité** : Surveillance via `safety_status`

### Flux de données typique

```
1. MAVROS → MavrosInterface → Position/Velocity/Status
2. MissionManager → MavrosInterface → cmd_pose (transfert vers MAVROS)
3. drone_interface → MAVROS → Services de commandes directs
4. MavrosInterface → System → drone_status, safety_status, odom
```

### Considérations de sécurité système

1. **Redondance** : Le nœud ne doit jamais être le seul point de défaillance
2. **Failsafe** : En cas de problème, retour automatique aux modes sûrs
3. **Validation** : Toutes les commandes sont validées avant transmission
4. **Monitoring** : Surveillance continue avec alertes appropriées

---

## Performance et optimisation

### Métriques de performance

#### Fréquences cibles
- **Position** : 50 Hz (20ms)
- **Vitesse** : 50 Hz (20ms)  
- **Statut** : 1 Hz (1000ms)
- **Heartbeat** : 10 Hz (100ms)

#### Latences acceptables
- **Commandes critiques** : < 100ms
- **Changements de mode** : < 500ms
- **Services de status** : < 200ms

### Optimisations recommandées

#### Configuration système
```bash
# Augmenter la priorité du processus
sudo nice -n -10 ros2 run drone_navigation mavros_interface_node

# Optimiser la configuration réseau
sudo sysctl -w net.core.rmem_max=134217728
sudo sysctl -w net.core.wmem_max=134217728
```

#### Paramètres de performance
```python
# Réduire la latence au détriment de la bande passante
qos_realtime = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    history=QoSHistoryPolicy.KEEP_LAST,
    depth=1  # Seule la dernière valeur
)
```

---

## Conclusion

Le MAVROS Interface Node constitue la couche de **collecte et distribution de données** entre le système de navigation autonome et l'autopilot ArduPilot. Sa conception focalisée assure :

- **Collecte fiable** : Réception et normalisation des données MAVROS
- **Distribution efficace** : Redistribution optimisée vers les nœuds de navigation
- **Abstraction claire** : Interface simplifiée masquant la complexité MAVROS
- **Monitoring continu** : Surveillance de l'état et diagnostic du système
- **Séparation des responsabilités** : Focus sur les données, commandes gérées par `drone_interface`

Cette spécialisation permet une architecture modulaire où chaque composant a une responsabilité claire et définie, facilitant la maintenance et l'évolution du système de drone autonome.
