# DRONE NAVIGATION NODE

## Description
Le nœud `drone_navigation` est spécialisé dans la navigation autonome et le contrôle de position du drone. Il fournit des services avancés de navigation, gère les waypoints, planifie les trajectoires et assure l'évitement d'obstacles basique.

## Responsabilités
- Navigation autonome vers des waypoints
- Contrôle de position et d'altitude
- Gestion des trajectoires
- Évitement d'obstacles basique
- Interface avec le nœud drone_interface central
- Planification de chemins
- Contrôle PID pour position et altitude

## Architecture interne

### Modules principaux
- **NavigationNode** : Nœud principal ROS2
- **TrajectoryPlanner** : Planificateur de trajectoire pour waypoints
- **PIDController** : Contrôleurs PID pour position/altitude
- **ObstacleAvoidance** : Module d'évitement d'obstacles
- **NavigationParams** : Paramètres de configuration

### États de navigation
- `IDLE` : Inactif, prêt à recevoir des commandes
- `TAKING_OFF` : En phase de décollage
- `NAVIGATING` : En navigation vers waypoint(s)
- `HOVERING` : Maintien de position
- `LANDING` : En phase d'atterrissage
- `EMERGENCY` : État d'urgence
- `RETURNING_HOME` : Retour à la position home

## Services exposés

### `/navigation/set_position` (SetPosition)
Navigation vers une position spécifique.
```bash
ros2 service call /navigation/set_position drone_navigation/srv/SetPosition "{position: {x: 2.0, y: 2.0, z: 5.0}, yaw: 0.0}"
```

### `/navigation/set_velocity` (SetVelocity)
Contrôle direct de vitesse.
```bash
ros2 service call /navigation/set_velocity drone_navigation/srv/SetVelocity "{velocity: {x: 1.0, y: 0.0, z: 0.5}, yaw_rate: 0.1}"
```

### `/navigation/set_waypoints` (SetWaypoints)
Définition d'une liste de waypoints à suivre.
```bash
ros2 service call /navigation/set_waypoints drone_navigation/srv/SetWaypoints "{waypoints: [{position: {x: 1.0, y: 1.0, z: 3.0}, yaw: 0.0, tolerance: 0.5, yaw_tolerance: 0.1}]}"
```

### `/navigation/get_waypoints` (GetWaypoints)
Récupération des waypoints actuels.
```bash
ros2 service call /navigation/get_waypoints drone_navigation/srv/GetWaypoints
```

### `/navigation/return_home` (Trigger)
Retour à la position home.
```bash
ros2 service call /navigation/return_home std_srvs/srv/Trigger
```

### `/navigation/hold_position` (Trigger)
Maintien de la position actuelle.
```bash
ros2 service call /navigation/hold_position std_srvs/srv/Trigger
```

## Actions exposées

### `/navigation/navigate_to_goal` (NavigateToGoal)
Navigation vers un goal avec feedback temps réel.
```bash
ros2 action send_goal /navigation/navigate_to_goal drone_navigation/action/NavigateToGoal "{target_position: {x: 5.0, y: 5.0, z: 10.0}, target_yaw: 1.57, position_tolerance: 0.3, yaw_tolerance: 0.1}"
```

### `/navigation/follow_path` (FollowPath)
Suivi d'un chemin de waypoints avec feedback.
```bash
ros2 action send_goal /navigation/follow_path drone_navigation/action/FollowPath "{waypoints: [{position: {x: 1.0, y: 1.0, z: 3.0}, yaw: 0.0, tolerance: 0.5, yaw_tolerance: 0.1}]}"
```

## Topics publiés

### `/navigation/status` (NavigationStatus)
État de navigation en temps réel.
```bash
ros2 topic echo /navigation/status --once
```

### `/navigation/current_path` (Path)
Chemin actuel planifié.
```bash
ros2 topic echo /navigation/current_path --once
```

### `/mavros/setpoint_velocity/cmd_vel` (TwistStamped)
Commandes de vitesse vers MAVROS.

## Topics souscrits

### `/mavros/local_position/pose` (PoseStamped)
Position locale du drone.
```bash
ros2 topic echo /mavros/local_position/pose --once
```

### `/mavros/local_position/velocity_local` (TwistStamped)
Vitesse locale du drone.
```bash
ros2 topic echo /mavros/local_position/velocity_local --once
```

### `/perception/obstacles` (ObstacleAlert)
Alertes d'obstacles pour évitement.

## Paramètres de configuration

### NavigationParams par défaut
- `max_velocity` : 2.0 m/s
- `max_acceleration` : 1.0 m/s²
- `position_tolerance` : 0.3 m
- `yaw_tolerance` : 0.1 rad
- `update_rate` : 20.0 Hz
- `safety_distance` : 2.0 m
- `max_altitude` : 50.0 m
- `min_altitude` : 2.0 m

### Contrôleurs PID
- **Position X/Y** : Kp=1.0, Ki=0.01, Kd=0.1
- **Position Z** : Kp=1.5, Ki=0.01, Kd=0.2
- **Yaw** : Kp=1.0, Ki=0.0, Kd=0.1

## Commandes de test complètes

### Tests de services de base
```bash
# Navigation simple
ros2 service call /navigation/set_position drone_navigation/srv/SetPosition "{position: {x: 2.0, y: 2.0, z: 5.0}, yaw: 0.0}"

# Contrôle de vitesse
ros2 service call /navigation/set_velocity drone_navigation/srv/SetVelocity "{velocity: {x: 1.0, y: 0.0, z: 0.5}, yaw_rate: 0.1}"

# Waypoints multiples
ros2 service call /navigation/set_waypoints drone_navigation/srv/SetWaypoints "{waypoints: [{position: {x: 1.0, y: 1.0, z: 3.0}, yaw: 0.0, tolerance: 0.5, yaw_tolerance: 0.1}, {position: {x: 3.0, y: 1.0, z: 4.0}, yaw: 1.57, tolerance: 0.5, yaw_tolerance: 0.1}]}"

# Récupération des waypoints
ros2 service call /navigation/get_waypoints drone_navigation/srv/GetWaypoints

# Retour maison
ros2 service call /navigation/return_home std_srvs/srv/Trigger

# Maintien position
ros2 service call /navigation/hold_position std_srvs/srv/Trigger
```

### Tests d'actions
```bash
# Navigation avec feedback
ros2 action send_goal /navigation/navigate_to_goal drone_navigation/action/NavigateToGoal "{target_position: {x: 5.0, y: 5.0, z: 10.0}, target_yaw: 1.57, position_tolerance: 0.3, yaw_tolerance: 0.1}"

# Suivi de chemin
ros2 action send_goal /navigation/follow_path drone_navigation/action/FollowPath "{waypoints: [{position: {x: 1.0, y: 1.0, z: 3.0}, yaw: 0.0, tolerance: 0.5, yaw_tolerance: 0.1}, {position: {x: 3.0, y: 3.0, z: 5.0}, yaw: 3.14, tolerance: 0.5, yaw_tolerance: 0.1}]}"
```

### Monitoring en temps réel
```bash
# État de navigation
ros2 topic echo /navigation/status

# Position du drone
ros2 topic echo /mavros/local_position/pose

# Vitesse du drone
ros2 topic echo /mavros/local_position/velocity_local

# Chemin planifié
ros2 topic echo /navigation/current_path
```

### Informations du nœud
```bash
# Services disponibles
ros2 service list | grep navigation

# Actions disponibles
ros2 action list | grep navigation

# Informations complètes
ros2 node info /navigation_node

# Paramètres
ros2 param list /navigation_node
```

## Séquence de test complète

### 1. Vérification de l'état initial
```bash
ros2 topic echo /navigation/status --once
```

### 2. Navigation simple
```bash
ros2 service call /navigation/set_position drone_navigation/srv/SetPosition "{position: {x: 2.0, y: 2.0, z: 5.0}, yaw: 0.0}"
```

### 3. Monitoring pendant navigation
```bash
ros2 topic echo /navigation/status
```

### 4. Test de waypoints multiples
```bash
# D'abord arrêter la navigation courante
ros2 service call /navigation/hold_position std_srvs/srv/Trigger

# Puis définir plusieurs waypoints
ros2 service call /navigation/set_waypoints drone_navigation/srv/SetWaypoints "{waypoints: [{position: {x: 1.0, y: 1.0, z: 3.0}, yaw: 0.0, tolerance: 0.5, yaw_tolerance: 0.1}, {position: {x: 3.0, y: 1.0, z: 4.0}, yaw: 1.57, tolerance: 0.5, yaw_tolerance: 0.1}, {position: {x: 3.0, y: 3.0, z: 3.0}, yaw: 3.14, tolerance: 0.5, yaw_tolerance: 0.1}]}"
```

### 5. Test de retour maison
```bash
ros2 service call /navigation/return_home std_srvs/srv/Trigger
```

## Gestion d'erreurs

### Erreurs courantes
- "Navigation déjà en cours" : Utilisez `/navigation/hold_position` d'abord
- "Position actuelle inconnue" : Vérifiez que MAVROS publie la position
- "Altitude supérieure/inférieure au limite" : Respectez les limites d'altitude

### Débogage
```bash
# Vérifier les logs du nœud
ros2 node info /navigation_node

# Vérifier la position MAVROS
ros2 topic echo /mavros/local_position/pose --once

# Vérifier l'état de navigation
ros2 topic echo /navigation/status --once
```

## Utilisation

### Lancement du nœud
```bash
ros2 run drone_navigation navigation_node.py
```

### Prérequis
- MAVROS doit être lancé et connecté
- Le drone doit publier sa position locale
- Le nœud `drone_interface` doit être opérationnel

## Intégration système

Ce nœud s'intègre avec :
- **drone_interface** : Interface centrale MAVROS
- **drone_mission** : Gestion de missions
- **drone_vision** : Perception et évitement d'obstacles

## Auteur
Adama Komi

## Version
1.0.0

## Licence
Apache License 2.0
