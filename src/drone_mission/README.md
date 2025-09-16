# Gestionnaires de Waypoints - Documentation

Ce package contient deux nœuds principaux pour la gestion de missions avec waypoints multiples :

## Nœuds disponibles

### 1. GPS Waypoint Manager (`gps_waypoint_manager_node.py`)

Ce nœud gère la navigation entre plusieurs waypoints GPS en utilisant le `gps_navigation_node`.

**Lancement :**
```bash
ros2 run drone_mission gps_waypoint_manager_node.py
```

**Services offerts :**
- `/drone_mission/start_gps_mission` - Démarre la mission GPS
- `/drone_mission/pause_gps_mission` - Met en pause la mission
- `/drone_mission/resume_gps_mission` - Reprend la mission
- `/drone_mission/cancel_gps_mission` - Annule la mission

**Topics publiés :**
- `/drone_mission/gps_mission_status` - Statut de la mission
- `/drone_mission/gps_waypoint_list` - Liste des waypoints

**Waypoints GPS par défaut :**
1. Point de départ : -35.363261, 149.165230, 30m
2. Point intermédiaire : -35.363300, 149.165300, 30m (HOLD)
3. Point de manœuvre : -35.363400, 149.165200, 25m
4. Point d'atterrissage : -35.363261, 149.165230, 20m (LAND)

### 2. Local Waypoint Manager (`local_waypoint_manager_node.py`)

Ce nœud gère la navigation entre plusieurs waypoints locaux en utilisant le `local_navigation_node`.

**Lancement :**
```bash
ros2 run drone_mission local_waypoint_manager_node.py
```

**Services offerts :**
- `/drone_mission/start_local_mission` - Démarre la mission locale
- `/drone_mission/pause_local_mission` - Met en pause la mission
- `/drone_mission/resume_local_mission` - Reprend la mission
- `/drone_mission/cancel_local_mission` - Annule la mission
- `/drone_mission/skip_local_waypoint` - Passe au waypoint suivant

**Topics publiés :**
- `/drone_mission/local_mission_status` - Statut de la mission
- `/drone_mission/local_waypoint_list` - Liste des waypoints

**Waypoints locaux par défaut (NED en mètres) :**
1. Décollage : (0, 0, 10) - TAKEOFF
2. Point Nord : (10, 0, 10)
3. Point Nord-Est : (10, 10, 10)
4. Point Ouest : (0, 10, 10) - HOLD
5. Retour origine : (0, 0, 10)
6. Atterrissage : (0, 0, 0.5) - LAND

## Utilisation avec Launch File

**Lancer les deux gestionnaires :**
```bash
ros2 launch drone_mission waypoint_managers.launch.py
```

**Lancer seulement le gestionnaire GPS :**
```bash
ros2 launch drone_mission waypoint_managers.launch.py local_manager:=false
```

**Lancer seulement le gestionnaire local :**
```bash
ros2 launch drone_mission waypoint_managers.launch.py gps_manager:=false
```

**Lancer en mode test :**
```bash
ros2 launch drone_mission waypoint_managers.launch.py test_mode:=true
```

## Scripts de Test

### Test GPS
```bash
ros2 run drone_mission test_gps_waypoint_manager.py
```

### Test Local
```bash
# Test automatique
ros2 run drone_mission test_local_waypoint_manager.py

# Test interactif
ros2 run drone_mission test_local_waypoint_manager.py --interactive
```

## Exemple d'utilisation

### 1. Démarrer les nœuds de navigation
```bash
# Terminal 1 - Navigation complète
ros2 launch drone_navigation full_navigation.launch.py

# Terminal 2 - Gestionnaires de waypoints
ros2 launch drone_mission waypoint_managers.launch.py
```

### 2. Activer la sécurité (requis)
```bash
ros2 topic pub /drone_nav/safe_to_navigate std_msgs/Bool "data: true" --once
```

### 3. Démarrer une mission GPS
```bash
ros2 service call /drone_mission/start_gps_mission std_srvs/SetBool "data: true"
```

### 4. Démarrer une mission locale
```bash
ros2 service call /drone_mission/start_local_mission std_srvs/SetBool "data: true"
```

### 5. Surveiller le statut
```bash
# Statut mission GPS
ros2 topic echo /drone_mission/gps_mission_status

# Statut mission locale
ros2 topic echo /drone_mission/local_mission_status
```

## Paramètres configurables

### GPS Waypoint Manager
- `auto_continue` (bool, défaut: true) - Continuer même si un waypoint échoue
- `max_retries` (int, défaut: 3) - Nombre de tentatives par waypoint
- `retry_delay` (float, défaut: 5.0) - Délai entre les tentatives (s)

### Local Waypoint Manager
- `auto_continue` (bool, défaut: true) - Continuer même si un waypoint échoue
- `max_retries` (int, défaut: 3) - Nombre de tentatives par waypoint
- `retry_delay` (float, défaut: 3.0) - Délai entre les tentatives (s)
- `hold_time` (float, défaut: 2.0) - Temps d'attente aux waypoints HOLD (s)

## États de Mission

- **IDLE** - En attente
- **RUNNING** - Mission en cours
- **PAUSED** - Mission en pause
- **COMPLETED** - Mission terminée avec succès
- **FAILED** - Mission échouée
- **CANCELLED** - Mission annulée

## États de Waypoint

- **PENDING** - En attente d'exécution
- **ACTIVE** - En cours d'exécution
- **COMPLETED** - Atteint avec succès
- **FAILED** - Échec
- **CANCELLED** - Annulé

## Dépendances

- `drone_navigation` - Nœuds de navigation GPS et locale
- `drone_msgs` - Messages, services et actions
- `rclpy` - Client Python ROS2
- `geometry_msgs` - Messages de géométrie
- `std_msgs` - Messages standards

## Architecture

```
GPS Waypoint Manager
    ↓ (utilise)
GPS Navigation Node
    ↓ (commande)
MAVROS GPS

Local Waypoint Manager
    ↓ (utilise)
Local Navigation Node
    ↓ (commande)
MAVROS Local Position
```

## Notes importantes

1. **Sécurité** : Les nœuds ne démarrent que si `/drone_nav/safe_to_navigate` est à `true`
2. **Ordre des waypoints** : Les waypoints sont exécutés séquentiellement
3. **Gestion d'erreurs** : Les échecs peuvent être automatiquement contournés selon la configuration
4. **Thread-safety** : Tous les nœuds utilisent des verrous pour l'accès concurrent aux données
5. **Feedback temps réel** : Progression et statut publiés en continu
