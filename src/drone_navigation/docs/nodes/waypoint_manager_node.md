# Documentation complète : `waypoint_manager_node`

## 1. Présentation

Le nœud `waypoint_manager_node` gère la liste des waypoints pour les missions de navigation autonome du drone. Il permet d’ajouter, supprimer, récupérer, et séquencer les waypoints, ainsi que de piloter le déroulement d’une mission multi-points.

## 2. Fonctionnalités principales
- Ajout, suppression, modification et récupération des waypoints
- Notification sur l’atteinte d’un waypoint
- Gestion des modes de mission (séquentiel, boucle, backtrack)
- Pause, reprise, et arrêt de mission
- Publication du statut de mission et de progression
- Testabilité indépendante via services et topics

## 3. Interfaces ROS2

### Services
- `/drone_nav/set_waypoints` : Définir la liste complète des waypoints
- `/drone_nav/get_waypoints` : Récupérer la liste des waypoints
- `/drone_nav/add_waypoint` : Ajouter un waypoint à la liste
- `/drone_nav/remove_waypoint` : Supprimer un waypoint
- `/drone_nav/next_waypoint` : Obtenir le prochain waypoint à atteindre
- `/drone_nav/clear_waypoints` : Vider la liste
- `/drone_nav/pause_mission` : Mettre la mission en pause
- `/drone_nav/resume_mission` : Reprendre la mission
- `/drone_nav/get_mission_status` : Récupérer le statut de la mission
- `/drone_nav/set_mission_mode` : Changer le mode de mission

### Topics publiés
- `/drone_nav/waypoint_reached` : Notification d’atteinte d’un waypoint (`drone_msgs/msg/WaypointReached`)
- `/drone_nav/mission_status` : Statut global de la mission (`drone_msgs/msg/MissionStatus`)

### Topics abonnés
- `/drone_nav/path_progress` : Suivi de la progression de navigation
- `/drone_nav/status` : Suivi du statut de navigation

## 4. Utilisation

### Exemples de commandes
```bash
# Ajouter un waypoint
ros2 service call /drone_nav/add_waypoint drone_msgs/srv/AddWaypoint "{waypoint: {position: {x: 10.0, y: 5.0, z: 20.0}, tolerance: 2.0, speed: 5.0, actions: []}, index: 4294967295}"

# Récupérer la liste
ros2 service call /drone_nav/get_waypoints drone_msgs/srv/GetWaypoints "{}"

# Démarrer la mission (via le nœud de navigation ou action)
# Simuler l’atteinte d’un waypoint
ros2 topic pub /drone_nav/path_progress drone_msgs/msg/PathProgress "{distance_remaining: 0.5}"

# Observer la notification
ros2 topic echo /drone_nav/waypoint_reached

# Changer le mode de mission
ros2 service call /drone_nav/set_mission_mode drone_msgs/srv/SetMissionMode "{mode: 'LOOP'}"

# Pause/Reprise
ros2 service call /drone_nav/pause_mission drone_msgs/srv/PauseMission "{}"
ros2 service call /drone_nav/resume_mission drone_msgs/srv/ResumeMission "{}"

# Statut de mission
ros2 service call /drone_nav/get_mission_status drone_msgs/srv/GetMissionStatus "{}"
ros2 topic echo /drone_nav/mission_status
```

## 5. Modes de mission
- **SEQUENTIAL** : Les waypoints sont suivis dans l’ordre
- **LOOP** : La liste est répétée en boucle
- **BACKTRACK** : Retour en inversant la liste

## 6. Bonnes pratiques
- Valider les waypoints (altitude, tolérance, vitesse) avant de lancer la mission
- Surveiller les topics pour le suivi en temps réel
- Utiliser les services pour adapter la mission dynamiquement

## 7. Testabilité
- Tous les services peuvent être testés sans drone réel
- La notification d’atteinte peut être simulée en publiant sur `/drone_nav/path_progress`

## 8. Référence code
Le code source complet est disponible dans :
`src/drone_navigation/drone_navigation/nodes/waypoint_manager_node.py`

---

Pour toute question ou amélioration, consulter la documentation du package ou le code source.
