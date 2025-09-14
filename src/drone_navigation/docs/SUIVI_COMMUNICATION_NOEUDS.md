# Points d'amélioration et suivi de la communication entre les nœuds

Ce fichier consigne les remarques et suggestions pour améliorer la robustesse et l'autonomie de la communication entre `goto_position_node` et `waypoint_manager_node`.

## 0. Responsabilités dupliquées et recommandations

### Analyse des doublons
- Les deux nœuds gèrent certains aspects de la mission (démarrage, arrêt, progression), ce qui peut créer des ambiguïtés ou des conflits.
- La logique de progression dans la séquence des waypoints peut être présente dans les deux nœuds.
- La validation des paramètres de navigation (tolérance, altitude, etc.) est parfois dupliquée.

### Recommandations pour une architecture claire
- **Centraliser la gestion de mission dans le manager** : Le `waypoint_manager_node` doit être le seul à piloter l'état de la mission (démarrage, arrêt, progression, mode).
- **goto_position_node** doit uniquement exécuter les ordres reçus (navigation vers une position ou un waypoint) et rapporter l'état d'exécution (succès, échec, progression).
- La séquence des waypoints doit être décidée par le manager, qui utilise le nœud de navigation comme un service d'exécution.
- La validation métier (tolérance, mode de mission, etc.) doit être centralisée dans le manager, la validation technique restant dans le nœud de navigation.
- Supprimer ou désactiver la gestion interne de mission dans `goto_position_node` (services `/start_mission`, `/stop_mission`), ou les rendre privés.
- Documenter le workflow :
  - Le manager initie chaque navigation.
  - Le nœud de navigation exécute et rapporte.
  - Le manager décide de la suite (prochain waypoint, fin de mission, etc.).

**Résumé** : Le manager doit être le chef d'orchestre, le nœud de navigation doit être passif et réactif, sans logique métier de mission. Cela garantit une architecture claire, modulaire et facilement testable.

## 1. Démarrage de mission
- Le service `/drone_nav/start_mission` est défini dans `goto_position_node.py`, mais il n’est pas appelé automatiquement par le `waypoint_manager_node`.
- **Action recommandée** : Ajouter une logique pour démarrer la mission depuis le manager, ou documenter le workflow attendu.

## 2. Fin de mission
- La gestion de la fin de mission doit être synchronisée :
  - Le manager publie le statut, mais le nœud de navigation doit aussi arrêter la navigation proprement (service `/drone_nav/stop_mission`).
- **Action recommandée** : Vérifier la synchronisation et l'arrêt propre des deux nœuds.

## 3. Retour d’état
- Le manager reçoit le statut via `/drone_nav/status`, mais il n’y a pas de retour explicite du nœud de navigation vers le manager pour indiquer l’échec ou l’abandon d’une navigation (hors topic).
- **Action recommandée** : S'assurer que le manager réagit bien à tous les statuts (FAILED, ABORTED, COMPLETED).

## 4. Actions associées aux waypoints
- Les actions (ex : TAKE_PHOTO, WAIT_5s) sont gérées dans le manager, mais il n’y a pas d’appel vers des nœuds externes (caméra, etc.).
- **Action recommandée** : Ajouter l’intégration avec les services/actions des autres modules si nécessaire.

## 5. Mise à jour dynamique de la cible
- Le service `/drone_nav/update_position` permet de changer la cible en cours de navigation, mais il n’est pas utilisé par le manager pour corriger la trajectoire en cas d’événement.
- **Action recommandée** : Envisager l'utilisation de ce service pour des corrections dynamiques.

## 6. Sécurité et validation
- Les deux nœuds valident les entrées (tolérance, altitude, etc.), mais il faut s’assurer que les erreurs sont bien propagées et traitées.
- **Action recommandée** : Vérifier la gestion des erreurs et leur propagation.

## 7. Documentation et workflow
- Documenter clairement le workflow :
  - Qui démarre la mission ?
  - Comment la séquence des waypoints est-elle synchronisée ?
  - Que se passe-t-il en cas d’erreur ou d’annulation ?

---

## 8. Interfaces personnalisées utilisées

### Messages

# Waypoint.msg
geometry_msgs/Point position  # Position (x,y,z or lat/lon/alt)
float32 tolerance             # Position tolerance for waypoint completion
string wp_type                # Waypoint type: "NORMAL", "HOLD", "TAKEOFF", "LAND"
string[] actions              # Actions to perform at waypoint
float32 speed                 # Recommended speed to approach waypoint
float32 yaw                   # Desired yaw orientation
float32 hold_time             # Time to hold at waypoint (for HOLD type)

# NavigationStatus.msg
builtin_interfaces/Time stamp
string status             # "IDLE", "NAVIGATING", "SUCCEEDED", "FAILED", "ABORTED"
string mode               # "GPS", "LOCAL"
geometry_msgs/Point target_position
float64 progress
string message

# PathProgress.msg
builtin_interfaces/Time stamp
float64 progress
float64 distance_remaining
geometry_msgs/Point current_position
geometry_msgs/Point target_position
string status

# MissionStatus.msg
builtin_interfaces/Time stamp
string status
string mode
uint32 current_waypoint
uint32 total_waypoints
float32 progress
string message

# WaypointReached.msg
uint32 index
Waypoint waypoint
builtin_interfaces/Time reached_time

### Services

# NextWaypoint.srv
---
Waypoint waypoint
bool success
string message
bool mission_completed

# GotoPosition.srv
float64 latitude
float64 longitude
float64 altitude
float64 yaw_angle
float64 tolerance
---
bool success
string message

# GotoLocal.srv
float64 x
float64 y
float64 z
float64 yaw_angle
---
bool success
string message

# SetWaypoints.srv
Waypoint[] waypoints
---
bool success
string message

# GetWaypoints.srv
---
Waypoint[] waypoints
bool success
string message

# AddWaypoint.srv
Waypoint waypoint
uint32 index
---
bool success
string message

# RemoveWaypoint.srv
int32 index
---
bool success
string message

# ClearWaypoints.srv
---
bool success
string message

# PauseMission.srv
---
bool success
string message

# ResumeMission.srv
---
bool success
string message

# GetMissionStatus.srv
---
MissionStatus status
bool success
string message

# SetMissionMode.srv
string mode
---
bool success
string message

### Actions

# GotoPositionAction.action
float64 latitude
float64 longitude
float64 altitude
float64 yaw_angle
float64 tolerance
---
# Result
bool success
string message
geometry_msgs/Point final_position
---
# Feedback
float64 progress
float64 distance_remaining
geometry_msgs/Point current_position
string current_status

**Ce fichier doit être mis à jour à chaque évolution ou correction apportée à la communication entre les nœuds.**


## 9. Division possible en nœuds plus petits

Il est possible et recommandé de diviser les nœuds actuels en nœuds plus petits pour améliorer la modularité, la maintenabilité et la testabilité.

### Pour `goto_position_node` (Navigation)
- **Nœud de navigation GPS** : navigation vers des positions GPS uniquement.
- **Nœud de navigation locale** : navigation dans le repère local (x, y, z).
- **Nœud de gestion de sécurité** : vérification de l’état MAVROS, batterie, altitude, etc.
- **Nœud d’action** : gestion de l’ActionServer pour les commandes complexes (goto, update, cancel).

### Pour `waypoint_manager_node` (Mission/Waypoints)
- **Nœud de gestion de la liste de waypoints** : ajout, suppression, validation, etc.
- **Nœud de gestion de mission** : démarrage, pause, reprise, fin, mode de mission.
- **Nœud d’exécution d’actions associées aux waypoints** : prise de photo, attente, etc.
- **Nœud de suivi de progression** : abonnement à la progression, publication du statut.

### Avantages
- Responsabilités claires : chaque nœud a une tâche précise.
- Testabilité : chaque composant peut être testé indépendamment.
- Scalabilité : ajout facile de nouveaux comportements (ex : gestion d’obstacles, intégration caméra).
- Robustesse : un bug dans un nœud n’affecte pas tout le système.

### Exemple d’architecture
- `gps_navigation_node.py`
- `local_navigation_node.py`
- `navigation_safety_node.py`
- `waypoint_list_node.py`
- `mission_manager_node.py`
- `waypoint_action_executor_node.py`
- `progress_monitor_node.py`

**Conclusion** :
Diviser en nœuds plus petits est possible et bénéfique. Il faut bien définir les interfaces (topics/services/actions) entre eux pour garder la cohérence du système.
