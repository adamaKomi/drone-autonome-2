# gps_navigation_node

## Description
Nœud ROS2 de navigation GPS pour drone autonome. Il reçoit des commandes de navigation vers une position GPS, publie les setpoints pour MAVROS, gère la progression et le statut, et fournit un serveur d'action pour la navigation asynchrone.

## Fonctionnalités principales
- Service `/drone_nav/goto_position` pour démarrer une navigation GPS
- Service `/drone_nav/update_position` pour mettre à jour la cible en cours de navigation
- Action `/drone_nav/goto_position_action` pour navigation avec feedback et résultat
- Publication du setpoint GPS sur `/mavros/setpoint_position/global` (GeoPoseStamped)
- Publication de la progression sur `/drone_nav/progress` (PathProgress)
- Publication du statut sur `/drone_nav/status` (NavigationStatus)
- Prise en compte de la sécurité via `/drone_nav/safe_to_navigate`
- MultiThreadedExecutor pour la gestion concurrente

## Interfaces
### Services
- **/drone_nav/goto_position** (`GotoPosition.srv`)
  - Démarre la navigation vers une position GPS
- **/drone_nav/update_position** (`UpdatePosition.srv`)
  - Met à jour la cible GPS en cours de navigation

### Action
- **/drone_nav/goto_position_action** (`GotoPositionAction.action`)
  - Permet de lancer une navigation GPS avec feedback et résultat

### Topics
- **/mavros/setpoint_position/global** (`geographic_msgs/GeoPoseStamped`)
  - Setpoint GPS envoyé à MAVROS
- **/drone_nav/progress** (`drone_msgs/PathProgress`)
  - Progression de la navigation (distance restante, position actuelle, cible)
- **/drone_nav/status** (`drone_msgs/NavigationStatus`)
  - Statut de la navigation (IDLE, NAVIGATING, SUCCEEDED, FAILED)
- **/drone_nav/safe_to_navigate** (`std_msgs/Bool`)
  - Indique si la navigation est autorisée

### Souscriptions
- **/mavros/global_position/global** (`sensor_msgs/NavSatFix`)
  - Position GPS actuelle du drone
- **/drone_nav/safe_to_navigate** (`std_msgs/Bool`)
  - Sécurité de navigation

## Paramètres
- `default_tolerance` : Tolérance par défaut pour l'arrivée à la cible (float, recommandé : 20.0 en simulation SITL)

## Utilisation
1. Lancer le nœud avec les paramètres YAML :
   ```bash
   ros2 run drone_navigation gps_navigation_node --ros-args --params-file src/drone_navigation/config/navigation_params.yaml
   ```
2. Publier la sécurité :
   ```bash
   ros2 topic pub /drone_nav/safe_to_navigate std_msgs/Bool "data: true"
   ```
3. Publier la position GPS (si besoin) :
   ```bash
   ros2 topic pub /mavros/global_position/global sensor_msgs/NavSatFix "{latitude: 48.85, longitude: 2.35, altitude: 50.0}"
   ```
4. Appeler le service de navigation (SITL recommandé : tolérance 20.0 m) :
  ```bash
  ros2 service call /drone_nav/goto_position drone_msgs/srv/GotoPosition "{latitude: 33.70913368 , longitude: -7.34989784, altitude: 55.0, yaw_angle: 0.0, tolerance: 20.0}"
  ``` 
5. Utiliser l'action pour navigation asynchrone (SITL recommandé : tolérance 20.0 m) :
  ```bash
  ros2 action send_goal /drone_nav/goto_position_action drone_msgs/action/GotoPositionAction "{latitude: 48.86, longitude: 2.36, altitude: 55.0, yaw_angle: 0.0, tolerance: 20.0}"
  ```


## Suggestions d'amélioration à prévoir
- Ajouter un service ROS2 dédié pour annuler une navigation en cours lancée via le service `/drone_nav/goto_position` (actuellement seule l'action peut être annulée).
  - Exemple : `/drone_nav/cancel_navigation` (type `std_srvs/Trigger` ou personnalisé)
  - Permettre à un superviseur ou à l'opérateur d'interrompre la navigation à tout moment.

## Notes
- Le nœud nécessite MAVROS et une source de position GPS (SITL ou drone réel).
- La sécurité doit être activée pour permettre la navigation.
- Les messages de progression et de statut sont publiés automatiquement pendant la navigation.

## Auteur
adamaKomi

## Fichier
`src/drone_navigation/drone_navigation/nodes/gps_navigation_node.py`
