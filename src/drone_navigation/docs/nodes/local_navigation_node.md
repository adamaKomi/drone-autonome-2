# local_navigation_node

## Description
Nœud ROS2 de navigation locale pour drone autonome. Il permet de déplacer le drone vers une position locale (x, y, z) en utilisant des setpoints locaux, avec gestion de la sécurité, du feedback et du statut. Il supporte les commandes synchrones (service) et asynchrones (action) et publie la progression et le statut de la navigation.

## Fonctionnalités principales
- Service `/drone_nav/goto_local` pour démarrer une navigation locale
- Service `/drone_nav/update_local` pour mettre à jour la cible locale en cours de navigation
- Action `/drone_nav/goto_local_action` pour navigation locale avec feedback et résultat
- Publication du setpoint local sur `/mavros/setpoint_position/local` (`PoseStamped`)
- Publication de la progression sur `/drone_nav/progress` (`PathProgress`)
- Publication du statut sur `/drone_nav/status` (`NavigationStatus`)
- Prise en compte de la sécurité via `/drone_nav/safe_to_navigate`
- MultiThreadedExecutor pour la gestion concurrente

## Interfaces
### Services
- **/drone_nav/goto_local** (`GotoLocal.srv`)
  - Démarre la navigation vers une position locale (x, y, z, yaw)
- **/drone_nav/update_local** (`UpdateLocal.srv`)
  - Met à jour la cible locale en cours de navigation

### Action
- **/drone_nav/goto_local_action** (`GotoLocalAction.action`)
  - Permet de lancer une navigation locale avec feedback et résultat

### Topics
- **/mavros/setpoint_position/local** (`geometry_msgs/PoseStamped`)
  - Setpoint local envoyé à MAVROS
- **/drone_nav/progress** (`drone_msgs/PathProgress`)
  - Progression de la navigation (distance restante, position actuelle, cible)
- **/drone_nav/status** (`drone_msgs/NavigationStatus`)
  - Statut de la navigation (IDLE, NAVIGATING, SUCCEEDED, FAILED)
- **/drone_nav/safe_to_navigate** (`std_msgs/Bool`)
  - Indique si la navigation est autorisée

### Souscriptions
- **/mavros/local_position/pose** (`geometry_msgs/PoseStamped`)
  - Position locale actuelle du drone
- **/drone_nav/safe_to_navigate** (`std_msgs/Bool`)
  - Sécurité de navigation

## Paramètres
- `default_tolerance` : Tolérance par défaut pour l'arrivée à la cible locale (float, recommandé : 1.0)
- `default_speed` : Vitesse par défaut pour la navigation locale (float, m/s)
- `max_velocity` : Vitesse maximale autorisée (float, m/s)

## Utilisation
1. Lancer le nœud avec les paramètres YAML :
   ```bash
   ros2 run drone_navigation local_navigation_node --ros-args --params-file src/drone_navigation/config/navigation_params.yaml
   ```
2. Publier la sécurité :
   ```bash
   ros2 topic pub /drone_nav/safe_to_navigate std_msgs/Bool "data: true" --once
   ```
3. Publier la position locale (si besoin) :
   ```bash
   ros2 topic pub /mavros/local_position/pose geometry_msgs/PoseStamped "{header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'map'}, pose: {position: {x: 1.0, y: 2.0, z: 3.0}, orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}}}"
   ```
4. Appeler le service de navigation locale :
   ```bash
  ros2 service call /drone_nav/goto_local drone_msgs/srv/GotoLocal "{x: 2000.0, y: 1000.0, z: 50.0, yaw_angle: 3.0}"
   ```
5. Utiliser l'action pour navigation asynchrone :
   ```bash
   ros2 action send_goal /drone_nav/goto_local_action drone_msgs/action/GotoLocalAction "{x: 5.0, y: 5.0, z: 5.0, yaw_angle: 0.0, tolerance: 1.0}"
   ```

## Suggestions d'amélioration à prévoir

- Ajouter un service ROS2 dédié pour annuler une navigation en cours lancée via le service `/drone_nav/goto_local` (actuellement seule l'action peut être annulée).
  - Exemple : `/drone_nav/cancel_local_navigation` (type `std_srvs/Trigger` ou personnalisé)
  - Permettre à un superviseur ou à l'opérateur d'interrompre la navigation à tout moment.

- Ajouter la possibilité de changer dynamiquement la cible locale pendant la navigation, même si le drone n'a pas encore atteint le point prévu.
  - Permettre d'appeler `/drone_nav/update_local` à tout moment pour modifier la cible en cours.
  - Gérer la logique de transition et de sécurité lors du changement de cible.

## Notes
- Le nœud nécessite MAVROS et une source de position locale (SITL ou drone réel).
- La sécurité doit être activée pour permettre la navigation.
- Les messages de progression et de statut sont publiés automatiquement pendant la navigation.

## Auteur
adamaKomi

## Fichier
`src/drone_navigation/drone_navigation/nodes/local_navigation_node.py`
