# navigation_supervisor_node

## Description
Nœud ROS2 de supervision globale pour la navigation drone. Il coordonne les différents modules de navigation (locale, GPS, mission), gère les transitions, surveille la sécurité et arbitre les priorités en fonction de l’état du drone et de l’environnement.

## Fonctionnalités principales
- Supervision des états de navigation (IDLE, NAVIGATING, PAUSED, SUCCEEDED, FAILED)
- Coordination entre navigation locale, GPS et mission manager
- Surveillance des conditions de sécurité via `/drone_nav/safe_to_navigate`
- Arbitrage des priorités (urgence, sécurité, mission, navigation)
- Gestion des transitions entre modes de navigation
- Publication du statut global et des alertes

## Interfaces
### Topics
- **/drone_nav/supervisor_status** (`NavigationStatus`)
  - Statut global de la navigation supervisée
- **/drone_nav/emergency_status** (`EmergencyStatus`)
  - Statut d’urgence ou d’alerte
- **/drone_nav/safe_to_navigate** (`std_msgs/Bool`)
  - Indique si la navigation est autorisée (abonnement)
- **/drone_nav/mission_status** (`MissionStatus`)
  - Statut de la mission (abonnement)
- **/drone_nav/gps_status** (`NavigationStatus`)
  - Statut de la navigation GPS (abonnement)
- **/drone_nav/local_status** (`NavigationStatus`)
  - Statut de la navigation locale (abonnement)

### Services
- **/drone_nav/request_navigation** (`RequestNavigation.srv`)
  - Demande de démarrage d’une navigation supervisée
- **/drone_nav/cancel_navigation** (`CancelNavigation.srv`)
  - Annulation d’une navigation en cours

## Paramètres
- `supervision_rate` : Fréquence de supervision (Hz)
- `emergency_timeout` : Délai avant passage en mode urgence (s)
- `priority_mode` : Mode d’arbitrage (sécurité, mission, navigation)

## Utilisation
1. Lancer le nœud avec les paramètres YAML :
   ```bash
   ros2 run drone_navigation navigation_supervisor_node --ros-args --params-file src/drone_navigation/config/navigation_params.yaml
   ```
2. Observer le statut global et les alertes :
   ```bash
   ros2 topic echo /drone_nav/supervisor_status
   ros2 topic echo /drone_nav/emergency_status
   ```
3. Demander une navigation supervisée :
   ```bash
   ros2 service call /drone_nav/request_navigation drone_msgs/srv/RequestNavigation "{...}"
   ```
4. Annuler une navigation en cours :
   ```bash
   ros2 service call /drone_nav/cancel_navigation drone_msgs/srv/CancelNavigation
   ```

## Logique de supervision
- Le nœud surveille en continu la sécurité, l’état des modules et la progression de la mission
- Il bloque ou autorise la navigation selon `/drone_nav/safe_to_navigate` et les priorités
- Il gère les transitions entre navigation locale, GPS et mission selon l’état du drone et les demandes
- Il publie des alertes en cas d’urgence ou de problème critique

## Notes
- Ce nœud doit être lancé avant le mission manager pour garantir la cohérence et la sécurité globale
- Les paramètres sont ajustables dans le fichier YAML
- Compatible SITL et drone réel

## Auteur
adamaKomi

## Fichier
`src/drone_navigation/drone_navigation/nodes/navigation_supervisor_node.py`
