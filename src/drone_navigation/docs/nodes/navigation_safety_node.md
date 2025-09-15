## Contexte d'exécution recommandé

Ce nœud doit être lancé en même temps que les autres nœuds critiques du système drone :

- MAVROS (doit être actif pour fournir les états, la batterie, le GPS, etc.)
- Les nœuds de navigation (local, GPS, mission manager, etc.)
- Les nœuds de commande ou de supervision

**Recommandation :**
Lance `navigation_safety_node` dès le démarrage du système, avant toute commande de navigation. Il doit tourner en continu pour garantir la sécurité et permettre aux autres nœuds de vérifier l'autorisation via `/drone_nav/safe_to_navigate`.

**Test idéal :**
1. Démarre MAVROS et les nœuds de navigation (local, GPS, mission manager).
2. Lance `navigation_safety_node`.
3. Observe `/drone_nav/safe_to_navigate` et `/drone_nav/safety_status` pendant l'exécution des autres nœuds.
4. Vérifie que la navigation est bloquée si une condition de sécurité n'est pas remplie (batterie, GPS, etc.).
5. Vérifie que la navigation reprend automatiquement quand tout est OK.
# navigation_safety_node

## Description
Nœud ROS2 de sécurité pour la navigation drone. Il vérifie en continu les conditions critiques (connexion MAVROS, armement, mode de vol, batterie, GPS, altitude) et publie l'autorisation de navigation sur `/drone_nav/safe_to_navigate`.

## Fonctionnalités principales
- Vérification périodique de la sécurité (connexion, armement, mode, batterie, GPS, altitude)
- Publication du statut de sécurité (`Bool`) sur `/drone_nav/safe_to_navigate`
- Publication du message d'état détaillé (`String`) sur `/drone_nav/safety_status`
- Paramètres configurables : seuil batterie, altitude minimale, fréquence de vérification
- Thread-safe pour tous les accès et publications

## Interfaces
### Publishers
- **/drone_nav/safe_to_navigate** (`std_msgs/Bool`)
  - Indique si la navigation est autorisée (True/False)
- **/drone_nav/safety_status** (`std_msgs/String`)
  - Message détaillé sur la raison du statut de sécurité

### Subscribers
- **/mavros/state** (`mavros_msgs/State`)
  - État MAVROS (connexion, armement, mode)
- **/mavros/battery** (`mavros_msgs/BatteryState`)
  - Niveau de batterie
- **/mavros/global_position/global** (`sensor_msgs/NavSatFix`)
  - Position GPS globale
- **/mavros/local_position/pose** (`geometry_msgs/PoseStamped`)
  - Position locale
- **/mavros/global_position/rel_alt** (`std_msgs/Float64`)
  - Altitude relative

## Paramètres
- `low_battery_threshold` : Seuil de batterie critique (%)
- `min_takeoff_altitude` : Altitude minimale pour navigation (m)
- `safety_check_rate` : Fréquence de vérification (Hz)

## Utilisation
1. Lancer le nœud avec les paramètres YAML :
   ```bash
   ros2 run drone_navigation navigation_safety_node --ros-args --params-file src/drone_navigation/config/navigation_params.yaml
   ```
2. Observer le statut sécurité :
   ```bash
   ros2 topic echo /drone_nav/safe_to_navigate
   ros2 topic echo /drone_nav/safety_status
   ```
3. Vérifier la réaction en cas de batterie faible, perte GPS, altitude insuffisante, etc.

## Logique de sécurité
- Le nœud refuse la navigation si :
  - MAVROS non connecté ou drone non armé
  - Mode de vol non supporté
  - Batterie sous le seuil
  - Altitude trop basse
  - GPS absent ou imprécis
  - Position locale indisponible
- Le message `/drone_nav/safety_status` détaille la cause du refus

## Notes
- Ce nœud doit tourner en continu pour garantir la sécurité.
- Il est compatible avec SITL et drone réel.
- Les paramètres sont ajustables dans le fichier YAML.

## Auteur
adamaKomi

## Fichier
`src/drone_navigation/drone_navigation/nodes/navigation_safety_node.py`
