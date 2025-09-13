# Guide d'utilisation des nœuds du package `drone_interface`

Ce document explique comment utiliser les principaux nœuds ROS2 du package `drone_interface` pour contrôler un drone via MAVROS.

## 1. `arm_disarm_node`
- **Fonction** : Armer ou désarmer le drone.
- **Commande** : Publiez un message `std_msgs/Bool` sur `/arm_control/arm_cmd` (True pour armer, False pour désarmer).
- **Exemple** :
  ```bash
  ros2 topic pub --once /arm_control/arm_cmd std_msgs/Bool "data: true"
  ```

## 2. `mode_node`
- **Fonction** : Changer le mode de vol du drone (ex: GUIDED, LOITER, AUTO).
- **Commande** : Publiez un message `std_msgs/String` sur `/mode_command` avec le nom du mode.
- **Exemple** :
  ```bash
  ros2 topic pub --once /mode_command std_msgs/String "data: 'GUIDED'"
  ```

## 3. `takeoff_land_node`
- **Fonction** : Décollage et atterrissage du drone.
- **Décollage** : Publiez `std_msgs/Bool` (True) sur `/takeoff_control/takeoff_cmd`.
- **Atterrissage** : Publiez `std_msgs/Bool` (True) sur `/takeoff_control/land_cmd`.
- **Exemple** :
  ```bash
  ros2 topic pub --once /takeoff_control/takeoff_cmd std_msgs/Bool "data: true"
  ros2 topic pub --once /takeoff_control/land_cmd std_msgs/Bool "data: true"
  ```

## 4. Statut et monitoring
- **Statut** : Le nœud `takeoff_land_node` publie l'état de la navigation sur `/drone_nav/status` (type `drone_msgs/msg/NavigationStatus`).
- **Position** : La position locale du drone est disponible sur `/mavros/local_position/pose` (type `geometry_msgs/PoseStamped`).

## 5. Lancement des nœuds
Utilisez le fichier de lancement ROS2 pour démarrer tous les nœuds :
```bash
ros2 launch drone_interface interface.launch.py
```

## 6. Remarques
- Assurez-vous que MAVROS est connecté et que le drone est armé avant de lancer des commandes de vol.
- Vérifiez les logs pour les messages d'erreur ou de confirmation.

---

Pour plus de détails, consultez le code source de chaque nœud dans `src/drone_interface/drone_interface/`.
