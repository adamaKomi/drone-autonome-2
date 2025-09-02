# DRONE INTERFACE NODE

## Description
Le nœud `drone_interface` est le point d'entrée central pour l'intégration ROS2 avec ArduPilot et MAVROS. Il gère la communication avec le drone, l'armement, la sécurité, la publication d'état et l'interfaçage avec les autres modules spécialisés (navigation, mission, vision).

## Architecture
- **drone_interface** : Interface centrale MAVROS (ce nœud)
- **drone_navigation** : Navigation et contrôle de position
- **drone_mission** : Gestion et exécution de missions
- **drone_vision** : Traitement de vision (futur)

## Responsabilités
- Communication avec MAVROS
- Armement/Désarmement sécurisé
- Changement de modes de vol
- Publication de l'état du drone
- Vérifications de sécurité
- Services de contrôle de base

## Fonctionnalités principales
- Souscription à l’état du drone (`/mavros/state`)
- Souscription à la position locale (`/mavros/local_position/pose`)
- Souscription à la batterie (`/mavros/battery`)
- Souscription à la position GPS (`/mavros/global_position/global`)
- Service d’armement du drone : `/drone/arm` (Trigger)
- Service de vérification de sécurité : `/drone/safety_check` (Trigger)
- Suivi de l’état, du mode, de la batterie, du GPS, de la position

## Services exposés
- `/drone/arm` : Armement du drone
- `/drone/safety_check` : Vérification des conditions de sécurité

## Commandes de test

### Vérification de sécurité
```bash
ros2 service call /drone/safety_check std_srvs/srv/Trigger
```

### Armement du drone
```bash
ros2 service call /drone/arm std_srvs/srv/Trigger
```
> **Remarque :** Pour que l'armement fonctionne, le mode du drone doit être `guided`.

### Changer le mode en 'guided'
```bash
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"
```

### Procédure complète de navigation
```bash
# 1. Préparer l'armement (publier des setpoints)
ros2 service call /navigation/prepare_arm std_srvs/srv/Trigger

# 2. Changer en mode GUIDED
ros2 service call /mavros/set_mode mavros_msgs/srv/SetMode "{custom_mode: 'GUIDED'}"

# 3. Armer le drone
ros2 service call /drone/arm std_srvs/srv/Trigger

# 4. Navigation (après décollage)
ros2 service call /navigation/set_position drone_navigation/srv/SetPosition "{position: {x: 10.0, y: 10.0, z: 15.0}, yaw: 0.0}"
```

### Vérification de l’état du drone
```bash
ros2 topic echo /mavros/state --once
ros2 topic echo /mavros/local_position/pose --once
ros2 topic echo /mavros/battery --once
ros2 topic echo /mavros/global_position/global --once
```

### Lister les services disponibles
```bash
ros2 service list | grep drone
```

### Lister les paramètres du nœud
```bash
ros2 param list /drone_interface
```

### Obtenir les valeurs des paramètres
```bash
ros2 param get /drone_interface <param_name>
```

### Vérifier l’état du nœud
```bash
ros2 node info /drone_interface
```

## Utilisation
Lancer le nœud :
```bash
ros2 run drone_interface interface_node
```

## Auteur
Adama Komi

## Version
2.0.3

## Licence
Apache License 2.0
