# DRONE MISSION NODE

## Description
Le nœud `drone_mission` est le chef d’orchestre de la mission autonome du drone. Il gère la planification, le séquencement, l’exécution et le suivi des missions complexes, en interfaçant les nœuds de navigation et d’interface. Il permet de charger, créer, contrôler et surveiller des missions structurées, tout en assurant la robustesse et la sécurité de l’exécution.

---

## Fonctionnalités principales
- Chargement et création de missions (JSON, services ROS2)
- Exécution séquentielle de tâches (armement, décollage, navigation, atterrissage, etc.)
- Gestion des conditions et événements (batterie, état du drone, mode de vol)
- Interface avec les nœuds `drone_interface` et `drone_navigation` (services ROS2)
- Publication d’état et de progression (topics ROS2)
- Gestion des erreurs, pauses, reprises et arrêts d’urgence
- Callbacks pour suivi et reporting

---

## Interfaces ROS2

### Services
- `/drone/mission/control` (`MissionControl.srv`)
  - Commandes : `START`, `PAUSE`, `RESUME`, `STOP`, `EMERGENCY`
  - Contrôle l’exécution de la mission
- `/drone/mission/load` (`LoadMission.srv`)
  - Charge une mission depuis un fichier JSON
- `/drone/mission/create` (`CreateMission.srv`)
  - Crée et sauvegarde une mission à partir de paramètres

### Topics
- `/drone/mission/status` (`MissionStatus.msg`)
  - Publie l’état détaillé de la mission (ID, nom, état, tâche courante, progression, temps écoulé)
- `/drone/mission/progress` (`std_msgs/String`)
  - Publie la progression simple (état, index, pourcentage)

### Souscriptions
- `/drone/status` (`std_msgs/String`)
  - Reçoit l’état du drone (mode, armement, connexion)
- `/drone/battery_percentage` (`std_msgs/Float64`)
  - Reçoit le niveau de batterie

---

## Cycle de vie d’une mission
1. **Chargement/création** : La mission est chargée ou créée via service, puis placée en état `READY`.
2. **Démarrage** : Commande `START` via service, passage en état `EXECUTING`.
3. **Exécution séquentielle** : Chaque tâche est exécutée selon son type et ses conditions.
4. **Gestion des erreurs** : Retry automatique, gestion des tâches optionnelles, arrêt sur échec critique.
5. **Pause/reprise/arrêt** : Commandes via service, gestion du thread d’exécution.
6. **Fin** : Passage en état `COMPLETED`, `FAILED` ou `ABORTED`, publication de l’état final.

---

## Types de tâches supportés
- `ARM`, `DISARM` : Armement/désarmement du drone
- `TAKEOFF`, `LAND` : Décollage/atterrissage
- `GOTO`, `WAYPOINT` : Navigation vers position ou waypoint
- `SET_MODE` : Changement de mode de vol
- `WAIT`, `WAIT_FOR_CONDITION` : Attente temporisée ou conditionnelle
- `RTL` : Retour au point de départ
- `SURVEY`, `ORBIT`, `CUSTOM` : Missions avancées ou personnalisées

---

## Exemple d’utilisation

### 1. Lancer le nœud
```bash
ros2 run drone_mission mission_node
```
ou
```bash
python3 src/drone_mission/drone_mission/mission_node.py
```

### 2. Charger une mission
```bash
ros2 service call /drone/mission/load drone_mission/srv/LoadMission "{mission_name: 'test_simple'}"
```

### 3. Démarrer la mission
```bash
ros2 service call /drone/mission/control drone_mission/srv/MissionControl "{command: 'START'}"
```

### 4. Suivre la progression
```bash
ros2 topic echo /drone/mission/status
ros2 topic echo /drone/mission/progress
```

---

## Personnalisation et extension
- Ajouter des tâches : Modifier le JSON de mission ou utiliser le service de création
- Ajouter des types de tâches : Étendre l’énumération `TaskType` et la logique d’exécution
- Intégrer d’autres nœuds : Ajouter des clients/services dans `MissionExecutor`
- Callbacks : Ajouter des fonctions de suivi pour reporting ou UI

---

## Sécurité et robustesse
- Vérification des conditions : Batterie, état, mode, armement
- Timeouts et retries : Configurables par tâche
- Arrêt d’urgence : Commande `EMERGENCY` pour interruption immédiate
- Gestion des erreurs : Log détaillé, reporting, passage en état `FAILED` ou `ABORTED`

---

## Dépendances
- ROS2 Humble
- Packages : `rclpy`, `std_msgs`, `geometry_msgs`, `drone_interface`, `drone_navigation`
- Fichiers de mission JSON dans `src/drone_mission/missions/`

---

## Conclusion
Ce nœud permet de piloter des missions autonomes complexes pour drone ROS2, avec une architecture modulaire, extensible et robuste, adaptée à l’intégration avec les nœuds de navigation et d’interface. Toutes les opérations sont pilotables via services et topics ROS2 pour une automatisation et un monitoring complet.
