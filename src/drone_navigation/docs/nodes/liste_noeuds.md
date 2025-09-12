# Liste exhaustive des nœuds ROS2 pour drone_navigation

---

1. navigation_lifecycle_node.py
   - Gestion du cycle de vie du système de navigation
   - Supervision et recovery automatique

2. goto_position_node.py
   - Navigation vers une position GPS ou locale
   - Publication d’état de progression

3. waypoint_manager_node.py
   - Gestion de la liste de waypoints
   - Détection d’événements "waypoint atteint"

4. trajectory_planner_node.py
   - Planification de trajectoire (A*, RRT*, Dijkstra, Dubins, Bézier)
   - Publication de trajectoire planifiée

5. path_optimizer_node.py
   - Optimisation de trajectoire (algorithme génétique, contraintes dynamiques)
   - Replanification temps réel

6. coverage_pattern_node.py
   - Génération de patterns de couverture (zigzag, spiral, lawn mower, boustrophedon, adaptatif)
   - Publication des waypoints de couverture

7. position_controller_node.py
   - Contrôle PID de position/altitude (XYZ, anti-windup)
   - Contrôle d’attitude et approche de précision

8. velocity_controller_node.py
   - Contrôle PID de vitesse (rampes d’accélération)
   - Contrôle de la vitesse en mode suivi de trajectoire

9. obstacle_detection_node.py
   - Détection d’obstacles (mapping 3D, perception)
   - Publication des obstacles détectés

10. obstacle_avoidance_node.py
    - Évitement d’obstacles (Potential Fields, DWA, emergency maneuvers)
    - Gestion des manœuvres d’urgence

11. geofence_manager_node.py
    - Gestion des géobarrières (zones, polygones, altitude)
    - Actions automatiques (RTL, LAND, STOP, warning)

12. mission_manager_node.py
    - Démarrage, pause, reprise, arrêt de mission de navigation
    - Suivi de progression de mission

13. status_publisher_node.py
    - Publication de l’état détaillé de la navigation (JSON)
    - Publication de diagnostics système

14. nav_diagnostics_node.py
    - Monitoring des performances et ressources
    - Détection et publication d’anomalies

15. mavros_integration_node.py
    - Interface avec MAVROS pour commandes et feedback
    - Gestion des modes de vol et armement

---

Chaque nœud : max 2 responsabilités, max 400 lignes, pas de duplication, interfaces via drone_msgs.
