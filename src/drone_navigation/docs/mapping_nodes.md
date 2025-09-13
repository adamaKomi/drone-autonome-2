# Mapping des services/actions/topics pour chaque nœud du package drone_navigation

---

## 1. navigation_lifecycle_node.py
- Services :
  - /drone_nav/configure
  - /drone_nav/activate
  - /drone_nav/deactivate
  - /drone_nav/cleanup
- Topics publiés :
  - /drone_nav/status
  - /diagnostics
- Responsabilités :
  - Gestion du cycle de vie
  - Recovery automatique

## 2. goto_position_node.py
- Services :
  - /drone_nav/goto_position
  - /drone_nav/goto_local
- Actions :
  - /drone_nav/goto_position_action
- Topics publiés :
  - /drone_nav/path_progress
- Responsabilités :
  - Navigation vers position GPS ou locale
  - Publication de progression

## 3. waypoint_manager_node.py
- Services :
  - /drone_nav/set_waypoints
  - /drone_nav/get_waypoints
- Topics publiés :
  - /drone_nav/waypoint_reached
- Responsabilités :
  - Gestion de la liste de waypoints
  - Détection d'événements

## 4. trajectory_planner_node.py
- Services :
  - /drone_nav/plan_path
- Topics publiés :
  - /drone_nav/trajectory
- Responsabilités :
  - Planification de trajectoire
  - Publication de trajectoire

## 5. path_optimizer_node.py
- Services :
  - /drone_nav/optimize_path
- Topics publiés :
  - /drone_nav/trajectory_optimized
- Responsabilités :
  - Optimisation de trajectoire
  - Replanification temps réel

## 6. coverage_pattern_node.py
- Services :
  - /drone_nav/generate_coverage_pattern
- Topics publiés :
  - /drone_nav/coverage_waypoints
- Responsabilités :
  - Génération de patterns de couverture
  - Publication des waypoints

## 7. position_controller_node.py
- Services :
  - /drone_nav/set_position_control_params
- Topics publiés :
  - /drone_nav/position_control_status
- Responsabilités :
  - Contrôle PID position/altitude
  - Approche de précision

## 8. velocity_controller_node.py
- Services :
  - /drone_nav/set_velocity_control_params
- Topics publiés :
  - /drone_nav/velocity_control_status
- Responsabilités :
  - Contrôle PID vitesse
  - Suivi de trajectoire

## 9. obstacle_detection_node.py
- Topics publiés :
  - /drone_nav/obstacles_detected
- Topics souscrits :
  - /perception/obstacles
- Responsabilités :
  - Détection d'obstacles
  - Publication obstacles

## 10. obstacle_avoidance_node.py
- Services :
  - /drone_nav/set_avoidance_params
- Topics publiés :
  - /drone_nav/avoidance_status
- Responsabilités :
  - Évitement d'obstacles
  - Manœuvres d'urgence

## 11. geofence_manager_node.py
- Services :
  - /drone_nav/set_geofence
  - /drone_nav/clear_geofence
- Topics publiés :
  - /drone_nav/geofence_status
- Responsabilités :
  - Gestion des géobarrières
  - Actions automatiques

## 12. mission_manager_node.py
- Services :
  - /drone_nav/start_mission
  - /drone_nav/pause_mission
  - /drone_nav/resume_mission
- Topics publiés :
  - /drone_nav/mission_progress
- Responsabilités :
  - Gestion des missions
  - Suivi de progression

## 13. status_publisher_node.py
- Topics publiés :
  - /drone_nav/status
  - /diagnostics
- Responsabilités :
  - Publication état navigation
  - Diagnostics système

## 14. nav_diagnostics_node.py
- Topics publiés :
  - /drone_nav/diagnostics
- Responsabilités :
  - Monitoring performances
  - Détection d'anomalies

## 15. mavros_integration_node.py
- Topics publiés :
  - /mavros/setpoint_position/local
  - /mavros/setpoint_velocity/cmd_vel
- Topics souscrits :
  - /mavros/local_position/pose
  - /mavros/local_position/velocity_local
- Responsabilités :
  - Interface MAVROS commandes/feedback
  - Gestion modes de vol

---

Chaque nœud doit utiliser les messages/services définis dans drone_msgs.
