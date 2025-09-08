# Trajectoire complète avec timing et contraintes
std_msgs/Header header

# Identification
string trajectory_id
string trajectory_name
string trajectory_type        # DIRECT, SMOOTH, OPTIMAL, EMERGENCY, SURVEY

# Points de trajectoire
Waypoint[] waypoints
geometry_msgs/Point[] interpolated_points # Points interpolés entre waypoints
builtin_interfaces/Duration[] segment_durations
float64[] segment_speeds
float64[] segment_accelerations

# Paramètres globaux
float64 total_distance        # mètres
builtin_interfaces/Duration total_duration
builtin_interfaces/Duration estimated_duration
float64 average_speed         # m/s
float64 max_speed            # m/s
float64 max_acceleration     # m/s²
float64 max_jerk            # m/s³

# Profil d'altitude
float64 min_altitude         # mètres
float64 max_altitude         # mètres
float64[] altitude_profile   # Altitude le long de la trajectoire

# Contraintes appliquées
float64 safety_margin        # mètres obstacles
bool avoid_obstacles
bool maintain_minimum_altitude
bool respect_geofences
string[] geofence_constraints
string[] weather_constraints

# Optimisation
string optimization_objective # TIME, ENERGY, SAFETY, PRECISION, COVERAGE
float64 optimization_score   # 0.0-1.0
string optimization_method   # GENETIC, GRADIENT, ASTAR, RRT
builtin_interfaces/Duration planning_time

# Validation et sécurité
bool trajectory_validated
string[] validation_checks   # Tests passés
string[] validation_warnings # Warnings générés
string[] safety_violations   # Violations détectées

# Exécution
string execution_status      # PLANNED, EXECUTING, PAUSED, COMPLETED, ABORTED
uint32 current_waypoint_index
float64 completion_percentage # 0.0-100.0
builtin_interfaces/Time start_time
builtin_interfaces/Time estimated_completion

# Métadonnées
string planner_version
string created_by
builtin_interfaces/Time creation_time
string mission_context
