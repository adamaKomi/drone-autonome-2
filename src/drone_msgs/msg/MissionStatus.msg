# État détaillé de la mission en cours
std_msgs/Header header

# Identification mission
string mission_id
string mission_name
string mission_description
string mission_type              # SINGLE_FLOWER, ZONE_COVERAGE, SURVEY, RESEARCH, EMERGENCY

# État d'exécution
string STATUS_PLANNING = "PLANNING"
string STATUS_READY = "READY"
string STATUS_STARTING = "STARTING"
string STATUS_EXECUTING = "EXECUTING"
string STATUS_PAUSED = "PAUSED"
string STATUS_COMPLETED = "COMPLETED"
string STATUS_ABORTED = "ABORTED"
string STATUS_FAILED = "FAILED"

string status
float64 progress_percentage      # 0.0-100.0
builtin_interfaces/Time planned_start_time
builtin_interfaces/Time actual_start_time
builtin_interfaces/Time estimated_completion_time
builtin_interfaces/Time actual_completion_time

# Objectifs et progression
uint32 total_waypoints
uint32 completed_waypoints
uint32 skipped_waypoints
uint32 failed_waypoints

uint32 total_flowers_targeted
uint32 flowers_approached
uint32 flowers_pollinated_successfully
uint32 flowers_pollinated_partially
uint32 flowers_failed
uint32 flowers_skipped

# Performance temps réel
float64 current_speed            # m/s
float64 average_speed           # m/s
float64 max_speed_achieved      # m/s
float64 distance_traveled       # mètres
float64 total_planned_distance  # mètres
float64 energy_consumed         # Wh
float64 estimated_total_energy  # Wh

# Navigation actuelle
Waypoint current_waypoint
Waypoint next_waypoint
uint32 current_waypoint_index
float64 distance_to_current_waypoint
float64 estimated_time_to_waypoint
string current_navigation_mode

# Zones et couverture
float64 area_planned            # m² planifiée
float64 area_covered           # m² couverte
float64 coverage_efficiency    # % efficacité couverture
geometry_msgs/Polygon current_zone
string[] zones_completed
string[] zones_remaining

# Données collectées
uint32 images_captured
uint32 video_segments_recorded
uint32 scientific_measurements
uint32 environmental_readings
float64 data_storage_used_mb

# Conditions et contraintes
EnvironmentData current_environment
BatteryStatus current_battery
SafetyStatus current_safety
string[] active_constraints
string[] violated_constraints
string[] warnings_active

# Problèmes et événements
string[] issues_encountered
string[] critical_events
builtin_interfaces/Time[] event_timestamps
string[] recovery_actions_taken
uint32 emergency_stops_triggered

# Optimisation adaptative
bool adaptive_optimization_enabled
string[] optimization_adjustments_made
float64 plan_efficiency_score   # 0.0-1.0
uint32 replanning_events

# Qualité mission
float64 mission_quality_score   # 0.0-1.0 qualité globale
float64 data_quality_score     # 0.0-1.0 qualité données
float64 efficiency_score       # 0.0-1.0 efficacité
float64 safety_score          # 0.0-1.0 respect sécurité
string[] quality_factors       # Facteurs impactant qualité

# Prédictions
builtin_interfaces/Duration estimated_remaining_time
float64 estimated_remaining_energy
float64 completion_probability  # 0.0-1.0
string[] completion_risks      # Risques pour complétion

# Métadonnées
string mission_operator
string mission_supervisor
builtin_interfaces/Time mission_created_time
string mission_priority        # LOW, NORMAL, HIGH, CRITICAL
string[] mission_tags
