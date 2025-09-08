# Waypoint spécialisé pour missions
std_msgs/Header header

# Héritage du waypoint de base
Waypoint base_waypoint

# Spécificités mission
string mission_id
string mission_phase              # APPROACH, SURVEY, POLLINATE, COLLECT, VERIFY
uint32 mission_sequence          # Ordre dans la mission
string mission_objective         # Objectif à ce waypoint

# Cibles spécifiques
FlowerTarget[] flower_targets    # Fleurs cibles à ce waypoint
geometry_msgs/Polygon survey_area # Zone à surveiller
string[] detection_requirements  # Exigences détection

# Paramètres opérationnels
float64 hover_altitude           # Altitude survol (m)
builtin_interfaces/Duration max_time_at_waypoint
string preferred_approach_direction # NORTH, SOUTH, EAST, WEST, OPTIMAL
float64 wind_tolerance           # Tolérance vent max (m/s)

# Actions mission spécifiques
string[] mission_actions         # Actions spécifiques mission
string[] sensor_activations     # Capteurs à activer
string[] data_collection_types  # Types données à collecter
bool require_image_capture
bool require_environmental_data
bool require_flower_detection

# Contraintes mission
bool critical_waypoint           # Waypoint critique
float64 minimum_battery_level    # Batterie min requise (%)
string[] weather_requirements    # Exigences météo
builtin_interfaces/Time time_window_start
builtin_interfaces/Time time_window_end

# Conditions de succès
string[] success_criteria        # Critères succès
float64[] success_thresholds     # Seuils associés
string[] failure_conditions     # Conditions d'échec
uint32 max_retry_attempts       # Tentatives max

# Optimisation locale
bool allow_local_optimization   # Optimisation locale autorisée
float64 optimization_radius     # Rayon optimisation (m)
string[] optimization_objectives # Objectifs optimisation

# Connectivité et communication
bool require_communication      # Communication requise
float64 min_signal_strength     # Force signal min
string[] communication_protocols # Protocoles requis
bool allow_autonomous_mode      # Mode autonome autorisé

# Coordination multi-drone
string[] coordinate_with_drones # Coordination avec autres drones
string synchronization_type     # Type synchronisation
builtin_interfaces/Duration sync_tolerance

# Sécurité et contingence
Waypoint emergency_alternative  # Alternative urgence
string[] escape_routes          # Routes évacuation
string[] contingency_plans      # Plans contingence
float64 safety_margin_multiplier # Multiplicateur marge sécurité

# Vérification et validation
bool require_verification       # Vérification requise
string verification_method      # Méthode vérification
float64 verification_threshold  # Seuil vérification
builtin_interfaces/Duration verification_timeout

# Apprentissage et adaptation
float64 historical_success_rate # Taux succès historique
builtin_interfaces/Duration average_completion_time
string[] learned_optimizations # Optimisations apprises
float64 difficulty_rating      # Note difficulté 0.0-1.0

# Métadonnées avancées
string waypoint_creator         # Créateur waypoint
string creation_algorithm       # Algorithme création
builtin_interfaces/Time last_updated
uint32 revision_number
string[] modification_history
