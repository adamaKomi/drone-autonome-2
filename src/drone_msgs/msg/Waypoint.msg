# Point de navigation 3D avec métadonnées complètes
std_msgs/Header header

# Identification
uint32 waypoint_id
string waypoint_name
uint32 sequence_number

# Position géographique
float64 latitude               # degrés WGS84
float64 longitude              # degrés WGS84
float64 altitude               # mètres au-dessus niveau mer
float64 relative_altitude      # mètres au-dessus point départ

# Position locale (optionnelle)
geometry_msgs/Point local_position
string local_frame            # Référentiel local utilisé

# Orientation au waypoint
float64 heading               # degrés (0-360)
geometry_msgs/Quaternion target_orientation

# Paramètres d'approche
float64 acceptance_radius     # mètres
float64 approach_speed        # m/s
float64 hover_time           # secondes
string approach_mode         # DIRECT, SMOOTH, PRECISION, SPIRAL

# Actions à effectuer
string[] actions             # "CAPTURE_IMAGE", "DETECT_FLOWERS", "POLLINATE", "HOVER", "WAIT"
float64[] action_durations   # Durées correspondantes en secondes
string[] action_parameters   # Paramètres additionnels JSON

# Contraintes de navigation
float64 max_approach_speed
float64 min_altitude
float64 max_altitude
float64 max_distance_from_home # mètres
bool require_gps_lock
bool avoid_obstacles
string[] required_sensors     # Capteurs requis

# Type et priorité
string waypoint_type         # NAVIGATION, FLOWER_TARGET, SAFETY_POINT, HOME, SURVEY
uint8 priority              # 0=LOW, 1=NORMAL, 2=HIGH, 3=CRITICAL
bool is_mandatory           # Waypoint obligatoire

# Conditions d'activation
string[] activation_conditions # Conditions pour activer ce waypoint
builtin_interfaces/Time earliest_activation
builtin_interfaces/Time latest_activation

# Métadonnées mission
string mission_id
string description
string notes
builtin_interfaces/Time created_time
string created_by
