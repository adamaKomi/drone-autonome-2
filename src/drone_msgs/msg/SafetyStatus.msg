# État complet de sécurité du système
std_msgs/Header header

# Niveau de sécurité global
uint8 SAFETY_LEVEL_SAFE = 0
uint8 SAFETY_LEVEL_WARNING = 1  
uint8 SAFETY_LEVEL_CRITICAL = 2
uint8 SAFETY_LEVEL_EMERGENCY = 3

uint8 overall_safety_level
string safety_message

# Vérifications pré-vol
bool preflight_complete
bool[] preflight_checks        # Résultats individuels
string[] preflight_items       # Noms des vérifications
string[] preflight_failures    # Détails des échecs

# Systèmes critiques
bool battery_safe
bool gps_safe
bool communication_safe
bool sensors_safe
bool actuators_safe
bool geofence_safe

# Conditions environnementales
bool weather_safe
float64 wind_speed             # m/s
float64 wind_direction         # degrés
float64 temperature            # °C
float64 humidity               # %
float64 pressure               # hPa
string weather_condition       # CLEAR, CLOUDY, RAIN, STORM

# Geofence et contraintes
bool inside_geofence
float64 distance_to_geofence   # mètres
bool altitude_limits_ok
bool speed_limits_ok
string[] active_constraints
string[] violated_constraints

# Actions de sécurité
bool emergency_stop_available
bool return_to_home_available
bool emergency_landing_available
string[] recommended_actions
string[] prohibited_actions

# Historique sécurité
uint32 total_safety_events
builtin_interfaces/Time last_safety_event
string last_safety_event_type
