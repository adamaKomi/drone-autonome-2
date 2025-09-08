# Mode de vol actuel et paramètres
std_msgs/Header header

# Mode principal
string mode                    # MANUAL, STABILIZE, ALT_HOLD, LOITER, AUTO, GUIDED, RTL, LAND, BRAKE, THROW
string custom_mode             # Mode personnalisé spécifique
uint8 base_mode               # Mode de base MAVLink

# Permissions et capacités
bool guided_enabled
bool manual_input_enabled
bool auto_mission_enabled
bool failsafe_enabled

# Paramètres mode actuel
float64 target_altitude       # Altitude cible (m)
float64 target_speed          # Vitesse cible (m/s)
geometry_msgs/Point target_position  # Position cible
float64 loiter_radius         # Rayon de loiter (m)

# Conditions de changement
string[] mode_requirements     # Conditions requises pour ce mode
string[] mode_restrictions     # Restrictions du mode actuel
bool can_arm                  # Armement possible
bool can_takeoff              # Décollage possible
bool can_land                 # Atterrissage possible

# Historique
string previous_mode
builtin_interfaces/Time mode_change_time
string mode_change_reason
