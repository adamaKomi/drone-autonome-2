# Commande de navigation pour le contrôleur
std_msgs/Header header

# Type de commande
uint8 COMMAND_IDLE = 0
uint8 COMMAND_POSITION = 1
uint8 COMMAND_VELOCITY = 2
uint8 COMMAND_ATTITUDE = 3
uint8 COMMAND_TRAJECTORY = 4
uint8 COMMAND_LAND = 5
uint8 COMMAND_TAKEOFF = 6
uint8 COMMAND_RTH = 7

uint8 command_type

# Cibles de position
geometry_msgs/Point target_position
geometry_msgs/Quaternion target_orientation
float64 target_yaw             # radians

# Cibles de vélocité
geometry_msgs/Vector3 target_linear_velocity
geometry_msgs/Vector3 target_angular_velocity

# Paramètres de contrôle
float64 position_tolerance     # mètres
float64 velocity_tolerance     # m/s
float64 yaw_tolerance         # radians
builtin_interfaces/Duration timeout

# Contraintes dynamiques
float64 max_speed             # m/s
float64 max_acceleration      # m/s²
float64 max_jerk             # m/s³
float64 max_yaw_rate         # rad/s

# Mode de navigation
string navigation_mode        # DIRECT, SMOOTH, PRECISION, AGGRESSIVE
string coordinate_frame       # LOCAL, GLOBAL, BODY

# Priorités et flags
uint8 priority               # 0=LOW, 1=NORMAL, 2=HIGH, 3=CRITICAL
bool force_execution         # Forcer malgré warnings
bool abort_on_obstacle       # Abandonner si obstacle
bool maintain_altitude       # Maintenir altitude actuelle

# Identification
uint32 command_id
string issuer                # Module émetteur de commande
builtin_interfaces/Time timestamp
builtin_interfaces/Time expiry_time
