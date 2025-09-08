# Vélocité 3D complète avec dérivées
std_msgs/Header header

# Vélocité linéaire
geometry_msgs/Vector3 linear   # m/s dans référentiel local
geometry_msgs/Vector3 body     # m/s dans référentiel body du drone

# Vélocité angulaire
geometry_msgs/Vector3 angular  # rad/s dans référentiel body

# Accélération
geometry_msgs/Vector3 linear_acceleration  # m/s² 
geometry_msgs/Vector3 angular_acceleration # rad/s²

# Vitesses dérivées
float64 ground_speed           # m/s vitesse par rapport au sol
float64 air_speed             # m/s vitesse par rapport à l'air
float64 vertical_speed        # m/s vitesse verticale (+ = montée)
float64 track_angle           # radians direction du mouvement

# Limites et contraintes
float64 max_horizontal_speed   # m/s limite horizontale
float64 max_vertical_speed     # m/s limite verticale
float64 max_angular_rate       # rad/s limite rotation

# Incertitudes
float64 linear_accuracy        # m/s incertitude vitesse linéaire
float64 angular_accuracy       # rad/s incertitude vitesse angulaire

# Source et qualité
string velocity_source         # GPS, IMU, OPTICAL_FLOW, LIDAR
float64 velocity_confidence    # 0.0-1.0
bool velocity_valid
builtin_interfaces/Time last_update

# Vent estimé
geometry_msgs/Vector3 wind_velocity # m/s vent estimé
float64 wind_speed            # m/s magnitude vent
float64 wind_direction        # radians direction vent
