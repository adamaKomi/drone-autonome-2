# Orientation 3D avec représentations multiples
std_msgs/Header header

# Quaternion (représentation principale)
geometry_msgs/Quaternion quaternion

# Angles d'Euler (pour lisibilité)
float64 roll                   # Roulis en radians
float64 pitch                  # Tangage en radians  
float64 yaw                    # Lacet en radians

# Angles en degrés (pour affichage)
float64 roll_degrees
float64 pitch_degrees
float64 yaw_degrees

# Vitesses angulaires
geometry_msgs/Vector3 angular_velocity # rad/s dans référentiel body
geometry_msgs/Vector3 angular_acceleration # rad/s² dans référentiel body

# Incertitudes
float64 roll_uncertainty       # radians (écart-type)
float64 pitch_uncertainty      # radians (écart-type)
float64 yaw_uncertainty        # radians (écart-type)

# Source d'orientation
string orientation_source      # IMU, AHRS, MAG, VIO, SLAM
bool gyroscope_healthy
bool accelerometer_healthy
bool magnetometer_healthy
float64 magnetic_declination   # radians

# Qualité
float64 orientation_confidence # 0.0-1.0
bool orientation_valid
builtin_interfaces/Time last_update

# Référentiel
string reference_frame         # "ned", "enu", "body"
