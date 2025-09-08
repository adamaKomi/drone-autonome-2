# Position 3D précise avec incertitudes
std_msgs/Header header

# Position géographique (WGS84)
float64 latitude               # Degrés
float64 longitude              # Degrés  
float64 altitude               # Mètres au-dessus du niveau de la mer
float64 altitude_relative      # Mètres au-dessus du point de départ

# Position dans le référentiel local
geometry_msgs/Point local_position

# Incertitudes et précision
float64 horizontal_accuracy    # Mètres (rayon 95% confiance)
float64 vertical_accuracy      # Mètres (95% confiance)
float64 speed_accuracy         # m/s (95% confiance)

# Source de positioning
string position_source         # GPS, RTK, VIO, SLAM, MOCAP
uint8 fix_type                # 0=NONE, 1=2D, 2=3D, 3=DGPS, 4=RTK_FLOAT, 5=RTK_FIXED
uint8 satellites_used         # Nombre de satellites utilisés
float64 hdop                  # Dilution horizontale précision
float64 vdop                  # Dilution verticale précision

# Dérivées
geometry_msgs/Vector3 velocity # m/s dans référentiel local
geometry_msgs/Vector3 acceleration # m/s² dans référentiel local
float64 ground_speed          # m/s vitesse sol
float64 track_angle           # Degrés direction mouvement

# Référentiels
string coordinate_frame       # "wgs84", "local_enu", "local_ned"
geometry_msgs/Point origin    # Origine référentiel local
geometry_msgs/Quaternion frame_rotation # Rotation référentiel

# Qualité et confiance
float64 position_confidence   # 0.0-1.0
bool position_valid
builtin_interfaces/Time last_update
builtin_interfaces/Duration time_since_fix
