# PathProgress
builtin_interfaces/Time stamp
float64 progress          # 0.0 à 1.0 (pourcentage)
float64 distance_remaining # distance restante en mètres
geometry_msgs/Point current_position
geometry_msgs/Point target_position
string status             # "NAVIGATING", "REACHED", "FAILED"
