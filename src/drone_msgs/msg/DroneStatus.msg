# État général du drone de pollinisation
std_msgs/Header header

# État général
bool is_armed
bool is_flying
bool is_connected
bool is_healthy

# Mode de vol
string flight_mode              # MANUAL, AUTO, GUIDED, RTL, LAND, etc.
string mission_state           # IDLE, PLANNING, EXECUTING, PAUSED, COMPLETED

# Position et orientation
geometry_msgs/Point position
geometry_msgs/Quaternion orientation
geometry_msgs/Twist velocity

# Systèmes
BatteryStatus battery
SafetyStatus safety
PerformanceMetrics performance

# Statut modules
bool navigation_active
bool vision_active
bool mission_active
bool communication_active

# Alertes et erreurs
SystemAlert[] active_alerts
string[] error_messages
uint8 severity_level           # 0=INFO, 1=WARNING, 2=ERROR, 3=CRITICAL
