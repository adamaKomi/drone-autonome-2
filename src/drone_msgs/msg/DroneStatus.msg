# Drone status message
std_msgs/Header header

# System state
string state                    # DISCONNECTED, CONNECTED, ARMED, FLYING, LANDING, EMERGENCY
string mode                     # MANUAL, GUIDED, AUTO, RTL, LAND
bool armed
bool connected
bool guided

# Position and navigation
geometry_msgs/Point position
geometry_msgs/Vector3 velocity
float64 altitude
float64 home_distance

# Power and safety
float64 battery_voltage
float64 battery_percentage
bool gps_fix
int32 gps_satellites

# Safety alerts
bool critical_battery_warning
bool low_battery_warning
bool altitude_warning
bool geofence_violation

# Mission context
string mission_id
string current_task
float64 last_heartbeat
