# Message for drone status information
# Used for communication between nodes

# Basic state information
string state                    # Current drone state (UNKNOWN, DISCONNECTED, CONNECTED, ARMED, FLYING, LANDING, EMERGENCY)
string mode                     # Current flight mode
bool armed                      # Armed status
bool connected                  # Connection status to flight controller
bool guided                     # Guided mode status

# Position and motion
geometry_msgs/Point position    # Current position (x, y, z) in local frame
geometry_msgs/Vector3 velocity  # Current velocity (vx, vy, vz)
geometry_msgs/Vector3 angular_velocity  # Angular velocity

# Orientation
geometry_msgs/Quaternion orientation  # Current orientation

# GPS information
bool gps_fix                    # GPS fix status
int32 gps_satellites           # Number of GPS satellites
geometry_msgs/Point gps_position  # GPS coordinates (lat, lon, alt)

# Battery information
float32 battery_voltage         # Battery voltage in volts
float32 battery_percentage      # Battery percentage (0-100)
float32 battery_current         # Battery current in amperes

# Sensor information
float32 altitude_barometric     # Barometric altitude
float32 altitude_relative       # Relative altitude from home
float32 home_distance          # Distance from home position

# Safety information
bool geofence_violation        # Geofence violation status
bool low_battery_warning       # Low battery warning
bool critical_battery_warning  # Critical battery warning
bool altitude_warning          # Altitude limit warning

# Timestamps
builtin_interfaces/Time last_heartbeat    # Last heartbeat from flight controller
builtin_interfaces/Time last_update       # Last status update
