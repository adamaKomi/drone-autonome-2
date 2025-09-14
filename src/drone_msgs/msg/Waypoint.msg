# Waypoint definition for autonomous navigation
geometry_msgs/Point position  # Position (x,y,z or lat/lon/alt)
float32 tolerance             # Position tolerance for waypoint completion
string wp_type                # Waypoint type: "NORMAL", "HOLD", "TAKEOFF", "LAND"
string[] actions              # Actions to perform at waypoint
float32 speed                 # Recommended speed to approach waypoint
float32 yaw                   # Desired yaw orientation
float32 hold_time             # Time to hold at waypoint (for HOLD type)