# Mission status information
builtin_interfaces/Time stamp
string status                 # "IDLE", "RUNNING", "PAUSED", "COMPLETED", "FAILED"
string mode                  # "SEQUENTIAL", "LOOP", "BACKTRACK"
uint32 current_waypoint      # Index of current waypoint
uint32 total_waypoints       # Total waypoints in mission
float32 progress             # Overall progress (0.0 to 1.0)
string message               # Additional status message