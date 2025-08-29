# Command message for drone control
# Used to send commands from other nodes to the interface node

# Command types
string ARM = "ARM"
string DISARM = "DISARM"
string TAKEOFF = "TAKEOFF"
string LAND = "LAND"
string SET_MODE = "SET_MODE"
string EMERGENCY = "EMERGENCY"
string HOLD_POSITION = "HOLD_POSITION"

# Command information
string command_type            # Type of command (ARM, DISARM, etc.)
string command_id             # Unique identifier for command tracking
int32 priority                # Command priority (0=lowest, 10=highest)

# Command parameters
string target_mode            # Target flight mode (for SET_MODE)
float32 target_altitude       # Target altitude for takeoff/landing
geometry_msgs/Point target_position    # Target position
geometry_msgs/Quaternion target_orientation  # Target orientation
float32 target_speed          # Target speed
bool force_command            # Force command execution (ignore safety checks)

# Optional parameters
string[] parameters_names     # Parameter names
string[] parameters_values    # Parameter values

# Safety overrides
bool emergency_override       # Emergency override flag
bool safety_override         # Safety check override flag

# Timeout
float32 timeout              # Command timeout in seconds (0 = no timeout)

# Timestamp
builtin_interfaces/Time timestamp  # Command timestamp
