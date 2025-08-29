# Response message for command execution
# Used to report command execution results

# Response information
string command_id             # Corresponding command ID
string command_type           # Type of command that was executed
bool success                  # Command execution success
string message               # Result message or error description
float32 execution_time       # Time taken to execute command in seconds

# Result data (optional)
geometry_msgs/Point final_position     # Final position after command
string final_mode            # Final flight mode after command
bool final_armed_state       # Final armed state after command

# Error information
int32 error_code             # Error code (0 = no error)
string error_details         # Detailed error information

# Timestamp
builtin_interfaces/Time timestamp  # Response timestamp
