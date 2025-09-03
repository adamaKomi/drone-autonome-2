# Pollination task message
std_msgs/Header header

# Task identification
string task_id
string flower_id

# Task parameters
geometry_msgs/Point target_position
string flower_color
string pollination_method       # HOVER, CONTACT, SPRAY
float64 approach_altitude
float64 pollination_duration

# Task status
string status                   # PENDING, APPROACHING, POLLINATING, COMPLETED, FAILED
float64 confidence_required     # Minimum detection confidence
bool force_execution            # Override safety checks

# Results
bool success
string result_message
float64 execution_time
float64 final_distance_to_target
