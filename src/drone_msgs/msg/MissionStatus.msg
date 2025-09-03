# Mission status message
std_msgs/Header header

# Mission identification
string mission_id
string mission_name
string mission_description

# Execution state
string state                    # READY, EXECUTING, PAUSED, COMPLETED, FAILED, ABORTED
float64 progress_percentage     # 0.0 to 100.0
int32 current_task_index
int32 total_tasks

# Current task
string current_task_id
string current_task_type
string current_task_description

# Mission metrics
float64 start_time
float64 elapsed_time
float64 estimated_remaining_time

# Results
int32 tasks_completed
int32 tasks_failed
int32 flowers_pollinated
float64 total_distance_traveled
