# Message pour l'état d'une mission

string mission_id
string mission_name
string state          # IDLE, LOADING, PLANNING, READY, EXECUTING, PAUSED, COMPLETED, FAILED, ABORTED
int32 current_task_index
int32 total_tasks
string current_task_type
string current_task_status
float64 progress_percentage
float64 elapsed_time
