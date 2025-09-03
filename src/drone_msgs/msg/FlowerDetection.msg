# Flower detection message
std_msgs/Header header

# Flower identification
string id
string color                    # red, yellow, pink, orange
float64 confidence              # 0.0 to 1.0

# Image coordinates (pixels)
float64 center_x
float64 center_y
float64 radius

# World coordinates (if calculated)
geometry_msgs/Point world_position
bool world_position_valid

# Detection metadata
float64 timestamp
string camera_frame_id
int32 image_width
int32 image_height
