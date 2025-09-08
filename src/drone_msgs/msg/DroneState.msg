# État détaillé du système drone
std_msgs/Header header

# Identification système
string system_id
string firmware_version
string hardware_version
builtin_interfaces/Time boot_time
builtin_interfaces/Duration uptime

# État autopilot
bool autopilot_connected
string autopilot_mode
uint8 autopilot_state          # 0=UNINIT, 1=BOOT, 2=CALIBRATING, 3=STANDBY, 4=ACTIVE, 5=CRITICAL, 6=EMERGENCY
float64 cpu_usage              # Pourcentage 0-100
float64 memory_usage           # Pourcentage 0-100
float64 temperature            # Degrés Celsius

# Communication
bool rc_connected
float64 rc_signal_strength     # dBm
bool telemetry_connected
float64 telemetry_signal_strength
uint32 total_packets_sent
uint32 total_packets_received
uint32 packets_lost

# Capteurs
bool gps_available
uint8 gps_satellites_visible
float64 gps_hdop
bool imu_healthy
bool barometer_healthy
bool magnetometer_healthy
bool camera_available

# Actuateurs
bool motors_armed
float64[] motor_outputs        # PWM 1000-2000
bool servo_healthy
float64[] servo_positions      # Degrés

# Stockage et logs
float64 storage_used_mb
float64 storage_available_mb
uint32 log_files_count
builtin_interfaces/Time last_log_entry
