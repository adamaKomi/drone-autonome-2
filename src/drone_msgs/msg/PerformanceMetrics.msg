# Métriques de performance système complètes
std_msgs/Header header

# Performance processeur
float64 cpu_usage_percent    # % utilisation CPU
float64 cpu_temperature      # °C température CPU
float64 cpu_frequency_mhz    # MHz fréquence CPU
float64[] cpu_core_usage     # % utilisation par cœur
float64 cpu_load_average_1m  # Charge moyenne 1min
float64 cpu_load_average_5m  # Charge moyenne 5min
float64 cpu_throttling_percent # % limitation performance

# Performance mémoire
float64 memory_usage_percent # % utilisation mémoire
uint64 memory_total_mb      # MB mémoire totale
uint64 memory_used_mb       # MB mémoire utilisée
uint64 memory_available_mb  # MB mémoire disponible
uint64 memory_cache_mb      # MB cache mémoire
uint64 memory_swap_used_mb  # MB swap utilisé
uint32 memory_allocation_failures # Échecs allocation

# Performance stockage
float64 storage_usage_percent # % utilisation stockage
uint64 storage_total_gb     # GB stockage total
uint64 storage_used_gb      # GB stockage utilisé
uint64 storage_available_gb # GB stockage disponible
float64 storage_read_speed_mbps # MB/s vitesse lecture
float64 storage_write_speed_mbps # MB/s vitesse écriture
uint32 storage_io_errors    # Erreurs I/O stockage
float64 storage_temperature # °C température stockage

# Performance réseau
float64 network_upload_mbps # MB/s débit montant
float64 network_download_mbps # MB/s débit descendant
uint32 network_latency_ms   # ms latence réseau
uint32 network_packet_loss_percent # % perte paquets
uint64 network_bytes_sent   # Octets envoyés
uint64 network_bytes_received # Octets reçus
uint32 network_errors       # Erreurs réseau
string[] active_connections # Connexions actives

# Performance GPU (si applicable)
float64 gpu_usage_percent   # % utilisation GPU
float64 gpu_memory_usage_percent # % mémoire GPU
float64 gpu_temperature     # °C température GPU
float64 gpu_frequency_mhz   # MHz fréquence GPU
float64 gpu_power_watts     # W consommation GPU

# Performance capteurs
float64 imu_update_rate_hz  # Hz taux mise à jour IMU
float64 gps_update_rate_hz  # Hz taux mise à jour GPS
float64 camera_fps          # FPS caméra
float64 lidar_scan_rate_hz  # Hz taux scan LiDAR
uint32 sensor_data_drops    # Pertes données capteur
float64 sensor_latency_ms   # ms latence capteurs

# Performance communication
float64 telemetry_rate_hz   # Hz taux télémétrie
float64 command_latency_ms  # ms latence commandes
uint32 communication_drops  # Pertes communication
float64 data_throughput_kbps # kbps débit données
uint32 protocol_errors      # Erreurs protocole
float64 signal_quality_percent # % qualité signal

# Performance navigation
float64 navigation_update_rate_hz # Hz taux navigation
float64 path_planning_time_ms # ms temps planification
float64 obstacle_detection_rate_hz # Hz détection obstacles
float64 localization_accuracy_m # m précision localisation
uint32 navigation_failures  # Échecs navigation
float64 trajectory_deviation_m # m déviation trajectoire

# Performance mission
float64 mission_progress_percent # % progression mission
float64 task_completion_rate # Hz taux achèvement tâches
float64 mission_efficiency  # 0.0-1.0 efficacité mission
uint32 mission_retries      # Tentatives mission
float64 waypoint_accuracy_m # m précision waypoints
builtin_interfaces/Duration mission_duration # Durée mission

# Performance énergétique
float64 power_consumption_watts # W consommation totale
float64 power_efficiency    # tâches/watt efficacité
float64 battery_drain_rate_percent_per_hour # %/h décharge
float64 energy_per_task_wh  # Wh énergie par tâche
float64 thermal_efficiency # 0.0-1.0 efficacité thermique

# Performance temps réel
float64 real_time_factor    # Facteur temps réel
uint32 missed_deadlines     # Échéances manquées
float64 jitter_ms          # ms gigue temporelle
float64 worst_case_latency_ms # ms latence pire cas
uint32 timing_violations    # Violations timing
float64 determinism_score   # 0.0-1.0 score déterminisme

# Qualité de service
float64 availability_percent # % disponibilité système
float64 reliability_score   # 0.0-1.0 score fiabilité
uint32 error_count_total    # Nombre erreurs total
uint32 warning_count_total  # Nombre avertissements
float64 mtbf_hours         # h temps moyen entre pannes
float64 recovery_time_s    # s temps récupération

# Métriques application
float64[] task_execution_times_ms # ms temps exécution tâches
float64[] queue_lengths     # Longueurs files attente
uint32[] thread_counts      # Nombres threads par processus
float64[] response_times_ms # ms temps réponse
uint32 active_processes     # Processus actifs
float64 context_switches_per_sec # Changements contexte/s

# Métriques prédictives
float64 predicted_failure_probability # Probabilité panne
builtin_interfaces/Duration predicted_maintenance_time # Temps maintenance prévu
float64 performance_degradation_rate # Taux dégradation
float64[] trending_metrics  # Métriques tendance
string[] performance_warnings # Avertissements performance

# Benchmarks et comparaisons
float64 performance_vs_baseline_percent # % vs baseline
float64 efficiency_vs_optimal_percent # % vs optimal
string[] performance_anomalies # Anomalies détectées
float64 overall_health_score # 0.0-1.0 score santé global

# Métadonnées
builtin_interfaces/Time measurement_start_time
builtin_interfaces/Duration measurement_period
string measurement_method   # Méthode mesure
float64 measurement_accuracy # Précision mesures
string[] data_quality_flags # Indicateurs qualité données
