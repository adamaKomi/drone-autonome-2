# Progression détaillée d'une mission
std_msgs/Header header

# Référence mission
string mission_id
builtin_interfaces/Time progress_timestamp

# Progression globale
float64 overall_progress_percentage    # 0.0-100.0
uint32 phase_current                  # Phase actuelle
uint32 phase_total                   # Nombre total de phases
string current_phase_name
string current_activity

# Progression par composants
float64 navigation_progress          # % navigation
float64 detection_progress          # % détection
float64 pollination_progress        # % pollinisation
float64 data_collection_progress    # % collecte données

# Waypoints et objectifs
uint32 waypoint_current_index
uint32 waypoints_completed
uint32 waypoints_remaining
float64 waypoint_completion_rate    # waypoints/heure

# Détections et pollinisations
uint32 flowers_detected_total
uint32 flowers_detected_session    # Cette session
uint32 pollinations_attempted
uint32 pollinations_successful
float64 pollination_success_rate   # %

# Métriques temporelles
builtin_interfaces/Duration elapsed_time
builtin_interfaces/Duration estimated_remaining_time
builtin_interfaces/Duration time_at_current_waypoint
float64 average_time_per_waypoint

# Métriques spatiales
float64 distance_traveled          # mètres
float64 distance_remaining         # mètres
float64 current_altitude          # mètres
geometry_msgs/Point current_position

# Efficacité opérationnelle
float64 time_efficiency           # % temps utilisé efficacement
float64 energy_efficiency         # km/Wh ou similaire
float64 detection_efficiency      # détections/km
float64 pollination_efficiency    # pollinisations/heure

# Ressources consommées
float64 battery_percentage_used
float64 energy_consumed_wh
float64 storage_used_mb
float64 bandwidth_used_kb

# Conditions actuelles
float64 wind_speed_current        # m/s
float64 temperature_current       # °C
string weather_current
float64 light_level_current       # lux ou %

# Qualité en temps réel
float64 positioning_accuracy      # mètres
float64 detection_confidence_avg  # moyenne confidences
float64 operation_stability       # stabilité opérations
uint32 system_warnings_count

# Adaptations et optimisations
uint32 route_optimizations_made
uint32 parameter_adjustments_made
string[] adaptive_changes_applied
float64 improvement_factor        # amélioration par adaptations

# Prédictions mise à jour
float64 completion_probability_updated
builtin_interfaces/Duration eta_updated
string[] risk_factors_current
string[] optimization_opportunities

# Événements récents
string[] recent_events            # 5 derniers événements
builtin_interfaces/Time[] event_times
string[] recent_achievements
string[] recent_issues

# Comparaison avec plan
float64 adherence_to_plan        # % respect plan original
builtin_interfaces/Duration schedule_variance # écart horaire
float64 route_variance           # écart route planifiée
string[] deviations_from_plan

# Feedback pour optimisation
string[] performance_insights
string[] suggested_improvements
float64 learning_factor          # facteur apprentissage 0.0-1.0
