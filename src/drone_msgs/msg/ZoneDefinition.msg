# Définition de zone géographique avec contraintes
std_msgs/Header header

# Identification
string zone_id
string zone_name
string zone_type                 # INCLUSION, EXCLUSION, RESTRICTED, SURVEY, LANDING

# Géométrie
geometry_msgs/Polygon boundary  # Frontière principale
geometry_msgs/Polygon[] holes   # Trous dans la zone
geometry_msgs/Point[] vertices  # Vertices en coordonnées GPS
float64 area_square_meters      # Superficie (m²)
float64 perimeter_meters        # Périmètre (m)

# Altitude et 3D
float64 min_altitude            # Altitude minimale (m)
float64 max_altitude            # Altitude maximale (m)
float64 default_altitude        # Altitude par défaut (m)
bool altitude_restrictions_apply # Restrictions altitude s'appliquent

# Permissions et restrictions
bool flight_allowed             # Vol autorisé
bool landing_allowed            # Atterrissage autorisé
bool takeoff_allowed           # Décollage autorisé
bool autonomous_allowed        # Mode autonome autorisé
string[] restricted_activities  # Activités restreintes
string[] required_permissions   # Permissions requises

# Contraintes opérationnelles
float64 max_speed_ms           # Vitesse max autorisée (m/s)
float64 max_noise_level_db     # Niveau bruit max (dB)
uint32 max_drones_simultaneous # Drones max simultanés
builtin_interfaces/Duration max_flight_time # Temps vol max

# Contraintes temporelles
builtin_interfaces/Time[] allowed_time_windows_start
builtin_interfaces/Time[] allowed_time_windows_end
string[] time_restrictions      # Restrictions temporelles
bool seasonal_restrictions     # Restrictions saisonnières
string[] seasonal_periods      # Périodes saisonnières

# Propriétés environnementales
string terrain_type            # FLAT, HILLY, MOUNTAINOUS, WATER, FOREST
float64 vegetation_density     # Densité végétation 0.0-1.0
string[] vegetation_types      # Types végétation
bool water_present            # Eau présente
string[] environmental_hazards # Dangers environnementaux

# Propriétés biologiques
string[] flora_species         # Espèces floristiques
string[] fauna_species         # Espèces faunistiques
string conservation_status     # Statut conservation
bool protected_area           # Zone protégée
string[] ecological_constraints # Contraintes écologiques

# Infrastructure et obstacles
geometry_msgs/Point[] permanent_obstacles # Obstacles permanents
float64[] obstacle_heights     # Hauteurs obstacles
string[] infrastructure_types  # Types infrastructure
geometry_msgs/Point[] safe_landing_spots # Zones atterrissage sûres

# Communication et navigation
bool gps_reliable             # GPS fiable
bool communication_coverage   # Couverture communication
string[] navigation_aids      # Aides navigation
float64 magnetic_declination  # Déclinaison magnétique

# Conditions météorologiques typiques
float64 typical_wind_speed    # Vitesse vent typique (m/s)
string prevailing_wind_direction # Direction vent dominante
float64 typical_temperature_range_min # Temp min typique (°C)
float64 typical_temperature_range_max # Temp max typique (°C)
string[] weather_patterns     # Patterns météo

# Sécurité et urgence
geometry_msgs/Point[] emergency_landing_zones # Zones atterrissage urgence
string[] emergency_contacts   # Contacts urgence
string[] safety_procedures    # Procédures sécurité
float64 emergency_response_time # Temps réponse urgence (min)

# Réglementation
string regulatory_authority   # Autorité réglementaire
string[] applicable_regulations # Règlements applicables
string authorization_required # Autorisation requise
string[] compliance_requirements # Exigences conformité

# Métadonnées
string created_by
builtin_interfaces/Time created_time
builtin_interfaces/Time last_updated
string data_source            # Source données
float64 data_accuracy_meters  # Précision données (m)
string[] validation_status    # Statut validation
