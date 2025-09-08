# Données environnementales complètes
std_msgs/Header header

# Conditions météorologiques
float64 temperature            # °C
float64 humidity              # % (0-100)
float64 pressure              # hPa
float64 wind_speed            # m/s
float64 wind_direction        # degrés (0-360)
float64 wind_gusts            # m/s rafales max
string weather_condition      # CLEAR, PARTLY_CLOUDY, CLOUDY, RAIN, STORM, FOG

# Visibilité et luminosité
float64 visibility_km         # km
float64 light_level_lux      # lux
float64 uv_index             # Index UV
string lighting_condition    # BRIGHT, NORMAL, DIM, DARK
bool shadows_present         # Ombres présentes

# Précipitations
bool precipitation_active    # Précipitations actives
float64 precipitation_rate   # mm/h
string precipitation_type    # RAIN, DRIZZLE, SNOW, HAIL
float64 precipitation_total_24h # mm dernières 24h

# Qualité de l'air
float64 air_quality_index    # Index qualité air
float64 pm25_concentration   # µg/m³ PM2.5
float64 pm10_concentration   # µg/m³ PM10
float64 ozone_concentration  # µg/m³ O3
float64 no2_concentration    # µg/m³ NO2

# Conditions magnétiques et électromagnétiques
float64 magnetic_declination # degrés
float64 magnetic_inclination # degrés
float64 magnetic_field_strength # nT
float64 electromagnetic_interference # dB niveau interférence

# Propriétés atmosphériques
float64 density_altitude     # m altitude densité
float64 dew_point           # °C point rosée
float64 cloud_ceiling       # m plafond nuageux
float64 atmospheric_pressure_trend # hPa/h tendance

# Conditions de vol
string flight_conditions    # VFR, MVFR, IFR, LIFR
bool turbulence_expected    # Turbulences attendues
float64 turbulence_intensity # 0.0-1.0 intensité
string[] flying_restrictions # Restrictions vol

# Géophysique locale
float64 terrain_elevation   # m élévation terrain
string terrain_type         # FLAT, ROLLING, HILLY, MOUNTAINOUS
float64 terrain_roughness   # 0.0-1.0 rugosité
geometry_msgs/Vector3 terrain_slope # Pente terrain

# Végétation et écosystème
float64 vegetation_density  # 0.0-1.0 densité
string vegetation_state     # DORMANT, GROWING, FLOWERING, FRUITING
float64 pollen_concentration # grains/m³
string[] active_flora_species # Espèces actives

# Faune
bool wildlife_activity_high # Activité faune élevée
string[] wildlife_species_active # Espèces actives
string[] wildlife_warnings  # Alertes faune
bool bird_migration_active  # Migration oiseaux active

# Pollution et contaminants
float64 noise_level_db      # dB niveau sonore
string[] pollution_sources  # Sources pollution
float64 dust_concentration  # µg/m³ poussière
string[] chemical_hazards   # Dangers chimiques

# Activité humaine
string human_activity_level # LOW, MODERATE, HIGH, VERY_HIGH
bool agricultural_activity  # Activité agricole
bool construction_nearby    # Construction proche
string[] human_restrictions # Restrictions dues activité humaine

# Conditions de communication
float64 signal_strength_gps # dB force signal GPS
float64 signal_strength_radio # dB force signal radio
bool communication_interference # Interférences communication
string[] communication_issues # Problèmes communication

# Historique et tendances
float64[] temperature_trend_1h # Tendance température 1h
float64[] wind_trend_1h       # Tendance vent 1h
float64[] pressure_trend_3h   # Tendance pression 3h
string weather_forecast_6h    # Prévisions 6h

# Alertes et warnings
string[] active_weather_warnings # Alertes météo actives
string[] environmental_warnings # Alertes environnementales
bool extreme_conditions     # Conditions extrêmes
string[] safety_recommendations # Recommandations sécurité

# Sources et qualité données
string[] data_sources       # Sources données
builtin_interfaces/Time last_update
float64 data_reliability    # 0.0-1.0 fiabilité
string[] data_limitations   # Limitations données

# Impact sur opérations drone
float64 flight_safety_score # 0.0-1.0 score sécurité vol
string[] operational_impacts # Impacts opérationnels
string recommended_action   # PROCEED, CAUTION, DELAY, ABORT
float64 mission_viability   # 0.0-1.0 viabilité mission
