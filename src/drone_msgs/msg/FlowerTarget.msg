# Cible fleur complète pour pollinisation
std_msgs/Header header

# Identification
uint32 target_id
string target_name
uint32 priority                       # 0=LOW, 1=NORMAL, 2=HIGH, 3=CRITICAL

# Données de base
FlowerDetection detection
FlowerClassification classification

# Position et accessibilité
geometry_msgs/Point target_position  # Position 3D cible
geometry_msgs/Point approach_position # Position d'approche
geometry_msgs/Vector3 approach_vector # Vecteur d'approche optimal
float64 approach_angle               # Angle d'approche optimal (radians)
float64 approach_distance            # Distance d'approche (m)

# Obstacles et contraintes
geometry_msgs/Point[] obstacles_nearby # Obstacles proches
float64 clearance_radius             # Rayon de dégagement requis (m)
float64 wind_factor                 # Facteur vent (impact sur approche)
string[] environmental_constraints   # Contraintes environnementales

# Stratégie de pollinisation
string pollination_method           # CONTACT, PROXIMITY, VIBRATION, SPRAYING
float64 optimal_hover_time          # Temps survol optimal (s)
float64 optimal_height              # Hauteur optimale (m)
geometry_msgs/Vector3 optimal_velocity # Vélocité optimale lors contact
string[] required_tools             # Outils requis
string[] procedure_steps            # Étapes procédure

# Timing et fenêtres
builtin_interfaces/Time earliest_approach # Plus tôt pour approche
builtin_interfaces/Time latest_approach   # Plus tard pour approche
builtin_interfaces/Duration optimal_duration # Durée optimale
string[] timing_constraints         # Contraintes temporelles
bool time_sensitive                 # Sensible au timing

# Qualité et potentiel
float64 pollination_potential       # Potentiel pollinisation 0.0-1.0
float64 success_probability         # Probabilité succès 0.0-1.0
float64 risk_assessment            # Évaluation risque 0.0-1.0
string[] risk_factors              # Facteurs de risque
string[] success_factors           # Facteurs de succès

# Historique et statut
string target_status               # IDENTIFIED, PLANNED, APPROACHING, POLLINATED, FAILED, SKIPPED
uint32 previous_attempts          # Tentatives précédentes
builtin_interfaces/Time last_attempt_time
string[] previous_outcomes        # Résultats tentatives précédentes
string[] lessons_learned         # Leçons apprises

# Données scientifiques
bool collect_scientific_data      # Collecter données scientifiques
string[] data_collection_requirements # Exigences collecte
sensor_msgs/Image[] reference_images # Images de référence
string[] measurement_requirements  # Mesures requises

# Contexte mission
string mission_id
string mission_phase              # SURVEY, SYSTEMATIC, TARGETED, VERIFICATION
uint32 mission_sequence_number
float64 mission_importance        # Importance dans mission 0.0-1.0

# Métadonnées
builtin_interfaces/Time created_time
builtin_interfaces/Time updated_time
string created_by
string updated_by
string notes
