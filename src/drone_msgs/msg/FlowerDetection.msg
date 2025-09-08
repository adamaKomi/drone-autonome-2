# Détection de fleur par le système de vision
std_msgs/Header header

# Identification unique
uint32 detection_id
string detection_session_id
builtin_interfaces/Time detection_timestamp

# Position dans l'image
geometry_msgs/Point image_center       # Centre en pixels
geometry_msgs/Polygon bounding_box     # Boîte englobante
sensor_msgs/RegionOfInterest roi       # Région d'intérêt
uint8[] segmentation_mask             # Masque de segmentation

# Position 3D estimée
geometry_msgs/Point world_position     # Position dans le monde
float64 distance_estimate             # Distance estimée (m)
float64 position_confidence           # Confiance position 0.0-1.0
geometry_msgs/Vector3 position_uncertainty # Incertitude XYZ (m)

# Caractéristiques visuelles
string flower_color                   # Couleur dominante
string[] secondary_colors            # Couleurs secondaires
float64 estimated_diameter           # Diamètre estimé (cm)
float64 estimated_height            # Hauteur estimée (cm)
uint32 petal_count                  # Nombre de pétales estimé
float64 detection_confidence        # Confiance détection 0.0-1.0

# Analyse de maturité
string flower_stage                 # BUD, OPENING, FULL_BLOOM, WILTING, SEED
float64 maturity_score             # Score maturité 0.0-1.0
float64 health_score              # Score santé 0.0-1.0
bool nectar_visible               # Nectar visible
bool pollen_visible              # Pollen visible

# Qualité pour pollinisation
float64 pollination_suitability    # 0.0-1.0
string pollination_recommendation  # PROCEED, SKIP, WAIT, INVESTIGATE
float64 accessibility_score       # Facilité d'accès 0.0-1.0
string[] obstacles_detected       # Obstacles près de la fleur

# Métadonnées caméra
string camera_id
sensor_msgs/CameraInfo camera_info
geometry_msgs/Pose camera_pose
sensor_msgs/Image source_image
float64 image_quality_score       # Qualité image 0.0-1.0

# Traitement et algorithmes
string detection_algorithm        # YOLO, CASCADE, COLOR, HYBRID
string model_version
float64 processing_time_ms
float64 inference_time_ms

# Contexte environnemental
float64 lighting_conditions       # Score éclairage 0.0-1.0
float64 wind_speed_estimate      # Vitesse vent estimée
bool shadow_present              # Ombre détectée
string weather_conditions        # Conditions météo
