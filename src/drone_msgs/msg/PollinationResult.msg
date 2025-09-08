# Résultat détaillé d'une opération de pollinisation
std_msgs/Header header

# Identification
uint32 result_id
uint32 target_id                     # ID de la cible traitée
string mission_id
builtin_interfaces/Time operation_start
builtin_interfaces/Time operation_end
builtin_interfaces/Duration total_duration

# Statut final
string OUTCOME_SUCCESS = "SUCCESS"
string OUTCOME_PARTIAL = "PARTIAL"  
string OUTCOME_FAILED = "FAILED"
string OUTCOME_ABORTED = "ABORTED"
string OUTCOME_SKIPPED = "SKIPPED"

string outcome
float64 success_confidence           # Confiance dans le succès 0.0-1.0
string[] success_indicators         # Indicateurs de succès observés
string[] failure_reasons            # Raisons d'échec si applicable

# Métriques d'exécution
float64 approach_accuracy           # Précision approche (m)
float64 position_stability          # Stabilité position durant opération
float64 operation_efficiency        # Efficacité opération 0.0-1.0
uint32 approach_attempts            # Nombre tentatives approche
builtin_interfaces/Duration actual_hover_time

# Contact et interaction
bool physical_contact_achieved      # Contact physique réalisé
float64 contact_duration           # Durée contact (s)
float64 contact_force              # Force contact estimée (N)
geometry_msgs/Vector3 contact_point # Point de contact
string contact_quality             # EXCELLENT, GOOD, FAIR, POOR

# Transfert de pollen
bool pollen_transfer_detected      # Transfert pollen détecté
float64 pollen_transfer_confidence # Confiance transfert 0.0-1.0
string pollen_transfer_method      # DIRECT, VIBRATION, BRUSH, SPRAY
float64 estimated_pollen_amount    # Quantité pollen estimée

# État fleur après pollinisation
FlowerClassification flower_state_after
bool flower_damage_detected        # Dommage fleur détecté
string[] damage_types              # Types de dommages
float64 flower_health_after        # Santé fleur après 0.0-1.0

# Données collectées
sensor_msgs/Image[] captured_images # Images capturées
sensor_msgs/PointCloud2[] point_clouds # Nuages de points
string[] sensor_data_files         # Fichiers données capteurs
uint32 scientific_samples_collected # Échantillons scientifiques

# Conditions environnementales
float64 wind_speed_during          # Vitesse vent durant opération
float64 temperature_during         # Température durant opération
float64 humidity_during           # Humidité durant opération
string lighting_conditions        # Conditions éclairage
string[] environmental_factors     # Facteurs environnementaux

# Performance drone
float64 battery_consumed           # Batterie consommée (%)
float64 energy_consumed           # Énergie consommée (Wh)
float64 positioning_accuracy       # Précision positionnement
string[] system_warnings          # Warnings système
string[] performance_issues        # Problèmes performance

# Analyse post-opération
string[] lessons_learned           # Leçons apprises
string[] recommendations          # Recommandations futures
float64 operation_difficulty      # Difficulté opération 0.0-1.0
string[] improvement_suggestions   # Suggestions amélioration

# Validation et vérification
bool post_verification_performed   # Vérification post effectuée
string verification_method        # Méthode vérification
float64 verification_confidence   # Confiance vérification 0.0-1.0
builtin_interfaces/Time verification_time

# Impact écologique estimé
float64 ecological_benefit         # Bénéfice écologique estimé 0.0-1.0
string[] ecosystem_contributions   # Contributions écosystème
bool biodiversity_enhanced        # Biodiversité améliorée

# Qualité données
float64 data_quality_score        # Qualité données collectées 0.0-1.0
bool data_scientifically_valuable # Données scientifiquement valables
string[] data_limitations         # Limitations données

# Métadonnées
string operator_notes
string automated_analysis_summary
builtin_interfaces/Time analysis_timestamp
