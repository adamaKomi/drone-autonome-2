# Classification détaillée d'une fleur détectée
std_msgs/Header header

# Référence à la détection
uint32 detection_id
FlowerDetection source_detection

# Classification taxonomique
string species_name                    # Nom commun
string species_scientific_name        # Nom scientifique
string genus                          # Genre
string family                         # Famille
float64 species_confidence            # Confiance espèce 0.0-1.0

# Alternatives de classification
string[] alternative_species          # Autres espèces possibles
float64[] alternative_confidences     # Confidences correspondantes
string classification_method          # CNN, FEATURE_MATCHING, ENSEMBLE

# Caractéristiques morphologiques
uint32 petal_count                    # Nombre de pétales
string petal_shape                    # ROUND, OVAL, ELONGATED, IRREGULAR
string flower_symmetry                # RADIAL, BILATERAL, ASYMMETRIC
float64 diameter_mm                   # Diamètre en millimètres
float64 petal_length_mm              # Longueur pétale moyenne
float64 petal_width_mm               # Largeur pétale moyenne

# Analyse colorimétrique
string primary_color                  # Couleur dominante
string[] secondary_colors             # Couleurs secondaires
uint8[] color_histogram_hsv          # Histogramme HSV
uint8[] color_histogram_lab          # Histogramme LAB
float64 color_saturation             # Saturation moyenne
float64 color_brightness             # Luminosité moyenne

# État phénologique
string phenological_stage            # BUD, EARLY_BLOOM, FULL_BLOOM, LATE_BLOOM, SENESCENT
float64 maturity_percentage          # Pourcentage maturité 0-100
float64 freshness_score             # Score fraîcheur 0.0-1.0
string[] maturity_indicators        # Indicateurs utilisés

# État sanitaire
float64 health_score                 # Score santé global 0.0-1.0
string[] health_issues               # PEST_DAMAGE, DISEASE, DROUGHT, NUTRIENT, PHYSICAL
float64[] health_issue_severities    # Sévérité de chaque problème
bool shows_stress_signs             # Signes de stress
string[] stress_indicators          # Types de stress détectés

# Analyse reproductive
bool stamens_visible                 # Étamines visibles
bool pistil_visible                 # Pistil visible
bool nectar_guide_visible           # Guide nectarifère visible
float64 nectar_accessibility        # Accessibilité nectar 0.0-1.0
float64 pollen_availability         # Disponibilité pollen 0.0-1.0
string pollination_syndrome          # INSECT, WIND, BIRD, BAT, WATER

# Recommandations pollinisation
float64 pollination_priority         # Priorité 0.0-1.0
string pollination_recommendation    # IMMEDIATE, DELAYED, SKIP, MONITOR
string[] pollination_requirements    # Exigences spéciales
builtin_interfaces/Duration optimal_time_window # Fenêtre optimale

# Valeur écologique
float64 ecological_value            # Valeur écologique 0.0-1.0
string[] ecosystem_services         # Services écosystémiques
bool rare_species                   # Espèce rare
string conservation_status          # Statut conservation

# Métadonnées ML
string model_name
string model_version
builtin_interfaces/Time inference_time
float64 processing_duration_ms
float64 feature_extraction_time_ms
uint32 features_count               # Nombre caractéristiques extraites

# Qualité de la classification
float64 image_quality_for_classification # Qualité pour classification
float64 lighting_quality            # Qualité éclairage
float64 angle_quality              # Qualité angle vue
float64 resolution_quality         # Qualité résolution
string[] quality_limitations       # Limitations qualité
