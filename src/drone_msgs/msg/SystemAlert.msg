# Alerte système complète
std_msgs/Header header

# Identification alerte
string alert_id             # ID unique alerte
string alert_type           # Type alerte principal
string alert_category       # SAFETY, OPERATIONAL, MAINTENANCE, COMMUNICATION, ENVIRONMENTAL
uint8 priority_level        # 1=LOW, 2=MEDIUM, 3=HIGH, 4=CRITICAL, 5=EMERGENCY

# Niveaux de priorité constants
uint8 PRIORITY_LOW = 1
uint8 PRIORITY_MEDIUM = 2
uint8 PRIORITY_HIGH = 3
uint8 PRIORITY_CRITICAL = 4
uint8 PRIORITY_EMERGENCY = 5

# Catégories d'alertes constants
string CATEGORY_SAFETY = "SAFETY"
string CATEGORY_OPERATIONAL = "OPERATIONAL"
string CATEGORY_MAINTENANCE = "MAINTENANCE"
string CATEGORY_COMMUNICATION = "COMMUNICATION"
string CATEGORY_ENVIRONMENTAL = "ENVIRONMENTAL"

# Statut et état
string alert_status         # ACTIVE, ACKNOWLEDGED, RESOLVED, ESCALATED, SUPPRESSED
builtin_interfaces/Time alert_time # Temps déclenchement
builtin_interfaces/Time acknowledgment_time # Temps accusé réception
builtin_interfaces/Time resolution_time # Temps résolution
string alert_state          # NEW, ONGOING, RECURRING, CLEARED

# Source et origine
string source_system        # Système source
string source_component     # Composant source
string source_sensor        # Capteur source
string source_process       # Processus source
geometry_msgs/Point source_location # Position source

# Description et détails
string alert_title          # Titre alerte
string alert_description    # Description détaillée
string alert_message        # Message utilisateur
string technical_details    # Détails techniques
string[] affected_systems   # Systèmes affectés
string[] affected_functions # Fonctions affectées

# Conditions déclenchement
string trigger_condition    # Condition déclenchement
float64 trigger_value       # Valeur déclenchement
float64 threshold_value     # Valeur seuil
string comparison_operator  # >, <, ==, >=, <=, !=
builtin_interfaces/Duration trigger_duration # Durée avant déclenchement

# Impact et conséquences
string impact_assessment    # NONE, MINOR, MODERATE, MAJOR, SEVERE
string[] operational_impact # Impact opérationnel
string[] safety_impact      # Impact sécurité
float64 mission_impact_percent # % impact mission
bool requires_immediate_action # Action immédiate requise
bool requires_human_intervention # Intervention humaine requise

# Actions recommandées
string[] recommended_actions # Actions recommandées
string[] automatic_actions_taken # Actions automatiques prises
string[] required_manual_actions # Actions manuelles requises
string escalation_procedure # Procédure escalade
string[] preventive_measures # Mesures préventives

# Données contextuelles
string flight_phase         # Phase vol lors alerte
string mission_context      # Contexte mission
float64 system_load_percent # % charge système
string weather_conditions   # Conditions météo
string[] environmental_factors # Facteurs environnementaux

# Historique et récurrence
uint32 occurrence_count     # Nombre occurrences
builtin_interfaces/Time first_occurrence # Première occurrence
builtin_interfaces/Time last_occurrence # Dernière occurrence
builtin_interfaces/Duration average_duration # Durée moyenne
string recurrence_pattern   # Modèle récurrence

# Diagnostic et données
float64[] diagnostic_values # Valeurs diagnostic
string[] diagnostic_params  # Paramètres diagnostic
string[] log_references     # Références logs
string[] trace_data         # Données trace
string error_code          # Code erreur

# Résolution et réparation
string resolution_method    # Méthode résolution
string[] resolution_steps   # Étapes résolution
string resolved_by          # Résolu par
string resolution_notes     # Notes résolution
bool requires_maintenance   # Maintenance requise
builtin_interfaces/Time next_check_time # Prochain contrôle

# Notification et communication
string[] notification_recipients # Destinataires notification
string notification_method  # Méthode notification
bool notification_sent      # Notification envoyée
string[] communication_channels # Canaux communication
bool requires_external_notification # Notification externe requise

# Corrélation et groupement
string[] related_alerts     # Alertes liées
string parent_alert_id      # ID alerte parent
string[] child_alert_ids    # IDs alertes enfants
string correlation_group    # Groupe corrélation
float64 correlation_confidence # Confiance corrélation

# Métriques et suivi
float64 response_time_s     # Temps réponse (s)
float64 resolution_time_s   # Temps résolution (s)
float64 downtime_s          # Temps arrêt (s)
float64 cost_impact         # Impact coût
string[] performance_impact # Impact performance

# Configuration et seuils
bool is_configurable        # Alerte configurable
float64[] threshold_levels  # Niveaux seuils
builtin_interfaces/Duration suppression_time # Temps suppression
bool auto_acknowledge       # Accusé réception auto
bool auto_resolve          # Résolution auto

# Métadonnées et validation
string alert_version        # Version définition alerte
string creator_system       # Système créateur
builtin_interfaces/Time expiry_time # Temps expiration
bool is_test_alert         # Alerte test
string[] validation_flags   # Indicateurs validation
float64 confidence_level    # Niveau confiance

# Archivage et persistance
bool should_archive         # Doit archiver
builtin_interfaces/Duration retention_period # Période rétention
string archive_location     # Emplacement archive
bool is_archived           # Est archivé
string archive_reason       # Raison archivage
