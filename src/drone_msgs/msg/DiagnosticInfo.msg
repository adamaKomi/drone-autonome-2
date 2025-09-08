# Informations diagnostiques système complètes
std_msgs/Header header

# Identification diagnostic
string diagnostic_id        # ID unique diagnostic
string diagnostic_name      # Nom diagnostic
string diagnostic_type      # SYSTEM, HARDWARE, SOFTWARE, NETWORK, SENSOR, PERFORMANCE
string diagnostic_category  # HEALTH_CHECK, FAULT_DETECTION, PERFORMANCE_ANALYSIS, MAINTENANCE
uint8 diagnostic_level      # 1=INFO, 2=WARNING, 3=ERROR, 4=CRITICAL

# Niveaux diagnostic constants
uint8 LEVEL_INFO = 1
uint8 LEVEL_WARNING = 2
uint8 LEVEL_ERROR = 3
uint8 LEVEL_CRITICAL = 4

# Types diagnostic constants
string TYPE_SYSTEM = "SYSTEM"
string TYPE_HARDWARE = "HARDWARE"
string TYPE_SOFTWARE = "SOFTWARE"
string TYPE_NETWORK = "NETWORK"
string TYPE_SENSOR = "SENSOR"
string TYPE_PERFORMANCE = "PERFORMANCE"

# Statut diagnostic
string status               # OK, WARNING, ERROR, CRITICAL, UNKNOWN
float64 health_score        # 0.0-1.0 score santé
float64 confidence_level    # 0.0-1.0 niveau confiance
builtin_interfaces/Time execution_time # Temps exécution diagnostic
builtin_interfaces/Duration execution_duration # Durée exécution

# Source et cible
string source_system        # Système source
string target_component     # Composant cible
string target_subsystem     # Sous-système cible
string[] tested_functions   # Fonctions testées
string test_method          # Méthode test

# Résultats généraux
string summary              # Résumé diagnostic
string detailed_description # Description détaillée
string[] findings           # Observations
string[] issues_detected    # Problèmes détectés
string[] recommendations    # Recommandations

# Données de test
string[] test_parameters    # Paramètres test
float64[] test_values       # Valeurs test
float64[] expected_values   # Valeurs attendues
float64[] tolerance_values  # Valeurs tolérance
string[] test_results       # Résultats test
bool[] test_passed          # Tests réussis

# Métriques système
float64 cpu_usage_percent   # % utilisation CPU
float64 memory_usage_percent # % utilisation mémoire
float64 disk_usage_percent  # % utilisation disque
float64 network_latency_ms  # ms latence réseau
float64 temperature_celsius # °C température
float64 voltage_volts       # V tension
float64 current_amperes     # A courant

# État composants
string[] component_names    # Noms composants
string[] component_states   # États composants
float64[] component_health_scores # Scores santé composants
string[] component_errors   # Erreurs composants
string[] component_warnings # Avertissements composants
builtin_interfaces/Time[] component_last_check # Dernière vérification composants

# Capteurs et instruments
string[] sensor_names       # Noms capteurs
string[] sensor_states      # États capteurs
float64[] sensor_values     # Valeurs capteurs
float64[] sensor_accuracies # Précisions capteurs
string[] sensor_calibration_status # États calibration capteurs
builtin_interfaces/Time[] sensor_last_calibration # Dernière calibration capteurs

# Communication et connectivité
string[] interface_names    # Noms interfaces
string[] interface_states   # États interfaces
float64[] signal_strengths  # Forces signaux
float64[] data_rates        # Débits données
uint32[] error_counts       # Nombres erreurs
string[] protocol_versions  # Versions protocoles

# Performances temporelles
float64[] response_times_ms # Temps réponse (ms)
float64[] processing_times_ms # Temps traitement (ms)
float64[] latency_times_ms  # Temps latence (ms)
float64 real_time_factor    # Facteur temps réel
uint32 missed_deadlines     # Échéances manquées

# Logs et traces
string[] log_entries        # Entrées log
string[] error_messages     # Messages erreur
string[] warning_messages   # Messages avertissement
string[] debug_information  # Informations debug
string[] stack_traces       # Traces pile

# Historique et tendances
float64[] historical_values # Valeurs historiques
builtin_interfaces/Time[] historical_timestamps # Horodatages historiques
string trend_analysis       # Analyse tendance
float64 degradation_rate    # Taux dégradation
string[] anomalies_detected # Anomalies détectées

# Environnement système
string operating_system     # Système exploitation
string kernel_version       # Version noyau
string[] installed_packages # Paquets installés
string[] running_processes  # Processus en cours
float64 system_uptime_hours # Temps fonctionnement (h)
float64 load_average        # Charge moyenne

# Configuration système
string[] configuration_parameters # Paramètres configuration
string[] configuration_values # Valeurs configuration
string[] configuration_files # Fichiers configuration
string[] environment_variables # Variables environnement
string[] active_services    # Services actifs

# Sécurité et intégrité
string[] security_checks    # Vérifications sécurité
bool[] security_results     # Résultats sécurité
string[] checksum_values    # Valeurs checksum
string[] integrity_checks   # Vérifications intégrité
string[] permission_issues  # Problèmes permissions

# Maintenance et prédictif
float64 maintenance_score   # Score maintenance
string[] maintenance_needs  # Besoins maintenance
builtin_interfaces/Time next_maintenance # Prochaine maintenance
float64 remaining_life_percent # % vie restante
string[] wear_indicators    # Indicateurs usure

# Calibration et étalonnage
string[] calibration_status # États calibration
builtin_interfaces/Time[] last_calibration # Dernières calibrations
builtin_interfaces/Time[] next_calibration # Prochaines calibrations
float64[] calibration_drift # Dérive calibration
string[] calibration_references # Références calibration

# Ressources et utilisation
float64[] resource_usage    # Utilisation ressources
float64[] resource_limits   # Limites ressources
string[] resource_warnings  # Avertissements ressources
float64[] allocation_efficiency # Efficacité allocation
string[] bottlenecks        # Goulots étranglement

# Actions et résolution
string[] suggested_actions  # Actions suggérées
string[] corrective_actions # Actions correctives
string[] preventive_actions # Actions préventives
uint8[] action_priorities   # Priorités actions
string[] resolution_steps   # Étapes résolution

# Métadonnées
string diagnostic_version   # Version diagnostic
string creator_tool         # Outil créateur
builtin_interfaces/Time creation_time # Temps création
bool is_automated          # Automatisé
string[] reference_documents # Documents référence
string[] contact_information # Informations contact
