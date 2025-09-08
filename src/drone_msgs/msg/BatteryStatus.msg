# État détaillé de la batterie
std_msgs/Header header

# Identification batterie
string battery_id
string battery_type            # LiPo, Li-ion, NiMH
uint8 cell_count
float64 nominal_voltage        # Volts
float64 nominal_capacity       # Ah

# État actuel
float64 voltage                # Volts total
float64 current                # Ampères (positif = décharge)
float64 power                  # Watts
float64 energy_consumed        # Wh depuis démarrage
float64 charge_remaining       # Ah restant
float64 percentage_remaining   # % (0-100)

# Cellules individuelles
float64[] cell_voltages        # Volts par cellule
float64 voltage_min            # Tension cellule minimale
float64 voltage_max            # Tension cellule maximale
float64 voltage_diff           # Différence max-min

# Température
float64 temperature            # °C
float64 temperature_max        # °C maximum cellules

# Prédictions et estimations
builtin_interfaces/Duration time_remaining     # Temps de vol restant
builtin_interfaces/Duration time_to_empty      # Temps jusqu'à décharge
float64 distance_remaining     # Distance possible (m)
float64 discharge_rate         # A/h actuel

# État de santé
uint8 HEALTH_EXCELLENT = 0
uint8 HEALTH_GOOD = 1
uint8 HEALTH_FAIR = 2
uint8 HEALTH_POOR = 3
uint8 HEALTH_CRITICAL = 4

uint8 health_status
float64 state_of_health        # % (0-100) santé générale
uint32 cycle_count             # Nombre de cycles charge/décharge
bool balancing_active          # Équilibrage cellules actif

# Alertes batterie
bool low_voltage_alert
bool high_temperature_alert
bool unbalanced_cells_alert
bool critical_level_alert
string[] active_warnings
string[] maintenance_recommendations
