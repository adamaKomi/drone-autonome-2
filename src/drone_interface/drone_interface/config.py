#!/usr/bin/env python3
"""
=============================================================================
CONFIGURATION POUR DRONE INTERFACE
=============================================================================
Auteur: Assistant IA
Date: 2025-08-29
Version: 1.0.0

Description:
    Configuration centralisée pour le système de contrôle de drone.
    Permet d'adapter facilement le comportement selon l'environnement.
=============================================================================
"""

from dataclasses import dataclass
from typing import Dict, List, Tuple
import os


@dataclass
class SafetyConfig:
    """Configuration de sécurité"""
    # Vérifications d'armement
    min_battery_voltage: float = 10.5        # Voltage minimum pour ArduPilot (V)
    min_battery_percentage: float = 20.0     # Pourcentage minimum de batterie
    min_gps_satellites: int = 6              # Satellites GPS minimum
    max_altitude: float = 120.0              # Altitude maximum légale (m)
    geofence_radius: float = 100.0           # Rayon de géofence (m)
    
    # Alertes en vol
    critical_battery_percentage: float = 20.0  # Seuil d'alerte batterie
    low_battery_percentage: float = 30.0       # Seuil d'avertissement batterie
    max_velocity: float = 15.0                 # Vitesse maximum (m/s)
    
    # Timeouts de sécurité
    connection_timeout: float = 5.0           # Timeout connexion MAVROS (s)
    command_timeout: float = 10.0             # Timeout commandes (s)
    
    # Options
    enable_safety_checks: bool = True         # Activer les vérifications de sécurité
    enable_geofence: bool = True              # Activer la géofence
    enable_altitude_limit: bool = True        # Activer la limite d'altitude


@dataclass
class FlightConfig:
    """Configuration de vol"""
    # Décollage
    default_takeoff_altitude: float = 2.5    # Altitude de décollage par défaut (m)
    takeoff_rate: float = 1.0                # Vitesse de montée (m/s)
    min_takeoff_altitude: float = 1.0        # Altitude minimum de décollage (m)
    max_takeoff_altitude: float = 50.0       # Altitude maximum de décollage (m)
    
    # Atterrissage
    landing_rate: float = 0.5                # Vitesse de descente (m/s)
    landing_step_size: float = 0.5           # Palier de descente (m)
    final_landing_altitude: float = 0.2      # Altitude finale avant désarmement (m)
    
    # Contrôle de position
    position_tolerance: float = 0.5          # Tolérance de position (m)
    setpoint_publish_rate: float = 20.0      # Fréquence publication setpoints (Hz)
    max_position_error: float = 5.0          # Erreur maximum de position (m)
    
    # Timeouts
    takeoff_timeout: float = 30.0            # Timeout décollage (s)
    landing_timeout: float = 60.0            # Timeout atterrissage (s)
    mode_change_timeout: float = 10.0        # Timeout changement de mode (s)
    
    # Modes préférés
    preferred_flight_mode: str = "GUIDED"    # Mode de vol préféré
    emergency_mode: str = "RTL"              # Mode d'urgence
    backup_emergency_mode: str = "LAND"      # Mode d'urgence de secours


@dataclass
class CommunicationConfig:
    """Configuration communication MAVROS"""
    # Topics MAVROS
    state_topic: str = "/mavros/state"
    position_topic: str = "/mavros/local_position/pose"
    velocity_topic: str = "/mavros/local_position/velocity_local"
    battery_topic: str = "/mavros/battery"
    gps_topic: str = "/mavros/global_position/global"
    setpoint_topic: str = "/mavros/setpoint_position/local"
    
    # Services MAVROS
    arming_service: str = "/mavros/cmd/arming"
    mode_service: str = "/mavros/set_mode"
    takeoff_service: str = "/mavros/cmd/takeoff"
    land_service: str = "/mavros/cmd/land"
    
    # QoS
    qos_depth: int = 10
    qos_reliable: bool = True
    
    # Timeouts
    service_timeout: float = 5.0             # Timeout services (s)
    topic_timeout: float = 1.0               # Timeout topics (s)


@dataclass
class MonitoringConfig:
    """Configuration monitoring"""
    # Fréquences de surveillance
    monitoring_rate: float = 1.0             # Fréquence surveillance générale (Hz)
    status_display_rate: float = 0.1         # Fréquence affichage status (Hz)
    safety_check_rate: float = 2.0           # Fréquence vérifications sécurité (Hz)
    
    # Logs
    log_level: str = "INFO"                  # Niveau de log (DEBUG, INFO, WARN, ERROR)
    enable_periodic_status: bool = True      # Affichage périodique du statut
    status_display_interval: float = 10.0    # Intervalle affichage statut (s)
    
    # Métriques
    enable_performance_monitoring: bool = True  # Surveillance des performances
    max_loop_time: float = 0.1               # Temps maximum boucle principale (s)


@dataclass
class SimulationConfig:
    """Configuration spécifique à la simulation SITL"""
    # Paramètres SITL
    home_position: Tuple[float, float, float] = (0.0, 0.0, 0.0)  # Position home (x, y, z)
    use_fake_gps: bool = True                # Utiliser un GPS simulé
    disable_safety_checks: bool = False      # Désactiver certaines vérifications
    
    # Délais simulés
    simulated_arm_delay: float = 2.0         # Délai armement simulé (s)
    simulated_mode_change_delay: float = 1.0 # Délai changement mode simulé (s)
    
    # Tolérances relaxées pour simulation
    relaxed_position_tolerance: float = 1.0  # Tolérance position relaxée (m)
    relaxed_altitude_tolerance: float = 1.0  # Tolérance altitude relaxée (m)


class DroneConfig:
    """Configuration principale du drone"""
    
    def __init__(self, profile: str = "default"):
        """
        Initialise la configuration selon le profil spécifié
        
        Profils disponibles:
        - "default": Configuration équilibrée
        - "simulation": Optimisé pour SITL
        - "conservative": Sécurité maximale
        - "aggressive": Performance maximale
        - "test": Configuration pour tests
        """
        self.profile = profile
        
        # Configurations de base
        self.safety = SafetyConfig()
        self.flight = FlightConfig()
        self.communication = CommunicationConfig()
        self.monitoring = MonitoringConfig()
        self.simulation = SimulationConfig()
        
        # Appliquer le profil spécifique
        self._apply_profile(profile)
        
        # Appliquer les variables d'environnement
        self._apply_environment_overrides()
    
    def _apply_profile(self, profile: str):
        """Applique un profil de configuration spécifique"""
        
        if profile == "simulation":
            # Configuration optimisée pour SITL
            self.safety.min_gps_satellites = 4      # SITL n'a pas toujours 6 satellites
            self.safety.enable_safety_checks = True
            self.safety.geofence_radius = 200.0      # Plus large pour les tests
            
            self.flight.position_tolerance = 1.0     # Plus tolérant
            self.flight.setpoint_publish_rate = 10.0 # Moins fréquent
            
            self.simulation.use_fake_gps = True
            self.simulation.disable_safety_checks = False
            
        elif profile == "conservative":
            # Configuration sécuritaire maximale
            self.safety.min_battery_voltage = 11.0
            self.safety.min_battery_percentage = 30.0
            self.safety.min_gps_satellites = 8
            self.safety.max_altitude = 50.0          # Plus conservateur
            self.safety.geofence_radius = 50.0       # Plus restreint
            self.safety.critical_battery_percentage = 30.0
            
            self.flight.default_takeoff_altitude = 2.0
            self.flight.max_takeoff_altitude = 20.0
            self.flight.position_tolerance = 0.3     # Plus strict
            
        elif profile == "aggressive":
            # Configuration performance maximale
            self.safety.min_battery_percentage = 15.0
            self.safety.max_altitude = 200.0
            self.safety.geofence_radius = 500.0
            self.safety.max_velocity = 25.0
            
            self.flight.takeoff_rate = 2.0
            self.flight.landing_rate = 1.0
            self.flight.position_tolerance = 1.0
            self.flight.setpoint_publish_rate = 50.0  # Plus fréquent
            
        elif profile == "test":
            # Configuration pour tests unitaires
            self.safety.enable_safety_checks = False
            self.safety.connection_timeout = 1.0
            self.safety.command_timeout = 5.0
            
            self.flight.takeoff_timeout = 10.0
            self.flight.landing_timeout = 15.0
            
            self.monitoring.monitoring_rate = 10.0    # Plus fréquent pour tests
            self.monitoring.enable_periodic_status = False
    
    def _apply_environment_overrides(self):
        """Applique les surcharges depuis les variables d'environnement"""
        
        # Sécurité
        if os.getenv('DRONE_MIN_BATTERY_VOLTAGE'):
            self.safety.min_battery_voltage = float(os.getenv('DRONE_MIN_BATTERY_VOLTAGE'))
        
        if os.getenv('DRONE_MAX_ALTITUDE'):
            self.safety.max_altitude = float(os.getenv('DRONE_MAX_ALTITUDE'))
        
        if os.getenv('DRONE_GEOFENCE_RADIUS'):
            self.safety.geofence_radius = float(os.getenv('DRONE_GEOFENCE_RADIUS'))
        
        # Vol
        if os.getenv('DRONE_TAKEOFF_ALTITUDE'):
            self.flight.default_takeoff_altitude = float(os.getenv('DRONE_TAKEOFF_ALTITUDE'))
        
        if os.getenv('DRONE_POSITION_TOLERANCE'):
            self.flight.position_tolerance = float(os.getenv('DRONE_POSITION_TOLERANCE'))
        
        # Communication
        if os.getenv('MAVROS_NAMESPACE'):
            namespace = os.getenv('MAVROS_NAMESPACE')
            self.communication.state_topic = f"{namespace}/state"
            self.communication.position_topic = f"{namespace}/local_position/pose"
            self.communication.velocity_topic = f"{namespace}/local_position/velocity_local"
            self.communication.battery_topic = f"{namespace}/battery"
            self.communication.gps_topic = f"{namespace}/global_position/global"
            self.communication.setpoint_topic = f"{namespace}/setpoint_position/local"
            self.communication.arming_service = f"{namespace}/cmd/arming"
            self.communication.mode_service = f"{namespace}/set_mode"
            self.communication.takeoff_service = f"{namespace}/cmd/takeoff"
            self.communication.land_service = f"{namespace}/cmd/land"
        
        # Désactivation sécurité
        if os.getenv('DRONE_DISABLE_SAFETY') == 'true':
            self.safety.enable_safety_checks = False
    
    def get_summary(self) -> Dict:
        """Retourne un résumé de la configuration"""
        return {
            'profile': self.profile,
            'safety': {
                'min_battery_voltage': self.safety.min_battery_voltage,
                'max_altitude': self.safety.max_altitude,
                'geofence_radius': self.safety.geofence_radius,
                'safety_checks_enabled': self.safety.enable_safety_checks
            },
            'flight': {
                'takeoff_altitude': self.flight.default_takeoff_altitude,
                'position_tolerance': self.flight.position_tolerance,
                'preferred_mode': self.flight.preferred_flight_mode
            },
            'communication': {
                'mavros_namespace': self.communication.state_topic.split('/')[1] if len(self.communication.state_topic.split('/')) > 2 else 'mavros'
            }
        }
    
    def validate(self) -> List[str]:
        """Valide la configuration et retourne les erreurs trouvées"""
        errors = []
        
        # Validation sécurité
        if self.safety.min_battery_voltage < 9.0:
            errors.append("min_battery_voltage trop faible (< 9.0V)")
        
        if self.safety.max_altitude > 400.0:  # Limite légale
            errors.append("max_altitude dépasse la limite légale (400m)")
        
        if self.safety.geofence_radius < 5.0:
            errors.append("geofence_radius trop petit (< 5m)")
        
        # Validation vol
        if self.flight.default_takeoff_altitude > self.safety.max_altitude:
            errors.append("takeoff_altitude dépasse max_altitude")
        
        if self.flight.position_tolerance <= 0:
            errors.append("position_tolerance doit être positive")
        
        # Validation communication
        if self.communication.service_timeout <= 0:
            errors.append("service_timeout doit être positif")
        
        return errors


def create_config(profile: str = None) -> DroneConfig:
    """
    Factory function pour créer une configuration
    
    Args:
        profile: Profil de configuration ("default", "simulation", "conservative", "aggressive", "test")
                Si None, utilise la variable d'environnement DRONE_CONFIG_PROFILE ou "default"
    
    Returns:
        DroneConfig: Instance de configuration
    """
    if profile is None:
        profile = os.getenv('DRONE_CONFIG_PROFILE', 'default')
    
    return DroneConfig(profile)


# Configuration par défaut (peut être importée directement)
DEFAULT_CONFIG = create_config("default")

# Configurations prédéfinies pour import direct
SIMULATION_CONFIG = create_config("simulation")
CONSERVATIVE_CONFIG = create_config("conservative")
AGGRESSIVE_CONFIG = create_config("aggressive")
TEST_CONFIG = create_config("test")


def print_config_info(config: DroneConfig):
    """Affiche les informations de configuration"""
    print("=" * 50)
    print(f"🔧 CONFIGURATION DRONE - PROFIL: {config.profile.upper()}")
    print("=" * 50)
    
    summary = config.get_summary()
    
    print("🛡️ SÉCURITÉ:")
    print(f"  • Batterie minimum: {summary['safety']['min_battery_voltage']}V")
    print(f"  • Altitude maximum: {summary['safety']['max_altitude']}m")
    print(f"  • Rayon géofence: {summary['safety']['geofence_radius']}m")
    print(f"  • Vérifications: {'✅ Activées' if summary['safety']['safety_checks_enabled'] else '❌ Désactivées'}")
    
    print("\n🚁 VOL:")
    print(f"  • Altitude décollage: {summary['flight']['takeoff_altitude']}m")
    print(f"  • Tolérance position: {summary['flight']['position_tolerance']}m")
    print(f"  • Mode préféré: {summary['flight']['preferred_mode']}")
    
    print("\n📡 COMMUNICATION:")
    print(f"  • Namespace MAVROS: /{summary['communication']['mavros_namespace']}")
    
    # Validation
    errors = config.validate()
    if errors:
        print("\n❌ ERREURS DE CONFIGURATION:")
        for error in errors:
            print(f"  • {error}")
    else:
        print("\n✅ Configuration valide")
    
    print("=" * 50)


if __name__ == '__main__':
    # Test des différents profils
    profiles = ["default", "simulation", "conservative", "aggressive", "test"]
    
    for profile in profiles:
        config = create_config(profile)
        print_config_info(config)
        print("\n")
