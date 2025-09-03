# 💾 PROMPT IA - TRANSFORMATION PACKAGE DRONE_DATA_COLLECTOR

## 🎯 OBJECTIF
Transforme complètement le package `drone_data_collector` en suivant EXACTEMENT la même structure, qualité et méthodologie que le package `drone_interface` fourni en exemple.

## 📋 CONTEXTE DU PROJET
Le package `drone_data_collector` est le **système de collecte et analyse de données scientifiques** du drone de pollinisation. Il doit :
- Collecter données de vol, environnementales, et de performance
- Analyser et traiter données en temps réel
- Stocker données structurées pour recherche scientifique
- Générer rapports et visualisations
- Synchroniser avec bases de données externes
- Fournir analytics prédictifs pour optimisation

## 🏗️ ARCHITECTURE REQUISE

### Structure EXACTE à implémenter :
```
drone_data_collector/
├── drone_data_collector/               # Package Python principal
│   ├── __init__.py
│   ├── collector_node.py               # Nœud principal avec lifecycle ROS2
│   ├── data_manager.py                 # Gestionnaire données centralisé
│   ├── flight_data_collector.py        # Collecteur données vol
│   ├── environmental_collector.py      # Collecteur données environnement
│   ├── performance_collector.py        # Collecteur métriques performance
│   ├── scientific_collector.py         # Collecteur données scientifiques
│   ├── image_data_manager.py           # Gestionnaire données images
│   ├── telemetry_processor.py          # Processeur télémétrie
│   ├── data_analyzer.py                # Analyseur données temps réel
│   ├── database_manager.py             # Interface bases de données
│   ├── export_manager.py               # Gestionnaire exports
│   └── tools/                          # Outils CLI complets
│       ├── __init__.py
│       ├── data_status.py              # État collecte CLI
│       ├── export_data.py              # Export données CLI
│       ├── analyze_mission.py          # Analyse mission CLI
│       ├── generate_report.py          # Génération rapport CLI
│       ├── data_cleanup.py             # Nettoyage données CLI
│       ├── sync_database.py            # Synchronisation DB CLI
│       └── data_diagnostics.py         # Diagnostics données CLI
├── launch/                             # Fichiers de lancement
│   └── data_collector_launch.py        # Lancement complet collecteur
├── config/                             # Configuration YAML
│   ├── collector_params.yaml           # Paramètres principaux
│   ├── database_config.yaml            # Configuration BDD
│   ├── export_formats.yaml             # Formats d'export
│   └── analytics_config.yaml           # Configuration analytics
├── schemas/                            # Schémas données
│   ├── flight_data_schema.json         # Schéma données vol
│   ├── environmental_schema.json       # Schéma données environnement
│   ├── performance_schema.json         # Schéma métriques
│   └── scientific_schema.json          # Schéma données scientifiques
├── reports/                            # Templates rapports
│   ├── mission_report.html             # Template rapport mission
│   ├── performance_report.html         # Template rapport performance
│   └── scientific_report.html          # Template rapport scientifique
├── test/                               # Tests complets
│   ├── test_collector.py               # Tests collecteur
│   ├── test_data_manager.py            # Tests gestionnaire
│   ├── test_analyzers.py               # Tests analyseurs
│   ├── test_exports.py                 # Tests exports
│   └── test_database.py                # Tests base données
├── scripts/                            # Scripts utilitaires
│   ├── migrate_data.py                 # Migration données
│   ├── validate_data.py                # Validation données
│   └── backup_data.py                  # Sauvegarde données
├── resource/                           # Ressources package
│   └── drone_data_collector
├── setup.py                           # Configuration Python
├── package.xml                        # Métadonnées ROS2
└── README.md                          # Documentation exhaustive
```

## 🔧 SPÉCIFICATIONS TECHNIQUES DÉTAILLÉES

### 1. COLLECTOR NODE (collector_node.py)
**Caractéristiques obligatoires :**
- **Lifecycle management complet** (configure/activate/deactivate/cleanup)
- **Multi-source data collection** simultanée
- **Real-time processing** avec buffering intelligent
- **Data validation** et integrity checks
- **Automatic backup** et recovery

**Services exposés (OBLIGATOIRES) :**
```python
/drone_data/start_collection         # Démarrer collecte
/drone_data/stop_collection          # Arrêter collecte
/drone_data/export_data              # Export données
/drone_data/generate_report          # Génération rapport
/drone_data/analyze_mission          # Analyse mission
/drone_data/get_statistics           # Statistiques collecte
/drone_data/validate_data            # Validation données
/drone_data/backup_data              # Sauvegarde données
/drone_data/sync_database            # Synchronisation BDD
/drone_data/cleanup_old_data         # Nettoyage données anciennes
```

**Topics souscrits (OBLIGATOIRES) :**
```python
/drone/status                       # État drone temps réel
/drone/telemetry                    # Télémétrie complète
/drone_navigation/position          # Position précise
/drone_navigation/trajectory        # Trajectoire suivie
/drone_vision/detections            # Détections fleurs
/drone_vision/classifications       # Classifications espèces
/drone_mission/status               # État mission
/drone_mission/progress             # Progression mission
/mavros/global_position/global      # Position GPS
/mavros/imu/data                    # Données IMU
/mavros/battery                     # État batterie
/mavros/wind_estimation             # Estimation vent
```

**Topics publiés (OBLIGATOIRES) :**
```python
/drone_data/collection_status       # État collecte
/drone_data/data_summary            # Résumé données
/drone_data/analytics_results       # Résultats analytics
/drone_data/performance_metrics     # Métriques performance
/drone_data/alerts                  # Alertes données
/diagnostics                        # Diagnostics système
```

### 2. DATA MANAGER (data_manager.py)
**Gestion centralisée OBLIGATOIRE :**
- **Unified data model** pour tous types
- **Schema validation** automatique
- **Data versioning** et tracking
- **Compression** intelligente
- **Indexing** pour recherche rapide

```python
class DataManager:
    def __init__(self):
        self.data_store = {}
        self.schemas = {}
        self.validators = {}
        self.indexes = {}
        
    def register_data_type(self, data_type: str, schema: dict)
    def validate_data(self, data_type: str, data: dict) -> bool
    def store_data(self, data_type: str, data: dict) -> str
    def retrieve_data(self, data_type: str, filters: dict) -> List[dict]
    def aggregate_data(self, data_type: str, aggregation: str) -> dict
    def export_data(self, data_type: str, format: str) -> str
    
    def create_index(self, data_type: str, field: str)
    def query_indexed(self, data_type: str, query: dict) -> List[dict]
    def compress_data(self, data: dict) -> bytes
    def decompress_data(self, compressed: bytes) -> dict
```

### 3. FLIGHT DATA COLLECTOR (flight_data_collector.py)
**Données vol COMPLÈTES :**
- **Position/Orientation** haute fréquence (10Hz)
- **Vitesse/Accélération** 3D
- **Commandes contrôle** appliquées
- **État systèmes** (GPS, IMU, magnétomètre)
- **Performance moteurs/hélices**

```python
class FlightDataCollector:
    def __init__(self):
        self.collection_rate = 10.0  # Hz
        self.data_buffer = CircularBuffer(size=1000)
        self.validators = FlightDataValidators()
        
    async def collect_position_data(self) -> dict:
        """Collecte données position précises"""
        return {
            'timestamp': time.time(),
            'latitude': self.get_latitude(),
            'longitude': self.get_longitude(),
            'altitude_msl': self.get_altitude_msl(),
            'altitude_agl': self.get_altitude_agl(),
            'heading': self.get_heading(),
            'ground_speed': self.get_ground_speed(),
            'vertical_speed': self.get_vertical_speed(),
            'gps_fix_type': self.get_gps_fix_type(),
            'hdop': self.get_hdop(),
            'vdop': self.get_vdop(),
            'satellites_visible': self.get_satellites_count()
        }
        
    async def collect_attitude_data(self) -> dict:
        """Collecte données attitude/orientation"""
        return {
            'timestamp': time.time(),
            'roll': self.get_roll(),
            'pitch': self.get_pitch(),
            'yaw': self.get_yaw(),
            'roll_rate': self.get_roll_rate(),
            'pitch_rate': self.get_pitch_rate(),
            'yaw_rate': self.get_yaw_rate(),
            'quaternion': self.get_quaternion(),
            'angular_velocity': self.get_angular_velocity(),
            'linear_acceleration': self.get_linear_acceleration()
        }
        
    async def collect_control_data(self) -> dict:
        """Collecte données commandes contrôle"""
        return {
            'timestamp': time.time(),
            'throttle': self.get_throttle(),
            'elevator': self.get_elevator(),
            'aileron': self.get_aileron(),
            'rudder': self.get_rudder(),
            'flight_mode': self.get_flight_mode(),
            'armed': self.get_armed_status(),
            'guided': self.get_guided_status(),
            'auto_mode': self.get_auto_mode()
        }
```

### 4. ENVIRONMENTAL COLLECTOR (environmental_collector.py)
**Données environnement OBLIGATOIRES :**
- **Conditions météo** (température, humidité, pression)
- **Vent** (vitesse, direction, turbulences)
- **Luminosité** et conditions éclairage
- **Qualité air** si capteurs disponibles
- **Conditions terrain** visibles

```python
class EnvironmentalCollector:
    def __init__(self):
        self.weather_api = WeatherAPI()
        self.sensor_manager = EnvironmentalSensors()
        
    async def collect_weather_data(self) -> dict:
        """Collecte données météorologiques"""
        return {
            'timestamp': time.time(),
            'temperature': self.sensor_manager.get_temperature(),
            'humidity': self.sensor_manager.get_humidity(),
            'pressure': self.sensor_manager.get_pressure(),
            'wind_speed': self.sensor_manager.get_wind_speed(),
            'wind_direction': self.sensor_manager.get_wind_direction(),
            'wind_gusts': self.sensor_manager.get_wind_gusts(),
            'visibility': self.weather_api.get_visibility(),
            'cloud_cover': self.weather_api.get_cloud_cover(),
            'precipitation': self.weather_api.get_precipitation(),
            'uv_index': self.sensor_manager.get_uv_index()
        }
        
    async def collect_lighting_data(self) -> dict:
        """Collecte données éclairage"""
        return {
            'timestamp': time.time(),
            'ambient_light': self.sensor_manager.get_ambient_light(),
            'sun_elevation': self.calculate_sun_elevation(),
            'sun_azimuth': self.calculate_sun_azimuth(),
            'shadow_factor': self.calculate_shadow_factor(),
            'lighting_quality': self.assess_lighting_quality()
        }
```

### 5. SCIENTIFIC COLLECTOR (scientific_collector.py)
**Données scientifiques SPÉCIALISÉES :**
- **Inventaire fleurs** détecté
- **Données pollinisation** (succès, échecs)
- **Biodiversité** observée
- **Couverture territoriale** effective
- **Conditions optimales** pollinisation

```python
class ScientificCollector:
    def __init__(self):
        self.flower_database = FlowerDatabase()
        self.species_classifier = SpeciesClassifier()
        
    async def collect_flower_data(self, detection: FlowerDetection) -> dict:
        """Collecte données scientifiques fleur"""
        classification = await self.species_classifier.classify(detection)
        
        return {
            'timestamp': time.time(),
            'detection_id': detection.id,
            'gps_coordinates': detection.gps_position,
            'species_name': classification.species,
            'species_confidence': classification.confidence,
            'flower_state': classification.state,
            'estimated_size': classification.size,
            'color_primary': classification.primary_color,
            'color_secondary': classification.secondary_color,
            'health_assessment': classification.health_score,
            'pollination_suitability': classification.pollination_score,
            'environmental_context': await self.get_environmental_context(),
            'habitat_type': self.classify_habitat(),
            'surrounding_vegetation': self.analyze_vegetation()
        }
        
    async def collect_pollination_data(self, event: PollinationEvent) -> dict:
        """Collecte données événement pollinisation"""
        return {
            'timestamp': time.time(),
            'flower_id': event.flower_id,
            'approach_method': event.approach_method,
            'success': event.success,
            'precision_error': event.precision_error,
            'duration': event.duration,
            'environmental_conditions': event.environmental_conditions,
            'follow_up_required': event.follow_up_required,
            'quality_score': event.quality_score
        }
```

### 6. DATA ANALYZER (data_analyzer.py)
**Analytics temps réel OBLIGATOIRES :**
- **Performance tracking** missions
- **Trend analysis** conditions optimales
- **Predictive modeling** pour optimisation
- **Anomaly detection** dans données
- **Correlation analysis** multi-variables

```python
class DataAnalyzer:
    def __init__(self):
        self.ml_models = AnalyticsModels()
        self.statistics_engine = StatisticsEngine()
        
    def analyze_mission_performance(self, mission_data: dict) -> dict:
        """Analyse performance mission"""
        return {
            'efficiency_score': self.calculate_efficiency(mission_data),
            'coverage_percentage': self.calculate_coverage(mission_data),
            'energy_efficiency': self.calculate_energy_efficiency(mission_data),
            'time_efficiency': self.calculate_time_efficiency(mission_data),
            'accuracy_metrics': self.calculate_accuracy(mission_data),
            'improvement_suggestions': self.suggest_improvements(mission_data)
        }
        
    def detect_patterns(self, historical_data: List[dict]) -> dict:
        """Détection patterns dans données historiques"""
        return {
            'seasonal_patterns': self.detect_seasonal_patterns(historical_data),
            'weather_correlations': self.analyze_weather_impact(historical_data),
            'optimal_conditions': self.identify_optimal_conditions(historical_data),
            'success_factors': self.identify_success_factors(historical_data)
        }
        
    def predict_performance(self, current_conditions: dict) -> dict:
        """Prédiction performance basée conditions actuelles"""
        prediction = self.ml_models.predict_performance(current_conditions)
        
        return {
            'predicted_success_rate': prediction.success_rate,
            'predicted_efficiency': prediction.efficiency,
            'confidence_level': prediction.confidence,
            'risk_factors': prediction.risk_factors,
            'recommendations': prediction.recommendations
        }
```

### 7. OUTILS CLI COMPLETS (tools/)

#### data_status.py
```bash
ros2 run drone_data_collector data_status
ros2 run drone_data_collector data_status --detailed --live
ros2 run drone_data_collector data_status --storage --performance
```

#### export_data.py
```bash
ros2 run drone_data_collector export_data --format csv --output data.csv
ros2 run drone_data_collector export_data --format json --date 2025-01-01
ros2 run drone_data_collector export_data --mission-id 12345 --scientific
```

#### analyze_mission.py
```bash
ros2 run drone_data_collector analyze_mission --mission-id 12345
ros2 run drone_data_collector analyze_mission --date-range "2025-01-01:2025-01-31"
ros2 run drone_data_collector analyze_mission --compare missions.json
```

## 📊 CONFIGURATION YAML COMPLÈTE

### collector_params.yaml
```yaml
data_collection:
  general:
    collection_rate: 10.0           # Hz
    buffer_size: 10000              # samples
    auto_start: true
    auto_export: false
    compression_enabled: true
    validation_strict: true
    
  flight_data:
    enabled: true
    rate: 10.0                      # Hz
    include_raw_sensor_data: true
    include_filtered_data: true
    
    position:
      gps_precision_required: 3.0   # m
      altitude_precision: 1.0       # m
      coordinate_system: "WGS84"
      
    attitude:
      quaternion_format: true
      euler_angles: true
      angular_rates: true
      
    control:
      log_commands: true
      log_responses: true
      log_mode_changes: true
      
  environmental:
    enabled: true
    rate: 1.0                       # Hz
    weather_api_enabled: true
    local_sensors_enabled: true
    
    weather:
      api_provider: "openweather"
      api_key: "${WEATHER_API_KEY}"
      update_interval: 300           # s
      
    sensors:
      temperature_sensor: "/dev/ttyUSB0"
      humidity_sensor: "/dev/ttyUSB1"
      pressure_sensor: "/dev/ttyUSB2"
      wind_sensor: "/dev/ttyUSB3"
      
  scientific:
    enabled: true
    rate: 5.0                       # Hz (during detection)
    detailed_classification: true
    habitat_analysis: true
    
    flower_data:
      min_confidence: 0.7
      include_images: true
      image_quality: "high"
      metadata_complete: true
      
    pollination:
      track_attempts: true
      track_success_rate: true
      environmental_correlation: true
      
  performance:
    enabled: true
    rate: 5.0                       # Hz
    cpu_monitoring: true
    memory_monitoring: true
    network_monitoring: true
    storage_monitoring: true
    
storage:
  local:
    enabled: true
    base_path: "/data/drone_missions"
    max_size_gb: 100
    retention_days: 90
    backup_enabled: true
    
  database:
    enabled: true
    type: "postgresql"              # postgresql, mysql, sqlite
    host: "localhost"
    port: 5432
    database: "drone_data"
    username: "${DB_USERNAME}"
    password: "${DB_PASSWORD}"
    
    connection_pool:
      min_connections: 5
      max_connections: 20
      timeout: 30                   # s
      
  cloud:
    enabled: false
    provider: "aws"                 # aws, gcp, azure
    bucket: "drone-data-bucket"
    region: "us-east-1"
    sync_interval: 3600             # s
    
export:
  formats:
    csv:
      enabled: true
      delimiter: ","
      include_headers: true
      
    json:
      enabled: true
      pretty_print: true
      compression: "gzip"
      
    parquet:
      enabled: true
      compression: "snappy"
      
    hdf5:
      enabled: true
      compression: "gzip"
      
  scientific:
    include_metadata: true
    include_analysis: true
    include_visualizations: false
    coordinate_precision: 8         # decimal places
    
analytics:
  real_time:
    enabled: true
    update_interval: 5.0            # s
    metrics_window: 300             # s
    
  machine_learning:
    enabled: true
    model_update_interval: 86400    # s (daily)
    prediction_confidence: 0.8
    
  reporting:
    auto_generate: true
    generate_interval: 3600         # s
    include_visualizations: true
    
alerts:
  enabled: true
  severity_levels: ["INFO", "WARNING", "ERROR", "CRITICAL"]
  
  thresholds:
    storage_usage: 0.9              # 90%
    memory_usage: 0.8               # 80%
    data_loss_rate: 0.01            # 1%
    validation_failure_rate: 0.05   # 5%
    
  notifications:
    email_enabled: false
    webhook_enabled: true
    webhook_url: "${ALERT_WEBHOOK_URL}"
```

## 📊 SCHÉMAS DONNÉES

### flight_data_schema.json
```json
{
  "type": "object",
  "properties": {
    "timestamp": {"type": "number"},
    "position": {
      "type": "object",
      "properties": {
        "latitude": {"type": "number", "minimum": -90, "maximum": 90},
        "longitude": {"type": "number", "minimum": -180, "maximum": 180},
        "altitude_msl": {"type": "number"},
        "altitude_agl": {"type": "number", "minimum": 0},
        "heading": {"type": "number", "minimum": 0, "maximum": 360},
        "ground_speed": {"type": "number", "minimum": 0},
        "vertical_speed": {"type": "number"}
      },
      "required": ["latitude", "longitude", "altitude_msl"]
    },
    "attitude": {
      "type": "object",
      "properties": {
        "roll": {"type": "number"},
        "pitch": {"type": "number"},
        "yaw": {"type": "number"},
        "quaternion": {
          "type": "array",
          "items": {"type": "number"},
          "minItems": 4,
          "maxItems": 4
        }
      }
    },
    "control": {
      "type": "object",
      "properties": {
        "flight_mode": {"type": "string"},
        "armed": {"type": "boolean"},
        "throttle": {"type": "number", "minimum": 0, "maximum": 1},
        "commands": {"type": "object"}
      }
    }
  },
  "required": ["timestamp", "position", "attitude", "control"]
}
```

## 🧪 TESTS OBLIGATOIRES

### test_collector.py
```python
class TestDataCollector(unittest.TestCase):
    def test_lifecycle_management(self)
    def test_multi_source_collection(self)
    def test_data_validation(self)
    def test_real_time_processing(self)
    def test_buffer_management(self)
    def test_error_recovery(self)
```

### test_data_manager.py
```python
class TestDataManager(unittest.TestCase):
    def test_schema_validation(self)
    def test_data_storage_retrieval(self)
    def test_indexing_performance(self)
    def test_compression_decompression(self)
    def test_concurrent_access(self)
```

## 🎯 MÉTRIQUES DE PERFORMANCE

### Objectifs OBLIGATOIRES :
- **Collection rate** : >10Hz pour données critiques
- **Data validation** : <10ms par échantillon
- **Storage efficiency** : >70% compression sans perte
- **Query performance** : <100ms pour requêtes indexées
- **Export speed** : >1MB/s pour formats standards
- **Analytics latency** : <5s pour analyse temps réel

## 🔗 INTÉGRATION SYSTÈME

### Avec tous les packages :
- Collecte données de tous les nœuds ROS2
- Corrélation temporelle multi-sources
- Analytics prédictifs pour optimisation
- Rapports complets performance mission

---

**IMPORTANT : Ce package est essentiel pour la recherche scientifique et l'amélioration continue du système. La qualité des données collectées détermine l'efficacité future du système de pollinisation.**
