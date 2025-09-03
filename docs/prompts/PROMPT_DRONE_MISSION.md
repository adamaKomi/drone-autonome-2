# 🎯 PROMPT IA - TRANSFORMATION PACKAGE DRONE_MISSION

## 🎯 OBJECTIF
Transforme complètement le package `drone_mission` en suivant EXACTEMENT la même structure, qualité et méthodologie que le package `drone_interface` fourni en exemple.

## 📋 CONTEXTE DU PROJET
Le package `drone_mission` est le **cerveau de planification et exécution** du drone de pollinisation. Il doit orchestrer toutes les missions de pollinisation avec :
- Planification intelligente de missions multi-fleurs
- Exécution autonome avec gestion d'erreurs
- Optimisation de trajectoires pour efficacité maximale
- Adaptation dynamique aux conditions terrain
- Collecte et analyse de données mission
- Intégration temps réel avec navigation et vision
- Gestion de missions complexes multi-zones

## 🏗️ ARCHITECTURE REQUISE

### Structure EXACTE à implémenter :
```
drone_mission/
├── drone_mission/                      # Package Python principal
│   ├── __init__.py
│   ├── mission_node.py                 # Nœud principal avec lifecycle ROS2
│   ├── mission_planner.py              # Planificateur missions intelligentes
│   ├── mission_executor.py             # Exécuteur missions autonome
│   ├── mission_monitor.py              # Monitoring temps réel
│   ├── zone_manager.py                 # Gestionnaire zones géographiques
│   ├── flower_mission.py               # Missions spécialisées fleurs
│   ├── data_collector.py               # Collecteur données scientifiques
│   ├── optimization_engine.py          # Optimiseur trajectoires/temps
│   ├── emergency_handler.py            # Gestionnaire urgences/échecs
│   ├── weather_adapter.py              # Adaptation conditions météo
│   └── tools/                          # Outils CLI complets
│       ├── __init__.py
│       ├── plan_mission.py             # Planification CLI
│       ├── execute_mission.py          # Exécution CLI
│       ├── mission_status.py           # État mission CLI
│       ├── mission_stats.py            # Statistiques CLI
│       ├── validate_mission.py         # Validation CLI
│       ├── optimize_route.py           # Optimisation CLI
│       └── mission_diagnostics.py      # Diagnostics mission CLI
├── launch/                             # Fichiers de lancement
│   └── mission_launch.py               # Lancement complet mission
├── config/                             # Configuration YAML
│   ├── mission_params.yaml             # Paramètres principaux
│   ├── optimization_config.yaml        # Configuration optimisation
│   ├── emergency_protocols.yaml        # Protocoles d'urgence
│   └── zone_definitions.yaml           # Définitions zones
├── missions/                           # Bibliothèque missions
│   ├── templates/                      # Templates missions types
│   │   ├── single_flower.yaml
│   │   ├── zone_coverage.yaml
│   │   ├── research_survey.yaml
│   │   └── emergency_return.yaml
│   ├── saved/                          # Missions sauvegardées
│   └── examples/                       # Exemples missions
├── data/                               # Données et logs missions
│   ├── mission_logs/                   # Logs exécution
│   ├── performance_data/               # Données performance
│   └── scientific_data/                # Données collectées
├── test/                               # Tests complets
│   ├── test_mission.py                 # Tests mission
│   ├── test_planner.py                 # Tests planification
│   ├── test_executor.py                # Tests exécution
│   ├── test_optimization.py            # Tests optimisation
│   └── test_emergency.py               # Tests urgences
├── scripts/                            # Scripts utilitaires
│   ├── analyze_missions.py             # Analyse performance
│   ├── generate_reports.py             # Génération rapports
│   └── benchmark_planner.py            # Benchmarks planificateur
├── resource/                           # Ressources package
│   └── drone_mission
├── setup.py                           # Configuration Python
├── package.xml                        # Métadonnées ROS2
└── README.md                          # Documentation exhaustive
```

## 🔧 SPÉCIFICATIONS TECHNIQUES DÉTAILLÉES

### 1. MISSION NODE (mission_node.py)
**Caractéristiques obligatoires :**
- **Lifecycle management complet** (configure/activate/deactivate/cleanup)
- **State machine** robuste pour phases mission
- **Real-time monitoring** toutes métriques
- **Multi-threading** pour exécution parallèle
- **Fault tolerance** et recovery automatique

**Services exposés (OBLIGATOIRES) :**
```python
/drone_mission/plan_mission              # Planification nouvelle mission
/drone_mission/start_mission             # Démarrage mission
/drone_mission/pause_mission             # Pause mission
/drone_mission/resume_mission            # Reprise mission
/drone_mission/abort_mission             # Abandon mission urgence
/drone_mission/get_mission_status        # État mission actuelle
/drone_mission/optimize_mission          # Optimisation mission
/drone_mission/add_waypoint              # Ajout point navigation
/drone_mission/remove_waypoint           # Suppression point
/drone_mission/update_mission_params     # MAJ paramètres
/drone_mission/generate_report           # Génération rapport
/drone_mission/emergency_return          # Retour urgence
```

**Topics publiés (OBLIGATOIRES) :**
```python
/drone_mission/status                   # État mission temps réel
/drone_mission/progress                 # Progression mission %
/drone_mission/current_objective        # Objectif actuel
/drone_mission/waypoint_reached         # Point atteint
/drone_mission/flower_targeted          # Fleur ciblée
/drone_mission/flower_pollinated        # Fleur pollinisée
/drone_mission/mission_metrics          # Métriques performance
/drone_mission/alerts                   # Alertes mission
/drone_mission/data_collected           # Données scientifiques
/diagnostics                           # Diagnostics système
```

### 2. MISSION PLANNER (mission_planner.py)
**Algorithmes OBLIGATOIRES :**
- **TSP solver** (Traveling Salesman) pour optimisation
- **A* pathfinding** pour trajectoires
- **Genetic algorithm** pour optimisation complexe
- **Greedy algorithms** pour planification rapide
- **Dynamic programming** pour sous-problèmes

**Capacités de planification :**
```python
class MissionPlanner:
    def plan_single_flower_mission(self, flower_location) -> Mission
    def plan_zone_coverage_mission(self, zone_polygon) -> Mission
    def plan_multi_zone_mission(self, zones) -> Mission
    def plan_research_survey(self, grid_params) -> Mission
    def plan_emergency_return(self, current_pos) -> Mission
    
    def optimize_waypoint_order(self, waypoints) -> List[Waypoint]
    def optimize_for_time(self, mission) -> Mission
    def optimize_for_energy(self, mission) -> Mission
    def optimize_for_coverage(self, mission) -> Mission
    
    def validate_mission_safety(self, mission) -> ValidationResult
    def estimate_mission_duration(self, mission) -> float
    def estimate_energy_consumption(self, mission) -> float
```

### 3. MISSION EXECUTOR (mission_executor.py)
**Exécution OBLIGATOIRE :**
- **State machine** pour phases mission
- **Real-time adaptation** aux conditions
- **Error recovery** automatique
- **Progress tracking** détaillé
- **Coordination** avec navigation/vision

**Phases d'exécution :**
```python
class MissionExecutor:
    # États mission
    STATES = ["IDLE", "PLANNING", "STARTING", "EXECUTING", 
              "PAUSED", "COMPLETED", "ABORTED", "EMERGENCY"]
    
    async def execute_mission(self, mission: Mission)
    async def execute_takeoff_sequence(self)
    async def execute_waypoint_navigation(self, waypoint)
    async def execute_flower_approach(self, flower_target)
    async def execute_pollination_sequence(self, flower)
    async def execute_data_collection(self, location)
    async def execute_emergency_landing(self)
    async def execute_return_to_home(self)
    
    def handle_mission_failure(self, error: Exception)
    def recover_from_error(self, error_type: str)
    def adapt_to_conditions(self, weather_data)
```

### 4. MISSION MONITOR (mission_monitor.py)
**Monitoring OBLIGATOIRE :**
- **Real-time metrics** toutes performances
- **Predictive analytics** pour problèmes
- **Resource monitoring** (batterie, mémoire)
- **Environmental monitoring** conditions terrain
- **Performance benchmarking**

```python
class MissionMonitor:
    def monitor_progress(self) -> MissionProgress
    def monitor_performance(self) -> PerformanceMetrics
    def monitor_resources(self) -> ResourceStatus
    def monitor_environment(self) -> EnvironmentStatus
    
    def predict_mission_completion(self) -> float
    def predict_battery_consumption(self) -> float
    def detect_performance_anomalies(self) -> List[Anomaly]
    def generate_real_time_alerts(self) -> List[Alert]
    
    def track_flower_targets(self) -> List[FlowerTarget]
    def track_pollination_success(self) -> PollinationStats
    def track_data_collection(self) -> DataStats
```

### 5. ZONE MANAGER (zone_manager.py)
**Gestion zones OBLIGATOIRE :**
- **Zone definition** polygones géographiques
- **Zone prioritization** selon critères
- **Overlap detection** entre zones
- **Coverage optimization** pour efficacité

```python
class ZoneManager:
    def define_zone(self, points: List[GeoPoint]) -> Zone
    def validate_zone(self, zone: Zone) -> bool
    def optimize_zone_coverage(self, zone: Zone) -> CoveragePattern
    def detect_zone_overlaps(self, zones: List[Zone]) -> List[Overlap]
    
    def generate_grid_pattern(self, zone, spacing) -> List[Waypoint]
    def generate_spiral_pattern(self, zone, center) -> List[Waypoint]
    def generate_random_pattern(self, zone, density) -> List[Waypoint]
    
    def calculate_zone_metrics(self, zone) -> ZoneMetrics
    def prioritize_zones(self, zones, criteria) -> List[Zone]
```

### 6. FLOWER MISSION (flower_mission.py)
**Missions fleurs SPÉCIALISÉES :**
- **Single flower targeting** précision maximale
- **Multi-flower sequences** optimisées
- **Flower quality assessment** avant approche
- **Pollination verification** après action
- **Adaptive behavior** selon espèce

```python
class FlowerMission:
    def create_single_flower_mission(self, flower: FlowerTarget) -> Mission
    def create_multi_flower_mission(self, flowers: List[FlowerTarget]) -> Mission
    def create_species_specific_mission(self, species: str) -> Mission
    
    async def approach_flower(self, flower: FlowerTarget)
    async def assess_flower_quality(self, flower: FlowerTarget) -> QualityScore
    async def execute_pollination(self, flower: FlowerTarget) -> PollinationResult
    async def verify_pollination_success(self, flower: FlowerTarget) -> bool
    
    def adapt_approach_by_species(self, species: str) -> ApproachParams
    def optimize_multi_flower_sequence(self, flowers) -> List[FlowerTarget]
```

### 7. DATA COLLECTOR (data_collector.py)
**Collecte données SCIENTIFIQUES :**
- **GPS coordinates** haute précision
- **Timestamp synchronization** précise
- **Environmental data** (température, humidité)
- **Performance metrics** mission
- **Scientific observations** fleurs

```python
class DataCollector:
    def collect_flower_data(self, flower: FlowerTarget) -> FlowerData
    def collect_environmental_data(self) -> EnvironmentData
    def collect_performance_data(self) -> PerformanceData
    def collect_navigation_data(self) -> NavigationData
    
    def synchronize_timestamps(self, data_points)
    def validate_data_quality(self, dataset) -> ValidationReport
    def export_scientific_dataset(self, format: str) -> str
    def generate_data_summary(self) -> DataSummary
```

### 8. OPTIMIZATION ENGINE (optimization_engine.py)
**Optimisation AVANCÉE :**
- **Multi-objective optimization** (temps, énergie, précision)
- **Genetic algorithms** pour problèmes complexes
- **Simulated annealing** pour optimums locaux
- **Dynamic optimization** en temps réel

```python
class OptimizationEngine:
    def optimize_waypoint_sequence(self, waypoints) -> List[Waypoint]
    def optimize_for_multiple_objectives(self, mission, objectives) -> Mission
    def optimize_energy_efficiency(self, mission) -> Mission
    def optimize_time_to_completion(self, mission) -> Mission
    
    def genetic_algorithm_optimization(self, population_size=50) -> Solution
    def simulated_annealing_optimization(self, temp_schedule) -> Solution
    def dynamic_reoptimization(self, current_state) -> Mission
```

### 9. OUTILS CLI COMPLETS (tools/)

#### plan_mission.py
```bash
ros2 run drone_mission plan_mission --type single_flower --target "lat,lon"
ros2 run drone_mission plan_mission --type zone_coverage --zone zone1.yaml
ros2 run drone_mission plan_mission --type multi_zone --zones "zone1,zone2"
ros2 run drone_mission plan_mission --optimize time --constraints energy
```

#### execute_mission.py
```bash
ros2 run drone_mission execute_mission --file mission1.yaml
ros2 run drone_mission execute_mission --live --monitor
ros2 run drone_mission execute_mission --resume mission_state.json
```

#### mission_status.py
```bash
ros2 run drone_mission mission_status
ros2 run drone_mission mission_status --detailed --metrics
ros2 run drone_mission mission_status --export status_report.json
```

## 📊 CONFIGURATION YAML COMPLÈTE

### mission_params.yaml
```yaml
mission:
  planning:
    default_altitude: 50.0      # m
    safety_margin: 10.0         # m
    max_mission_duration: 3600  # s (1 hour)
    max_waypoints: 100
    
    optimization:
      primary_objective: "time"   # time, energy, coverage, precision
      secondary_objective: "energy"
      algorithm: "genetic"       # genetic, annealing, greedy
      
      genetic:
        population_size: 50
        generations: 100
        mutation_rate: 0.1
        crossover_rate: 0.8
      
      annealing:
        initial_temperature: 1000
        cooling_rate: 0.95
        min_temperature: 1.0
    
    constraints:
      max_flight_time: 1800       # s (30 min)
      min_battery_reserve: 0.2    # 20%
      max_wind_speed: 10.0        # m/s
      min_visibility: 1000        # m
      
  execution:
    waypoint_tolerance: 2.0       # m
    approach_speed: 2.0           # m/s
    precision_speed: 0.5          # m/s
    hover_time: 5.0               # s
    
    flower_approach:
      detection_distance: 10.0    # m
      approach_distance: 2.0      # m
      precision_distance: 0.5     # m
      verification_time: 3.0      # s
      
    error_recovery:
      max_retries: 3
      retry_delay: 10.0           # s
      fallback_altitude: 100.0    # m
      
  monitoring:
    update_frequency: 1.0         # Hz
    metrics_logging: true
    real_time_alerts: true
    
    thresholds:
      battery_warning: 0.3        # 30%
      battery_critical: 0.2       # 20%
      position_error: 5.0         # m
      altitude_error: 3.0         # m
      
  data_collection:
    auto_collect: true
    sampling_rate: 1.0            # Hz
    data_formats: ["csv", "json"]
    
    flower_data:
      gps_precision: 0.1          # m
      timestamp_precision: 0.001  # s
      environmental_sensors: true
      image_capture: true
      
    mission_data:
      performance_metrics: true
      navigation_logs: true
      error_logs: true
      resource_usage: true
      
  emergency:
    return_to_home:
      trigger_battery: 0.15       # 15%
      trigger_weather: true
      trigger_error: true
      
    emergency_landing:
      safe_zones: []              # List of safe landing zones
      min_landing_space: 10.0     # m radius
      max_descent_rate: 2.0       # m/s
      
  communication:
    telemetry_rate: 10.0          # Hz
    heartbeat_timeout: 5.0        # s
    retry_attempts: 3
    
  zones:
    default_coverage_pattern: "grid"  # grid, spiral, random
    grid_spacing: 5.0             # m
    overlap_tolerance: 1.0        # m
    
    priority_factors:
      flower_density: 0.4
      accessibility: 0.3
      weather_conditions: 0.2
      previous_coverage: 0.1
```

### optimization_config.yaml
```yaml
optimization:
  objectives:
    time:
      weight: 1.0
      target: "minimize"
      units: "seconds"
      
    energy:
      weight: 0.8
      target: "minimize"
      units: "joules"
      
    coverage:
      weight: 0.6
      target: "maximize"
      units: "percentage"
      
    precision:
      weight: 0.9
      target: "maximize"
      units: "meters"
      
  algorithms:
    genetic:
      enabled: true
      parameters:
        population_size: 50
        generations: 100
        mutation_rate: 0.1
        crossover_rate: 0.8
        elitism_rate: 0.1
        
    simulated_annealing:
      enabled: true
      parameters:
        initial_temperature: 1000
        final_temperature: 1
        cooling_rate: 0.95
        max_iterations: 1000
        
    greedy:
      enabled: true
      parameters:
        lookahead_depth: 3
        randomization: 0.1
        
  constraints:
    hard_constraints:
      - "battery_reserve >= 0.2"
      - "flight_time <= 1800"
      - "altitude >= 20"
      - "altitude <= 120"
      
    soft_constraints:
      - "wind_speed <= 8.0"
      - "visibility >= 500"
      - "temperature >= -10"
      - "temperature <= 40"
      
  performance:
    timeout: 60.0                 # s for optimization
    memory_limit: 1024            # MB
    parallel_threads: 4
    
  validation:
    simulate_mission: true
    check_safety: true
    verify_reachability: true
    estimate_resources: true
```

## 🎯 MÉTRIQUES DE PERFORMANCE

### Objectifs OBLIGATOIRES :
- **Mission success rate** : >95% missions complétées
- **Planning time** : <30s pour mission 50 waypoints
- **Execution accuracy** : <2m erreur position waypoint
- **Energy efficiency** : >80% énergie utilisée efficacement
- **Pollination rate** : >10 fleurs/heure en conditions normales
- **Data collection completeness** : >99% données critiques
- **Recovery time** : <30s après erreur non-critique

## 🧪 TESTS OBLIGATOIRES

### test_mission.py
```python
class TestMission(unittest.TestCase):
    def test_lifecycle_management(self)
    def test_mission_state_machine(self)
    def test_real_time_monitoring(self)
    def test_emergency_handling(self)
    def test_data_integrity(self)
```

### test_planner.py
```python
class TestPlanner(unittest.TestCase):
    def test_single_flower_planning(self)
    def test_zone_coverage_planning(self)
    def test_multi_zone_planning(self)
    def test_optimization_algorithms(self)
    def test_constraint_satisfaction(self)
```

### test_executor.py
```python
class TestExecutor(unittest.TestCase):
    def test_mission_execution(self)
    def test_waypoint_navigation(self)
    def test_flower_approach(self)
    def test_error_recovery(self)
    def test_adaptive_behavior(self)
```

## 🔗 INTÉGRATION SYSTÈME

### Avec drone_navigation :
- Envoi waypoints et trajectoires optimisées
- Réception confirmations navigation
- Coordination approches précision

### Avec drone_vision :
- Réception détections fleurs temps réel
- Intégration données visuelles
- Validation succès pollinisation

### Avec drone_interface :
- Monitoring état drone en continu
- Gestion modes vol selon phase mission
- Contrôle sécurité système complet

## 🚀 EXEMPLES D'UTILISATION

### Mission zone complète :
```python
# Planification et exécution mission zone
async def execute_zone_mission():
    # 1. Définition zone
    zone = zone_manager.define_zone([
        GeoPoint(lat=45.1234, lon=-73.5678),
        GeoPoint(lat=45.1244, lon=-73.5678),
        GeoPoint(lat=45.1244, lon=-73.5688),
        GeoPoint(lat=45.1234, lon=-73.5688)
    ])
    
    # 2. Planification mission
    mission = mission_planner.plan_zone_coverage_mission(
        zone=zone,
        pattern="grid",
        spacing=5.0
    )
    
    # 3. Optimisation
    optimized_mission = optimization_engine.optimize_for_multiple_objectives(
        mission=mission,
        objectives=["time", "energy", "coverage"]
    )
    
    # 4. Validation
    validation = mission_planner.validate_mission_safety(optimized_mission)
    if not validation.is_safe:
        raise Exception(f"Mission unsafe: {validation.issues}")
    
    # 5. Exécution
    result = await mission_executor.execute_mission(optimized_mission)
    
    # 6. Rapport
    report = data_collector.generate_mission_report(result)
    return report
```

### Mission fleur spécifique :
```python
# Mission ciblée sur fleur détectée
async def target_specific_flower():
    # Attendre détection vision
    flower_detection = await vision_node.wait_for_flower_detection()
    
    # Créer mission fleur
    flower_mission = flower_mission.create_single_flower_mission(
        flower=flower_detection
    )
    
    # Approche adaptée espèce
    approach_params = flower_mission.adapt_approach_by_species(
        species=flower_detection.species
    )
    
    # Exécution avec monitoring
    monitor_task = asyncio.create_task(
        mission_monitor.monitor_flower_approach(flower_detection)
    )
    
    result = await flower_mission.execute_pollination(flower_detection)
    
    # Vérification succès
    success = await flower_mission.verify_pollination_success(flower_detection)
    
    return PollinationResult(
        flower=flower_detection,
        success=success,
        data=result
    )
```

## 📚 DOCUMENTATION README.md

Le README.md doit inclure :
- **Vue d'ensemble** système mission complet
- **Architecture** planification/exécution/monitoring
- **Mission types** supportés avec exemples
- **Configuration** paramètres et optimisation
- **API reference** tous services et topics
- **Integration** avec autres packages
- **Performance** métriques et benchmarks
- **Troubleshooting** problèmes courants

---

**IMPORTANT : Ce package est le cerveau du système - il doit démontrer intelligence et robustesse exceptionnelles pour missions critiques de pollinisation autonome.**
