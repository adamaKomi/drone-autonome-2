# 📚 GUIDE DE DÉVELOPPEMENT - ARCHITECTURE MICRO-NŒUDS

## 🎯 ORDRE DE DÉVELOPPEMENT RECOMMANDÉ

### **PHASE 1 : Fondations (Semaine 1)**

#### **Jour 1-2 : Parameter Manager Node**
```bash
# Développer et tester
cd ~/ros2_ws/src/drone_navigation/drone_navigation/nodes/
# Éditer parameter_manager_node.py (FAIT)

# Tests unitaires
python3 -m pytest test/test_parameter_manager.py -v

# Test d'intégration
ros2 run drone_navigation parameter_manager_node
ros2 service call /drone_nav/parameter_manager_node/reload_config std_srvs/srv/Trigger
```

#### **Jour 3-4 : MAVROS Interface Node**
```bash
# Développer l'interface MAVROS
# Éditer mavros_interface_node.py (FAIT)

# Test avec SITL
sim_vehicle.py -v ArduCopter --map --console
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555
ros2 run drone_navigation mavros_interface_node

# Vérifier la communication
ros2 topic echo /drone_nav/drone_status
ros2 service call /drone_nav/mavros_interface_node/get_drone_status
```

#### **Jour 5-7 : Core Navigation Node**
```bash
# Développer l'orchestrateur principal
# Éditer core_navigation_node.py (FAIT)

# Test lifecycle
ros2 run drone_navigation core_navigation_node
ros2 lifecycle set /drone_nav/core_navigation_node configure
ros2 lifecycle set /drone_nav/core_navigation_node activate

# Vérifier coordination
ros2 topic echo /drone_nav/system_state
```

### **PHASE 2 : Planification (Semaine 2)**

#### **Jour 8-10 : Trajectory Planner Node**
```bash
# Implémenter les algorithmes A*, RRT*, Dijkstra
# Éditer trajectory_planner_node.py (FAIT)

# Tests d'algorithmes
python3 test_trajectory_algorithms.py

# Test de performance
ros2 run drone_navigation trajectory_planner_node
ros2 service call /drone_nav/trajectory_planner_node/plan_trajectory
```

#### **Jour 11-12 : Coverage Pattern Node**
```bash
# Développer patterns zigzag, spiral, adaptatif
touch drone_navigation/nodes/coverage_pattern_node.py

# Implementation pattern zigzag
def generate_zigzag_pattern(area, spacing, altitude):
    # Logique de génération
    pass

# Test patterns
ros2 run drone_navigation coverage_pattern_node
```

#### **Jour 13-14 : Path Optimizer Node**
```bash
# Implémenter optimisation génétique
touch drone_navigation/nodes/path_optimizer_node.py

# Algorithme génétique TSP
class GeneticOptimizer:
    def optimize_path(self, waypoints):
        # Optimisation multi-objectifs
        pass
```

### **PHASE 3 : Contrôle (Semaine 3)**

#### **Jour 15-17 : Position Controller Node**
```bash
# Utiliser le contrôleur PID existant mais modulaire
touch drone_navigation/nodes/position_controller_node.py

# Adapter le contrôleur existant
from drone_navigation.position_controller import PositionController

# Test en boucle fermée
ros2 run drone_navigation position_controller_node
```

#### **Jour 18-21 : Trajectory Follower Node**
```bash
# Développer le suiveur de trajectoire
touch drone_navigation/nodes/trajectory_follower_node.py

# Logique de suivi
class TrajectoryFollower:
    def follow_path(self, path, current_position):
        # Logique de suivi avec anticipation
        pass
```

### **PHASE 4 : Sécurité (Semaine 4)**

#### **Jour 22-24 : Geofence Monitor Node**
```bash
# Adapter le gestionnaire de géobarrières existant
touch drone_navigation/nodes/geofence_monitor_node.py

# Integration avec Shapely
from drone_navigation.geofence_manager import GeofenceManager

# Test violations
ros2 run drone_navigation geofence_monitor_node
```

#### **Jour 25-26 : Emergency Manager Node**
```bash
# Développer gestion d'urgence
touch drone_navigation/nodes/emergency_manager_node.py

# Actions d'urgence
class EmergencyManager:
    def emergency_stop(self):
        pass
    
    def return_home(self):
        pass
    
    def emergency_land(self):
        pass
```

#### **Jour 27-28 : Obstacle Avoidance Node**
```bash
# Adapter l'évitement existant
touch drone_navigation/nodes/obstacle_avoidance_node.py

# Champs de potentiel
from drone_navigation.obstacle_avoidance import ObstacleAvoidance
```

## 🔗 RELATIONS INTER-NŒUDS

### **Flux de Données Principal**

```mermaid
sequenceDiagram
    participant PM as Parameter Manager
    participant MI as MAVROS Interface
    participant CN as Core Navigation
    participant TP as Trajectory Planner
    participant PC as Position Controller
    
    PM->>CN: Configuration chargée
    MI->>CN: Position actuelle
    CN->>TP: Demande planification
    TP->>CN: Trajectoire planifiée
    CN->>PC: Commandes de mouvement
    PC->>MI: Commandes de vitesse
    MI->>CN: Statut d'exécution
```

### **Communication d'Urgence**

```mermaid
sequenceDiagram
    participant GM as Geofence Monitor
    participant OA as Obstacle Avoidance
    participant EM as Emergency Manager
    participant CN as Core Navigation
    participant MI as MAVROS Interface
    
    GM->>EM: Violation géobarrière
    OA->>EM: Obstacle critique
    EM->>CN: Alerte d'urgence
    CN->>MI: Arrêt d'urgence
    MI->>EM: Confirmation arrêt
```

## 🧪 STRATÉGIE DE TESTS

### **Tests Unitaires par Nœud**

#### **Parameter Manager**
```python
def test_parameter_loading():
    node = ParameterManagerNode()
    config = node.get_config()
    assert config.max_velocity > 0
    assert config.safety_distance > 0

def test_parameter_validation():
    # Test validation des paramètres
    pass
```

#### **MAVROS Interface**
```python
def test_mavros_connection():
    node = MavrosInterfaceNode()
    # Simuler MAVROS
    status = node.get_drone_status()
    assert status is not None

def test_command_forwarding():
    # Test transmission commandes
    pass
```

#### **Core Navigation**
```python
def test_lifecycle_transitions():
    node = CoreNavigationNode()
    # Test transitions configure->activate
    result = node.on_configure(None)
    assert result == TransitionCallbackReturn.SUCCESS

def test_emergency_handling():
    # Test gestion d'urgence
    pass
```

### **Tests d'Intégration par Couche**

#### **Test Couche Infrastructure**
```bash
# Launch file test infrastructure
ros2 launch drone_navigation test_infrastructure.launch.py

# Vérifications
ros2 service list | grep drone_nav
ros2 topic list | grep drone_nav
```

#### **Test Couche Planification**
```bash
ros2 launch drone_navigation test_planning.launch.py

# Test planification bout en bout
ros2 service call /drone_nav/trajectory_planner_node/plan_trajectory
```

#### **Test Couche Contrôle**
```bash
ros2 launch drone_navigation test_control.launch.py

# Test contrôle position
ros2 topic pub /drone_nav/cmd_pose geometry_msgs/PoseStamped "..."
```

### **Tests Système Complet**

#### **Test Navigation Complète**
```bash
# Démarrage système complet
ros2 launch drone_navigation navigation_full.launch.py simulation_mode:=true

# Test mission simple
ros2 service call /drone_nav/goto_point geometry_msgs/srv/GetPose "..."

# Vérifier exécution
ros2 topic echo /drone_nav/navigation_status
```

#### **Test Stress et Performance**
```bash
# Test charge importante
for i in {1..100}; do
    ros2 service call /drone_nav/plan_trajectory std_srvs/srv/Trigger &
done

# Monitoring performance
htop
ros2 topic hz /drone_nav/position
```

## 📋 PRÉREQUIS POUR CHAQUE NŒUD

### **Parameter Manager Node**
- **Dépendances** : Fichiers YAML de configuration
- **Services requis** : Aucun
- **Topics requis** : Aucun
- **Tests** : Validation des paramètres, rechargement

### **MAVROS Interface Node**
- **Dépendances** : MAVROS running, ArduPilot SITL
- **Services requis** : `/mavros/cmd/arming`, `/mavros/set_mode`
- **Topics requis** : `/mavros/state`, `/mavros/local_position/pose`
- **Tests** : Communication MAVROS, transmission commandes

### **Core Navigation Node**
- **Dépendances** : Parameter Manager, MAVROS Interface
- **Services requis** : Services des nœuds dépendants
- **Topics requis** : `/drone_nav/drone_status`
- **Tests** : Lifecycle, coordination, gestion d'urgence

### **Trajectory Planner Node**
- **Dépendances** : Core Navigation
- **Services requis** : `/drone_nav/get_system_status`
- **Topics requis** : `/drone_nav/position`, `/drone_nav/goal_position`
- **Tests** : Algorithmes de planification, performance

### **Position Controller Node**
- **Dépendances** : MAVROS Interface, Parameter Manager
- **Services requis** : `/drone_nav/get_parameter`
- **Topics requis** : `/drone_nav/position`, `/drone_nav/target_waypoint`
- **Tests** : Contrôle PID, précision, stabilité

## 🚦 COORDINATION SYSTÈME

### **Séquence de Démarrage Optimale**

1. **Phase Bootstrap (0-3s)**
   ```bash
   # Démarrer Parameter Manager
   ros2 run drone_navigation parameter_manager_node
   
   # Attendre configuration chargée
   sleep 1
   ```

2. **Phase Infrastructure (3-6s)**
   ```bash
   # Démarrer MAVROS Interface
   ros2 run drone_navigation mavros_interface_node
   
   # Démarrer Core Navigation
   ros2 run drone_navigation core_navigation_node
   
   # Configuration lifecycle
   ros2 lifecycle set /drone_nav/core_navigation_node configure
   ```

3. **Phase Planification (6-10s)**
   ```bash
   # Démarrer planificateurs
   ros2 run drone_navigation trajectory_planner_node &
   ros2 run drone_navigation coverage_pattern_node &
   ros2 run drone_navigation path_optimizer_node &
   ```

4. **Phase Contrôle (10-14s)**
   ```bash
   # Démarrer contrôleurs
   ros2 run drone_navigation position_controller_node &
   ros2 run drone_navigation trajectory_follower_node &
   ```

5. **Phase Sécurité (14-18s)**
   ```bash
   # Démarrer sécurité (priorité critique)
   ros2 run drone_navigation geofence_monitor_node &
   ros2 run drone_navigation emergency_manager_node &
   ros2 run drone_navigation obstacle_avoidance_node &
   ```

6. **Phase Interface (18-22s)**
   ```bash
   # Démarrer interfaces utilisateur
   ros2 run drone_navigation mission_manager_node &
   ros2 run drone_navigation status_monitor_node &
   ros2 run drone_navigation cli_bridge_node &
   ```

7. **Phase Activation (22-25s)**
   ```bash
   # Activer les nœuds lifecycle
   ros2 lifecycle set /drone_nav/core_navigation_node activate
   
   # Vérification système
   ros2 run drone_navigation system_health_check
   ```

### **Gestion des Dépendances**

#### **Attente de Services**
```python
def wait_for_dependencies(node, services, timeout=30.0):
    """Attend que les services requis soient disponibles"""
    for service_name in services:
        client = node.create_client(Trigger, service_name)
        if not client.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"Service {service_name} non disponible")
        node.get_logger().info(f"Service {service_name} disponible")
```

#### **Vérification de Santé**
```python
def check_node_health(node_name, timeout=5.0):
    """Vérifie la santé d'un nœud"""
    try:
        # Ping le nœud
        client = create_client(Trigger, f'/{node_name}/health_check')
        future = client.call_async(Trigger.Request())
        
        rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
        
        if future.result().success:
            return True
    except:
        pass
    
    return False
```

### **Mécanismes de Récupération**

#### **Redémarrage Automatique**
```bash
# Script de monitoring et redémarrage
#!/bin/bash
while true; do
    if ! ros2 node list | grep -q "core_navigation_node"; then
        echo "Redémarrage Core Navigation Node"
        ros2 run drone_navigation core_navigation_node &
        sleep 5
        ros2 lifecycle set /drone_nav/core_navigation_node configure
        ros2 lifecycle set /drone_nav/core_navigation_node activate
    fi
    sleep 10
done
```

#### **Dégradation Gracieuse**
```python
class GracefulDegradation:
    def handle_node_failure(self, failed_node):
        if failed_node == "trajectory_planner_node":
            # Basculer vers planification simple
            self.enable_simple_planning()
        elif failed_node == "obstacle_avoidance_node":
            # Activer mode sécurité renforcée
            self.enable_safety_mode()
```

## 📊 MÉTRIQUES ET KPIs

### **Métriques de Performance**

#### **Latence de Communication**
```python
def measure_communication_latency():
    start_time = time.time()
    # Appel service
    response = client.call(request)
    latency = time.time() - start_time
    
    assert latency < 0.1, f"Latence trop élevée: {latency}s"
```

#### **Débit de Messages**
```bash
# Mesurer débit topics
ros2 topic hz /drone_nav/position
ros2 topic bw /drone_nav/planned_path
```

#### **Utilisation Ressources**
```bash
# CPU et mémoire par nœud
ps aux | grep drone_navigation
htop -p $(pgrep -f drone_navigation)
```

### **Métriques de Qualité**

#### **Précision de Navigation**
- Erreur RMS position : < 0.5m
- Temps d'établissement : < 5s
- Dépassement : < 10%

#### **Fiabilité Système**
- Disponibilité : > 99%
- MTBF : > 24h
- Temps de récupération : < 30s

#### **Performance Algorithmes**
- Temps planification A* : < 100ms
- Succès planification : > 95%
- Optimisation chemin : > 90%

## 🛠️ OUTILS DE DÉVELOPPEMENT

### **Scripts de Debug**
```bash
# Debug communication inter-nœuds
ros2 run drone_navigation debug_communication

# Profiling performance
ros2 run drone_navigation profile_system

# Validation configuration
ros2 run drone_navigation validate_config
```

### **Visualisation**
```bash
# Dashboard RQT
rqt --perspective-file navigation_debug.perspective

# Graphique de nœuds
ros2 run rqt_graph rqt_graph

# Monitoring topics
ros2 run rqt_topic rqt_topic
```

Cette architecture modulaire garantit une évolutivité maximale et une maintenance simplifiée du système de navigation autonome.
