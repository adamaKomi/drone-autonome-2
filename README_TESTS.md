# DroneInterface v3.0 - Guide de Test Complet

## 📋 Vue d'ensemble

Ce guide présente toutes les fonctionnalités du système `drone_interface` et les étapes pour les tester de manière complète.

## 🚁 Architecture du système

### Services disponibles
- `/drone/arm` - Armement du drone
- `/drone/disarm` - Désarmement du drone  
- `/drone/set_mode` - Changement de mode de vol
- `/drone/safety_check` - Vérification de sécurité
- `/drone/health_check` - Vérification de santé système
- `/drone/emergency_stop` - Arrêt d'urgence

### Topics publiés
- `/drone/status` - Statut complet du drone (JSON)
- `/drone/safety_status` - Événements de sécurité
- `/diagnostics` - Diagnostics système

## 🔧 Configuration préalable

### 1. Compilation du workspace
```bash
cd /home/adama133/ros2_ws
colcon build --packages-select drone_msgs drone_interface
source install/setup.bash
```

### 2. Démarrage de MAVROS (optionnel pour tests réels)
```bash
# Terminal 1 - SITL ArduPilot
sim_vehicle.py -v ArduCopter --console --map

# Terminal 2 - MAVROS
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555
```

### 3. Lancement du DroneInterface
```bash
# Terminal 3 - DroneInterface
ros2 launch drone_interface drone_interface_launch.py
```

## 🧪 Tests automatisés

### Script de test complet
```bash
cd /home/adama133/ros2_ws
python3 test_drone_functionalities.py
```

Ce script teste automatiquement:
- ✅ Disponibilité des services
- ✅ Monitoring du statut
- ✅ Vérification de santé
- ✅ Vérification de sécurité
- ✅ Changement de mode
- ✅ Cycle armement/désarmement
- ✅ Tests de sécurité

## 🔍 Tests manuels détaillés

### 1. Vérification du statut

#### Consulter le statut en temps réel
```bash
ros2 topic echo /drone/status
```

#### Données attendues
```json
{
  "is_connected": true,
  "is_armed": false,
  "is_flying": false,
  "is_healthy": true,
  "flight_mode": "STABILIZE",
  "battery": {
    "voltage": 12.6,
    "current": 0.0,
    "percentage_remaining": 100.0
  },
  "position": {
    "x": 0.0,
    "y": 0.0,
    "z": 0.0
  },
  "safety": {
    "overall_safety_level": "SAFE"
  }
}
```

### 2. Test de santé système

```bash
ros2 service call /drone/health_check std_srvs/srv/Trigger "{}"
```

**Réponse attendue:**
```
success: true
message: "Système sain - Tous les composants opérationnels"
```

### 3. Vérification de sécurité

#### Test de sécurité pré-vol
```bash
ros2 service call /drone/safety_check drone_msgs/srv/SafetyCheck "{
  check_type: 'PREFLIGHT',
  check_all_systems: true
}"
```

**Réponse attendue:**
```
success: true
message: "Vérification sécurité réussie"
safety_score: 0.95
warnings: []
```

#### Test de sécurité en vol
```bash
ros2 service call /drone/safety_check drone_msgs/srv/SafetyCheck "{
  check_type: 'INFLIGHT',
  check_all_systems: true
}"
```

### 4. Changement de mode de vol

#### Mode GUIDED
```bash
ros2 service call /drone/set_mode drone_msgs/srv/SetFlightMode "{
  custom_mode: 'GUIDED'
}"
```

#### Mode STABILIZE
```bash
ros2 service call /drone/set_mode drone_msgs/srv/SetFlightMode "{
  custom_mode: 'STABILIZE'
}"
```

#### Mode LAND
```bash
ros2 service call /drone/set_mode drone_msgs/srv/SetFlightMode "{
  custom_mode: 'LAND'
}"
```

#### Mode RTL (Return to Launch)
```bash
ros2 service call /drone/set_mode drone_msgs/srv/SetFlightMode "{
  custom_mode: 'RTL'
}"
```

### 5. Armement du drone

#### Armement normal
```bash
ros2 service call /drone/arm drone_msgs/srv/ArmDrone "{
  arm_mode: 'NORMAL',
  force_arm: false,
  skip_preflight: false
}"
```

#### Armement forcé (si vérifications échouent)
```bash
ros2 service call /drone/arm drone_msgs/srv/ArmDrone "{
  arm_mode: 'NORMAL',
  force_arm: true,
  skip_preflight: false
}"
```

#### Mode test (bypass sécurité)
```bash
ros2 service call /drone/arm drone_msgs/srv/ArmDrone "{
  arm_mode: 'TEST',
  force_arm: false,
  skip_preflight: true
}"
```

### 6. Désarmement du drone

#### Désarmement normal
```bash
ros2 service call /drone/disarm drone_msgs/srv/DisarmDrone "{
  force_disarm: false
}"
```

#### Désarmement forcé
```bash
ros2 service call /drone/disarm drone_msgs/srv/DisarmDrone "{
  force_disarm: true
}"
```

### 7. Arrêt d'urgence

#### Arrêt immédiat
```bash
ros2 service call /drone/emergency_stop drone_msgs/srv/EmergencyStop "{
  emergency_type: 'IMMEDIATE',
  reason: 'Test arrêt urgence'
}"
```

#### Arrêt avec atterrissage
```bash
ros2 service call /drone/emergency_stop drone_msgs/srv/EmergencyStop "{
  emergency_type: 'EMERGENCY_LAND',
  reason: 'Atterrissage sécurisé'
}"
```

## 🛠️ Lifecycle du nœud

### Commandes de gestion du cycle de vie

#### Vérifier l'état actuel
```bash
ros2 lifecycle get /drone_interface
```

#### Configuration
```bash
ros2 lifecycle set /drone_interface configure
```

#### Activation
```bash
ros2 lifecycle set /drone_interface activate
```

#### Désactivation
```bash
ros2 lifecycle set /drone_interface deactivate
```

#### Nettoyage
```bash
ros2 lifecycle set /drone_interface cleanup
```

#### Arrêt
```bash
ros2 lifecycle set /drone_interface shutdown
```

## 📊 Monitoring et diagnostics

### 1. Diagnostics système
```bash
ros2 topic echo /diagnostics
```

### 2. Événements de sécurité
```bash
ros2 topic echo /drone/safety_status
```

### 3. Logs détaillés
```bash
# Logs en temps réel
ros2 run rqt_console rqt_console

# Ou via terminal
ros2 topic echo /rosout
```

## 🎯 Scénarios de test recommandés

### Scénario 1: Vérification complète du système
1. Démarrer le système
2. Vérifier le statut de base
3. Tester la santé système
4. Effectuer une vérification de sécurité
5. Changer de mode vers GUIDED
6. Tenter un armement
7. Désarmer si réussi
8. Retour en mode STABILIZE

### Scénario 2: Test de sécurité
1. Simuler des conditions d'erreur
2. Vérifier les réponses de sécurité
3. Tester l'arrêt d'urgence
4. Vérifier la récupération

### Scénario 3: Test de performance
1. Stress test des services
2. Monitoring de la latence
3. Vérification de la stabilité

## ⚠️ Problèmes connus et solutions

### Le drone ne s'arme pas
- Vérifier que MAVROS est connecté
- S'assurer que le mode est GUIDED ou STABILIZE
- Utiliser `force_arm: true` si nécessaire
- Vérifier les paramètres de sécurité ArduPilot

### Services non disponibles
- Vérifier que le nœud est activé: `ros2 lifecycle get /drone_interface`
- Redémarrer le système si nécessaire

### Problèmes de MAVROS
- Vérifier la connexion SITL
- Redémarrer MAVROS si nécessaire

## 📈 Métriques de performance attendues

- **Latence des services**: < 100ms
- **Fréquence de publication du statut**: 10Hz
- **Temps de réponse MAVROS**: < 50ms
- **Disponibilité des services**: 99.9%

## 🔄 Procédure de redémarrage complète

```bash
# 1. Arrêter tous les processus
pkill -f drone_interface
pkill -f mavros

# 2. Nettoyer l'environnement
cd /home/adama133/ros2_ws
rm -rf build/ install/ log/

# 3. Recompiler
colcon build --packages-select drone_msgs drone_interface

# 4. Redémarrer MAVROS
source install/setup.bash
ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555 &

# 5. Redémarrer DroneInterface
ros2 launch drone_interface drone_interface_launch.py
```

## 📞 Support et débogage

### Logs système
```bash
# Voir tous les logs
find ~/.ros/log -name "*.log" | head -5 | xargs cat

# Logs spécifiques au drone_interface
grep -r "drone_interface" ~/.ros/log
```

### Commandes de débogage utiles
```bash
# Lister tous les nœuds
ros2 node list

# Lister tous les topics
ros2 topic list

# Lister tous les services
ros2 service list

# Informations sur un service
ros2 service type /drone/arm
ros2 interface show drone_msgs/srv/ArmDrone
```

---

**Version**: 3.0  
**Dernière mise à jour**: 2025-09-06  
**Auteur**: Adama Komi
