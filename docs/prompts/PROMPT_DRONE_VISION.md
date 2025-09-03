# 👁️ PROMPT IA - TRANSFORMATION PACKAGE DRONE_VISION

## 🎯 OBJECTIF
Transforme complètement le package `drone_vision` en suivant EXACTEMENT la même structure, qualité et méthodologie que le package `drone_interface` fourni en exemple.

## 📋 CONTEXTE DU PROJET
Le package `drone_vision` est le **système de vision artificielle** du drone de pollinisation. Il doit fournir des capacités visuelles avancées pour :
- Détecter et classifier les fleurs avec haute précision (>90%)
- Localiser précisément les fleurs dans l'espace 3D
- Évaluer la qualité et maturité des fleurs pour optimiser la pollinisation
- Fournir du visual servoing pour approche de précision
- Collecter des données visuelles scientifiques complètes
- S'intégrer parfaitement avec navigation et mission

## 🏗️ ARCHITECTURE REQUISE

### Structure EXACTE à implémenter :
```
drone_vision/
├── drone_vision/                       # Package Python principal
│   ├── __init__.py
│   ├── vision_node.py                  # Nœud principal avec lifecycle ROS2
│   ├── flower_detector.py              # Détecteur de fleurs avancé
│   ├── flower_classifier.py            # Classificateur espèces ML
│   ├── flower_analyzer.py              # Analyseur qualité/maturité
│   ├── visual_servoing.py              # Contrôle visuel pour précision
│   ├── stereo_vision.py                # Vision stéréo pour profondeur
│   ├── image_processor.py              # Traitement d'images avancé
│   ├── calibration_manager.py          # Calibration caméras
│   ├── tracking_system.py              # Suivi temporel fleurs
│   └── tools/                          # Outils CLI complets
│       ├── __init__.py
│       ├── detect_flowers.py           # Détection CLI
│       ├── classify_image.py           # Classification CLI
│       ├── calibrate_camera.py         # Calibration CLI
│       ├── vision_status.py            # État vision CLI
│       ├── capture_dataset.py          # Capture données CLI
│       └── vision_diagnostics.py       # Diagnostics vision CLI
├── launch/                             # Fichiers de lancement
│   └── vision_launch.py                # Lancement complet vision
├── config/                             # Configuration YAML
│   ├── vision_params.yaml              # Paramètres principaux
│   ├── detection_models.yaml           # Configuration modèles
│   ├── camera_calibration.yaml         # Calibration caméras
│   └── flower_database.yaml            # Base données espèces
├── models/                             # Modèles IA/ML
│   ├── flower_classification.onnx      # Modèle classification
│   ├── flower_detection.pt            # Modèle détection YOLO
│   └── quality_assessment.pkl          # Modèle évaluation qualité
├── test/                               # Tests complets
│   ├── test_vision.py                  # Tests vision
│   ├── test_detection.py               # Tests détection
│   ├── test_classification.py          # Tests classification
│   ├── test_calibration.py             # Tests calibration
│   └── test_tracking.py                # Tests suivi
├── scripts/                            # Scripts utilitaires
│   ├── train_classifier.py             # Entraînement modèles
│   ├── evaluate_models.py              # Évaluation performance
│   └── create_dataset.py               # Création datasets
├── resource/                           # Ressources package
│   └── drone_vision
├── setup.py                           # Configuration Python
├── package.xml                        # Métadonnées ROS2
└── README.md                          # Documentation exhaustive
```

## 🔧 SPÉCIFICATIONS TECHNIQUES DÉTAILLÉES

### 1. VISION NODE (vision_node.py)
**Caractéristiques obligatoires :**
- **Lifecycle management complet** (configure/activate/deactivate/cleanup)
- **Multi-threading** pour traitement temps réel (>10 FPS)
- **Multiple cameras support** (RGB, stereo, thermal)
- **Real-time processing** avec optimisations GPU
- **Memory management** intelligent pour images HD

**Services exposés (OBLIGATOIRES) :**
```python
/drone_vision/detect_flowers         # Détection fleurs dans image
/drone_vision/classify_flower        # Classification espèce fleur
/drone_vision/analyze_quality        # Analyse qualité fleur
/drone_vision/track_flower           # Suivi fleur spécifique
/drone_vision/visual_servo           # Contrôle visuel position
/drone_vision/calibrate_camera       # Calibration caméra
/drone_vision/capture_image          # Capture image manuelle
/drone_vision/start_recording        # Démarrer enregistrement
/drone_vision/stop_recording         # Arrêter enregistrement
/drone_vision/get_3d_position        # Position 3D fleur
```

**Topics publiés (OBLIGATOIRES) :**
```python
/drone_vision/detections            # Détections temps réel
/drone_vision/classifications       # Classifications espèces
/drone_vision/tracked_flowers       # Fleurs suivies
/drone_vision/visual_servo_error    # Erreur visual servoing
/drone_vision/image_processed       # Images traitées
/drone_vision/stereo_pointcloud     # Nuage de points stéréo
/drone_vision/quality_assessment    # Évaluations qualité
/diagnostics                        # Diagnostics système
```

### 2. FLOWER DETECTOR (flower_detector.py)
**Algorithmes OBLIGATOIRES :**
- **YOLO v8** pour détection temps réel
- **Mask R-CNN** pour segmentation précise
- **Color-based detection** HSV pour robustesse
- **Edge detection** Canny pour contours
- **Blob detection** pour formes circulaires

**Méthodes de détection :**
```python
class FlowerDetector:
    def detect_yolo(self, image) -> List[Detection]
    def detect_color_based(self, image, color_ranges) -> List[Detection]
    def detect_shape_based(self, image) -> List[Detection]
    def detect_hybrid(self, image) -> List[Detection]
    def filter_detections(self, detections, criteria) -> List[Detection]
    def merge_overlapping(self, detections) -> List[Detection]
```

### 3. FLOWER CLASSIFIER (flower_classifier.py)
**Classifications OBLIGATOIRES :**
- **Espèces de fleurs** (>50 espèces communes)
- **Couleurs** (rouge, jaune, rose, blanc, violet, etc.)
- **Tailles** (petit, moyen, grand)
- **États** (bourgeon, ouvert, fané)
- **Confiance** score 0-1 pour chaque classification

**Modèles supportés :**
```python
class FlowerClassifier:
    def load_model(self, model_type: str)  # "resnet", "efficientnet", "vit"
    def classify_species(self, flower_image) -> SpeciesResult
    def classify_color(self, flower_image) -> ColorResult
    def classify_state(self, flower_image) -> StateResult
    def batch_classify(self, images) -> List[ClassificationResult]
    def fine_tune_model(self, training_data)
```

### 4. FLOWER ANALYZER (flower_analyzer.py)
**Analyses OBLIGATOIRES :**
- **Qualité de la fleur** (santé, vitalité)
- **Maturité** (prête pour pollinisation)
- **Défauts** (maladies, dégâts)
- **Attractivité pour pollinisateurs**
- **Recommandations d'action**

```python
class FlowerAnalyzer:
    def analyze_health(self, flower_image) -> HealthScore
    def assess_maturity(self, flower_image) -> MaturityLevel
    def detect_diseases(self, flower_image) -> List[Disease]
    def evaluate_pollination_readiness(self, flower_data) -> ReadinessScore
    def generate_recommendations(self, analysis) -> List[Action]
```

### 5. VISUAL SERVOING (visual_servoing.py)
**Contrôle visuel OBLIGATOIRE :**
- **Position-based visual servoing (PBVS)**
- **Image-based visual servoing (IBVS)**
- **Hybrid visual servoing**
- **Kalman filtering** pour prédiction
- **PID controllers** pour stabilité

```python
class VisualServoController:
    def pbvs_control(self, target_pose, current_pose) -> ControlCommand
    def ibvs_control(self, target_features, current_features) -> ControlCommand
    def hybrid_control(self, target, current) -> ControlCommand
    def predict_motion(self, tracking_history) -> PredictedMotion
    def stabilize_approach(self, flower_target) -> StabilizationCommand
```

### 6. STEREO VISION (stereo_vision.py)
**Vision stéréo OBLIGATOIRE :**
- **Calibration stereo** automatique
- **Disparity mapping** en temps réel
- **3D reconstruction** nuages de points
- **Depth estimation** pour fleurs
- **Obstacle detection** 3D

```python
class StereoVisionSystem:
    def calibrate_stereo_pair(self, left_images, right_images)
    def compute_disparity(self, left_img, right_img) -> DisparityMap
    def reconstruct_3d(self, disparity_map) -> PointCloud
    def estimate_flower_depth(self, flower_detection) -> float
    def detect_3d_obstacles(self, pointcloud) -> List[Obstacle]
```

### 7. OUTILS CLI COMPLETS (tools/)

#### detect_flowers.py
```bash
ros2 run drone_vision detect_flowers --image image.jpg
ros2 run drone_vision detect_flowers --camera 0 --live
ros2 run drone_vision detect_flowers --method yolo --confidence 0.8
```

#### classify_image.py
```bash
ros2 run drone_vision classify_image --image flower.jpg
ros2 run drone_vision classify_image --batch images/ --output results.json
ros2 run drone_vision classify_image --species --color --state
```

#### calibrate_camera.py
```bash
ros2 run drone_vision calibrate_camera --camera 0 --pattern chessboard
ros2 run drone_vision calibrate_camera --stereo --left 0 --right 1
ros2 run drone_vision calibrate_camera --save calibration.yaml
```

## 📊 CONFIGURATION YAML COMPLÈTE

### vision_params.yaml
```yaml
vision:
  cameras:
    main:
      device_id: 0
      resolution: [1920, 1080]
      fps: 30
      exposure: auto
      gain: auto
      white_balance: auto
    stereo_left:
      device_id: 1
      resolution: [1280, 720]
      fps: 30
    stereo_right:
      device_id: 2
      resolution: [1280, 720]
      fps: 30
  
  detection:
    methods:
      primary: "yolo"        # yolo, color, shape, hybrid
      fallback: "color"
      ensemble: true
    
    yolo:
      model_path: "models/flower_detection.pt"
      confidence_threshold: 0.7
      iou_threshold: 0.5
      max_detections: 50
    
    color_detection:
      color_ranges:
        red: [[0, 100, 100], [10, 255, 255]]
        yellow: [[20, 100, 100], [30, 255, 255]]
        pink: [[140, 100, 100], [170, 255, 255]]
        white: [[0, 0, 200], [180, 30, 255]]
        purple: [[120, 100, 100], [140, 255, 255]]
      
      morphology:
        kernel_size: 5
        iterations: 2
      
      filtering:
        min_area: 500
        max_area: 50000
        min_circularity: 0.3
        min_convexity: 0.7
  
  classification:
    models:
      species:
        path: "models/flower_classification.onnx"
        input_size: [224, 224]
        num_classes: 52
        confidence_threshold: 0.8
      
      quality:
        path: "models/quality_assessment.pkl"
        features: ["color_histogram", "texture", "shape"]
        threshold: 0.6
    
    preprocessing:
      normalize: true
      augmentation: false
      resize_method: "bilinear"
  
  tracking:
    algorithm: "kalman"      # kalman, particle, optical_flow
    max_tracks: 10
    track_lifetime: 30       # frames
    association_threshold: 0.7
    
    kalman:
      process_noise: 0.1
      measurement_noise: 0.5
      initial_uncertainty: 1.0
  
  visual_servoing:
    control_method: "hybrid"  # pbvs, ibvs, hybrid
    pid_gains:
      p: [1.0, 1.0, 1.0]     # x, y, z
      i: [0.1, 0.1, 0.1]
      d: [0.05, 0.05, 0.05]
    
    target_size: 0.1          # m (diameter of flower)
    approach_distance: 1.0    # m
    stabilization_time: 2.0   # s
  
  stereo:
    enabled: true
    baseline: 0.12            # m
    focal_length: 500         # pixels
    disparity_range: [0, 128]
    block_size: 15
    uniqueness_ratio: 10
  
  performance:
    processing_threads: 4
    gpu_acceleration: true
    memory_limit: 2048        # MB
    queue_size: 10
    
    target_fps: 15
    max_latency: 100          # ms
    
  quality:
    save_detections: true
    save_classifications: true
    save_raw_images: false
    
    output_directory: "/tmp/drone_vision"
    image_format: "jpg"
    compression_quality: 85
```

## 🧠 MODÈLES IA/ML INTÉGRÉS

### Modèles OBLIGATOIRES :
1. **YOLOv8** pour détection temps réel
2. **ResNet50** pour classification espèces
3. **EfficientNet** pour évaluation qualité
4. **Vision Transformer** pour analyse fine
5. **Custom CNN** pour maturité fleurs

### Pipeline ML :
```python
class VisionPipeline:
    def __init__(self):
        self.detector = YOLODetector("models/flower_detection.pt")
        self.classifier = ResNetClassifier("models/species_classification.onnx")
        self.quality_assessor = QualityNet("models/quality_assessment.pkl")
        
    def process_frame(self, image):
        # 1. Détection
        detections = self.detector.detect(image)
        
        # 2. Classification pour chaque détection
        results = []
        for detection in detections:
            flower_crop = self.crop_flower(image, detection.bbox)
            
            species = self.classifier.classify(flower_crop)
            quality = self.quality_assessor.assess(flower_crop)
            
            results.append(FlowerResult(
                detection=detection,
                species=species,
                quality=quality,
                timestamp=time.time()
            ))
        
        return results
```

## 🧪 TESTS OBLIGATOIRES

### test_vision.py
```python
class TestVision(unittest.TestCase):
    def test_lifecycle_management(self)
    def test_camera_initialization(self)
    def test_real_time_processing(self)
    def test_memory_management(self)
    def test_performance_metrics(self)
```

### test_detection.py
```python
class TestDetection(unittest.TestCase):
    def test_yolo_detection(self)
    def test_color_detection(self)
    def test_detection_accuracy(self)
    def test_detection_speed(self)
    def test_false_positive_rate(self)
```

## 🎯 MÉTRIQUES DE PERFORMANCE

### Objectifs OBLIGATOIRES :
- **Précision détection** : >90% sur dataset test
- **Vitesse traitement** : >10 FPS en temps réel
- **Classification accuracy** : >85% espèces communes
- **Visual servoing precision** : ±5cm erreur position
- **Latence totale** : <100ms detection → action
- **Memory usage** : <2GB avec GPU
- **CPU usage** : <50% sur processeur mobile

## 🔗 INTÉGRATION SYSTÈME

### Avec drone_navigation :
- Positions 3D des fleurs détectées
- Visual servoing pour approche précision
- Correction trajectoire basée vision

### Avec drone_mission :
- Rapport détections en temps réel
- Validation réussite pollinisation
- Collecte données scientifiques

### Avec drone_interface :
- Contrôle caméras et capteurs
- Synchronisation avec télémétrie
- Gestion modes de vol vision

## 🚀 EXEMPLES D'UTILISATION

### Détection en vol :
```python
# Pipeline détection temps réel
async def continuous_detection():
    while mission_active:
        frame = await camera.capture_frame()
        detections = await vision_node.detect_flowers(frame)
        
        for flower in detections:
            if flower.confidence > 0.8:
                # Position 3D de la fleur
                position_3d = await vision_node.get_3d_position(flower)
                
                # Classification espèce
                species = await vision_node.classify_flower(flower)
                
                # Analyse qualité
                quality = await vision_node.analyze_quality(flower)
                
                # Envoi à navigation pour approche
                await navigation_node.add_flower_target(
                    position=position_3d,
                    species=species,
                    quality=quality
                )
```

### Visual servoing pour pollinisation :
```python
# Contrôle visuel précis
async def precision_approach(flower_target):
    servo_controller = VisualServoController()
    
    while not approach_complete:
        current_frame = await camera.capture_frame()
        flower_position = await vision_node.track_flower(
            current_frame, flower_target.id
        )
        
        # Calcul commande contrôle
        control_command = servo_controller.compute_control(
            target_position=flower_target.position,
            current_position=flower_position,
            desired_distance=1.0  # 1m au-dessus
        )
        
        # Envoi commande navigation
        await navigation_node.execute_control_command(control_command)
        
        # Vérification précision
        if control_command.error < 0.05:  # 5cm
            approach_complete = True
```

## 📚 DOCUMENTATION README.md

Le README.md doit inclure :
- **Vue d'ensemble** du système vision
- **Capabilities** détaillées (détection, classification, tracking)
- **Installation** et dépendances (OpenCV, PyTorch, ONNX)
- **Configuration** caméras et modèles
- **Exemples pratiques** d'utilisation
- **Performance benchmarks**
- **Troubleshooting** commun
- **API reference** complète

---

**IMPORTANT : Ce package doit avoir la même qualité exceptionnelle que drone_interface, avec focus sur performance temps réel et précision pour mission critique de pollinisation.**
