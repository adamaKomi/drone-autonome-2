# 🎯 MVP - Système Drone de Pollinisation Autonome

## 📋 **Objectif MVP**

Développer un **Produit Minimum Viable** fonctionnel pour valider le concept de pollinisation par drone en simulation complète (ROS2 + ArduPilot SITL).

**Délai cible : 6-8 semaines**

---

## 🏗️ **Architecture MVP Simplifiée**

### **Structure des Packages**
```
📦 mvp_drone_pollination/
├── 🚁 mvp_drone_control/          # Contrôle drone basique
├── 🗺️ mvp_mission_planner/        # Planification simple
├── 👁️ mvp_flower_detector/        # Détection visuelle
├── 🌸 mvp_pollinator/             # Simulation pollinisation
├── 📊 mvp_data_collector/         # Collecte données
└── 🌐 mvp_web_interface/          # Interface web minimale
```

---

## 🔧 **Composants MVP Détaillés**

### **1. MVP Drone Control**

#### **Fonctionnalités Essentielles**
- ✅ Décollage/atterrissage automatique
- ✅ Navigation par waypoints GPS
- ✅ Communication MAVROS stable
- ✅ Mode RTL (Return To Launch) automatique
- ✅ Surveillance batterie basique

#### **Code MVP Drone Controller**
```python
#!/usr/bin/env python3
"""
MVP Drone Controller - Contrôle de base du drone
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from mavros_msgs.srv import CommandBool, SetMode, CommandTOL
import time

class MVPDroneController(Node):
    """Contrôleur drone MVP - fonctionnalités essentielles"""
    
    def __init__(self):
        super().__init__('mvp_drone_controller')
        
        # État du drone
        self.current_state = State()
        self.current_pose = PoseStamped()
        
        # Services MAVROS
        self.arming_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        self.set_mode_client = self.create_client(SetMode, '/mavros/set_mode')
        self.takeoff_client = self.create_client(CommandTOL, '/mavros/cmd/takeoff')
        self.land_client = self.create_client(CommandTOL, '/mavros/cmd/land')
        
        # Subscribers
        self.state_sub = self.create_subscription(
            State, '/mavros/state', self.state_callback, 10
        )
        self.pose_sub = self.create_subscription(
            PoseStamped, '/mavros/local_position/pose', self.pose_callback, 10
        )
        
        # Publisher pour commandes de position
        self.position_pub = self.create_publisher(
            PoseStamped, '/mavros/setpoint_position/local', 10
        )
        
        self.get_logger().info("🚁 MVP Drone Controller initialisé")
        
    def state_callback(self, msg):
        self.current_state = msg
        
    def pose_callback(self, msg):
        self.current_pose = msg
        
    def arm_drone(self):
        """Arme le drone"""
        if not self.current_state.armed:
            arm_cmd = CommandBool.Request()
            arm_cmd.value = True
            
            future = self.arming_client.call_async(arm_cmd)
            rclpy.spin_until_future_complete(self, future)
            
            if future.result().success:
                self.get_logger().info("✅ Drone armé")
                return True
            else:
                self.get_logger().error("❌ Échec armement")
                return False
        return True
        
    def takeoff(self, altitude=10.0):
        """Décollage automatique"""
        # 1. Vérifier que le drone est armé
        if not self.arm_drone():
            return False
            
        # 2. Passer en mode GUIDED
        if not self.set_mode("GUIDED"):
            return False
            
        # 3. Commande de décollage
        takeoff_cmd = CommandTOL.Request()
        takeoff_cmd.altitude = altitude
        
        future = self.takeoff_client.call_async(takeoff_cmd)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info(f"🚀 Décollage à {altitude}m en cours...")
            
            # Attendre que l'altitude soit atteinte
            while abs(self.current_pose.pose.position.z - altitude) > 1.0:
                time.sleep(0.5)
                rclpy.spin_once(self, timeout_sec=0.1)
                
            self.get_logger().info("✅ Altitude de mission atteinte")
            return True
        else:
            self.get_logger().error("❌ Échec décollage")
            return False
            
    def land(self):
        """Atterrissage automatique"""
        land_cmd = CommandTOL.Request()
        
        future = self.land_client.call_async(land_cmd)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info("🛬 Atterrissage en cours...")
            return True
        else:
            self.get_logger().error("❌ Échec atterrissage")
            return False
            
    def set_mode(self, mode):
        """Change le mode de vol"""
        mode_cmd = SetMode.Request()
        mode_cmd.custom_mode = mode
        
        future = self.set_mode_client.call_async(mode_cmd)
        rclpy.spin_until_future_complete(self, future)
        
        return future.result().mode_sent
        
    def goto_position(self, x, y, z):
        """Navigation vers une position locale"""
        target = PoseStamped()
        target.header.stamp = self.get_clock().now().to_msg()
        target.header.frame_id = "map"
        target.pose.position.x = x
        target.pose.position.y = y
        target.pose.position.z = z
        
        # Publier la commande
        self.position_pub.publish(target)
        
        # Attendre d'atteindre la position (tolérance 1m)
        while self.distance_to_target(target) > 1.0:
            self.position_pub.publish(target)
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.1)
            
        self.get_logger().info(f"✅ Position atteinte: ({x:.1f}, {y:.1f}, {z:.1f})")
        
    def distance_to_target(self, target):
        """Calcule la distance à la cible"""
        dx = self.current_pose.pose.position.x - target.pose.position.x
        dy = self.current_pose.pose.position.y - target.pose.position.y
        dz = self.current_pose.pose.position.z - target.pose.position.z
        return (dx*dx + dy*dy + dz*dz)**0.5
```

### **2. MVP Mission Planner**

#### **Fonctionnalités**
- ✅ Définition de zone rectangulaire
- ✅ Génération pattern zigzag
- ✅ Calcul automatique de waypoints
- ✅ Estimation temps de mission

#### **Code MVP Mission Planner**
```python
#!/usr/bin/env python3
"""
MVP Mission Planner - Planification simple de missions
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point
import math

class MVPMissionPlanner(Node):
    """Planificateur de mission MVP"""
    
    def __init__(self):
        super().__init__('mvp_mission_planner')
        
        # Paramètres par défaut
        self.default_altitude = 20.0  # mètres
        self.default_speed = 5.0      # m/s
        self.line_spacing = 10.0      # mètres entre lignes
        
        self.get_logger().info("🗺️ MVP Mission Planner initialisé")
        
    def plan_rectangular_mission(self, corner1, corner2, altitude=None):
        """
        Planifie une mission sur zone rectangulaire
        
        Args:
            corner1: Point(x, y, z) - Premier coin
            corner2: Point(x, y, z) - Coin opposé
            altitude: Altitude de vol (optionnel)
            
        Returns:
            waypoints: Liste de points de passage
            mission_info: Informations sur la mission
        """
        if altitude is None:
            altitude = self.default_altitude
            
        # Calcul des limites de la zone
        min_x = min(corner1.x, corner2.x)
        max_x = max(corner1.x, corner2.x)
        min_y = min(corner1.y, corner2.y)
        max_y = max(corner1.y, corner2.y)
        
        # Génération du pattern zigzag
        waypoints = []
        current_y = min_y
        direction = 1  # 1 = droite, -1 = gauche
        
        while current_y <= max_y:
            if direction == 1:
                # Ligne de gauche à droite
                start_x, end_x = min_x, max_x
            else:
                # Ligne de droite à gauche
                start_x, end_x = max_x, min_x
                
            # Ajouter les points de la ligne
            waypoints.append(Point(x=start_x, y=current_y, z=altitude))
            waypoints.append(Point(x=end_x, y=current_y, z=altitude))
            
            # Ligne suivante
            current_y += self.line_spacing
            direction *= -1
            
        # Calcul des informations de mission
        total_distance = self.calculate_total_distance(waypoints)
        estimated_time = total_distance / self.default_speed
        
        mission_info = {
            'total_waypoints': len(waypoints),
            'total_distance': total_distance,
            'estimated_time': estimated_time,
            'area_covered': (max_x - min_x) * (max_y - min_y),
            'pattern': 'zigzag'
        }
        
        self.get_logger().info(f"📋 Mission planifiée: {len(waypoints)} waypoints, "
                              f"{estimated_time:.1f}s estimées")
        
        return waypoints, mission_info
        
    def calculate_total_distance(self, waypoints):
        """Calcule la distance totale du parcours"""
        total = 0.0
        for i in range(1, len(waypoints)):
            dx = waypoints[i].x - waypoints[i-1].x
            dy = waypoints[i].y - waypoints[i-1].y
            total += math.sqrt(dx*dx + dy*dy)
        return total
        
    def optimize_waypoint_order(self, waypoints, flower_positions):
        """Optimise l'ordre des waypoints selon les fleurs détectées"""
        # Version MVP: simple réorganisation par proximité
        # Version avancée: algorithme du voyageur de commerce
        
        if not flower_positions:
            return waypoints
            
        # Pour le MVP, on priorise les waypoints proches des fleurs
        prioritized_waypoints = []
        remaining_waypoints = waypoints.copy()
        
        for flower_pos in flower_positions:
            # Trouver le waypoint le plus proche de cette fleur
            closest_wp = min(remaining_waypoints, 
                           key=lambda wp: self.distance_2d(wp, flower_pos))
            prioritized_waypoints.append(closest_wp)
            remaining_waypoints.remove(closest_wp)
            
        # Ajouter les waypoints restants
        prioritized_waypoints.extend(remaining_waypoints)
        
        return prioritized_waypoints
        
    def distance_2d(self, point1, point2):
        """Distance 2D entre deux points"""
        dx = point1.x - point2.x
        dy = point1.y - point2.y
        return math.sqrt(dx*dx + dy*dy)
```

### **3. MVP Flower Detector**

#### **Fonctionnalités**
- ✅ Détection par couleur (3-4 couleurs)
- ✅ Filtrage par taille et forme
- ✅ Calcul de confiance simple
- ✅ Localisation 2D dans l'image

#### **Code MVP Flower Detector**
```python
#!/usr/bin/env python3
"""
MVP Flower Detector - Détection simple mais efficace
"""

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from cv_bridge import CvBridge
import json
import time

class MVPFlowerDetector(Node):
    """Détecteur de fleurs MVP"""
    
    def __init__(self):
        super().__init__('mvp_flower_detector')
        
        self.cv_bridge = CvBridge()
        
        # Couleurs de fleurs à détecter (HSV)
        self.flower_colors = {
            'red': {
                'ranges': [
                    (np.array([0, 100, 100]), np.array([10, 255, 255])),
                    (np.array([170, 100, 100]), np.array([180, 255, 255]))
                ],
                'priority': 1
            },
            'yellow': {
                'ranges': [(np.array([20, 100, 100]), np.array([30, 255, 255]))],
                'priority': 2
            },
            'pink': {
                'ranges': [(np.array([140, 100, 100]), np.array([170, 255, 255]))],
                'priority': 3
            }
        }
        
        # Paramètres de détection
        self.min_area = 500          # pixels²
        self.max_area = 50000        # pixels²
        self.min_confidence = 0.6    # seuil de confiance
        
        # ROS2 interface
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10
        )
        
        self.flowers_pub = self.create_publisher(
            String, '/mvp/flowers_detected', 10
        )
        
        self.debug_image_pub = self.create_publisher(
            Image, '/mvp/debug_image', 10
        )
        
        self.detection_active = True
        self.detection_count = 0
        
        self.get_logger().info("👁️ MVP Flower Detector initialisé")
        
    def image_callback(self, msg):
        """Traite les images de la caméra"""
        if not self.detection_active:
            return
            
        try:
            # Conversion ROS -> OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Détection des fleurs
            detections = self.detect_flowers(cv_image)
            
            # Publication des résultats
            if detections:
                self.publish_detections(detections, msg.header)
                
            # Image de debug
            debug_image = self.draw_detections(cv_image, detections)
            self.publish_debug_image(debug_image, msg.header)
            
        except Exception as e:
            self.get_logger().error(f"❌ Erreur traitement image: {e}")
            
    def detect_flowers(self, image):
        """Détecte les fleurs dans une image"""
        detections = []
        
        # Conversion en HSV
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        height, width = image.shape[:2]
        
        for color_name, color_config in self.flower_colors.items():
            # Création du masque pour cette couleur
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            
            for lower, upper in color_config['ranges']:
                color_mask = cv2.inRange(hsv, lower, upper)
                mask = cv2.bitwise_or(mask, color_mask)
                
            # Nettoyage du masque
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Détection de contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                area = cv2.contourArea(contour)
                
                # Filtrage par taille
                if self.min_area <= area <= self.max_area:
                    # Calcul des propriétés
                    moments = cv2.moments(contour)
                    if moments['m00'] > 0:
                        cx = int(moments['m10'] / moments['m00'])
                        cy = int(moments['m01'] / moments['m00'])
                        
                        # Calcul de la confiance
                        confidence = self.calculate_confidence(contour, area)
                        
                        if confidence >= self.min_confidence:
                            self.detection_count += 1
                            
                            detection = {
                                'id': f"{color_name}_{self.detection_count}_{int(time.time()*1000)}",
                                'color': color_name,
                                'center_x': cx,
                                'center_y': cy,
                                'area': area,
                                'confidence': confidence,
                                'priority': color_config['priority'],
                                'image_width': width,
                                'image_height': height,
                                'timestamp': time.time()
                            }
                            
                            detections.append(detection)
                            
        # Tri par priorité et confiance
        detections.sort(key=lambda d: (d['priority'], -d['confidence']))
        
        return detections
        
    def calculate_confidence(self, contour, area):
        """Calcule la confiance de détection basée sur la forme"""
        # Calcul de la circularité
        perimeter = cv2.arcLength(contour, True)
        if perimeter == 0:
            return 0.0
            
        circularity = 4 * np.pi * area / (perimeter * perimeter)
        
        # Calcul du ratio d'aspect
        x, y, w, h = cv2.boundingRect(contour)
        aspect_ratio = float(w) / h if h > 0 else 0
        aspect_score = 1.0 - abs(1.0 - aspect_ratio)  # Proche de 1 = carré/rond
        
        # Score de remplissage
        bounding_area = w * h
        fill_ratio = area / bounding_area if bounding_area > 0 else 0
        
        # Combinaison des scores
        confidence = (circularity * 0.4 + aspect_score * 0.3 + fill_ratio * 0.3)
        return min(confidence, 1.0)
        
    def draw_detections(self, image, detections):
        """Dessine les détections sur l'image"""
        debug_image = image.copy()
        
        for detection in detections:
            color_map = {
                'red': (0, 0, 255),
                'yellow': (0, 255, 255),
                'pink': (255, 0, 255)
            }
            
            color = color_map.get(detection['color'], (0, 255, 0))
            center = (detection['center_x'], detection['center_y'])
            
            # Cercle principal
            radius = int(np.sqrt(detection['area'] / np.pi))
            cv2.circle(debug_image, center, radius, color, 2)
            cv2.circle(debug_image, center, 3, color, -1)
            
            # Texte d'information
            text = f"{detection['color']} ({detection['confidence']:.2f})"
            cv2.putText(debug_image, text, 
                       (center[0] - 50, center[1] - radius - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
                       
        # Informations générales
        info_text = f"Fleurs detectees: {len(detections)}"
        cv2.putText(debug_image, info_text, (10, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                   
        return debug_image
        
    def publish_detections(self, detections, header):
        """Publie les détections trouvées"""
        detection_msg = {
            'timestamp': time.time(),
            'header': {
                'stamp': header.stamp.sec + header.stamp.nanosec * 1e-9,
                'frame_id': header.frame_id
            },
            'count': len(detections),
            'flowers': detections
        }
        
        msg = String()
        msg.data = json.dumps(detection_msg)
        self.flowers_pub.publish(msg)
        
        if detections:
            self.get_logger().info(f"🌸 {len(detections)} fleur(s) détectée(s)")
            
    def publish_debug_image(self, image, header):
        """Publie l'image de debug"""
        try:
            debug_msg = self.cv_bridge.cv2_to_imgmsg(image, "bgr8")
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
        except Exception as e:
            self.get_logger().error(f"❌ Erreur publication debug: {e}")
```

### **4. MVP Web Interface**

#### **Fonctionnalités**
- ✅ Carte interactive avec sélection de zone
- ✅ Contrôles mission (start/stop/pause)
- ✅ Affichage position drone temps réel
- ✅ Liste des fleurs détectées
- ✅ Métriques de mission basiques

#### **Code MVP Web Interface**
```html
<!DOCTYPE html>
<html lang="fr">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>🌸 MVP Drone Pollinisation</title>
    <link rel="stylesheet" href="https://unpkg.com/leaflet/dist/leaflet.css" />
    <style>
        body {
            font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f5f5f5;
        }
        
        .container {
            max-width: 1200px;
            margin: 0 auto;
            background: white;
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
        }
        
        .header {
            text-align: center;
            margin-bottom: 30px;
            color: #2c3e50;
        }
        
        .controls {
            display: flex;
            gap: 15px;
            margin-bottom: 20px;
            align-items: center;
        }
        
        .btn {
            padding: 10px 20px;
            border: none;
            border-radius: 5px;
            cursor: pointer;
            font-weight: bold;
            transition: all 0.3s;
        }
        
        .btn-primary { background: #3498db; color: white; }
        .btn-success { background: #27ae60; color: white; }
        .btn-warning { background: #f39c12; color: white; }
        .btn-danger { background: #e74c3c; color: white; }
        
        .btn:hover { transform: translateY(-2px); }
        .btn:disabled { opacity: 0.5; cursor: not-allowed; }
        
        .status-panel {
            display: grid;
            grid-template-columns: 1fr 1fr 1fr;
            gap: 20px;
            margin-bottom: 20px;
        }
        
        .status-card {
            background: #ecf0f1;
            padding: 15px;
            border-radius: 8px;
            text-align: center;
        }
        
        .status-value {
            font-size: 24px;
            font-weight: bold;
            color: #2c3e50;
        }
        
        .status-label {
            color: #7f8c8d;
            font-size: 14px;
        }
        
        .main-content {
            display: grid;
            grid-template-columns: 2fr 1fr;
            gap: 20px;
        }
        
        #map {
            height: 500px;
            border-radius: 8px;
            border: 2px solid #bdc3c7;
        }
        
        .sidebar {
            display: flex;
            flex-direction: column;
            gap: 20px;
        }
        
        .flower-list {
            background: #f8f9fa;
            border-radius: 8px;
            padding: 15px;
            max-height: 300px;
            overflow-y: auto;
        }
        
        .flower-item {
            display: flex;
            justify-content: space-between;
            padding: 8px;
            margin: 5px 0;
            background: white;
            border-radius: 5px;
            border-left: 4px solid;
        }
        
        .flower-red { border-left-color: #e74c3c; }
        .flower-yellow { border-left-color: #f1c40f; }
        .flower-pink { border-left-color: #e91e63; }
        
        .log-panel {
            background: #2c3e50;
            color: #ecf0f1;
            padding: 15px;
            border-radius: 8px;
            height: 200px;
            overflow-y: auto;
            font-family: 'Courier New', monospace;
            font-size: 12px;
        }
        
        .connection-status {
            position: fixed;
            top: 20px;
            right: 20px;
            padding: 10px 15px;
            border-radius: 20px;
            color: white;
            font-weight: bold;
        }
        
        .connected { background: #27ae60; }
        .disconnected { background: #e74c3c; }
    </style>
</head>
<body>
    <div class="connection-status" id="connection-status">
        🔴 Déconnecté
    </div>
    
    <div class="container">
        <div class="header">
            <h1>🌸 MVP Drone de Pollinisation</h1>
            <p>Interface de contrôle et surveillance</p>
        </div>
        
        <!-- Contrôles principaux -->
        <div class="controls">
            <button class="btn btn-primary" id="define-mission">📍 Définir Zone</button>
            <button class="btn btn-success" id="start-mission" disabled>🚀 Démarrer Mission</button>
            <button class="btn btn-warning" id="pause-mission" disabled>⏸️ Pause</button>
            <button class="btn btn-danger" id="stop-mission" disabled>⏹️ Arrêter</button>
            <button class="btn btn-primary" id="rtl-mission">🏠 Retour Base</button>
        </div>
        
        <!-- Tableau de bord status -->
        <div class="status-panel">
            <div class="status-card">
                <div class="status-value" id="mission-status">IDLE</div>
                <div class="status-label">Statut Mission</div>
            </div>
            <div class="status-card">
                <div class="status-value" id="flower-count">0</div>
                <div class="status-label">Fleurs Détectées</div>
            </div>
            <div class="status-card">
                <div class="status-value" id="battery-level">--</div>
                <div class="status-label">Batterie (%)</div>
            </div>
        </div>
        
        <!-- Contenu principal -->
        <div class="main-content">
            <!-- Carte -->
            <div>
                <h3>🗺️ Carte de Mission</h3>
                <div id="map"></div>
            </div>
            
            <!-- Panneau latéral -->
            <div class="sidebar">
                <!-- Liste des fleurs -->
                <div>
                    <h3>🌸 Fleurs Détectées</h3>
                    <div class="flower-list" id="flower-list">
                        <p>Aucune fleur détectée</p>
                    </div>
                </div>
                
                <!-- Log des événements -->
                <div>
                    <h3>📋 Journal d'Événements</h3>
                    <div class="log-panel" id="log-panel"></div>
                </div>
            </div>
        </div>
    </div>
    
    <script src="https://unpkg.com/leaflet/dist/leaflet.js"></script>
    <script>
        // =================================================================
        // INITIALISATION DE LA CARTE
        // =================================================================
        const map = L.map('map').setView([45.5017, -73.5673], 16);
        
        L.tileLayer('https://{s}.tile.openstreetmap.org/{z}/{x}/{y}.png', {
            attribution: '© OpenStreetMap contributors'
        }).addTo(map);
        
        // Variables globales
        let missionArea = null;
        let droneMarker = null;
        let flowerMarkers = [];
        let websocket = null;
        let missionStatus = 'IDLE';
        let isDefiningMission = false;
        
        // =================================================================
        // GESTION DES MISSIONS
        // =================================================================
        const defineMissionBtn = document.getElementById('define-mission');
        const startMissionBtn = document.getElementById('start-mission');
        const pauseMissionBtn = document.getElementById('pause-mission');
        const stopMissionBtn = document.getElementById('stop-mission');
        const rtlMissionBtn = document.getElementById('rtl-mission');
        
        defineMissionBtn.addEventListener('click', () => {
            if (isDefiningMission) {
                // Annuler la définition
                isDefiningMission = false;
                defineMissionBtn.textContent = '📍 Définir Zone';
                defineMissionBtn.className = 'btn btn-primary';
            } else {
                // Commencer la définition
                isDefiningMission = true;
                defineMissionBtn.textContent = '❌ Annuler';
                defineMissionBtn.className = 'btn btn-warning';
                addLog('🖱️ Cliquez sur la carte pour définir la zone de mission');
            }
        });
        
        startMissionBtn.addEventListener('click', () => {
            startMission();
        });
        
        pauseMissionBtn.addEventListener('click', () => {
            pauseMission();
        });
        
        stopMissionBtn.addEventListener('click', () => {
            stopMission();
        });
        
        rtlMissionBtn.addEventListener('click', () => {
            returnToLaunch();
        });
        
        // =================================================================
        // GESTION DE LA CARTE
        // =================================================================
        let missionCorners = [];
        
        map.on('click', (e) => {
            if (!isDefiningMission) return;
            
            missionCorners.push(e.latlng);
            
            if (missionCorners.length === 1) {
                addLog('📍 Premier coin défini. Cliquez pour le coin opposé.');
            } else if (missionCorners.length === 2) {
                // Créer la zone de mission
                createMissionArea(missionCorners[0], missionCorners[1]);
                isDefiningMission = false;
                defineMissionBtn.textContent = '📍 Définir Zone';
                defineMissionBtn.className = 'btn btn-primary';
                missionCorners = [];
            }
        });
        
        function createMissionArea(corner1, corner2) {
            // Supprimer l'ancienne zone
            if (missionArea) {
                map.removeLayer(missionArea);
            }
            
            // Créer la nouvelle zone
            const bounds = [corner1, corner2];
            missionArea = L.rectangle(bounds, {
                color: '#3498db',
                weight: 3,
                fillOpacity: 0.2
            }).addTo(map);
            
            // Activer le bouton de démarrage
            startMissionBtn.disabled = false;
            
            addLog(`✅ Zone de mission définie: ${calculateArea(corner1, corner2)} m²`);
        }
        
        function calculateArea(corner1, corner2) {
            // Calcul approximatif de l'aire
            const latDiff = Math.abs(corner1.lat - corner2.lat);
            const lngDiff = Math.abs(corner1.lng - corner2.lng);
            const approxArea = latDiff * lngDiff * 111000 * 111000; // Conversion degrés -> mètres
            return Math.round(approxArea);
        }
        
        // =================================================================
        // COMMUNICATION AVEC L'API
        // =================================================================
        function connectWebSocket() {
            const wsUrl = 'ws://localhost:8000/ws/live_data';
            websocket = new WebSocket(wsUrl);
            
            websocket.onopen = () => {
                updateConnectionStatus(true);
                addLog('🔗 Connexion WebSocket établie');
            };
            
            websocket.onmessage = (event) => {
                const data = JSON.parse(event.data);
                handleLiveData(data);
            };
            
            websocket.onclose = () => {
                updateConnectionStatus(false);
                addLog('❌ Connexion WebSocket fermée');
                
                // Tentative de reconnexion
                setTimeout(connectWebSocket, 5000);
            };
            
            websocket.onerror = (error) => {
                addLog('❌ Erreur WebSocket: ' + error);
            };
        }
        
        function updateConnectionStatus(connected) {
            const statusEl = document.getElementById('connection-status');
            if (connected) {
                statusEl.textContent = '🟢 Connecté';
                statusEl.className = 'connection-status connected';
            } else {
                statusEl.textContent = '🔴 Déconnecté';
                statusEl.className = 'connection-status disconnected';
            }
        }
        
        function handleLiveData(data) {
            // Mise à jour de la position du drone
            if (data.drone_position) {
                updateDronePosition(data.drone_position);
            }
            
            // Mise à jour du statut de mission
            if (data.mission_status) {
                updateMissionStatus(data.mission_status);
            }
            
            // Mise à jour des fleurs détectées
            if (data.flower_detections) {
                updateFlowerList(data.flower_detections);
            }
            
            // Mise à jour de la batterie
            if (data.battery_level !== undefined) {
                document.getElementById('battery-level').textContent = data.battery_level + '%';
            }
        }
        
        // =================================================================
        // GESTION DES MISSIONS
        // =================================================================
        async function startMission() {
            if (!missionArea) {
                addLog('❌ Aucune zone de mission définie');
                return;
            }
            
            const bounds = missionArea.getBounds();
            const missionData = {
                area: [
                    [bounds.getSouthWest().lat, bounds.getSouthWest().lng],
                    [bounds.getNorthEast().lat, bounds.getNorthEast().lng]
                ],
                altitude: 20,
                pattern: 'zigzag',
                speed: 5
            };
            
            try {
                const response = await fetch('/api/missions/create', {
                    method: 'POST',
                    headers: {'Content-Type': 'application/json'},
                    body: JSON.stringify(missionData)
                });
                
                const result = await response.json();
                
                if (result.success) {
                    addLog('🚀 Mission démarrée: ' + result.mission_id);
                    updateMissionStatus('RUNNING');
                } else {
                    addLog('❌ Échec démarrage mission: ' + result.error);
                }
            } catch (error) {
                addLog('❌ Erreur communication API: ' + error);
            }
        }
        
        async function pauseMission() {
            // Implémentation pause mission
            addLog('⏸️ Mission mise en pause');
            updateMissionStatus('PAUSED');
        }
        
        async function stopMission() {
            // Implémentation arrêt mission
            addLog('⏹️ Mission arrêtée');
            updateMissionStatus('STOPPED');
        }
        
        async function returnToLaunch() {
            // Implémentation RTL
            addLog('🏠 Retour à la base initié');
        }
        
        // =================================================================
        // MISE À JOUR DE L'INTERFACE
        // =================================================================
        function updateMissionStatus(status) {
            missionStatus = status;
            document.getElementById('mission-status').textContent = status;
            
            // Mise à jour des boutons
            switch (status) {
                case 'IDLE':
                    startMissionBtn.disabled = !missionArea;
                    pauseMissionBtn.disabled = true;
                    stopMissionBtn.disabled = true;
                    break;
                case 'RUNNING':
                    startMissionBtn.disabled = true;
                    pauseMissionBtn.disabled = false;
                    stopMissionBtn.disabled = false;
                    break;
                case 'PAUSED':
                    startMissionBtn.disabled = false;
                    pauseMissionBtn.disabled = true;
                    stopMissionBtn.disabled = false;
                    break;
                case 'STOPPED':
                    startMissionBtn.disabled = !missionArea;
                    pauseMissionBtn.disabled = true;
                    stopMissionBtn.disabled = true;
                    break;
            }
        }
        
        function updateDronePosition(position) {
            if (!droneMarker) {
                // Créer le marqueur du drone
                const droneIcon = L.divIcon({
                    html: '🚁',
                    iconSize: [30, 30],
                    className: 'drone-marker'
                });
                
                droneMarker = L.marker([position.lat, position.lng], {
                    icon: droneIcon
                }).addTo(map);
            } else {
                // Mettre à jour la position
                droneMarker.setLatLng([position.lat, position.lng]);
            }
        }
        
        function updateFlowerList(flowers) {
            const flowerListEl = document.getElementById('flower-list');
            const flowerCountEl = document.getElementById('flower-count');
            
            if (flowers.length === 0) {
                flowerListEl.innerHTML = '<p>Aucune fleur détectée</p>';
                flowerCountEl.textContent = '0';
                return;
            }
            
            flowerCountEl.textContent = flowers.length;
            
            let html = '';
            flowers.forEach(flower => {
                html += `
                    <div class="flower-item flower-${flower.color}">
                        <span>${flower.color}</span>
                        <span>${(flower.confidence * 100).toFixed(0)}%</span>
                    </div>
                `;
            });
            
            flowerListEl.innerHTML = html;
        }
        
        function addLog(message) {
            const logPanel = document.getElementById('log-panel');
            const timestamp = new Date().toLocaleTimeString();
            const logEntry = `[${timestamp}] ${message}\n`;
            
            logPanel.textContent += logEntry;
            logPanel.scrollTop = logPanel.scrollHeight;
        }
        
        // =================================================================
        // INITIALISATION
        // =================================================================
        document.addEventListener('DOMContentLoaded', () => {
            addLog('🌸 Interface MVP Drone Pollinisation initialisée');
            connectWebSocket();
        });
    </script>
</body>
</html>
```

---

## 📋 **Plan de Développement MVP (6-8 semaines)**

### **Semaine 1-2: Infrastructure de Base**
- ✅ Setup environnement ROS2 + SITL
- ✅ Package MVP drone_control
- ✅ Communication MAVROS basique
- ✅ Tests de vol simple (décollage/atterrissage)

### **Semaine 3-4: Vision et Détection**
- ✅ Package MVP flower_detector
- ✅ Algorithmes détection par couleur
- ✅ Interface caméra simulée
- ✅ Tests de détection avec images

### **Semaine 5-6: Mission et Navigation**
- ✅ Package MVP mission_planner
- ✅ Génération patterns de vol
- ✅ Navigation par waypoints
- ✅ Intégration vision + navigation

### **Semaine 7-8: Interface Web et Tests**
- ✅ Interface web MVP
- ✅ API REST basique
- ✅ Tests système complet
- ✅ Documentation et démonstrations

---

## 🎯 **Critères de Succès MVP**

### **Fonctionnalités Validées**
1. ✅ **Vol Autonome**: Décollage, navigation, atterrissage automatique
2. ✅ **Détection Fleurs**: Reconnaissance de 3 couleurs avec >70% précision
3. ✅ **Mission Simple**: Couverture zone rectangulaire en zigzag
4. ✅ **Interface Web**: Définition zone et contrôle mission
5. ✅ **Collecte Données**: GPS, images, logs de mission
6. ✅ **Simulation**: Fonctionnement complet en SITL

### **Métriques de Performance**
- **Précision détection**: >70% sur fleurs simulées
- **Navigation**: Erreur de position <2m
- **Couverture**: >90% de la zone définie
- **Interface**: Latence <500ms pour commandes
- **Données**: 100% des événements logués

---

## 🚀 **Extensions Post-MVP**

### **Améliorations Prioritaires**
1. **IA Avancée**: Classification ML des espèces
2. **Navigation Optimisée**: Algorithmes génétiques
3. **Multi-Drones**: Coordination d'essaims
4. **Analyse Prédictive**: Apprentissage automatique
5. **Interface Avancée**: Tableau de bord complet

Ce plan MVP fournit une base solide et fonctionnelle pour valider le concept de pollinisation par drone, tout en gardant la complexité maîtrisée et les délais réalistes.
