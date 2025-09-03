#!/usr/bin/env python3
"""
=============================================================================
DRONE VISION NODE - Nœud de détection visuelle des fleurs
=============================================================================
Auteur: Système Drone Autonome
Date: 2025-09-02
Version: 1.0.0

Description:
    Nœud ROS2 spécialisé pour la détection visuelle des fleurs.
    Utilise OpenCV pour le traitement d'image et la détection par couleur.
    
Responsabilités:
    - Capture vidéo en temps réel
    - Détection de fleurs par couleur/forme
    - Localisation des fleurs dans le repère du drone
    - Interface avec le système de navigation

Utilisation:
    ros2 run drone_vision vision_node
=============================================================================
"""

import cv2
import numpy as np
import time
import math
from typing import List, Tuple, Optional, Dict, Any
from dataclasses import dataclass

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Message types
from std_msgs.msg import String, Bool, Header
from sensor_msgs.msg import Image, CompressedImage
from geometry_msgs.msg import Point, PointStamped
from cv_bridge import CvBridge

# Services personnalisés (à créer)
from std_srvs.srv import SetBool, Trigger


@dataclass
class FlowerDetection:
    """Structure pour une fleur détectée"""
    id: str
    center_x: float         # Position X dans l'image (pixels)
    center_y: float         # Position Y dans l'image (pixels)
    radius: float           # Rayon approximatif (pixels)
    color: str              # Couleur détectée
    confidence: float       # Confiance de la détection (0-1)
    timestamp: float        # Timestamp de détection
    world_position: Optional[Point] = None  # Position dans le monde (si calculée)


class FlowerDetector:
    """Classe pour la détection de fleurs par couleur"""
    
    def __init__(self):
        # Définition des plages de couleurs en HSV
        self.color_ranges = {
            'red': [
                (np.array([0, 50, 50]), np.array([10, 255, 255])),
                (np.array([170, 50, 50]), np.array([180, 255, 255]))
            ],
            'yellow': [
                (np.array([20, 50, 50]), np.array([30, 255, 255]))
            ],
            'pink': [
                (np.array([140, 50, 50]), np.array([170, 255, 255]))
            ],
            'orange': [
                (np.array([10, 50, 50]), np.array([20, 255, 255]))
            ]
        }
        
        # Paramètres de détection
        self.min_radius = 10
        self.max_radius = 100
        self.min_confidence = 0.5
        
    def detect_flowers(self, image: np.ndarray, target_colors: List[str] = None) -> List[FlowerDetection]:
        """Détecte les fleurs dans une image"""
        if target_colors is None:
            target_colors = ['red', 'yellow', 'pink']
            
        detections = []
        
        # Conversion en HSV pour la détection par couleur
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        
        for color in target_colors:
            if color not in self.color_ranges:
                continue
                
            # Créer un masque pour cette couleur
            mask = np.zeros(hsv.shape[:2], dtype=np.uint8)
            
            for lower, upper in self.color_ranges[color]:
                color_mask = cv2.inRange(hsv, lower, upper)
                mask = cv2.bitwise_or(mask, color_mask)
            
            # Nettoyage du masque
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
            
            # Détection de contours
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for contour in contours:
                # Filtrer par taille
                area = cv2.contourArea(contour)
                if area < 100:  # Trop petit
                    continue
                    
                # Approximation circulaire
                (x, y), radius = cv2.minEnclosingCircle(contour)
                
                if self.min_radius <= radius <= self.max_radius:
                    # Calculer la confiance basée sur la circularité
                    perimeter = cv2.arcLength(contour, True)
                    circularity = 4 * math.pi * area / (perimeter * perimeter) if perimeter > 0 else 0
                    confidence = min(circularity * 2, 1.0)  # Normaliser
                    
                    if confidence >= self.min_confidence:
                        detection = FlowerDetection(
                            id=f"{color}_{int(x)}_{int(y)}_{int(time.time()*1000)}",
                            center_x=float(x),
                            center_y=float(y),
                            radius=float(radius),
                            color=color,
                            confidence=confidence,
                            timestamp=time.time()
                        )
                        detections.append(detection)
        
        return detections
    
    def draw_detections(self, image: np.ndarray, detections: List[FlowerDetection]) -> np.ndarray:
        """Dessine les détections sur l'image"""
        result = image.copy()
        
        for detection in detections:
            # Cercle principal
            center = (int(detection.center_x), int(detection.center_y))
            radius = int(detection.radius)
            
            # Couleur en fonction du type détecté
            color_map = {
                'red': (0, 0, 255),
                'yellow': (0, 255, 255),
                'pink': (255, 0, 255),
                'orange': (0, 165, 255)
            }
            color = color_map.get(detection.color, (0, 255, 0))
            
            # Dessiner le cercle et les infos
            cv2.circle(result, center, radius, color, 2)
            cv2.circle(result, center, 3, color, -1)
            
            # Texte d'information
            text = f"{detection.color} ({detection.confidence:.2f})"
            cv2.putText(result, text, (center[0] - 50, center[1] - radius - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
        
        return result


class VisionNode(Node):
    """
    Nœud de vision pour la détection de fleurs
    """
    
    def __init__(self):
        super().__init__('drone_vision')
        
        self.logger = self.get_logger()
        self.logger.info("🔍 Initialisation du nœud de vision...")
        
        # Initialisation des composants
        self.cv_bridge = CvBridge()
        self.flower_detector = FlowerDetector()
        
        # État de détection
        self.detection_active = False
        self.current_detections: List[FlowerDetection] = []
        
        # Configuration
        self._setup_qos_profiles()
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_services()
        
        # Timer pour traitement
        self.processing_timer = self.create_timer(0.1, self._process_detection)  # 10 Hz
        
        self.logger.info("✅ Nœud de vision initialisé!")
        self._print_node_info()
        
    def _setup_qos_profiles(self):
        """Configure les profils QoS"""
        self.qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
    def _setup_subscribers(self):
        """Configure les souscripteurs"""
        # Image de la caméra (simulée ou réelle)
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._image_callback,
            self.qos_sensor
        )
        
        # Alternative: image compressée
        self.compressed_image_sub = self.create_subscription(
            CompressedImage,
            '/camera/image_raw/compressed',
            self._compressed_image_callback,
            self.qos_sensor
        )
        
    def _setup_publishers(self):
        """Configure les publishers"""
        # Fleurs détectées
        self.flowers_pub = self.create_publisher(
            String,
            '/vision/flowers_detected',
            10
        )
        
        # Image avec annotations pour debug
        self.debug_image_pub = self.create_publisher(
            Image,
            '/vision/debug_image',
            self.qos_sensor
        )
        
        # Position de la fleur la plus proche
        self.target_flower_pub = self.create_publisher(
            PointStamped,
            '/vision/target_flower_position',
            10
        )
        
    def _setup_services(self):
        """Configure les services"""
        # Démarrer/arrêter la détection
        self.detection_service = self.create_service(
            SetBool,
            '/vision/set_detection',
            self._handle_set_detection
        )
        
        # Obtenir la position de la fleur la plus proche
        self.get_flower_service = self.create_service(
            Trigger,
            '/vision/get_closest_flower',
            self._handle_get_closest_flower
        )
        
    def _image_callback(self, msg: Image):
        """Callback pour les images non compressées"""
        if not self.detection_active:
            return
            
        try:
            # Conversion ROS -> OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, "bgr8")
            self._process_image(cv_image, msg.header)
            
        except Exception as e:
            self.logger.error(f"❌ Erreur traitement image: {e}")
            
    def _compressed_image_callback(self, msg: CompressedImage):
        """Callback pour les images compressées"""
        if not self.detection_active:
            return
            
        try:
            # Décompression
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            self._process_image(cv_image, msg.header)
            
        except Exception as e:
            self.logger.error(f"❌ Erreur traitement image compressée: {e}")
            
    def _process_image(self, image: np.ndarray, header: Header):
        """Traite une image pour détecter les fleurs"""
        # Détection des fleurs
        detections = self.flower_detector.detect_flowers(image)
        self.current_detections = detections
        
        if detections:
            self.logger.info(f"🌸 {len(detections)} fleur(s) détectée(s)")
            
            # Publier les détections
            self._publish_detections(detections, header)
            
        # Publier l'image de debug
        debug_image = self.flower_detector.draw_detections(image, detections)
        self._publish_debug_image(debug_image, header)
        
    def _publish_detections(self, detections: List[FlowerDetection], header: Header):
        """Publie les détections de fleurs"""
        # Format JSON simple pour les détections
        detection_data = {
            "timestamp": header.stamp.sec + header.stamp.nanosec * 1e-9,
            "count": len(detections),
            "flowers": []
        }
        
        for detection in detections:
            flower_data = {
                "id": detection.id,
                "center_x": detection.center_x,
                "center_y": detection.center_y,
                "radius": detection.radius,
                "color": detection.color,
                "confidence": detection.confidence
            }
            detection_data["flowers"].append(flower_data)
            
        # Publication
        msg = String()
        msg.data = str(detection_data)  # JSON string
        self.flowers_pub.publish(msg)
        
        # Publier la fleur la plus proche comme cible
        if detections:
            closest_flower = min(detections, key=lambda f: f.radius)  # Plus grande = plus proche
            target_msg = PointStamped()
            target_msg.header = header
            target_msg.point.x = closest_flower.center_x
            target_msg.point.y = closest_flower.center_y
            target_msg.point.z = closest_flower.confidence
            self.target_flower_pub.publish(target_msg)
            
    def _publish_debug_image(self, image: np.ndarray, header: Header):
        """Publie l'image avec annotations"""
        try:
            # Conversion OpenCV -> ROS
            debug_msg = self.cv_bridge.cv2_to_imgmsg(image, "bgr8")
            debug_msg.header = header
            self.debug_image_pub.publish(debug_msg)
            
        except Exception as e:
            self.logger.error(f"❌ Erreur publication debug image: {e}")
            
    def _process_detection(self):
        """Timer de traitement périodique"""
        # Ici on pourrait ajouter du post-traitement ou des calculs de position
        pass
        
    def _handle_set_detection(self, request, response):
        """Gère l'activation/désactivation de la détection"""
        self.detection_active = request.data
        response.success = True
        
        if request.data:
            response.message = "Détection de fleurs activée"
            self.logger.info("🔍 Détection de fleurs ACTIVÉE")
        else:
            response.message = "Détection de fleurs désactivée"
            self.logger.info("⏸️ Détection de fleurs DÉSACTIVÉE")
            
        return response
        
    def _handle_get_closest_flower(self, request, response):
        """Retourne la position de la fleur la plus proche"""
        if not self.current_detections:
            response.success = False
            response.message = "Aucune fleur détectée"
            return response
            
        # Trouver la fleur la plus proche (plus grande dans l'image)
        closest_flower = max(self.current_detections, key=lambda f: f.radius)
        
        response.success = True
        response.message = f"Fleur {closest_flower.color} à ({closest_flower.center_x:.1f}, {closest_flower.center_y:.1f}), confiance: {closest_flower.confidence:.2f}"
        
        return response
        
    def _print_node_info(self):
        """Affiche les informations du nœud"""
        self.logger.info("=" * 60)
        self.logger.info("🔍 DRONE VISION - DÉTECTION DE FLEURS")
        self.logger.info("=" * 60)
        self.logger.info("📡 Services disponibles:")
        self.logger.info("  • /vision/set_detection - Activer/désactiver détection")
        self.logger.info("  • /vision/get_closest_flower - Obtenir fleur la plus proche")
        self.logger.info("")
        self.logger.info("📤 Topics publiés:")
        self.logger.info("  • /vision/flowers_detected - Liste des fleurs détectées")
        self.logger.info("  • /vision/debug_image - Image avec annotations")
        self.logger.info("  • /vision/target_flower_position - Position de la cible")
        self.logger.info("")
        self.logger.info("📥 Topics souscrits:")
        self.logger.info("  • /camera/image_raw - Flux vidéo principal")
        self.logger.info("  • /camera/image_raw/compressed - Flux vidéo compressé")
        self.logger.info("=" * 60)


def main(args=None):
    """Point d'entrée principal"""
    rclpy.init(args=args)
    
    try:
        node = VisionNode()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n🛑 Arrêt demandé par l'utilisateur")
    except Exception as e:
        print(f"❌ Erreur fatale: {e}")
    finally:
        try:
            if 'node' in locals():
                node.destroy_node()
        except:
            pass
        rclpy.shutdown()
        print("👋 VisionNode arrêté")


if __name__ == '__main__':
    main()
