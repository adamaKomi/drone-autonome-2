#!/usr/bin/env python3
"""
=============================================================================
DRONE DATA COLLECTOR - Collecteur de données de mission
=============================================================================
Auteur: Système Drone Autonome
Date: 2025-09-02
Version: 1.0.0

Description:
    Nœud ROS2 pour la collecte et l'enregistrement des données de mission.
    Enregistre les positions GPS, images, détections de fleurs et métriques.
    
Responsabilités:
    - Enregistrement GPS continu
    - Capture d'images aux points d'intérêt
    - Logging des détections de fleurs
    - Export des données au format JSON/CSV
    - Sauvegarde des images avec métadonnées

Utilisation:
    ros2 run drone_vision data_collector_node
=============================================================================
"""

import os
import json
import csv
import time
import uuid
from datetime import datetime
from typing import Dict, Any, List, Optional
from dataclasses import dataclass, asdict
import threading

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

# Message types
from std_msgs.msg import String, Bool
from sensor_msgs.msg import Image, NavSatFix
from geometry_msgs.msg import Point, PointStamped, PoseStamped
from mavros_msgs.msg import State
from cv_bridge import CvBridge

# Services
from std_srvs.srv import SetBool, Trigger


@dataclass
class GPSDataPoint:
    """Point de données GPS"""
    timestamp: float
    latitude: float
    longitude: float
    altitude: float
    accuracy: float
    mission_phase: str = "unknown"


@dataclass
class FlowerRecord:
    """Enregistrement d'une fleur détectée"""
    id: str
    timestamp: float
    gps_lat: float
    gps_lon: float
    gps_alt: float
    image_x: float
    image_y: float
    radius: float
    color: str
    confidence: float
    image_filename: Optional[str] = None
    mission_phase: str = "detection"


@dataclass
class MissionMetrics:
    """Métriques de mission"""
    mission_id: str
    start_time: float
    end_time: Optional[float] = None
    total_distance: float = 0.0
    flowers_detected: int = 0
    images_captured: int = 0
    areas_scanned: int = 0
    mission_success: bool = False


class DataCollectorNode(Node):
    """
    Nœud de collecte de données de mission
    """
    
    def __init__(self):
        super().__init__('drone_data_collector')
        
        self.logger = self.get_logger()
        self.logger.info("📊 Initialisation du collecteur de données...")
        
        # Configuration des chemins
        self.data_dir = os.path.expanduser("~/ros2_ws/mission_data")
        self.images_dir = os.path.join(self.data_dir, "images")
        self._ensure_directories()
        
        # État de collecte
        self.collecting_active = False
        self.mission_id = str(uuid.uuid4())[:8]
        self.current_mission_phase = "idle"
        
        # Données collectées
        self.gps_data: List[GPSDataPoint] = []
        self.flower_records: List[FlowerRecord] = []
        self.mission_metrics = MissionMetrics(
            mission_id=self.mission_id,
            start_time=time.time()
        )
        
        # Dernières données reçues
        self.last_gps: Optional[NavSatFix] = None
        self.last_image: Optional[Image] = None
        
        # Utilitaires
        self.cv_bridge = CvBridge()
        self.data_lock = threading.Lock()
        
        # Configuration ROS2
        self._setup_qos_profiles()
        self._setup_subscribers()
        self._setup_publishers()
        self._setup_services()
        
        # Timers
        self.gps_timer = self.create_timer(1.0, self._collect_gps_data)  # 1 Hz
        self.save_timer = self.create_timer(30.0, self._periodic_save)   # Sauvegarde toutes les 30s
        
        self.logger.info("✅ Collecteur de données initialisé!")
        self._print_node_info()
        
    def _ensure_directories(self):
        """Crée les répertoires nécessaires"""
        os.makedirs(self.data_dir, exist_ok=True)
        os.makedirs(self.images_dir, exist_ok=True)
        
    def _setup_qos_profiles(self):
        """Configure les profils QoS"""
        self.qos_sensor = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )
        
    def _setup_subscribers(self):
        """Configure les souscripteurs"""
        # Position GPS
        self.gps_sub = self.create_subscription(
            NavSatFix,
            '/mavros/global_position/global',
            self._gps_callback,
            self.qos_sensor
        )
        
        # Images de la caméra
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self._image_callback,
            self.qos_sensor
        )
        
        # Fleurs détectées
        self.flowers_sub = self.create_subscription(
            String,
            '/vision/flowers_detected',
            self._flowers_callback,
            10
        )
        
        # État de la mission
        self.mission_state_sub = self.create_subscription(
            String,
            '/mission/current_task',
            self._mission_state_callback,
            10
        )
        
        # État du drone
        self.drone_state_sub = self.create_subscription(
            State,
            '/mavros/state',
            self._drone_state_callback,
            self.qos_sensor
        )
        
    def _setup_publishers(self):
        """Configure les publishers"""
        # Statut de collecte
        self.status_pub = self.create_publisher(
            String,
            '/data_collector/status',
            10
        )
        
    def _setup_services(self):
        """Configure les services"""
        # Démarrer/arrêter la collecte
        self.collection_service = self.create_service(
            SetBool,
            '/data_collector/set_collection',
            self._handle_set_collection
        )
        
        # Sauvegarder les données
        self.save_service = self.create_service(
            Trigger,
            '/data_collector/save_data',
            self._handle_save_data
        )
        
        # Nouvelle mission
        self.new_mission_service = self.create_service(
            Trigger,
            '/data_collector/new_mission',
            self._handle_new_mission
        )
        
        # Capturer une image
        self.capture_service = self.create_service(
            Trigger,
            '/data_collector/capture_image',
            self._handle_capture_image
        )
        
    def _gps_callback(self, msg: NavSatFix):
        """Callback pour les données GPS"""
        self.last_gps = msg
        
    def _image_callback(self, msg: Image):
        """Callback pour les images"""
        self.last_image = msg
        
    def _flowers_callback(self, msg: String):
        """Callback pour les fleurs détectées"""
        if not self.collecting_active or not self.last_gps:
            return
            
        try:
            # Parser les données de fleurs (format JSON string)
            data = eval(msg.data)  # Simple parsing - en production utiliser json.loads
            
            if data.get("count", 0) > 0:
                with self.data_lock:
                    for flower_data in data.get("flowers", []):
                        flower_record = FlowerRecord(
                            id=flower_data["id"],
                            timestamp=time.time(),
                            gps_lat=self.last_gps.latitude,
                            gps_lon=self.last_gps.longitude,
                            gps_alt=self.last_gps.altitude,
                            image_x=flower_data["center_x"],
                            image_y=flower_data["center_y"],
                            radius=flower_data["radius"],
                            color=flower_data["color"],
                            confidence=flower_data["confidence"],
                            mission_phase=self.current_mission_phase
                        )
                        
                        # Capturer l'image si disponible
                        if self.last_image:
                            image_filename = self._save_image(flower_record.id)
                            flower_record.image_filename = image_filename
                            
                        self.flower_records.append(flower_record)
                        self.mission_metrics.flowers_detected += 1
                        
                self.logger.info(f"🌸 Fleur enregistrée: {flower_data['color']} (confiance: {flower_data['confidence']:.2f})")
                
        except Exception as e:
            self.logger.error(f"❌ Erreur traitement fleurs: {e}")
            
    def _mission_state_callback(self, msg: String):
        """Callback pour l'état de la mission"""
        self.current_mission_phase = msg.data
        
    def _drone_state_callback(self, msg: State):
        """Callback pour l'état du drone"""
        # Démarrer automatiquement la collecte quand armé
        if msg.armed and not self.collecting_active:
            self._start_collection()
        elif not msg.armed and self.collecting_active:
            self._stop_collection()
            
    def _collect_gps_data(self):
        """Collecte périodique des données GPS"""
        if not self.collecting_active or not self.last_gps:
            return
            
        with self.data_lock:
            gps_point = GPSDataPoint(
                timestamp=time.time(),
                latitude=self.last_gps.latitude,
                longitude=self.last_gps.longitude,
                altitude=self.last_gps.altitude,
                accuracy=getattr(self.last_gps, 'position_covariance', [0])[0],
                mission_phase=self.current_mission_phase
            )
            self.gps_data.append(gps_point)
            
        # Mettre à jour les métriques
        if len(self.gps_data) >= 2:
            self._update_distance_metrics()
            
    def _update_distance_metrics(self):
        """Met à jour les métriques de distance"""
        if len(self.gps_data) < 2:
            return
            
        # Calcul de distance simplifiée (Haversine approximé)
        p1 = self.gps_data[-2]
        p2 = self.gps_data[-1]
        
        # Distance approximative en mètres
        lat_diff = abs(p2.latitude - p1.latitude) * 111000  # ~111km par degré
        lon_diff = abs(p2.longitude - p1.longitude) * 111000
        distance = (lat_diff**2 + lon_diff**2)**0.5
        
        self.mission_metrics.total_distance += distance
        
    def _save_image(self, flower_id: str) -> Optional[str]:
        """Sauvegarde une image avec l'ID de la fleur"""
        if not self.last_image:
            return None
            
        try:
            # Conversion ROS -> OpenCV
            cv_image = self.cv_bridge.imgmsg_to_cv2(self.last_image, "bgr8")
            
            # Nom de fichier avec timestamp
            timestamp = int(time.time() * 1000)
            filename = f"flower_{flower_id}_{timestamp}.jpg"
            filepath = os.path.join(self.images_dir, filename)
            
            # Sauvegarde
            import cv2
            cv2.imwrite(filepath, cv_image)
            
            self.mission_metrics.images_captured += 1
            self.logger.info(f"📸 Image sauvegardée: {filename}")
            
            return filename
            
        except Exception as e:
            self.logger.error(f"❌ Erreur sauvegarde image: {e}")
            return None
            
    def _periodic_save(self):
        """Sauvegarde périodique des données"""
        if self.collecting_active and (self.gps_data or self.flower_records):
            self._save_data_to_files()
            
    def _save_data_to_files(self):
        """Sauvegarde les données dans des fichiers"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        
        with self.data_lock:
            # Sauvegarde GPS (CSV)
            if self.gps_data:
                gps_filename = f"gps_data_{self.mission_id}_{timestamp}.csv"
                gps_filepath = os.path.join(self.data_dir, gps_filename)
                
                with open(gps_filepath, 'w', newline='') as csvfile:
                    fieldnames = ['timestamp', 'latitude', 'longitude', 'altitude', 'accuracy', 'mission_phase']
                    writer = csv.DictWriter(csvfile, fieldnames=fieldnames)
                    writer.writeheader()
                    
                    for point in self.gps_data:
                        writer.writerow(asdict(point))
                        
            # Sauvegarde fleurs (JSON)
            if self.flower_records:
                flowers_filename = f"flowers_{self.mission_id}_{timestamp}.json"
                flowers_filepath = os.path.join(self.data_dir, flowers_filename)
                
                flowers_data = [asdict(record) for record in self.flower_records]
                with open(flowers_filepath, 'w') as jsonfile:
                    json.dump(flowers_data, jsonfile, indent=2)
                    
            # Sauvegarde métriques (JSON)
            metrics_filename = f"metrics_{self.mission_id}_{timestamp}.json"
            metrics_filepath = os.path.join(self.data_dir, metrics_filename)
            
            metrics_data = asdict(self.mission_metrics)
            with open(metrics_filepath, 'w') as jsonfile:
                json.dump(metrics_data, jsonfile, indent=2)
                
        self.logger.info(f"💾 Données sauvegardées dans {self.data_dir}")
        
    def _start_collection(self):
        """Démarre la collecte de données"""
        self.collecting_active = True
        self.mission_metrics.start_time = time.time()
        self.logger.info(f"🚀 Collecte de données DÉMARRÉE - Mission ID: {self.mission_id}")
        
    def _stop_collection(self):
        """Arrête la collecte de données"""
        self.collecting_active = False
        self.mission_metrics.end_time = time.time()
        self._save_data_to_files()
        self.logger.info("⏹️ Collecte de données ARRÊTÉE")
        
    def _handle_set_collection(self, request, response):
        """Gère l'activation/désactivation de la collecte"""
        if request.data:
            self._start_collection()
            response.message = "Collecte de données activée"
        else:
            self._stop_collection()
            response.message = "Collecte de données désactivée"
            
        response.success = True
        return response
        
    def _handle_save_data(self, request, response):
        """Force la sauvegarde des données"""
        try:
            self._save_data_to_files()
            response.success = True
            response.message = f"Données sauvegardées: {len(self.gps_data)} points GPS, {len(self.flower_records)} fleurs"
        except Exception as e:
            response.success = False
            response.message = f"Erreur sauvegarde: {e}"
            
        return response
        
    def _handle_new_mission(self, request, response):
        """Démarre une nouvelle mission"""
        # Sauvegarder l'ancienne mission
        if self.gps_data or self.flower_records:
            self._save_data_to_files()
            
        # Réinitialiser
        with self.data_lock:
            self.mission_id = str(uuid.uuid4())[:8]
            self.gps_data.clear()
            self.flower_records.clear()
            self.mission_metrics = MissionMetrics(
                mission_id=self.mission_id,
                start_time=time.time()
            )
            
        response.success = True
        response.message = f"Nouvelle mission créée: {self.mission_id}"
        self.logger.info(f"🆕 Nouvelle mission: {self.mission_id}")
        
        return response
        
    def _handle_capture_image(self, request, response):
        """Capture une image manuellement"""
        if not self.last_image:
            response.success = False
            response.message = "Aucune image disponible"
            return response
            
        # Capturer avec ID unique
        capture_id = f"manual_{int(time.time()*1000)}"
        filename = self._save_image(capture_id)
        
        if filename:
            response.success = True
            response.message = f"Image capturée: {filename}"
        else:
            response.success = False
            response.message = "Erreur capture image"
            
        return response
        
    def _print_node_info(self):
        """Affiche les informations du nœud"""
        self.logger.info("=" * 60)
        self.logger.info("📊 DRONE DATA COLLECTOR")
        self.logger.info("=" * 60)
        self.logger.info(f"🆔 Mission ID: {self.mission_id}")
        self.logger.info(f"📁 Répertoire données: {self.data_dir}")
        self.logger.info("")
        self.logger.info("📡 Services disponibles:")
        self.logger.info("  • /data_collector/set_collection - Activer/désactiver collecte")
        self.logger.info("  • /data_collector/save_data - Forcer sauvegarde")
        self.logger.info("  • /data_collector/new_mission - Nouvelle mission")
        self.logger.info("  • /data_collector/capture_image - Capturer image")
        self.logger.info("")
        self.logger.info("📤 Topics publiés:")
        self.logger.info("  • /data_collector/status - Statut de collecte")
        self.logger.info("")
        self.logger.info("📥 Topics souscrits:")
        self.logger.info("  • /mavros/global_position/global - Position GPS")
        self.logger.info("  • /camera/image_raw - Images caméra")
        self.logger.info("  • /vision/flowers_detected - Fleurs détectées")
        self.logger.info("  • /mission/current_task - État mission")
        self.logger.info("=" * 60)


def main(args=None):
    """Point d'entrée principal"""
    rclpy.init(args=args)
    
    try:
        node = DataCollectorNode()
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
        print("👋 DataCollectorNode arrêté")


if __name__ == '__main__':
    main()
