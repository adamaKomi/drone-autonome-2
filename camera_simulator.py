#!/usr/bin/env python3
"""
=============================================================================
SIMULATEUR DE CAMÉRA - Images avec fleurs simulées
=============================================================================
Génère et publie des images simulées contenant des fleurs colorées
pour tester le système de détection.

Usage:
    python3 camera_simulator.py
=============================================================================
"""

import cv2
import numpy as np
import time
import random
import math

# ROS2 imports
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class CameraSimulator(Node):
    """Simulateur de caméra avec fleurs"""
    
    def __init__(self):
        super().__init__('camera_simulator')
        
        self.logger = self.get_logger()
        self.logger.info("📹 Initialisation du simulateur de caméra...")
        
        # Configuration
        self.image_width = 640
        self.image_height = 480
        self.fps = 10  # Images par seconde
        
        # Utilitaires
        self.cv_bridge = CvBridge()
        
        # Publisher d'images
        self.image_pub = self.create_publisher(
            Image,
            '/camera/image_raw',
            10
        )
        
        # Timer pour génération d'images
        self.timer = self.create_timer(1.0 / self.fps, self._generate_and_publish)
        
        # État de simulation
        self.frame_count = 0
        self.flowers = []
        
        self.logger.info("✅ Simulateur de caméra initialisé!")
        self._print_info()
        
    def _generate_flowers(self):
        """Génère des fleurs aléatoirement"""
        # Réinitialiser de temps en temps
        if self.frame_count % 100 == 0:
            self.flowers.clear()
            
        # Ajouter de nouvelles fleurs occasionnellement
        if random.random() < 0.05:  # 5% de chance par frame
            flower = {
                'x': random.randint(50, self.image_width - 50),
                'y': random.randint(50, self.image_height - 50),
                'radius': random.randint(15, 35),
                'color': random.choice(['red', 'yellow', 'pink', 'orange']),
                'lifetime': random.randint(50, 150)  # Frames
            }
            self.flowers.append(flower)
            
        # Mettre à jour les fleurs existantes
        self.flowers = [f for f in self.flowers if f['lifetime'] > 0]
        for flower in self.flowers:
            flower['lifetime'] -= 1
            # Petit mouvement aléatoire
            flower['x'] += random.randint(-1, 1)
            flower['y'] += random.randint(-1, 1)
            
            # Garder dans les limites
            flower['x'] = max(30, min(self.image_width - 30, flower['x']))
            flower['y'] = max(30, min(self.image_height - 30, flower['y']))
            
    def _draw_flower(self, image, flower):
        """Dessine une fleur sur l'image"""
        x, y = flower['x'], flower['y']
        radius = flower['radius']
        color_name = flower['color']
        
        # Couleurs BGR pour OpenCV
        colors = {
            'red': (0, 0, 255),
            'yellow': (0, 255, 255),
            'pink': (255, 100, 255),
            'orange': (0, 165, 255)
        }
        
        color = colors.get(color_name, (0, 255, 0))
        
        # Fleur simple: centre + pétales
        center = (x, y)
        
        # Pétales (cercles autour du centre)
        petal_positions = [
            (x + int(radius * 0.6 * math.cos(angle)), 
             y + int(radius * 0.6 * math.sin(angle)))
            for angle in [0, math.pi/2, math.pi, 3*math.pi/2]
        ]
        
        # Dessiner les pétales
        for petal_pos in petal_positions:
            cv2.circle(image, petal_pos, radius // 2, color, -1)
            
        # Centre de la fleur (couleur légèrement différente)
        center_color = (
            max(0, color[0] - 50),
            max(0, color[1] - 50),
            min(255, color[2] + 50)
        )
        cv2.circle(image, center, radius // 3, center_color, -1)
        
        # Contour pour meilleure visibilité
        cv2.circle(image, center, radius, (0, 0, 0), 2)
        
    def _generate_background(self):
        """Génère un arrière-plan herbu"""
        # Fond vert avec texture
        image = np.full((self.image_height, self.image_width, 3), (40, 120, 40), dtype=np.uint8)
        
        # Ajouter du bruit pour simuler l'herbe
        noise = np.random.randint(-20, 20, (self.image_height, self.image_width, 3))
        image = np.clip(image.astype(np.int16) + noise, 0, 255).astype(np.uint8)
        
        # Quelques lignes pour simuler des brins d'herbe
        for _ in range(50):
            x1 = random.randint(0, self.image_width)
            y1 = random.randint(0, self.image_height)
            x2 = x1 + random.randint(-10, 10)
            y2 = y1 + random.randint(-20, 5)
            
            cv2.line(image, (x1, y1), (x2, y2), (20, 100, 20), 1)
            
        return image
        
    def _generate_and_publish(self):
        """Génère et publie une image"""
        # Générer les fleurs
        self._generate_flowers()
        
        # Créer l'image de base
        image = self._generate_background()
        
        # Dessiner les fleurs
        for flower in self.flowers:
            self._draw_flower(image, flower)
            
        # Ajouter des informations de debug
        timestamp = time.strftime("%H:%M:%S")
        cv2.putText(image, f"Frame: {self.frame_count}", (10, 30), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(image, f"Time: {timestamp}", (10, 60), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(image, f"Flowers: {len(self.flowers)}", (10, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Convertir et publier
        try:
            img_msg = self.cv_bridge.cv2_to_imgmsg(image, "bgr8")
            img_msg.header.stamp = self.get_clock().now().to_msg()
            img_msg.header.frame_id = "camera_link"
            
            self.image_pub.publish(img_msg)
            
            if self.frame_count % 50 == 0:  # Log toutes les 5 secondes
                self.logger.info(f"📸 Frame {self.frame_count}, {len(self.flowers)} fleur(s) visible(s)")
                
        except Exception as e:
            self.logger.error(f"❌ Erreur publication image: {e}")
            
        self.frame_count += 1
        
    def _print_info(self):
        """Affiche les informations du simulateur"""
        self.logger.info("=" * 50)
        self.logger.info("📹 SIMULATEUR DE CAMÉRA")
        self.logger.info("=" * 50)
        self.logger.info(f"📐 Résolution: {self.image_width}x{self.image_height}")
        self.logger.info(f"🎬 FPS: {self.fps}")
        self.logger.info("📤 Topic publié: /camera/image_raw")
        self.logger.info("🌸 Génération automatique de fleurs colorées")
        self.logger.info("=" * 50)


def main(args=None):
    """Point d'entrée principal"""
    rclpy.init(args=args)
    
    try:
        simulator = CameraSimulator()
        
        print("📹 Simulateur de caméra démarré!")
        print("🌸 Génération d'images avec fleurs en cours...")
        print("Press Ctrl+C pour arrêter")
        
        rclpy.spin(simulator)
        
    except KeyboardInterrupt:
        print("\n🛑 Arrêt du simulateur")
    except Exception as e:
        print(f"❌ Erreur: {e}")
    finally:
        try:
            if 'simulator' in locals():
                simulator.destroy_node()
        except:
            pass
        rclpy.shutdown()
        print("👋 Simulateur arrêté")


if __name__ == '__main__':
    main()
