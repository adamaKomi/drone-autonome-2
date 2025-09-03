# 🔍 DRONE VISION NODE - Plan de développement

## Objectif
Ajouter la capacité de détection visuelle des fleurs au système existant.

## Fonctionnalités requises
- Capture vidéo en temps réel
- Détection de fleurs par traitement d'image (OpenCV)
- Localisation des fleurs dans le repère du drone
- Interface avec le système de navigation pour guidage précis

## Architecture proposée

### Services ROS2
- `/vision/detect_flowers` - Démarre/arrête la détection
- `/vision/get_flower_position` - Retourne la position relative d'une fleur détectée
- `/vision/set_detection_params` - Configure les paramètres de détection

### Topics ROS2
- `/vision/camera/image_raw` - Flux vidéo brut
- `/vision/flowers_detected` - Liste des fleurs détectées
- `/vision/debug_image` - Image avec annotations pour debug

### Méthodes de détection
1. **Détection par couleur** (simple, rapide)
   - Filtrage HSV pour couleurs spécifiques
   - Détection de contours et formes circulaires
   
2. **Détection par forme** (plus robuste)
   - Analyse de texture et patterns floraux
   - Classification basique ML

## Intégration avec l'existant
- Le `mission_node` inclura des tâches `DETECT_FLOWERS` et `APPROACH_FLOWER`
- Le `navigation_node` recevra des positions relatives pour approche précise
- Interface avec caméra simulée dans Gazebo
