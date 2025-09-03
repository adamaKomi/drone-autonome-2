# 📊 DRONE DATA COLLECTOR NODE - Plan de développement

## Objectif
Collecter et enregistrer toutes les données de mission selon les spécifications du projet.

## Données à collecter
1. **Positions GPS** des fleurs détectées
2. **Images/vidéos** capturées lors des détections
3. **Logs de mission** (timestamps, événements, performances)
4. **Données de vol** (trajectoires, vitesses, altitudes)
5. **Résultats de pollinisation** (succès/échec, durée)

## Architecture proposée

### Services ROS2
- `/data/start_collection` - Démarre l'enregistrement
- `/data/stop_collection` - Arrête et sauvegarde
- `/data/log_event` - Enregistre un événement spécifique
- `/data/export_mission` - Exporte les données en format standard

### Topics ROS2
- `/data/mission_events` - Événements de mission en temps réel
- `/data/flower_database` - Base de données des fleurs trouvées
- `/data/flight_telemetry` - Télémétrie de vol

### Formats de stockage
- **JSON** pour les métadonnées et logs
- **CSV** pour les données tabulaires
- **Images** PNG/JPG avec géolocalisation
- **Base de données SQLite** pour les requêtes complexes

## Intégration
- Souscrit aux topics de tous les autres nœuds
- Triggered par les événements du `mission_node`
- Synchronisation des timestamps avec GPS
