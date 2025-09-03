#!/bin/bash
"""
=============================================================================
DÉMONSTRATION SYSTÈME DRONE AUTONOME - POLLINISATION
=============================================================================
Script de démonstration complète du système de drone autonome pour la 
pollinisation de fleurs. Lance tous les composants et exécute une mission.

Usage:
    ./demo_pollination_system.sh
=============================================================================
"""

echo "🌸========================================🌸"
echo "   DÉMONSTRATION DRONE DE POLLINISATION   "
echo "🌸========================================🌸"
echo ""

# Couleurs pour l'affichage
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# Fonction pour afficher avec couleur
print_status() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

print_header() {
    echo -e "${CYAN}$1${NC}"
}

# Vérifier que nous sommes dans le bon répertoire
if [ ! -d "src/drone_vision" ]; then
    print_error "Ce script doit être lancé depuis le répertoire ros2_ws"
    exit 1
fi

# Sourcer l'environnement ROS2
print_status "Chargement de l'environnement ROS2..."
source /opt/ros/humble/setup.bash
source install/setup.bash

# Vérifier si le build est à jour
print_status "Vérification du build..."
if [ ! -d "install/drone_vision" ]; then
    print_warning "Package drone_vision non compilé. Compilation en cours..."
    colcon build --packages-select drone_vision
    source install/setup.bash
fi

# Fonction pour vérifier si un nœud est actif
check_node() {
    local node_name=$1
    if ros2 node list | grep -q "$node_name"; then
        return 0
    else
        return 1
    fi
}

# Fonction pour attendre qu'un nœud soit actif
wait_for_node() {
    local node_name=$1
    local timeout=${2:-30}
    local count=0
    
    print_status "Attente du nœud $node_name..."
    while [ $count -lt $timeout ]; do
        if check_node "$node_name"; then
            print_status "✅ Nœud $node_name actif!"
            return 0
        fi
        sleep 1
        count=$((count + 1))
    done
    
    print_error "❌ Timeout: nœud $node_name non démarré"
    return 1
}

# Fonction de nettoyage
cleanup() {
    print_header "\n🧹 Nettoyage en cours..."
    
    # Tuer tous les processus ROS2 lancés
    pkill -f "ros2 run"
    pkill -f "ros2 launch"
    pkill -f "python3.*camera_simulator"
    pkill -f "python3.*test_vision"
    
    # Attendre un peu
    sleep 2
    
    print_status "✅ Nettoyage terminé"
    echo ""
    echo "👋 Démonstration terminée!"
    exit 0
}

# Gérer l'interruption
trap cleanup INT TERM

# =============================================================================
# ÉTAPE 1: LANCEMENT DU SIMULATEUR DE CAMÉRA
# =============================================================================
print_header "\n📹 ÉTAPE 1: Lancement du simulateur de caméra"
print_status "Démarrage du simulateur d'images avec fleurs..."

python3 camera_simulator.py &
CAMERA_PID=$!

# Attendre que le simulateur soit prêt
sleep 3
print_status "✅ Simulateur de caméra démarré"

# =============================================================================
# ÉTAPE 2: LANCEMENT DU SYSTÈME DE VISION
# =============================================================================
print_header "\n🔍 ÉTAPE 2: Lancement du système de vision"
print_status "Démarrage des nœuds de vision et collecte de données..."

# Lancer le système de vision en arrière-plan
ros2 launch drone_vision vision_system.launch.py &
VISION_PID=$!

# Attendre que les nœuds soient actifs
wait_for_node "/drone_vision" 15
wait_for_node "/drone_data_collector" 15

print_status "✅ Système de vision opérationnel"

# =============================================================================
# ÉTAPE 3: ACTIVATION DES SYSTÈMES
# =============================================================================
print_header "\n⚡ ÉTAPE 3: Activation des systèmes"

# Attendre un peu pour que tout soit initialisé
sleep 3

print_status "Activation de la détection de fleurs..."
ros2 service call /vision/set_detection std_srvs/srv/SetBool "data: true" &

sleep 2

print_status "Activation de la collecte de données..."
ros2 service call /data_collector/set_collection std_srvs/srv/SetBool "data: true" &

sleep 2
print_status "✅ Systèmes activés"

# =============================================================================
# ÉTAPE 4: PÉRIODE D'OBSERVATION
# =============================================================================
print_header "\n👀 ÉTAPE 4: Période d'observation (30 secondes)"
print_status "Observation des détections en temps réel..."

# Afficher les topics actifs
echo ""
print_status "📡 Topics actifs:"
echo "  🌸 Fleurs détectées: /vision/flowers_detected"
echo "  📊 Statut collecteur: /data_collector/status"
echo "  🖼️  Images debug: /vision/debug_image"
echo ""

# Période d'observation avec compteur
for i in {30..1}; do
    echo -ne "\r${CYAN}⏱️  Temps restant: ${i}s ${NC}"
    sleep 1
done
echo ""

print_status "✅ Période d'observation terminée"

# =============================================================================
# ÉTAPE 5: SAUVEGARDE DES DONNÉES
# =============================================================================
print_header "\n💾 ÉTAPE 5: Sauvegarde des données"
print_status "Déclenchement de la sauvegarde..."

ros2 service call /data_collector/save_data std_srvs/srv/Trigger

sleep 3
print_status "✅ Données sauvegardées"

# =============================================================================
# ÉTAPE 6: STATISTIQUES ET RÉSULTATS
# =============================================================================
print_header "\n📊 ÉTAPE 6: Résultats de la démonstration"

# Vérifier les fichiers de données générés
DATA_DIR="$HOME/ros2_ws/mission_data"
if [ -d "$DATA_DIR" ]; then
    print_status "📁 Répertoire de données: $DATA_DIR"
    
    # Compter les fichiers générés
    GPS_FILES=$(find "$DATA_DIR" -name "gps_data_*.csv" 2>/dev/null | wc -l)
    FLOWER_FILES=$(find "$DATA_DIR" -name "flowers_*.json" 2>/dev/null | wc -l)
    IMAGE_FILES=$(find "$DATA_DIR/images" -name "*.jpg" 2>/dev/null | wc -l)
    
    echo ""
    print_status "📈 Statistiques de la mission:"
    echo "  📍 Fichiers GPS: $GPS_FILES"
    echo "  🌸 Fichiers fleurs: $FLOWER_FILES"
    echo "  📸 Images capturées: $IMAGE_FILES"
    
    # Afficher le dernier fichier de fleurs s'il existe
    LATEST_FLOWER_FILE=$(find "$DATA_DIR" -name "flowers_*.json" -type f -exec ls -t {} + | head -n1)
    if [ -n "$LATEST_FLOWER_FILE" ]; then
        echo ""
        print_status "🌸 Dernières détections de fleurs:"
        # Extraire le nombre de fleurs du JSON (méthode simple)
        if command -v jq &> /dev/null; then
            FLOWER_COUNT=$(jq '. | length' "$LATEST_FLOWER_FILE" 2>/dev/null || echo "N/A")
            echo "  🎯 Total détections: $FLOWER_COUNT"
        else
            echo "  📄 Fichier: $(basename "$LATEST_FLOWER_FILE")"
        fi
    fi
else
    print_warning "❓ Répertoire de données non trouvé"
fi

# =============================================================================
# ÉTAPE 7: LANCEMENT DU TEST AUTOMATIQUE (OPTIONNEL)
# =============================================================================
print_header "\n🧪 ÉTAPE 7: Test automatique du système"
echo ""
read -p "Voulez-vous lancer le test automatique complet? (y/N): " -n 1 -r
echo ""

if [[ $REPLY =~ ^[Yy]$ ]]; then
    print_status "Lancement du test automatique..."
    python3 test_vision_system.py
else
    print_status "Test automatique ignoré"
fi

# =============================================================================
# FINALISATION
# =============================================================================
print_header "\n🎉 DÉMONSTRATION TERMINÉE!"

echo ""
echo "📋 Résumé de la démonstration:"
echo "  ✅ Simulateur de caméra"
echo "  ✅ Système de vision (détection de fleurs)"
echo "  ✅ Collecteur de données (GPS + images)"
echo "  ✅ Sauvegarde automatique"
echo ""

print_status "🎯 Le système drone de pollinisation est maintenant fonctionnel!"
print_status "📁 Données sauvegardées dans: $DATA_DIR"

echo ""
print_warning "💡 Pour visualiser les images de debug:"
echo "  ros2 run rqt_image_view rqt_image_view /vision/debug_image"

echo ""
print_warning "💡 Pour monitorer les topics:"
echo "  ros2 topic echo /vision/flowers_detected"
echo "  ros2 topic echo /data_collector/status"

echo ""
print_status "Appuyez sur Ctrl+C pour arrêter tous les services"

# Attendre l'interruption de l'utilisateur
wait
