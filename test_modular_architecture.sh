#!/bin/bash
"""
=============================================================================
SCRIPT DE TEST - Architecture Modulaire Drone
=============================================================================
Auteur: Assistant IA
Date: 2025-08-29

Description:
    Script de test pour valider l'architecture modulaire du système drone.
    Lance les nœuds séparément et teste les communications inter-nœuds.
    
Utilisation:
    ./test_modular_architecture.sh [start|stop|test|status]
=============================================================================
"""

# Couleurs pour l'affichage
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
PURPLE='\033[0;35m'
CYAN='\033[0;36m'
WHITE='\033[1;37m'
NC='\033[0m' # No Color

# Configuration
ROS2_WS_PATH="/home/adama133/ros2_ws"
LOG_DIR="$ROS2_WS_PATH/logs/test_$(date +%Y%m%d_%H%M%S)"

# Fonction d'affichage
print_header() {
    echo -e "${BLUE}============================================${NC}"
    echo -e "${WHITE}  $1${NC}"
    echo -e "${BLUE}============================================${NC}"
}

print_success() {
    echo -e "${GREEN}✅ $1${NC}"
}

print_error() {
    echo -e "${RED}❌ $1${NC}"
}

print_warning() {
    echo -e "${YELLOW}⚠️  $1${NC}"
}

print_info() {
    echo -e "${CYAN}ℹ️  $1${NC}"
}

# Fonction de vérification ROS2
check_ros2_environment() {
    print_header "Vérification de l'environnement ROS2"
    
    # Vérifier que ROS2 est sourcé
    if [ -z "$ROS_DISTRO" ]; then
        print_error "ROS2 n'est pas sourcé. Exécutez: source /opt/ros/humble/setup.bash"
        return 1
    fi
    print_success "ROS2 $ROS_DISTRO détecté"
    
    # Vérifier le workspace
    if [ -f "$ROS2_WS_PATH/install/setup.bash" ]; then
        source "$ROS2_WS_PATH/install/setup.bash"
        print_success "Workspace sourcé: $ROS2_WS_PATH"
    else
        print_error "Workspace non trouvé ou non compilé"
        return 1
    fi
    
    return 0
}

# Fonction de démarrage des nœuds
start_nodes() {
    print_header "Démarrage des nœuds modulaires"
    
    # Créer le répertoire de logs
    mkdir -p "$LOG_DIR"
    
    # Démarrer le nœud interface (central)
    print_info "Démarrage du nœud interface central..."
    ros2 run drone_interface interface_node > "$LOG_DIR/interface.log" 2>&1 &
    INTERFACE_PID=$!
    echo $INTERFACE_PID > "$LOG_DIR/interface.pid"
    sleep 2
    
    if kill -0 $INTERFACE_PID 2>/dev/null; then
        print_success "Nœud interface démarré (PID: $INTERFACE_PID)"
    else
        print_error "Échec démarrage nœud interface"
        return 1
    fi
    
    # Démarrer le nœud navigation
    print_info "Démarrage du nœud navigation..."
    ros2 run drone_navigation navigation_node > "$LOG_DIR/navigation.log" 2>&1 &
    NAVIGATION_PID=$!
    echo $NAVIGATION_PID > "$LOG_DIR/navigation.pid"
    sleep 2
    
    if kill -0 $NAVIGATION_PID 2>/dev/null; then
        print_success "Nœud navigation démarré (PID: $NAVIGATION_PID)"
    else
        print_error "Échec démarrage nœud navigation"
        return 1
    fi
    
    # Démarrer le nœud mission
    print_info "Démarrage du nœud mission..."
    ros2 run drone_mission mission_node > "$LOG_DIR/mission.log" 2>&1 &
    MISSION_PID=$!
    echo $MISSION_PID > "$LOG_DIR/mission.pid"
    sleep 2
    
    if kill -0 $MISSION_PID 2>/dev/null; then
        print_success "Nœud mission démarré (PID: $MISSION_PID)"
    else
        print_error "Échec démarrage nœud mission"
        return 1
    fi
    
    print_success "Tous les nœuds sont démarrés"
    print_info "Logs disponibles dans: $LOG_DIR"
    
    return 0
}

# Fonction d'arrêt des nœuds
stop_nodes() {
    print_header "Arrêt des nœuds"
    
    # Chercher et arrêter tous les nœuds
    for node in interface_node navigation_node mission_node; do
        PIDS=$(pgrep -f "$node")
        if [ ! -z "$PIDS" ]; then
            print_info "Arrêt de $node..."
            echo $PIDS | xargs kill -TERM
            sleep 2
            # Force kill si nécessaire
            PIDS=$(pgrep -f "$node")
            if [ ! -z "$PIDS" ]; then
                echo $PIDS | xargs kill -KILL
            fi
            print_success "$node arrêté"
        else
            print_warning "$node n'était pas en cours d'exécution"
        fi
    done
    
    return 0
}

# Fonction de test des communications
test_communications() {
    print_header "Test des communications inter-nœuds"
    
    # Vérifier que les nœuds sont actifs
    print_info "Vérification des nœuds actifs..."
    ros2 node list > /tmp/nodes.txt
    
    if grep -q "drone_interface" /tmp/nodes.txt; then
        print_success "Nœud interface détecté"
    else
        print_error "Nœud interface non trouvé"
        return 1
    fi
    
    if grep -q "drone_navigation" /tmp/nodes.txt; then
        print_success "Nœud navigation détecté"
    else
        print_error "Nœud navigation non trouvé"
        return 1
    fi
    
    if grep -q "drone_mission" /tmp/nodes.txt; then
        print_success "Nœud mission détecté"
    else
        print_error "Nœud mission non trouvé"
        return 1
    fi
    
    # Tester les topics
    print_info "Vérification des topics..."
    ros2 topic list > /tmp/topics.txt
    
    TOPICS_TO_CHECK=(
        "/drone/status"
        "/drone/armed"
        "/drone/flight_mode"
        "/drone/battery_percentage"
        "/drone/navigation/status"
        "/drone/mission/status"
    )
    
    for topic in "${TOPICS_TO_CHECK[@]}"; do
        if grep -q "$topic" /tmp/topics.txt; then
            print_success "Topic $topic disponible"
        else
            print_warning "Topic $topic non trouvé"
        fi
    done
    
    # Tester les services
    print_info "Vérification des services..."
    ros2 service list > /tmp/services.txt
    
    SERVICES_TO_CHECK=(
        "/drone/control"
        "/drone/safety_check"
        "/drone/navigation/control"
        "/drone/navigation/goto"
        "/drone/mission/control"
        "/drone/mission/load"
    )
    
    for service in "${SERVICES_TO_CHECK[@]}"; do
        if grep -q "$service" /tmp/services.txt; then
            print_success "Service $service disponible"
        else
            print_warning "Service $service non trouvé"
        fi
    done
    
    return 0
}

# Fonction de test des services
test_services() {
    print_header "Test des appels de services"
    
    # Test du service de contrôle interface
    print_info "Test du service de contrôle interface..."
    RESULT=$(ros2 service call /drone/control std_msgs/srv/SetString "{data: 'ARM'}" --timeout 5)
    if [ $? -eq 0 ]; then
        print_success "Service /drone/control répond"
    else
        print_error "Service /drone/control ne répond pas"
    fi
    
    # Test du service de navigation
    print_info "Test du service de navigation..."
    RESULT=$(ros2 service call /drone/navigation/goto std_msgs/srv/SetString "{data: '1.0:1.0:3.0:0.0'}" --timeout 5)
    if [ $? -eq 0 ]; then
        print_success "Service /drone/navigation/goto répond"
    else
        print_error "Service /drone/navigation/goto ne répond pas"
    fi
    
    # Test du service de mission
    print_info "Test du service de mission..."
    RESULT=$(ros2 service call /drone/mission/load std_msgs/srv/SetString "{data: 'test_mission'}" --timeout 5)
    if [ $? -eq 0 ]; then
        print_success "Service /drone/mission/load répond"
    else
        print_error "Service /drone/mission/load ne répond pas"
    fi
    
    return 0
}

# Fonction de vérification du statut
check_status() {
    print_header "Statut du système modulaire"
    
    # Vérifier les processus
    print_info "Processus actifs:"
    for node in interface_node navigation_node mission_node; do
        PIDS=$(pgrep -f "$node")
        if [ ! -z "$PIDS" ]; then
            print_success "$node: PID $PIDS"
        else
            print_warning "$node: Non actif"
        fi
    done
    
    # Afficher les topics actifs
    print_info "Topics avec données:"
    ros2 topic list | grep "/drone" | while read topic; do
        # Vérifier si le topic publie (timeout court)
        timeout 2 ros2 topic echo "$topic" --once >/dev/null 2>&1
        if [ $? -eq 0 ]; then
            print_success "$topic: Actif"
        else
            print_warning "$topic: Pas de données"
        fi
    done
    
    return 0
}

# Fonction d'aide
show_help() {
    echo -e "${WHITE}Script de test Architecture Modulaire Drone${NC}"
    echo ""
    echo -e "${CYAN}Usage:${NC}"
    echo "  $0 [COMMAND]"
    echo ""
    echo -e "${CYAN}Commands:${NC}"
    echo -e "  ${GREEN}start${NC}     - Démarre tous les nœuds modulaires"
    echo -e "  ${GREEN}stop${NC}      - Arrête tous les nœuds"
    echo -e "  ${GREEN}test${NC}      - Lance les tests de communication"
    echo -e "  ${GREEN}services${NC}  - Teste les appels de services"
    echo -e "  ${GREEN}status${NC}    - Affiche le statut du système"
    echo -e "  ${GREEN}help${NC}      - Affiche cette aide"
    echo ""
    echo -e "${CYAN}Exemples:${NC}"
    echo "  $0 start          # Démarre l'architecture"
    echo "  $0 test           # Teste les communications"
    echo "  $0 stop           # Arrête tout"
    echo ""
}

# Fonction principale
main() {
    case "${1:-help}" in
        "start")
            check_ros2_environment && start_nodes
            ;;
        "stop")
            stop_nodes
            ;;
        "test")
            check_ros2_environment && test_communications
            ;;
        "services")
            check_ros2_environment && test_services
            ;;
        "status")
            check_ros2_environment && check_status
            ;;
        "help"|"--help"|"-h")
            show_help
            ;;
        *)
            print_error "Commande inconnue: $1"
            show_help
            exit 1
            ;;
    esac
}

# Point d'entrée
main "$@"
