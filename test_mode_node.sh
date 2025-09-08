#!/bin/bash

# Script de test pour le mode_node
echo "=== Test du mode_node ===" 

echo "1. Vérification de l'état MAVROS..."
ros2 topic echo /mavros/state --once

echo "2. Test changement vers GUIDED..."
ros2 service call /drone/set_flight_mode drone_msgs/srv/SetFlightMode "{flight_mode: 'GUIDED'}" --once

echo "3. Attendre 2 secondes..."
sleep 2

echo "4. Test changement vers STABILIZE..."
ros2 service call /drone/set_flight_mode drone_msgs/srv/SetFlightMode "{flight_mode: 'STABILIZE'}" --once

echo "5. Affichage du statut du mode..."
ros2 topic echo /drone/mode_status --once

echo "=== Test terminé ==="
