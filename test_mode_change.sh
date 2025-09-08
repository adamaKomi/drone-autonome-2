#!/bin/bash
echo "🔄 Compilation du drone_interface..."
cd /home/adama133/ros2_ws
colcon build --packages-select drone_interface
echo "✅ Compilation terminée"

echo "🔄 Source de l'environnement..."
source install/setup.bash
echo "✅ Environnement sourcé"

echo "🚀 Test changement de mode GUIDED..."
ros2 service call /drone/set_mode drone_msgs/srv/SetFlightMode "{flight_mode: 'GUIDED'}" --timeout 10

echo "📊 Vérification du statut..."
ros2 topic echo /drone/status --once | grep flight_mode
