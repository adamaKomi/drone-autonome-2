#!/bin/bash

# Script de test pour la coordination des nœuds de navigation

echo "🚁 Testing Drone Navigation System Coordination 🚁"
echo "=================================================="

# Vérifier que ROS2 est configuré
if ! command -v ros2 &> /dev/null; then
    echo "❌ ROS2 not found. Please source your ROS2 setup."
    exit 1
fi

# Sourcer le workspace
cd /home/adama133/ros2_ws
source install/setup.bash

echo "✅ Workspace sourced"

# Construire le package si nécessaire
echo "🔨 Building navigation package..."
colcon build --packages-select drone_navigation --cmake-clean-cache

if [ $? -ne 0 ]; then
    echo "❌ Build failed. Please check for errors."
    exit 1
fi

echo "✅ Build successful"

# Re-sourcer après le build
source install/setup.bash

echo ""
echo "🚀 Starting navigation system test..."
echo ""

# Lancer les nœuds en arrière-plan
echo "Starting navigation safety node..."
ros2 run drone_navigation navigation_safety_node &
SAFETY_PID=$!

sleep 2

echo "Starting GPS navigation node..."
ros2 run drone_navigation gps_navigation_node &
GPS_PID=$!

sleep 2

echo "Starting local navigation node..."
ros2 run drone_navigation local_navigation_node &
LOCAL_PID=$!

sleep 2

echo "Starting waypoint manager node..."
ros2 run drone_navigation waypoint_manager_node &
WP_PID=$!

sleep 2

echo "Starting mission manager node..."
ros2 run drone_navigation mission_manager_node &
MISSION_PID=$!

sleep 2

echo "Starting emergency handler node..."
ros2 run drone_navigation emergency_handler_node &
EMERGENCY_PID=$!

sleep 2

echo "Starting navigation supervisor node..."
ros2 run drone_navigation navigation_supervisor_node &
SUPERVISOR_PID=$!

sleep 5

echo ""
echo "🧪 All nodes started. Running coordination test..."
echo ""

# Lancer le test
python3 /home/adama133/ros2_ws/src/drone_navigation/scripts/test_coordination.py

echo ""
echo "🛑 Stopping all nodes..."

# Arrêter tous les nœuds
kill $SAFETY_PID $GPS_PID $LOCAL_PID $WP_PID $MISSION_PID $EMERGENCY_PID $SUPERVISOR_PID 2>/dev/null

sleep 2

echo "✅ Test completed!"
echo ""
echo "📋 To run individual tests:"
echo "  ros2 launch drone_navigation full_navigation.launch.py"
echo "  python3 /home/adama133/ros2_ws/src/drone_navigation/scripts/test_coordination.py"
echo ""
echo "📋 To test with MAVROS simulation:"
echo "  1. Start ArduPilot SITL: sim_vehicle.py -v ArduCopter --console --map --out=127.0.0.1:14550"
echo "  2. Start MAVROS: ros2 launch mavros apm.launch fcu_url:=udp://127.0.0.1:14550@14555"
echo "  3. Run this test script"
