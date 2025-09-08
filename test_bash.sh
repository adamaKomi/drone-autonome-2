#!/bin/bash
echo "🚀 Test des fonctionnalités DroneInterface"
echo "=========================================="

echo "📋 1. Liste des services:"
ros2 service list | grep drone

echo -e "\n🔍 2. Test health check:"
ros2 service call /drone/health_check std_srvs/srv/Trigger "{}"

echo -e "\n📊 3. Statut du drone:"
timeout 3 ros2 topic echo /drone/status --once

echo -e "\n🎯 4. Test changement mode GUIDED:"
ros2 service call /drone/set_mode drone_msgs/srv/SetFlightMode "{flight_mode: 'GUIDED'}" --timeout 10

echo -e "\n🔫 5. Test armement:"
ros2 service call /drone/arm drone_msgs/srv/ArmDrone "{arm_mode: 'NORMAL', force_arm: false, skip_preflight: false}" --timeout 15

echo -e "\n✅ Tests terminés!"
