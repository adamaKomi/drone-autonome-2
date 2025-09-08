#!/usr/bin/env python3
"""
Script de test rapide pour diagnostiquer les services
"""

import rclpy
from rclpy.node import Node
from drone_msgs.srv import SetFlightMode
from std_srvs.srv import Trigger

def test_services():
    rclpy.init()
    node = Node('test_node')
    
    # Test health check
    print("🔍 Test health check...")
    health_client = node.create_client(Trigger, '/drone/health_check')
    if health_client.wait_for_service(timeout_sec=5.0):
        request = Trigger.Request()
        future = health_client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
        if future.result():
            print(f"✅ Health: {future.result().message}")
        else:
            print("❌ Health timeout")
    else:
        print("❌ Health service not available")
    
    # Test mode change
    print("\n🎯 Test changement de mode...")
    mode_client = node.create_client(SetFlightMode, '/drone/set_mode')
    if mode_client.wait_for_service(timeout_sec=5.0):
        request = SetFlightMode.Request()
        request.flight_mode = "GUIDED"
        request.sub_mode = ""
        request.max_speed = 10.0
        request.max_altitude = 100.0
        request.max_distance = 1000.0
        request.enable_obstacle_avoidance = True
        request.enable_geofence = True  
        request.enable_return_to_launch = True
        request.failsafe_altitude = 20.0
        request.operator_id = "test"
        request.override_restrictions = False
        request.custom_parameters = []
        
        future = mode_client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=10.0)
        if future.result():
            result = future.result()
            print(f"✅ Mode change: success={result.success}, message={result.message}")
        else:
            print("❌ Mode change timeout")
    else:
        print("❌ Mode service not available")
    
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    test_services()
