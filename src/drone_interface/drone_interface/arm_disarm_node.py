#!/usr/bin/env python3
"""
Nœud ROS2 pour armer/désarmer un drone ArduPilot via MAVROS
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from mavros_msgs.srv import CommandBool

class ArmDisarmNode(Node):
    def __init__(self):
        super().__init__('arm_disarm_node')
        
        # Souscription au topic au lieu d'argument
        self.arm_sub = self.create_subscription(
            Bool,
            '/arm_control/arm_cmd',
            self.arm_callback,
            10
        )
        
        self.disarm_sub = self.create_subscription(
            Bool,
            '/arm_control/disarm_cmd',
            self.disarm_callback,
            10
        )
        
        # Client service MAVROS
        self.arm_client = self.create_client(CommandBool, '/mavros/cmd/arming')
        
        self.get_logger().info("Nœud arm/disarm prêt")
    
    def arm_callback(self, msg):
        if msg.data:
            self.send_arm_command(True)
    
    def disarm_callback(self, msg):
        if msg.data:
            self.send_arm_command(False)
    
    def send_arm_command(self, arm):
        if not self.arm_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service arm/disarm non disponible")
            return
        
        request = CommandBool.Request()
        request.value = arm
        
        future = self.arm_client.call_async(request)
        future.add_done_callback(lambda f: self.arm_response_callback(f, arm))
    
    def arm_response_callback(self, future, arm):
        try:
            response = future.result()
            action = "armement" if arm else "désarmement"
            if response.success:
                self.get_logger().info(f"✅ {action.capitalize()} réussi")
            else:
                self.get_logger().error(f"❌ {action.capitalize()} échoué")
        except Exception as e:
            self.get_logger().error(f"⚠️ Erreur {action}: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ArmDisarmNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()