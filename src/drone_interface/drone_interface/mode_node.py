#!/usr/bin/env python3
"""
Nœud ROS2 pour changer le mode de vol d'un drone ArduPilot via MAVROS
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from mavros_msgs.srv import SetMode


class ModeNode(Node):
    def __init__(self):
        super().__init__('mode_node')
        
        # Souscription au topic au lieu d'argument
        self.mode_sub = self.create_subscription(
            String,
            '/mode_command',
            self.mode_callback,
            10
        )
        
        # Client service MAVROS
        self.mode_client = self.create_client(SetMode, '/mavros/set_mode')
        
        self.get_logger().info("Nœud mode_change prêt")
    
    def mode_callback(self, msg):
        mode = msg.data.upper()
        self.change_mode(mode)
    
    def change_mode(self, mode):
        if not self.mode_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error("Service set_mode non disponible")
            return
        
        request = SetMode.Request()
        request.custom_mode = mode
        
        future = self.mode_client.call_async(request)
        future.add_done_callback(lambda f: self.mode_response_callback(f, mode))
    
    def mode_response_callback(self, future, mode):
        try:
            response = future.result()
            if response.mode_sent:
                self.get_logger().info(f"✅ Mode changé vers {mode}")
            else:
                self.get_logger().error(f"❌ Changement de mode vers {mode} échoué")
        except Exception as e:
            self.get_logger().error(f"⚠️ Erreur changement mode: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ModeNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()