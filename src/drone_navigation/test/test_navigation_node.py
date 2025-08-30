#!/usr/bin/env python3

import rclpy
from drone_navigation.navigation_node import NavigationNode

def test_node_initialization():
    """Test que le nœud peut être initialisé"""
    rclpy.init()
    try:
        node = NavigationNode()
        assert node is not None
        print("✅ Test d'initialisation réussi")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    test_node_initialization()