# drone_navigation/test/test_navigation.py
import unittest
import rclpy
from rclpy.lifecycle import LifecycleNode

from drone_navigation.navigation_node import NavigationNode

class TestNavigation(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.node = NavigationNode()
        self.node.trigger_transition(rclpy.lifecycle.Transition(id=1))  # Configure

    def test_lifecycle_management(self):
        self.assertTrue(self.node.get_current_state().label == 'inactive')

    # Ajouter autres tests: position_accuracy, waypoint_navigation, etc.