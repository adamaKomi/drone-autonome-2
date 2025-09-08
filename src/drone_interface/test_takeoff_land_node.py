import unittest
from unittest.mock import MagicMock
import rclpy
from std_msgs.msg import Bool
from mavros_msgs.msg import State
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Float64

from drone_interface.drone_interface.takeoff_land_node import TakeoffLandNode

class TestTakeoffLandNode(unittest.TestCase):
    def setUp(self):
        rclpy.init()
        self.node = TakeoffLandNode()

    def tearDown(self):
        self.node.destroy_node()
        rclpy.shutdown()

    def test_takeoff_callback_not_armed(self):
        # Simule un état non armé
        state = State()
        state.connected = True
        state.mode = 'GUIDED'
        state.armed = False
        self.node.mavros_state_callback(state)
        msg = Bool()
        msg.data = True
        # Doit refuser le décollage
        self.node.get_logger = MagicMock()
        self.node.takeoff_callback(msg)
        self.node.get_logger().error.assert_any_call("🔒 Le drone n'est pas armé")

    def test_takeoff_callback_nan_position(self):
        # Simule un état armé et position locale NaN
        state = State()
        state.connected = True
        state.mode = 'GUIDED'
        state.armed = True
        self.node.mavros_state_callback(state)
        pose = PoseStamped()
        pose.pose.position.x = float('nan')
        pose.pose.position.y = 0.0
        pose.pose.position.z = 2.0
        self.node.local_pose_callback(pose)
        self.node.rel_alt_callback(Float64(data=10.0))
        msg = Bool()
        msg.data = True
        self.node.get_logger = MagicMock()
        self.node.takeoff_callback(msg)
        self.node.get_logger().error.assert_any_call('❌ Position locale non valide (NaN)')

    def test_takeoff_callback_nan_altitude(self):
        # Simule un état armé et altitude NaN
        state = State()
        state.connected = True
        state.mode = 'GUIDED'
        state.armed = True
        self.node.mavros_state_callback(state)
        pose = PoseStamped()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 2.0
        self.node.local_pose_callback(pose)
        self.node.rel_alt_callback(Float64(data=float('nan')))
        msg = Bool()
        msg.data = True
        self.node.get_logger = MagicMock()
        self.node.takeoff_callback(msg)
        self.node.get_logger().error.assert_any_call('❌ Altitude relative non valide (NaN ou 0.0)')

    def test_takeoff_callback_success(self):
        # Simule un état correct
        state = State()
        state.connected = True
        state.mode = 'GUIDED'
        state.armed = True
        self.node.mavros_state_callback(state)
        pose = PoseStamped()
        pose.pose.position.x = 0.0
        pose.pose.position.y = 0.0
        pose.pose.position.z = 2.0
        self.node.local_pose_callback(pose)
        self.node.rel_alt_callback(Float64(data=10.0))
        msg = Bool()
        msg.data = True
        self.node.get_logger = MagicMock()
        # On mock aussi le client pour éviter l'appel réel
        self.node.takeoff_client = MagicMock()
        self.node.takeoff_client.wait_for_service.return_value = True
        self.node.takeoff_client.call_async.return_value = MagicMock()
        self.node.takeoff_callback(msg)
        self.node.get_logger().info.assert_any_call('🛫 Décollage demandé')

if __name__ == '__main__':
    unittest.main()
