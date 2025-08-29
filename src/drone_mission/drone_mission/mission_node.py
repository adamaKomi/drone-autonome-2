import time
import yaml
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Header

class MissionNode(Node):
    def __init__(self):
        super().__init__('mission_node')
        self.declare_parameter('mission_file', 'mission.yaml')
        self.declare_parameter('frame_id', 'map')

        self.pub = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.timer = self.create_timer(0.1, self._tick)

        self.points = []
        self.rate_hz = 10.0
        self.hold_time_s = 3.0
        self.idx = 0
        self.last_switch = self.get_clock().now()

        self._load()

    def _load(self):
        pkg_share = self.get_parameter('mission_file').get_parameter_value().string_value
        try:
            with open(pkg_share, 'r') as f:
                data = yaml.safe_load(f)
            self.points = data.get('points', [])
            self.rate_hz = float(data.get('rate_hz', 10))
            self.hold_time_s = float(data.get('hold_time_s', 3))
            self.get_logger().info(f'{len(self.points)} waypoints chargés')
        except Exception as e:
            self.get_logger().error(f'Erreur chargement mission: {e}')

    def _tick(self):
        if not self.points:
            return
        x, y, z = self.points[self.idx]
        msg = PoseStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = float(z)
        msg.pose.orientation.w = 1.0
        self.pub.publish(msg)

        if (self.get_clock().now() - self.last_switch).nanoseconds * 1e-9 > self.hold_time_s:
            self.idx = (self.idx + 1) % len(self.points)
            self.last_switch = self.get_clock().now()
            self.get_logger().info(f'Next WP idx={self.idx}')

def main():
    rclpy.init()
    node = MissionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
