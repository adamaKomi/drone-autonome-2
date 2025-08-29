import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped

class NavNode(Node):
    def __init__(self):
        super().__init__('nav_node')
        self.goal_sub = self.create_subscription(PoseStamped, '/goal', self.on_goal, 10)
        self.sp_pub   = self.create_publisher(PoseStamped, '/mavros/setpoint_position/local', 10)
        self.active_goal = None
        self.timer = self.create_timer(0.1, self._tick)  # 10 Hz

    def on_goal(self, msg: PoseStamped):
        self.active_goal = msg
        self.get_logger().info(f'Goal reçu: ({msg.pose.position.x:.1f},{msg.pose.position.y:.1f},{msg.pose.position.z:.1f})')

    def _tick(self):
        if self.active_goal is None:
            return
        # republie tel quel (pipeline minimal)
        self.active_goal.header.stamp = self.get_clock().now().to_msg()
        self.sp_pub.publish(self.active_goal)

def main():
    rclpy.init()
    node = NavNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
