import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.bridge = CvBridge()
        # adapte le topic selon ta source camera
        self.sub = self.create_subscription(Image, '/camera/image_raw', self.on_img, 10)

    def on_img(self, msg: Image):
        try:
            cvimg = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            h, w = cvimg.shape[:2]
            self.get_logger().info_throttle(2.0, f'Image {w}x{h} reçue')
            # TODO: IA ici → publier consignes/infos
        except Exception as e:
            self.get_logger().error(f'CV Bridge error: {e}')

def main():
    rclpy.init()
    node = VisionNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
