import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point

class VisionNode(Node):
    def __init__(self):
        super().__init__('vision_node')
        self.pub = self.create_publisher(Point, 'vision/target', 10)
        self.timer = self.create_timer(5.0, self.publish_target)

    def publish_target(self):
        pt = Point()
        pt.x, pt.y, pt.z = 3.0, 7.0, 0.0
        self.pub.publish(pt)
        self.get_logger().info("Published dummy vision target (3,7)")

def main():
    rclpy.init()
    node = VisionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
