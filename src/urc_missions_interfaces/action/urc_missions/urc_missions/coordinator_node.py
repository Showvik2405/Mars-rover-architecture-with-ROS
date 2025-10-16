import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rcl_interfaces.msg import SetParametersResult  

class Coordinator(Node):
    def __init__(self):
        super().__init__('coordinator')
        # Declare a parameter that defines which mission mode we are in
        self.declare_parameter('mission_mode', 'science')

        # Publisher to broadcast the current mission mode
        self.mode_pub = self.create_publisher(String, 'mission/mode', 10)

        # Callback for when parameters are updated (e.g. mission_mode is changed)
        self.add_on_set_parameters_callback(self._on_param)

        # Publish the initial mode
        self._publish_mode(self.get_parameter('mission_mode').value)

    def _on_param(self, params):
        """Called whenever a parameter is updated"""
        for p in params:
            if p.name == 'mission_mode':
                self._publish_mode(p.value)
        return SetParametersResult(successful=True)

    def _publish_mode(self, mode: str):
        """Broadcast the mission mode on a topic"""
        msg = String()
        msg.data = mode
        self.mode_pub.publish(msg)
        self.get_logger().info(f"[Coordinator] Mode switched to: {mode}")

def main():
    rclpy.init()
    node = Coordinator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
