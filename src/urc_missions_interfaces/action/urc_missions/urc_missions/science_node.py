import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from turtlesim.srv import TeleportAbsolute

class ScienceMission(Node):
    def __init__(self):
        super().__init__('science_mission')
        self.sub = self.create_subscription(String, 'mission/mode', self.mode_callback, 10)
        self.client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for turtlesim...')
        self.mode = 'idle'

    def mode_callback(self, msg):
        self.mode = msg.data
        if self.mode == 'science':
            self.do_science()

    def do_science(self):
        req = TeleportAbsolute.Request()
        req.x, req.y, req.theta = 5.0, 5.0, 0.0
        self.client.call_async(req)
        self.get_logger().info("Science: moved turtle to site (5,5).")
        with open("gnss_log.txt", "a") as f:
            f.write("GNSS: 23.78N, 90.42E\n")

def main():
    rclpy.init()
    node = ScienceMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
