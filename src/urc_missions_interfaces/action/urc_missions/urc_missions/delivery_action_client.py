import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from urc_missions.action import DeliverItem

class DeliveryClient(Node):
    def __init__(self):
        super().__init__('delivery_client')
        self._client = ActionClient(self, DeliverItem, 'delivery/deliver_item')

    def send_goal(self, item, x, y):
        goal = DeliverItem.Goal()
        goal.item, goal.x, goal.y = item, x, y
        self._client.wait_for_server()
        self._client.send_goal_async(goal)

def main():
    rclpy.init()
    node = DeliveryClient()
    node.send_goal("sample", 5.0, 5.0)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
