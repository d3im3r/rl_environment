import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32


class SubscriberNode(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.subscription = self.create_subscription(Int32, 
                    'contador',self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'Recibido: {msg.data}')

def main():
    rclpy.init()
    node = SubscriberNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()