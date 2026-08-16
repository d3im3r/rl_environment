import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class NameSubNode(Node):
    def __init__(self):
        super().__init__('name_sub_node')
        self.subscription = self.create_subscription(String, 
                    '/name',self.listener_callback, 10)

    def listener_callback(self, msg):
        self.get_logger().info(f'He recibido el mensaje: {msg.data}')

def main():
    rclpy.init()
    node = NameSubNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()