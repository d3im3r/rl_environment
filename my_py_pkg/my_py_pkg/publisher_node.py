import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32

class PublisherNode(Node):
    def __init__(self):
        super().__init__('publisher_node')
        self.publisher = self.create_publisher(Int32, 'contador', 10)
        self.timer = self.create_timer(1.0, self.publish_msg)
        self.count = 0
    def publish_msg(self):
        msg = Int32()
        msg.data = self.count
        self.publisher.publish(msg)
        self.get_logger().info(f'Publicado: {msg.data}')
        self.count += 1

def main():
    rclpy.init()
    node = PublisherNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()