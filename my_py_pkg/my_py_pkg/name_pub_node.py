import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class NameNode(Node):
    def __init__(self):
        super().__init__('name_node')
        self.publisher = self.create_publisher(String, 
                                               '/name', 10)
        self.timer = self.create_timer(0.5, self.publish_name)

    def publish_name(self):
        name = String()
        name.data = 'Deimer'
        self.publisher.publish(name)
        self.get_logger().info(f'Publicado: {name.data}')

def main():
    rclpy.init()
    node = NameNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()