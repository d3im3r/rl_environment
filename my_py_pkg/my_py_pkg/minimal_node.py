import rclpy   
from rclpy.node import Node

class MinimalNode(Node):
    def __init__(self):
        super().__init__('minimal_node')
        self.timer = self.create_timer(1.0, 
                        self.on_timer)
        self.count=0

    def on_timer(self):
        self.count += 1
        self.get_logger().info(f'Hello ROS2: {self.count}')

def main():
    rclpy.init()
    node = MinimalNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


