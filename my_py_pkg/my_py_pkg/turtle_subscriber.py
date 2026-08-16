import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose
class TurtlePoseSubscriber(Node):
    def __init__(self):
        super().__init__('turtle_subscriber')
        self.sub = self.create_subscription(Pose, '/turtle1/pose', self.on_pose, 10)
        self.last_x = None
    
    def on_pose(self, msg: Pose):
        if self.last_x is None or abs(msg.x - self.last_x) > 0.01:
            self.get_logger().info(f'Pose -> x:{msg.x:.2f} y:{msg.y:.2f}, th:{msg.theta:.2f} 'f'v:{msg.linear_velocity:.2f}, w:{msg.angular_velocity:.2f}')
            self.last_x = msg.x
def main():
    rclpy.init()
    node = TurtlePoseSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()