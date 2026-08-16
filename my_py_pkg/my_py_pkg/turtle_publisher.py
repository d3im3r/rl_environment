import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

class TurtlePublisher(Node):
    def __init__(self):
        super().__init__('turtle_publisher')
        self.pub = self.create_publisher(Twist, '/turtle1/cmd_vel', 10)
        # Parámetros (velocidades por defecto)
        self.declare_parameter('lin', 1.0)
        self.declare_parameter('ang', 1.0)
        self.timer = self.create_timer(0.1, self.on_timer)
    def on_timer(self):
        lin = float(self.get_parameter('lin').value)
        ang = float(self.get_parameter('ang').value)
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self.pub.publish(msg)
        self.get_logger().info(f'cmd_vel -> lin: {lin:.2f}, ang: {ang:.2f}')
def main():
    rclpy.init()
    node = TurtlePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()