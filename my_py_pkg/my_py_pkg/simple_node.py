import rclpy
from rclpy.node import Node

def main():
    rclpy.init()
    node = Node("mi_nodo_simple")
    node.get_logger().info("Nodo en ejecución...")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

