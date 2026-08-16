from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
    Node(package='turtlesim', executable='turtlesim_node', name='sim'), 
    Node(package='my_py_pkg', executable='turtle_publisher', name='pub',
         parameters=[{'lin': 1.0, 'ang': 1.0}]),
    Node(package='my_py_pkg', executable='turtle_subscriber', name='sub'),
    ])