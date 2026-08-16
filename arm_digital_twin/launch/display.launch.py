from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg = get_package_share_directory('arm_digital_twin')

    xacro_file = os.path.join(pkg, 'urdf', 'arm_digital_twin.urdf.xacro')
    rviz_cfg   = os.path.join(pkg, 'rviz', 'arm_digital_twin.rviz')

    robot_description = ParameterValue(
        Command(['xacro', ' ', xacro_file]),
        value_type=str
    )

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    desc_pub = Node(
        package='arm_digital_twin',
        executable='robot_description_pub',
        output='screen',
        parameters=[{'robot_description': robot_description}]
    )

    bridge = Node(
        package='arm_digital_twin',
        executable='cmd_to_joint_states',
        output='screen'
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_cfg]
    )

    return LaunchDescription([rsp, desc_pub, bridge, rviz])