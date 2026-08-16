from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():
    publish_tf = LaunchConfiguration('publish_tf')
    start_teleop = LaunchConfiguration('start_teleop')

    declare_publish_tf = DeclareLaunchArgument(
        'publish_tf',
        default_value='true',
        description='Publish TF odom -> base_link from odom bridge.',
    )

    declare_start_teleop = DeclareLaunchArgument(
        'start_teleop',
        default_value='false',
        description='Start safe teleop node.',
    )

    odom_bridge_node = Node(
        package='d3im3r_bot_bringup',
        executable='odom_bridge',
        name='d3im3r_odom_bridge',
        output='screen',
        parameters=[
            {
                'odom_pose_topic': '/odom_pose',
                'odom_twist_topic': '/odom_twist',
                'odom_topic': '/odom',
                'reset_topic': '/reset_odom_topic',
                'reset_service': '/reset_odom',
                'odom_frame': 'odom',
                'base_frame': 'base_link',
                'publish_tf': publish_tf,
                'publish_rate_hz': 30.0,
            }
        ],
    )

    safe_teleop_node = Node(
        package='d3im3r_bot_bringup',
        executable='safe_teleop',
        name='d3im3r_safe_teleop',
        output='screen',
        parameters=[
            {
                'cmd_vel_topic': '/cmd_vel',
                'tof_topic': '/tof_distances_m',
                'safety_status_topic': '/safety_status',
                'publish_rate_hz': 20.0,
                'max_linear_m_s': 0.12,
                'max_angular_rad_s': 0.8,
                'linear_step_m_s': 0.02,
                'angular_step_rad_s': 0.10,
                'front_stop_m': 0.12,
                'front_slow_m': 0.30,
                'side_stop_m': 0.10,
                'side_slow_m': 0.18,
                'enable_pc_safety_filter': True,
            }
        ],
        condition=None,
    )

    # Nota:
    # Para mantener simple este launch inicial, el nodo de teleop
    # queda declarado abajo. Si no quieres iniciarlo con launch,
    # puedes correrlo manualmente con:
    #
    # ros2 run d3im3r_bot_bringup safe_teleop
    #
    # En una siguiente iteración podemos agregar una condición real
    # con IfCondition(start_teleop).

    return LaunchDescription(
        [
            declare_publish_tf,
            declare_start_teleop,
            odom_bridge_node,
            # safe_teleop_node,
        ]
    )