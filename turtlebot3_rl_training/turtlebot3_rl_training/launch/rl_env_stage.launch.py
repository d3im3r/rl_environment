import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_custom_worlds = get_package_share_directory('turtlebot3_custom_worlds')

    custom_world_launch = os.path.join(
        pkg_custom_worlds,
        'launch',
        'd3im3r_world.launch.py'
    )

    # ------------------------------------------------------------
    # Argumentos generales
    # ------------------------------------------------------------
    stage_arg = DeclareLaunchArgument(
        'stage',
        default_value='1',
        description='Custom world stage number.'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI.'
    )

    # ------------------------------------------------------------
    # Argumentos para la interfaz RL
    # ------------------------------------------------------------
    max_sensor_range_arg = DeclareLaunchArgument(
        'max_sensor_range',
        default_value='3.5',
        description='Maximum laser sensor range used for normalization.'
    )

    max_goal_distance_arg = DeclareLaunchArgument(
        'max_goal_distance',
        default_value='5.0',
        description='Maximum goal distance used for normalization.'
    )

    goal_tolerance_arg = DeclareLaunchArgument(
        'goal_tolerance',
        default_value='0.30',
        description='Goal tolerance in meters.'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='10.0',
        description='RL state publishing rate.'
    )

    # ------------------------------------------------------------
    # Argumentos para el controlador de movimiento de 5 acciones
    # ------------------------------------------------------------
    control_rate_arg = DeclareLaunchArgument(
        'control_rate',
        default_value='30.0',
        description='Control loop rate for RL motion controller.'
    )

    action_duration_arg = DeclareLaunchArgument(
        'action_duration',
        default_value='0.60',
        description='Duration of each discrete action in seconds.'
    )

    action_timeout_arg = DeclareLaunchArgument(
        'action_timeout',
        default_value='2.0',
        description='Maximum time allowed for one discrete action.'
    )

    linear_speed_forward_arg = DeclareLaunchArgument(
        'linear_speed_forward',
        default_value='0.18',
        description='Linear speed for forward action.'
    )

    linear_speed_soft_turn_arg = DeclareLaunchArgument(
        'linear_speed_soft_turn',
        default_value='0.14',
        description='Linear speed for soft turn actions.'
    )

    angular_speed_soft_turn_arg = DeclareLaunchArgument(
        'angular_speed_soft_turn',
        default_value='0.60',
        description='Angular speed for soft turn actions.'
    )

    linear_speed_hard_turn_arg = DeclareLaunchArgument(
        'linear_speed_hard_turn',
        default_value='0.10',
        description='Linear speed for hard turn actions.'
    )

    angular_speed_hard_turn_arg = DeclareLaunchArgument(
        'angular_speed_hard_turn',
        default_value='1.20',
        description='Angular speed for hard turn actions.'
    )

    # ------------------------------------------------------------
    # Gazebo + mundo personalizado
    # ------------------------------------------------------------
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(custom_world_launch),
        launch_arguments={
            'stage': LaunchConfiguration('stage'),
            'gui': LaunchConfiguration('gui'),
            'x_pose': '-1.5',
            'y_pose': '0.0',
            'z_pose': '0.01',
            'yaw': '0.0',
        }.items()
    )

    # ------------------------------------------------------------
    # Nodo de interfaz RL
    # Espera 3 segundos para dar tiempo a Gazebo.
    # ------------------------------------------------------------
    rl_interface_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='turtlebot3_rl_training',
                executable='rl_interface_node',
                name='rl_interface_node',
                output='screen',
                parameters=[
                    {
                        'max_sensor_range': LaunchConfiguration('max_sensor_range'),
                        'max_goal_distance': LaunchConfiguration('max_goal_distance'),
                        'goal_tolerance': LaunchConfiguration('goal_tolerance'),
                        'goal_model_name': 'goal_marker',
                        'robot_model_name': 'turtlebot3',
                        'publish_rate': LaunchConfiguration('publish_rate'),
                    }
                ]
            )
        ]
    )

    # ------------------------------------------------------------
    # Controlador de movimiento RL de 5 acciones
    # Espera 4 segundos para asegurar que /odom y /rl_goal_reached existan.
    # ------------------------------------------------------------
    rl_motion_controller = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='turtlebot3_rl_training',
                executable='rl_motion_controller',
                name='rl_motion_controller',
                output='screen',
                parameters=[
                    {
                        'control_rate': LaunchConfiguration('control_rate'),
                        'action_duration': LaunchConfiguration('action_duration'),
                        'action_timeout': LaunchConfiguration('action_timeout'),
                        'linear_speed_forward': LaunchConfiguration('linear_speed_forward'),
                        'linear_speed_soft_turn': LaunchConfiguration('linear_speed_soft_turn'),
                        'angular_speed_soft_turn': LaunchConfiguration('angular_speed_soft_turn'),
                        'linear_speed_hard_turn': LaunchConfiguration('linear_speed_hard_turn'),
                        'angular_speed_hard_turn': LaunchConfiguration('angular_speed_hard_turn'),
                    }
                ]
            )
        ]
    )

    return LaunchDescription([
        stage_arg,
        gui_arg,

        max_sensor_range_arg,
        max_goal_distance_arg,
        goal_tolerance_arg,
        publish_rate_arg,

        control_rate_arg,
        action_duration_arg,
        action_timeout_arg,
        linear_speed_forward_arg,
        linear_speed_soft_turn_arg,
        angular_speed_soft_turn_arg,
        linear_speed_hard_turn_arg,
        angular_speed_hard_turn_arg,

        world_launch,
        rl_interface_node,
        rl_motion_controller,
    ])