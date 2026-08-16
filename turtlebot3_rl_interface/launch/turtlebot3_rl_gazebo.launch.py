import math

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.substitutions import PathJoinSubstitution

from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ============================================================
    # Modelo de TurtleBot3
    # ============================================================

    turtlebot3_model = LaunchConfiguration(
        'turtlebot3_model',
        default='burger'
    )

    # ============================================================
    # Parámetros de meta
    # ============================================================

    goal_x = LaunchConfiguration(
        'goal_x',
        default='2.0'
    )

    goal_y = LaunchConfiguration(
        'goal_y',
        default='1.0'
    )

    goal_tolerance = LaunchConfiguration(
        'goal_tolerance',
        default='0.10'#'0.10'
    )

    # ============================================================
    # Parámetros de normalización
    # ============================================================

    max_sensor_range = LaunchConfiguration(
        'max_sensor_range',
        default='3.5'
    )

    max_goal_distance = LaunchConfiguration(
        'max_goal_distance',
        default='5.0'
    )

    # ============================================================
    # Parámetros de acciones discretas
    # ============================================================

    forward_distance = LaunchConfiguration(
        'forward_distance',
        default='0.20'
    )

    rotation_angle = LaunchConfiguration(
        'rotation_angle',
        default=str(math.pi / 12.0)
    )

    linear_speed = LaunchConfiguration(
        'linear_speed',
        default='0.08'
    )

    angular_speed = LaunchConfiguration(
        'angular_speed',
        default='0.25'
    )

    # ============================================================
    # Ángulos de lectura del LiDAR
    # ============================================================

    front_angle = LaunchConfiguration(
        'front_angle',
        default='0.0'
    )

    left_angle = LaunchConfiguration(
        'left_angle',
        default=str(math.pi / 4.0)
    )

    right_angle = LaunchConfiguration(
        'right_angle',
        default=str(-math.pi / 4.0)
    )

    # ============================================================
    # Periodos de ejecución
    # ============================================================

    control_period = LaunchConfiguration(
        'control_period',
        default='0.05'
    )

    state_publish_period = LaunchConfiguration(
        'state_publish_period',
        default='0.10'
    )

    # ============================================================
    # Launch de Gazebo con TurtleBot3
    # ============================================================

    turtlebot3_gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare('turtlebot3_gazebo'),
            # '/launch/turtlebot3_world.launch.py'
            '/launch/turtlebot3_dqn_stage1.launch.py'
        ])
    )

    # ============================================================
    # Modelo visual de la meta
    # ============================================================

    goal_model_path = PathJoinSubstitution([
        FindPackageShare('turtlebot3_rl_interface'),
        'models',
        'goal_marker',
        'model.sdf'
    ])

    spawn_goal_marker = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_goal_marker',
        output='screen',
        arguments=[
            '-entity', 'goal_marker',
            '-file', goal_model_path,
            '-x', goal_x,
            '-y', goal_y,
            '-z', '0.01'
        ]
    )

    # ============================================================
    # Nodo de interfaz RL
    # ============================================================

    rl_interface_node = Node(
        package='turtlebot3_rl_interface',
        executable='rl_interface_node',
        name='turtlebot3_rl_interface',
        output='screen',
        parameters=[
            {
                'goal_x': goal_x,
                'goal_y': goal_y,
                'goal_tolerance': goal_tolerance,

                'max_sensor_range': max_sensor_range,
                'max_goal_distance': max_goal_distance,

                'forward_distance': forward_distance,
                'rotation_angle': rotation_angle,

                'linear_speed': linear_speed,
                'angular_speed': angular_speed,

                'front_angle': front_angle,
                'left_angle': left_angle,
                'right_angle': right_angle,

                'control_period': control_period,
                'state_publish_period': state_publish_period,
            }
        ]
    )

    # ============================================================
    # Descripción final del launch
    # ============================================================

    return LaunchDescription([

        SetEnvironmentVariable(
            name='TURTLEBOT3_MODEL',
            value=turtlebot3_model
        ),

        turtlebot3_gazebo_launch,

        spawn_goal_marker,

        rl_interface_node
    ])
