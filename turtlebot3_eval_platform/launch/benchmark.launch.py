import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition

from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_custom_worlds = get_package_share_directory('turtlebot3_custom_worlds')

    custom_world_launch = os.path.join(
        pkg_custom_worlds,
        'launch',
        'd3im3r_world.launch.py'
    )

    # Declaración de argumentos para pruebas / benchmarking
    agent_arg = DeclareLaunchArgument(
        'agent',
        default_value='rule-based',
        description='Agent type to evaluate: dqn, fuzzy, rule-based.'
    )

    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='',
        description='Path to the model checkpoint (.pth) for DQN agent.'
    )

    stage_arg = DeclareLaunchArgument(
        'stage',
        default_value='1',
        description='Custom world stage number (1 to 7).'
    )

    episodes_arg = DeclareLaunchArgument(
        'episodes',
        default_value='3',
        description='Number of episodes to evaluate per goal.'
    )

    max_steps_arg = DeclareLaunchArgument(
        'max_steps',
        default_value='250',
        description='Maximum steps allowed per episode.'
    )

    csv_path_arg = DeclareLaunchArgument(
        'csv_path',
        default_value='/home/d3im3r/ros2_ws/src/eval_history.csv',
        description='Path to centralized CSV file.'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Launch Gazebo GUI.'
    )

    launch_gazebo_arg = DeclareLaunchArgument(
        'launch_gazebo',
        default_value='true',
        description='Whether to launch Gazebo simulation world or not.'
    )

    # Lanzamiento opcional de Gazebo
    world_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(custom_world_launch),
        condition=IfCondition(LaunchConfiguration('launch_gazebo')),
        launch_arguments={
            'stage': LaunchConfiguration('stage'),
            'gui': LaunchConfiguration('gui'),
            'x_pose': '-1.5',
            'y_pose': '0.0',
            'z_pose': '0.01',
            'yaw': '0.0',
        }.items()
    )

    # Nodo del runner de benchmarking (con retardo para dar tiempo a Gazebo si se lanza)
    benchmark_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='turtlebot3_eval_platform',
                executable='benchmark',
                name='benchmark_runner',
                output='screen',
                arguments=[
                    '--agent', LaunchConfiguration('agent'),
                    '--model-path', LaunchConfiguration('model_path'),
                    '--stage', LaunchConfiguration('stage'),
                    '--episodes', LaunchConfiguration('episodes'),
                    '--max-steps', LaunchConfiguration('max_steps'),
                    '--csv-path', LaunchConfiguration('csv_path'),
                ]
            )
        ]
    )

    return LaunchDescription([
        agent_arg,
        model_path_arg,
        stage_arg,
        episodes_arg,
        max_steps_arg,
        csv_path_arg,
        gui_arg,
        launch_gazebo_arg,
        world_launch,
        benchmark_node,
    ])
