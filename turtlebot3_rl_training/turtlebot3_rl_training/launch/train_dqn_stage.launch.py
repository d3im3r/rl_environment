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

    stage_arg = DeclareLaunchArgument(
        'stage',
        default_value='1',
        description='Custom world stage number.'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='false',
        description='Launch Gazebo GUI.'
    )

    linear_speed_arg = DeclareLaunchArgument(
        'linear_speed',
        default_value='0.25',
        description='Linear speed for RL action controller.'
    )

    angular_speed_arg = DeclareLaunchArgument(
        'angular_speed',
        default_value='0.75',
        description='Angular speed for RL action controller.'
    )

    forward_distance_arg = DeclareLaunchArgument(
        'forward_distance',
        default_value='0.20',
        description='Forward distance per discrete action.'
    )

    turn_angle_arg = DeclareLaunchArgument(
        'turn_angle',
        default_value='0.1745329',
        description='Turn angle per discrete action in radians. 0.1745329 rad = 10 deg.'
    )

    # Nuevos argumentos de entrenamiento
    episodes_arg = DeclareLaunchArgument(
        'episodes',
        default_value='100',
        description='Number of training episodes.'
    )

    max_steps_arg = DeclareLaunchArgument(
        'max_steps',
        default_value='40',
        description='Max steps per episode.'
    )

    batch_size_arg = DeclareLaunchArgument(
        'batch_size',
        default_value='64',
        description='DQN batch size.'
    )

    learning_rate_arg = DeclareLaunchArgument(
        'learning_rate',
        default_value='0.001',
        description='Learning rate.'
    )

    epsilon_decay_arg = DeclareLaunchArgument(
        'epsilon_decay',
        default_value='0.995',
        description='Epsilon decay rate.'
    )

    base_dir_arg = DeclareLaunchArgument(
        'base_dir',
        default_value='/home/d3im3r/ros2_ws/src/train_runs',
        description='Base directory to save training runs.'
    )

    resume_checkpoint_arg = DeclareLaunchArgument(
        'resume_checkpoint',
        default_value='',
        description='Path to .pth checkpoint file to resume or fine-tune training from.'
    )

    epsilon_start_arg = DeclareLaunchArgument(
        'epsilon_start',
        default_value='1.0',
        description='Initial epsilon exploration rate (reduce to 0.2-0.3 for fine-tuning).'
    )

    goal_mode_arg = DeclareLaunchArgument(
        'goal_mode',
        default_value='single',
        description='Goal placement mode (single, soft, medium, separated).'
    )

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

    rl_interface_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='turtlebot3_rl_training',
                executable='rl_interface_node',
                name='rl_interface_node',
                output='log',
                parameters=[
                    {
                        'max_sensor_range': 3.5,
                        'max_goal_distance': 5.0,
                        'goal_tolerance': 0.18,
                        'goal_model_name': 'goal_marker',
                        'robot_model_name': 'turtlebot3',
                        'publish_rate': 10.0,
                    }
                ]
            )
        ]
    )

    rl_motion_controller = TimerAction(
        period=4.0,
        actions=[
            Node(
                package='turtlebot3_rl_training',
                executable='rl_motion_controller',
                name='rl_motion_controller',
                output='log',
                parameters=[
                    {
                        'linear_speed': LaunchConfiguration('linear_speed'),
                        'angular_speed': LaunchConfiguration('angular_speed'),
                        'forward_distance': LaunchConfiguration('forward_distance'),
                        'turn_angle': LaunchConfiguration('turn_angle'),
                        'action_timeout': 4.0,
                        'control_rate': 30.0,
                    }
                ]
            )
        ]
    )

    train_node = TimerAction(
        period=6.0,
        actions=[
            Node(
                package='turtlebot3_rl_training',
                executable='train_dqn_ros',
                name='train_dqn_ros',
                output='screen',
                emulate_tty=True,
                arguments=[
                    '--stage', LaunchConfiguration('stage'),
                    '--goal-mode', LaunchConfiguration('goal_mode'),
                    '--episodes', LaunchConfiguration('episodes'),
                    '--max-steps', LaunchConfiguration('max_steps'),
                    '--batch-size', LaunchConfiguration('batch_size'),
                    '--learning-rate', LaunchConfiguration('learning_rate'),
                    '--epsilon-decay', LaunchConfiguration('epsilon_decay'),
                    '--epsilon-start', LaunchConfiguration('epsilon_start'),
                    '--resume-checkpoint', LaunchConfiguration('resume_checkpoint'),
                    '--base-dir', LaunchConfiguration('base_dir')
                ]
            )
        ]
    )

    return LaunchDescription([
        stage_arg,
        gui_arg,
        linear_speed_arg,
        angular_speed_arg,
        forward_distance_arg,
        turn_angle_arg,
        episodes_arg,
        max_steps_arg,
        batch_size_arg,
        learning_rate_arg,
        epsilon_decay_arg,
        resume_checkpoint_arg,
        epsilon_start_arg,
        goal_mode_arg,
        base_dir_arg,
        world_launch,
        rl_interface_node,
        rl_motion_controller,
        train_node,
    ])