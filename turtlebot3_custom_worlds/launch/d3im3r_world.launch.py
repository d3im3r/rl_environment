import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.actions import SetEnvironmentVariable
from launch.actions import OpaqueFunction
from launch.actions import LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_custom_worlds = get_package_share_directory('turtlebot3_custom_worlds')
    pkg_gazebo_ros = get_package_share_directory('gazebo_ros')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    stage = LaunchConfiguration('stage').perform(context)
    model = LaunchConfiguration('model').perform(context)

    x_pose = LaunchConfiguration('x_pose')
    y_pose = LaunchConfiguration('y_pose')
    z_pose = LaunchConfiguration('z_pose')
    yaw = LaunchConfiguration('yaw')

    stage_worlds = {
        '0': 'd3im3r_stage_00_empty.world',
        '1': 'd3im3r_stage_01_direct_goal.world',
        '2': 'd3im3r_stage_02_front_obstacle.world',
        '3': 'd3im3r_stage_03_left_right_choice.world',
        '4': 'd3im3r_stage_04_corridor.world',
        '5': 'd3im3r_stage_05_narrow_door.world',
        '6': 'd3im3r_stage_06_random_obstacles.world',
        '7': 'd3im3r_stage_07_simple_maze.world',
    }

    world_name = stage_worlds.get(stage, stage_worlds['0'])

    world_file = os.path.join(
        pkg_custom_worlds,
        'worlds',
        world_name
    )

    robot_sdf = os.path.join(
        pkg_turtlebot3_gazebo,
        'models',
        f'turtlebot3_{model}',
        'model.sdf'
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo_ros, 'launch', 'gazebo.launch.py')
        ),
        launch_arguments={
            'world': world_file,
            'verbose': 'false',
            'gui': LaunchConfiguration('gui')
        }.items()
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        name='spawn_turtlebot3',
        output='screen',
        arguments=[
            '-entity', 'turtlebot3',
            '-file', robot_sdf,
            '-x', x_pose,
            '-y', y_pose,
            '-z', z_pose,
            '-Y', yaw
        ]
    )

    return [
        LogInfo(msg=f'Loading custom world: {world_file}'),
        LogInfo(msg=f'Spawning TurtleBot3 model from: {robot_sdf}'),
        gazebo,
        spawn_robot
    ]


def generate_launch_description():
    pkg_custom_worlds = get_package_share_directory('turtlebot3_custom_worlds')
    pkg_turtlebot3_gazebo = get_package_share_directory('turtlebot3_gazebo')

    custom_models_path = os.path.join(pkg_custom_worlds, 'models')
    turtlebot3_models_path = os.path.join(pkg_turtlebot3_gazebo, 'models')

    return LaunchDescription([
        SetEnvironmentVariable(
            name='TURTLEBOT3_MODEL',
            value='burger'
        ),

        SetEnvironmentVariable(
            name='GAZEBO_MODEL_PATH',
            value=[
                custom_models_path,
                ':',
                turtlebot3_models_path
            ]
        ),

        SetEnvironmentVariable(
            name='GAZEBO_MODEL_DATABASE_URI',
            value=''
        ),

        # Note: Gazebo GUI can be disabled for faster performance, but it may limit visualization and debugging capabilities
        DeclareLaunchArgument(
            'gui',
            default_value='true',
            description='Whether to launch Gazebo GUI'
        ),

        DeclareLaunchArgument(
            'stage',
            default_value='0',
            description='Custom world stage: 0 to 7'
        ),

        DeclareLaunchArgument(
            'model',
            default_value='burger',
            description='TurtleBot3 model: burger, waffle, waffle_pi'
        ),

        DeclareLaunchArgument(
            'x_pose',
            default_value='-1.5',
            description='Initial TurtleBot3 x position'
        ),

        DeclareLaunchArgument(
            'y_pose',
            default_value='0.0',
            description='Initial TurtleBot3 y position'
        ),

        DeclareLaunchArgument(
            'z_pose',
            default_value='0.01',
            description='Initial TurtleBot3 z position'
        ),

        DeclareLaunchArgument(
            'yaw',
            default_value='0.0',
            description='Initial TurtleBot3 yaw angle'
        ),

        OpaqueFunction(function=launch_setup)
    ])