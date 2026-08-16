from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot3_rl_training'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Deymer Miranda',
    maintainer_email='d3im3r@gmail.com',
    description='DQN training package for TurtleBot3 with custom Gazebo worlds.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
    'console_scripts': [
        'train_dqn_ros = turtlebot3_rl_training.train_dqn_ros:main',
        'evaluate_dqn_ros = turtlebot3_rl_training.evaluate_dqn_ros:main',
        'render_episode_video = turtlebot3_rl_training.video_renderer:main',
        'rl_interface_node = turtlebot3_rl_training.rl_interface_node:main',
        'rl_motion_controller = turtlebot3_rl_training.rl_motion_controller:main',
        'test_gazebo_env = turtlebot3_rl_training.test_gazebo_env:main',
        'plot_training_metrics = turtlebot3_rl_training.plot_training_metrics:main',
        'postprocess_training_run = turtlebot3_rl_training.postprocess_training_run:main',
        'test_rule_based_agent = turtlebot3_rl_training.test_rule_based_agent:main',
        'action_sequence_monitor = turtlebot3_rl_training.action_sequence_monitor:main',
    ],
},
)