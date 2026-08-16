import os
from glob import glob
from setuptools import setup

package_name = 'turtlebot3_rl_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name]
        ),
        (
            'share/' + package_name,
            ['package.xml']
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py')
        ),
        (
            os.path.join('share', package_name, 'models', 'goal_marker'),
            glob('models/goal_marker/*')
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='d3im3r',
    maintainer_email='d3im3r@gmail.com',
    description='Interfaz RL para TurtleBot3 en Gazebo usando ROS 2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_interface_node = turtlebot3_rl_interface.rl_interface_node:main',
        ],
    },
)