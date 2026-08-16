from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'd3im3r_bot_bringup'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        (
            'share/ament_index/resource_index/packages',
            ['resource/' + package_name],
        ),
        (
            'share/' + package_name,
            ['package.xml'],
        ),
        (
            os.path.join('share', package_name, 'launch'),
            glob('launch/*.launch.py'),
        ),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Deymer Miranda',
    maintainer_email='deymer@example.com',
    description='ROS 2 bringup utilities for d3im3r_bot.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_bridge = d3im3r_bot_bringup.odom_bridge_node:main',
            'safe_teleop = d3im3r_bot_bringup.safe_teleop_node:main',
        ],
    },
)