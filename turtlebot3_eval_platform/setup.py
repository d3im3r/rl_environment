from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'turtlebot3_eval_platform'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Deimer Miranda',
    maintainer_email='demiranda@unal.edu.co',
    description='Framework de benchmarking continuo para TurtleBot3 con DQN y Lógica Difusa.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'benchmark = turtlebot3_eval_platform.benchmark_runner:main',
        ],
    },
)
