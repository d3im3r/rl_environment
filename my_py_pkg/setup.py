from setuptools import find_packages, setup
import os 
from glob import glob

package_name = 'my_py_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='d3im3r_rl',
    maintainer_email='demiranda@unal.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        'simple_node = my_py_pkg.simple_node:main',
        'minimal_node= my_py_pkg.minimal_node:main',
        'publisher_node = my_py_pkg.publisher_node:main',
        'subscriber_node = my_py_pkg.subscriber_node:main',
        'name_pub_node = my_py_pkg.name_pub_node:main',
        'name_sub_node = my_py_pkg.name_sub_node:main',
        'turtle_publisher = my_py_pkg.turtle_publisher:main',
        'turtle_subscriber = my_py_pkg.turtle_subscriber:main'
        ],
    },
)
