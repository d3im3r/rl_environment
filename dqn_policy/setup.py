from setuptools import find_packages, setup

package_name = 'dqn_policy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='d3im3r-2',
    maintainer_email='d3im3r-2@todo.todo',
    description='DQN policy node for RL action inference',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'dqn_policy_node = dqn_policy.dqn_policy_node:main',
        ],
    },
)
