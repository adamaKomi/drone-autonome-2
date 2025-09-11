# drone_navigation/setup.py
from setuptools import setup, find_packages

package_name = 'drone_navigation'

setup(
    name=package_name,
    version='3.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/navigation_complete.launch.py']),
        ('share/' + package_name + '/launch', ['launch/navigation_test.launch.py']),
        ('share/' + package_name + '/config', ['config/drone_config.yaml']),
        ('share/' + package_name + '/config', ['config/navigation_params.yaml']),
        ('share/' + package_name + '/config', ['config/pid_tuning.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adama Komi',
    maintainer_email='adama.komi@example.com',
    description='Package de navigation avancée pour drone de pollinisation autonome',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Micro-nœuds principaux
            'parameter_manager_node = drone_navigation.nodes.parameter_manager_node:main',
            'mavros_interface_node = drone_navigation.nodes.mavros_interface_node:main',
            'core_navigation_node = drone_navigation.nodes.core_navigation_node:main',
            'trajectory_planner_node = drone_navigation.nodes.trajectory_planner_node:main',
            'coverage_pattern_node = drone_navigation.nodes.coverage_pattern_node:main',
            'path_optimizer_node = drone_navigation.nodes.path_optimizer_node:main',
            'position_controller_node = drone_navigation.nodes.position_controller_node:main',
            'trajectory_follower_node = drone_navigation.nodes.trajectory_follower_node:main',
            'cli_bridge_node = drone_navigation.nodes.cli_bridge_node:main',
            
            # Outils CLI
            'drone-status = drone_navigation.cli.status:main',
        ],
    },
)