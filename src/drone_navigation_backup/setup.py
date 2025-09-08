# drone_navigation/setup.py
from setuptools import setup

package_name = 'drone_navigation'

setup(
    name=package_name,
    version='3.0.0',
    packages=[package_name, f'{package_name}.tools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/navigation_launch.py']),
        ('share/' + package_name + '/config', [
            'config/navigation_params.yaml',
            'config/pid_tuning.yaml',
            'config/geofence_zones.yaml',
            'config/coverage_patterns.yaml'
        ]),
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
            f'navigation_node = {package_name}.navigation_node:main',
            # f'trajectory_planner_node = {package_name}.trajectory_planner:main',
            # f'position_controller_node = {package_name}.position_controller:main',
            # f'path_optimizer_node = {package_name}.path_optimizer:main',
            # f'obstacle_avoidance_node = {package_name}.obstacle_avoidance:main',
            # f'geofence_manager_node = {package_name}.geofence_manager:main',
            # f'coverage_patterns_node = {package_name}.coverage_patterns:main',
            # Outils CLI
            f'goto_position = {package_name}.tools.goto_position:main',
            f'plan_mission = {package_name}.tools.plan_mission:main',
            f'nav_status = {package_name}.tools.nav_status:main',
            f'nav_diagnostics = {package_name}.tools.nav_diagnostics:main',
            f'test_waypoints = {package_name}.tools.test_waypoints:main',
            f'calibrate_navigation = {package_name}.calibrate_navigation:main',
            f'tune_pid = {package_name}.tune_pid:main',
        ],
    },
)