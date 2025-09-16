from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'drone_mission'
setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/missions', glob('missions/*.json')),
        ('share/' + package_name + '/missions', glob('missions/*.yaml')),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='Adama Komi',
    maintainer_email='adamakomi15@gmail.com',
    description='Mission planning and execution node for autonomous drone operations',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'gps_waypoint_manager_node = drone_mission.nodes.gps_waypoint_manager_node:main',
            'local_waypoint_manager_node = drone_mission.nodes.local_waypoint_manager_node:main',
            'test_gps_waypoint_manager = drone_mission.nodes.test_gps_waypoint_manager:main',
            'test_local_waypoint_manager = drone_mission.nodes.test_local_waypoint_manager:main',
        ],
    },
)
