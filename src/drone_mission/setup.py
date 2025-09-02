from setuptools import setup
import os
from glob import glob

package_name = 'drone_mission'
setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
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
            'mission_node.py = drone_mission.mission_node:main',
        ],
    },
)
