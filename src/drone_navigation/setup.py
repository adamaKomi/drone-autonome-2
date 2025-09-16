# drone_navigation/setup.py
from setuptools import setup, find_packages
from glob import glob

package_name = 'drone_navigation'

setup(
    name=package_name,
    version='3.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/drone_navigation/launch', glob('launch/*.launch.py')),
        ('share/drone_navigation/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'transforms3d'],
    zip_safe=True,
    maintainer='Adama Komi',
    maintainer_email='adama.komi@example.com',
    description='Package de navigation avancée pour drone de pollinisation autonome',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'gps_navigation_node = drone_navigation.nodes.gps_navigation_node:main',
            'local_navigation_node = drone_navigation.nodes.local_navigation_node:main',
            'navigation_safety_node = drone_navigation.nodes.navigation_safety_node:main',
            'emergency_handler_node = drone_navigation.nodes.emergency_handler_node:main',
            'navigation_supervisor_node = drone_navigation.nodes.navigation_supervisor_node:main',
        ],
    },
)