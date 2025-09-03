#!/usr/bin/env python3

from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'drone_interface'

setup(
    name='drone-interface',  # Nom pour setuptools (avec tiret)
    version='3.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Installation des métadonnées du package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # Installation des fichiers de lancement
        (os.path.join('share', package_name, 'launch'),
            glob(os.path.join('launch', '*launch.[pyx][yma]*'))),
        
        # Installation des fichiers de configuration
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.yml'))),
        (os.path.join('share', package_name, 'config'),
            glob(os.path.join('config', '*.json'))),
            
        # Installation des scripts utilitaires
        (os.path.join('share', package_name, 'scripts'),
            glob(os.path.join('scripts', '*.py'))),
    ],
    install_requires=[
        'setuptools',
        'rclpy',
        'std_msgs',
        'geometry_msgs', 
        'sensor_msgs',
        'nav_msgs',
        'diagnostic_msgs',
        'mavros_msgs',
        'lifecycle_msgs',
        'std_srvs',
        'psutil',  # Pour monitoring système
        'pyyaml',  # Pour configuration
    ],
    zip_safe=True,
    maintainer='Adama Komi',
    maintainer_email='adama.komi@example.com',
    description='Interface ROS2 robuste pour contrôle de drones ArduPilot via MAVROS',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Nœud principal
            'interface_node = drone_interface.interface_node:main',
            
            # Outils CLI
            'drone_diagnostics = drone_interface.tools.diagnostics:main',
            'drone_safety_check = drone_interface.tools.safety_check:main',
            'drone_arm = drone_interface.tools.arm_drone:main',
            'drone_status = drone_interface.tools.status:main',
            
            # Scripts de configuration
            'setup_environment = drone_interface.scripts.setup_environment:main',
        ],
    },
    python_requires='>=3.8',
    classifiers=[
        'Development Status :: 5 - Production/Stable',
        'Intended Audience :: Developers',
        'License :: OSI Approved :: MIT License',
        'Programming Language :: Python :: 3',
        'Programming Language :: Python :: 3.8',
        'Programming Language :: Python :: 3.9',
        'Programming Language :: Python :: 3.10',
        'Programming Language :: Python :: 3.11',
        'Topic :: Scientific/Engineering :: Artificial Intelligence',
        'Topic :: Software Development :: Libraries :: Python Modules',
        'Topic :: Scientific/Engineering :: Robotics',
    ],
)