from setuptools import find_packages, setup

package_name = 'drone_interface'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/interface.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adama Komi',
    maintainer_email='adamakomi15@gmail.com',
    description='Interface de contrôle de base pour drone avec MAVROS - ROS2 Humble',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mode_node = drone_interface.mode_node:main',
            'arm_disarm_node = drone_interface.arm_disarm_node:main',
            'takeoff_land_node = drone_interface.takeoff_land_node:main'
        ],
    },
)
