from setuptools import setup

package_name = 'drone_mission'
setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/mission.launch.py']),
        ('share/' + package_name + '/missions', ['missions/mission.yaml']),
    ],
    install_requires=['setuptools', 'pyyaml'],
    zip_safe=True,
    maintainer='Adama Komi',
    maintainer_email='adamakomi15@gmail.com',
    description='Mission sequencer',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'mission_node = drone_mission.mission_node:main',
        ],
    },
)
