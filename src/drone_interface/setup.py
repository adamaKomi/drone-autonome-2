from setuptools import setup

package_name = 'drone_interface'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/bringup.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Adama Komi',
    maintainer_email='adamakomi15@gmail.com',
    description='Simple MAVROS wrapper (arm/mode/setpoints)',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'interface_node = drone_interface.interface_node:main',
        ],
    },
)
