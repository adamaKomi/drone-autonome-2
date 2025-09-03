from setuptools import find_packages, setup

package_name = 'drone_vision'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'opencv-python',
        'numpy',
        'cv-bridge',
    ],
    zip_safe=True,
    maintainer='adama133',
    maintainer_email='adama133@todo.todo',
    description='Système de vision et collecte de données pour drone autonome',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'vision_node = drone_vision.vision_node:main',
            'data_collector_node = drone_vision.data_collector_node:main',
        ],
    },
)
