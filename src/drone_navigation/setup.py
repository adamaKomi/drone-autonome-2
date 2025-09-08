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
        ],
    },
)