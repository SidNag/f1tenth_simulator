from glob import glob

from setuptools import find_packages, setup

package_name = 'ece484_racing'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
        ('share/' + package_name + '/maps', glob('maps/*')),
        ('share/' + package_name + '/waypoints', glob('waypoints/*.csv')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='sid2404',
    maintainer_email='smn6@illinois.edu',
    description='ECE484 team Rogue: pure pursuit, SLAM/AMCL configs, maps and waypoints for the F1TENTH car and sim',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pure_pursuit = ece484_racing.pure_pursuit:main',
            'waypoint_logger = ece484_racing.waypoint_logger:main',
        ],
    },
)
