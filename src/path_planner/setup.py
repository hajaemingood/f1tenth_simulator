import os
from glob import glob
from setuptools import setup

package_name = 'path_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'maps'), glob('maps/*')),
        (os.path.join('share', package_name, 'data'), glob('path_planner/*.csv')),
        (os.path.join('share', package_name, 'data'), glob('path_planner/*.npy')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='hjm021123@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoints = path_planner.waypoints:main',
            'base_link_pub = path_planner.base_link_pub:main',
            'pure_pursuit = path_planner.pure_pursuit:main',
            'waypoint_visualization = path_planner.waypoint_visualization:main',
        ],
    },
)
