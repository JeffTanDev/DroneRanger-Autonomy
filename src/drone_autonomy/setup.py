from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'drone_autonomy'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        # Config files (if any)
    ],
    install_requires=['setuptools', 'numpy', 'scipy'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@example.com',
    description='Advanced drone autonomy system',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # Core executables
            'drone_controller = drone_autonomy.core.drone_controller:main',
            'mission_executor = drone_autonomy.core.mission_executor:main',

            # Mission executables
            'simple_takeoff = drone_autonomy.missions.simple_takeoff:main',
            'square_flight = drone_autonomy.missions.square_flight:main',
            'circle_flight = drone_autonomy.missions.circle_flight:main',
            'figure_eight = drone_autonomy.missions.figure_eight:main',
            'orbit_point = drone_autonomy.missions.orbit_point:main',
            'pursuit_flight = drone_autonomy.missions.pursuit_flight:main',
            'rrt_pursuit_single = drone_autonomy.missions.rrt_pursuit_single:main',
            'waypoint_mission = drone_autonomy.missions.waypoint_mission:main',
            'house_inspection = drone_autonomy.missions.house_inspection:main',

            # Utility executables
            'obstacle_detector = drone_autonomy.perception.obstacle_detector:main',
            'data_recorder = drone_autonomy.utils.data_recorder:main',
        ],
    },
)