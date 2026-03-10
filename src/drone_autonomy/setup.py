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
            'rrt_pursuit_single = drone_autonomy.missions.rrt_pursuit_single:main',
            'rrt_pursuit_single_v1_1 = drone_autonomy.missions.rrt_pursuit_single_v1_1:main',
            'astar_grid_pursuit = drone_autonomy.missions.astar_grid_pursuit:main',
            'astar_grid_pursuit_3d = drone_autonomy.missions.astar_grid_pursuit_3D:main',
            # Haoran custom algorithm entry point (Unity test friendly)
            'haoran_unity_pursuit = drone_autonomy.haoran_alg_update.haoran_unity_pursuit:main',
        ],
    },
)