from setuptools import setup

package_name = 'offboard_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/obstacle_mission.launch.py']),
        ('share/' + package_name + '/launch', ['launch/around_house_mission.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lingzp',
    maintainer_email='ling.zh@northeastern.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'offboard_takeoff = offboard_test.offboard_takeoff:main',
            'offboard_square_mission = offboard_test.offboard_square_mission:main',
            'obstacle_avoidance_mission = offboard_test.obstacle_avoidance_mission:main',
            'obstacle_publisher = offboard_test.obstacle_publisher:main',
        ],
    },
)
