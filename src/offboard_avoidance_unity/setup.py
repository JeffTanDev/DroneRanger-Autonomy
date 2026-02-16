from setuptools import setup

package_name = 'offboard_avoidance_unity'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='jefft',
    maintainer_email='jefft@example.com',
    description='Offboard avoidance mission around Unity house.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'unity_house_avoidance = offboard_avoidance_unity.unity_house_avoidance:main',
        ],
    },
)
