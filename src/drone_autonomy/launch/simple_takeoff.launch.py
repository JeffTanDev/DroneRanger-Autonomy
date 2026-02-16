"""
Launch file for simple takeoff mission.

This launch file starts the simple takeoff mission with configurable parameters.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for simple takeoff mission."""

    # Declare launch arguments
    takeoff_altitude_arg = DeclareLaunchArgument(
        'takeoff_altitude',
        default_value='3.0',
        description='Takeoff altitude in meters'
    )

    hover_duration_arg = DeclareLaunchArgument(
        'hover_duration',
        default_value='10.0',
        description='Hover duration in seconds'
    )

    safety_timeout_arg = DeclareLaunchArgument(
        'safety_timeout',
        default_value='30.0',
        description='Safety timeout in seconds'
    )

    # Simple takeoff mission node
    simple_takeoff_node = Node(
        package='drone_autonomy',
        executable='simple_takeoff',
        name='simple_takeoff_mission',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'takeoff_altitude': LaunchConfiguration('takeoff_altitude'),
            'hover_duration': LaunchConfiguration('hover_duration'),
            'safety_timeout': LaunchConfiguration('safety_timeout'),
        }]
    )

    return LaunchDescription([
        takeoff_altitude_arg,
        hover_duration_arg,
        safety_timeout_arg,
        simple_takeoff_node
    ])