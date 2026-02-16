from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        # Mission node
        Node(
            package='offboard_test',
            executable='obstacle_avoidance_mission',
            name='obstacle_avoidance_mission',
            output='screen'
        ),

        # Obstacle publisher node
        Node(
            package='offboard_test',
            executable='obstacle_publisher',
            name='obstacle_publisher',
            output='screen'
        ),
    ])
