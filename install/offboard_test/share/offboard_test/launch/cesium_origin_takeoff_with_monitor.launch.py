from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    enable_ros_tcp = LaunchConfiguration("enable_ros_tcp")
    ros_tcp_port = LaunchConfiguration("ros_tcp_port")
    enable_px4_odom = LaunchConfiguration("enable_px4_odom")

    takeoff_altitude_m = LaunchConfiguration("takeoff_altitude_m")
    hover_s = LaunchConfiguration("hover_s")

    log_interval_s = LaunchConfiguration("log_interval_s")
    min_delta_m = LaunchConfiguration("min_delta_m")

    return LaunchDescription(
        [
            DeclareLaunchArgument("enable_ros_tcp", default_value="true"),
            DeclareLaunchArgument("ros_tcp_port", default_value="10000"),
            DeclareLaunchArgument("enable_px4_odom", default_value="true"),
            DeclareLaunchArgument("takeoff_altitude_m", default_value="10.0"),
            DeclareLaunchArgument("hover_s", default_value="2.0"),
            DeclareLaunchArgument("log_interval_s", default_value="0.5"),
            DeclareLaunchArgument("min_delta_m", default_value="0.02"),

            # ROS-TCP Endpoint (Unity side connects to this)
            Node(
                package="ros_tcp_endpoint",
                executable="default_server_endpoint",
                name="ros_tcp_endpoint",
                emulate_tty=True,
                output="screen",
                parameters=[
                    {"ROS_IP": "0.0.0.0"},
                    {"ROS_TCP_PORT": ros_tcp_port},
                ],
                condition=IfCondition(enable_ros_tcp),
            ),

            # Odom monitor: prints changes to log
            Node(
                package="offboard_test",
                executable="odom_monitor",
                name="odom_monitor",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"log_interval_s": log_interval_s},
                    {"min_delta_m": min_delta_m},
                    {"enable_px4_odom": enable_px4_odom},
                ],
            ),

            # Mission: takeoff at Cesium origin (WorldOrigin) -> hover -> land
            Node(
                package="offboard_test",
                executable="offboard_cesium_origin_takeoff_land",
                name="offboard_cesium_origin_takeoff_land",
                output="screen",
                emulate_tty=True,
                parameters=[
                    {"takeoff_altitude_m": takeoff_altitude_m},
                    {"hover_s": hover_s},
                ],
            ),
        ]
    )

