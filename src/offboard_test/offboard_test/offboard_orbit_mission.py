"""
Takeoff -> orbit around current position -> land.

This mission is relative to the current vehicle pose (Cesium/Unity-aligned Cartesian NED),
using /fmu/out/vehicle_odometry as the reference when the mission starts.
"""

import rclpy
import math
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from px4_msgs.msg import (
    VehicleCommand,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleOdometry,
)


class OffboardOrbitMission(Node):
    def __init__(self):
        super().__init__('offboard_orbit_mission')

        self.declare_parameter('takeoff_altitude_m', 8.0)
        self.declare_parameter('orbit_radius_m', 10.0)
        self.declare_parameter('orbit_angular_velocity', 0.2)
        self.declare_parameter('num_orbits', 2.0)
        self.declare_parameter('takeoff_hold_s', 3.0)

        self.takeoff_alt_m = float(self.get_parameter('takeoff_altitude_m').value)
        self.orbit_radius_m = float(self.get_parameter('orbit_radius_m').value)
        self.omega = float(self.get_parameter('orbit_angular_velocity').value)
        self.num_orbits = float(self.get_parameter('num_orbits').value)
        self.takeoff_hold_s = float(self.get_parameter('takeoff_hold_s').value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10
        )
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10
        )
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odom_callback,
            qos,
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.home_ned = None
        self.current_odom = None
        self.phase = 'WAIT_HOME'
        self.phase_start_time = None
        self.orbit_start_angle = 0.0

    def odom_callback(self, msg: VehicleOdometry):
        self.current_odom = msg
        if self.phase != 'WAIT_HOME' or self.home_ned is not None:
            return
        self.home_ned = (
            float(msg.position[0]),
            float(msg.position[1]),
            float(msg.position[2]),
        )
        self.get_logger().info(
            f"Home (Cesium-aligned NED): x={self.home_ned[0]:.2f} y={self.home_ned[1]:.2f} z={self.home_ned[2]:.2f}"
        )
        self.phase = 'PRE_ARM'
        self.phase_start_time = time.time()

    def timer_callback(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = timestamp
        offboard_mode.position = True
        self.offboard_control_mode_pub.publish(offboard_mode)

        if self.phase == 'WAIT_HOME':
            return

        hx, hy, hz = self.home_ned
        target_z = -self.takeoff_alt_m

        if self.phase == 'PRE_ARM':
            traj = TrajectorySetpoint()
            traj.timestamp = timestamp
            traj.position = [hx, hy, target_z]
            traj.yaw = 0.0
            self.trajectory_setpoint_pub.publish(traj)
            if self.phase_start_time and time.time() - self.phase_start_time > 2.0:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                self.get_logger().info("Armed, Offboard; taking off.")
                self.phase = 'TAKEOFF'
                self.phase_start_time = time.time()

        elif self.phase == 'TAKEOFF':
            traj = TrajectorySetpoint()
            traj.timestamp = timestamp
            traj.position = [hx, hy, target_z]
            traj.yaw = 0.0
            self.trajectory_setpoint_pub.publish(traj)
            if self.phase_start_time and time.time() - self.phase_start_time >= self.takeoff_hold_s:
                self.get_logger().info("Orbiting around current position.")
                self.phase = 'ORBIT'
                self.phase_start_time = time.time()
                self.orbit_start_angle = 0.0

        elif self.phase == 'ORBIT':
            t = time.time() - self.phase_start_time
            angle = self.orbit_start_angle + self.omega * t
            x = hx + self.orbit_radius_m * math.cos(angle)
            y = hy + self.orbit_radius_m * math.sin(angle)
            traj = TrajectorySetpoint()
            traj.timestamp = timestamp
            traj.position = [x, y, target_z]
            traj.yaw = float(angle + math.pi / 2)
            self.trajectory_setpoint_pub.publish(traj)
            laps = (self.omega * t) / (2.0 * math.pi)
            if laps >= self.num_orbits:
                self.get_logger().info("Orbit done; landing.")
                self.phase = 'LAND'
                self.phase_start_time = time.time()

        elif self.phase == 'LAND':
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info("Land command sent.")
            rclpy.shutdown()

    def publish_vehicle_command(self, command, **kwargs):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(kwargs.get('param1', 0.0))
        msg.param2 = float(kwargs.get('param2', 0.0))
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardOrbitMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
