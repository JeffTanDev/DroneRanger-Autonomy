"""
Offboard square mission: fly a square **relative to current drone position**.
Uses /fmu/out/vehicle_odometry so the square is centered where the drone is
when the mission starts (e.g. Unity World Origin / takeoff place in NED).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from px4_msgs.msg import (
    VehicleCommand,
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleOdometry,
)
import time


class OffboardSquareMission(Node):
    def __init__(self):
        super().__init__('offboard_square_mission')

        self.declare_parameter('side_m', 10.0)
        self.declare_parameter('waypoint_hold_s', 5.0)
        self.declare_parameter('use_current_altitude', True)
        self.declare_parameter('fixed_altitude', 10.0)

        self.side_m = float(self.get_parameter('side_m').value)
        self.waypoint_hold_s = float(self.get_parameter('waypoint_hold_s').value)
        self.use_current_altitude = bool(self.get_parameter('use_current_altitude').value)
        self.fixed_altitude = float(self.get_parameter('fixed_altitude').value)

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

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.home_ned = None
        self.current_odom = None
        self.step = 0
        self.waypoint_start_time = None
        self.square_waypoints_ned = []

    def odom_callback(self, msg: VehicleOdometry):
        self.current_odom = msg
        if self.home_ned is None and self.step == 0:
            return
        if self.home_ned is not None:
            return
        home_x = float(msg.position[0])
        home_y = float(msg.position[1])
        home_z = float(msg.position[2])
        self.home_ned = (home_x, home_y, home_z)
        self.get_logger().info(
            f"Home set from current position (NED): x={home_x:.2f} y={home_y:.2f} z={home_z:.2f}"
        )
        self._build_square_waypoints()

    def _build_square_waypoints(self):
        if self.home_ned is None:
            return
        hx, hy, hz = self.home_ned
        s = self.side_m
        if not self.use_current_altitude:
            hz = -self.fixed_altitude
        self.square_waypoints_ned = [
            (hx, hy, hz),
            (hx + s, hy, hz),
            (hx + s, hy + s, hz),
            (hx, hy + s, hz),
            (hx, hy, hz),
        ]
        self.get_logger().info(
            f"Square waypoints (relative, side={s}m): {len(self.square_waypoints_ned)} points"
        )

    def timer_callback(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = timestamp
        offboard_mode.position = True
        self.offboard_control_mode_pub.publish(offboard_mode)

        if self.step == 0:
            if not self.square_waypoints_ned:
                return
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.get_logger().info("Armed and Offboard; starting square from current position.")
            self.step = 1
            self.waypoint_start_time = time.time()
            return

        if 1 <= self.step <= len(self.square_waypoints_ned):
            traj = TrajectorySetpoint()
            traj.timestamp = timestamp
            traj.position = list(self.square_waypoints_ned[self.step - 1])
            traj.yaw = 0.0
            self.trajectory_setpoint_pub.publish(traj)

            if self.waypoint_start_time is not None and time.time() - self.waypoint_start_time >= self.waypoint_hold_s:
                self.step += 1
                self.waypoint_start_time = time.time()
                self.get_logger().info(f"Waypoint {self.step - 1} done, next {self.step}/{len(self.square_waypoints_ned)}")

        elif self.step == len(self.square_waypoints_ned) + 1:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info("Landing...")
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
    node = OffboardSquareMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
