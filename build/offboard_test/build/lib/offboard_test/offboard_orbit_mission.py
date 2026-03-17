"""
Takeoff -> orbit -> land (NED Cartesian).

- **Setpoints** are published to PX4 as local NED: (x=North, y=East, z=Down).
- **Origin** for the orbit is the Unity `WorldOrigin` (ROS /drone/odom frame origin).
  That means: when /drone/odom position is (0,0,0) in ENU, we treat it as (0,0,0) in NED
  after ENU->NED conversion.

We use /drone/odom (nav_msgs/Odometry) for "world origin is (0,0,0)" semantics and convert:
  ENU (x=East, y=North, z=Up) -> NED (x=North, y=East, z=Down)
    x_ned = y_enu
    y_ned = x_enu
    z_ned = -z_enu
"""

import rclpy
import math
import time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from nav_msgs.msg import Odometry
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
        # Unity/ROS odom in ENU, aligned to Unity `WorldOrigin` as (0,0,0).
        self.odom_sub = self.create_subscription(Odometry, '/drone/odom', self.odom_callback, 10)
        # PX4 local position in NED (EKF local origin). We need this to compute a constant offset
        # between Unity(WorldOrigin) local NED and PX4 local NED so we can command setpoints correctly.
        self.px4_odom_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry', self.px4_odom_callback, qos
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

        # Orbit center is WorldOrigin => (0,0,0) in Unity-local NED.
        self.home_world_ned = (0.0, 0.0, 0.0)

        # Latest states
        self.have_world_odom = False
        self.have_px4_odom = False
        self.current_world_ned = None  # derived from /drone/odom (ENU->NED)
        self.current_px4_ned = None    # from /fmu/out/vehicle_odometry

        # Mapping: px4_local_ned = world_ned + world_to_px4_offset_ned
        self.world_to_px4_offset_ned = None
        self.phase = 'WAIT_HOME'
        self.phase_start_time = None
        self.orbit_start_angle = 0.0

    @staticmethod
    def enu_to_ned(x_enu: float, y_enu: float, z_enu: float):
        # ENU (E,N,U) -> NED (N,E,D)
        return (y_enu, x_enu, -z_enu)

    def odom_callback(self, msg: Odometry):
        p = msg.pose.pose.position
        self.current_world_ned = self.enu_to_ned(float(p.x), float(p.y), float(p.z))
        if not self.have_world_odom:
            self.have_world_odom = True
            self.get_logger().info(
                "Received /drone/odom. Using Unity WorldOrigin as NED origin (0,0,0). "
                f"Current world NED now: x={self.current_world_ned[0]:.2f} y={self.current_world_ned[1]:.2f} z={self.current_world_ned[2]:.2f}"
            )

    def px4_odom_callback(self, msg: VehicleOdometry):
        self.current_px4_ned = (float(msg.position[0]), float(msg.position[1]), float(msg.position[2]))
        if not self.have_px4_odom:
            self.have_px4_odom = True
            self.get_logger().info(
                f"Received /fmu/out/vehicle_odometry. Current PX4 local NED: x={self.current_px4_ned[0]:.2f} y={self.current_px4_ned[1]:.2f} z={self.current_px4_ned[2]:.2f}"
            )

    def timer_callback(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)
        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = timestamp
        offboard_mode.position = True
        self.offboard_control_mode_pub.publish(offboard_mode)

        if self.phase == 'WAIT_HOME':
            if not (self.have_world_odom and self.have_px4_odom):
                return
            if self.world_to_px4_offset_ned is None:
                wx, wy, wz = self.current_world_ned
                px, py, pz = self.current_px4_ned
                self.world_to_px4_offset_ned = (px - wx, py - wy, pz - wz)
                ox, oy, oz = self.world_to_px4_offset_ned
                self.get_logger().info(
                    "Computed world->PX4 NED offset: "
                    f"ox={ox:.2f} oy={oy:.2f} oz={oz:.2f} (so setpoint_px4 = setpoint_world + offset)"
                )
                self.phase = 'PRE_ARM'
                self.phase_start_time = time.time()
            return

        hx, hy, hz = self.home_world_ned
        target_z = -self.takeoff_alt_m
        ox, oy, oz = self.world_to_px4_offset_ned

        def to_px4_setpoint(world_x, world_y, world_z):
            return [world_x + ox, world_y + oy, world_z + oz]

        if self.phase == 'PRE_ARM':
            traj = TrajectorySetpoint()
            traj.timestamp = timestamp
            traj.position = to_px4_setpoint(hx, hy, target_z)
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
            traj.position = to_px4_setpoint(hx, hy, target_z)
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
            traj.position = to_px4_setpoint(x, y, target_z)
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
