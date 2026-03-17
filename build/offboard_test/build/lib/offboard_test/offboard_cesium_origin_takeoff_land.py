"""
Takeoff at Cesium/Unity WorldOrigin -> hover -> land.

This mission treats the Unity/Cesium `WorldOrigin` as (0,0,0) in a "world-local" frame
coming from `/drone/odom` (nav_msgs/Odometry) in ENU, then converts to NED and maps it
to PX4 local NED by estimating a constant offset:

  px4_local_ned = world_ned + world_to_px4_offset_ned

So commanding (0,0,-10) in world_ned means "take off 10m above Cesium origin".
"""

import time

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand, VehicleOdometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class OffboardCesiumOriginTakeoffLand(Node):
    def __init__(self):
        super().__init__("offboard_cesium_origin_takeoff_land")

        self.declare_parameter("takeoff_altitude_m", 10.0)
        self.declare_parameter("takeoff_settle_s", 2.0)
        self.declare_parameter("hover_s", 2.0)
        self.declare_parameter("shutdown_delay_s", 3.0)

        self.takeoff_alt_m = float(self.get_parameter("takeoff_altitude_m").value)
        self.takeoff_settle_s = float(self.get_parameter("takeoff_settle_s").value)
        self.hover_s = float(self.get_parameter("hover_s").value)
        self.shutdown_delay_s = float(self.get_parameter("shutdown_delay_s").value)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", 10
        )
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", 10
        )
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, "/fmu/in/vehicle_command", 10)

        # Unity/ROS odom in ENU, aligned to Unity `WorldOrigin` as (0,0,0).
        self.odom_sub = self.create_subscription(Odometry, "/drone/odom", self.odom_callback, 10)
        # PX4 local position in NED (EKF local origin).
        self.px4_odom_sub = self.create_subscription(
            VehicleOdometry, "/fmu/out/vehicle_odometry", self.px4_odom_callback, qos
        )

        self.timer = self.create_timer(0.05, self.timer_callback)

        self.have_world_odom = False
        self.have_px4_odom = False
        self.current_world_ned = None
        self.current_px4_ned = None
        self.world_to_px4_offset_ned = None

        self.phase = "WAIT_HOME"
        self.phase_start_time = None
        self.land_sent = False

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
                "Received /drone/odom. Treating Unity/Cesium WorldOrigin as world NED origin (0,0,0)."
            )

    def px4_odom_callback(self, msg: VehicleOdometry):
        self.current_px4_ned = (float(msg.position[0]), float(msg.position[1]), float(msg.position[2]))
        if not self.have_px4_odom:
            self.have_px4_odom = True
            self.get_logger().info("Received /fmu/out/vehicle_odometry. PX4 local position ready.")

    def publish_vehicle_command(self, command, **kwargs):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(kwargs.get("param1", 0.0))
        msg.param2 = float(kwargs.get("param2", 0.0))
        msg.param7 = float(kwargs.get("param7", 0.0))
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def timer_callback(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = timestamp
        offboard_mode.position = True
        self.offboard_control_mode_pub.publish(offboard_mode)

        if self.phase == "WAIT_HOME":
            if not (self.have_world_odom and self.have_px4_odom):
                return
            if self.world_to_px4_offset_ned is None:
                wx, wy, wz = self.current_world_ned
                px, py, pz = self.current_px4_ned
                self.world_to_px4_offset_ned = (px - wx, py - wy, pz - wz)
                ox, oy, oz = self.world_to_px4_offset_ned
                self.get_logger().info(
                    f"Computed world->PX4 offset (NED): ox={ox:.2f} oy={oy:.2f} oz={oz:.2f}"
                )
                self.phase = "PRE_ARM"
                self.phase_start_time = time.time()
            return

        target_world_x = 0.0
        target_world_y = 0.0
        target_world_z = -self.takeoff_alt_m  # NED: negative Down means up
        ox, oy, oz = self.world_to_px4_offset_ned

        def to_px4_setpoint(world_x, world_y, world_z):
            return [world_x + ox, world_y + oy, world_z + oz]

        if self.phase == "PRE_ARM":
            traj = TrajectorySetpoint()
            traj.timestamp = timestamp
            traj.position = to_px4_setpoint(target_world_x, target_world_y, target_world_z)
            traj.yaw = 0.0
            self.trajectory_setpoint_pub.publish(traj)

            if self.phase_start_time and time.time() - self.phase_start_time >= self.takeoff_settle_s:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                self.get_logger().info(f"Armed + Offboard. Taking off to {self.takeoff_alt_m:.1f}m above origin.")
                self.phase = "TAKEOFF_HOLD"
                self.phase_start_time = time.time()

        elif self.phase == "TAKEOFF_HOLD":
            traj = TrajectorySetpoint()
            traj.timestamp = timestamp
            traj.position = to_px4_setpoint(target_world_x, target_world_y, target_world_z)
            traj.yaw = 0.0
            self.trajectory_setpoint_pub.publish(traj)

            if self.phase_start_time and time.time() - self.phase_start_time >= self.hover_s:
                self.get_logger().info("Hover done; landing.")
                self.phase = "LAND"
                self.phase_start_time = time.time()

        elif self.phase == "LAND":
            if not self.land_sent:
                self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.land_sent = True
                self.get_logger().info("Land command sent.")

            if self.phase_start_time and time.time() - self.phase_start_time >= self.shutdown_delay_s:
                rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = OffboardCesiumOriginTakeoffLand()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

