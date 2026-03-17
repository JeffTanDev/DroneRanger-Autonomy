import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleOdometry
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
import math
import time


class OffboardTakeoffLand(Node):
    def __init__(self):
        super().__init__('offboard_takeoff_land')

        # Parameters
        self.declare_parameter('hover_time', 10.0)      # Hover time (seconds)
        self.declare_parameter('takeoff_height', 5.0)   # Takeoff height (meters)

        self.hover_time = self.get_parameter('hover_time').value
        self.takeoff_height = self.get_parameter('takeoff_height').value

        # QoS to match PX4 /fmu/out/* topics (typically best-effort)
        qos_profile_px4 = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Subscriber: PX4 vehicle odometry (NED)
        self.odom_sub = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile_px4
        )

        # Timer (20Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.offboard_setpoint_counter = 0
        self.takeoff_time = None
        self.landed = False

        # Cached state
        self.have_start = False
        self.start_n = 0.0
        self.start_e = 0.0
        self.start_d = 0.0  # NED Down (positive down)
        self.last_wait_log_sec = 0
        self.last_sp_log_sec = 0

    def odometry_callback(self, msg: VehicleOdometry):
        # VehicleOdometry.position is NED: [north, east, down] (meters)
        if not self.have_start:
            n = float(msg.position[0])
            e = float(msg.position[1])
            d = float(msg.position[2])
            if math.isfinite(n) and math.isfinite(e) and math.isfinite(d):
                self.start_n = n
                self.start_e = e
                self.start_d = d
                self.have_start = True
                self.get_logger().info(
                    f"📍 Got vehicle_odometry (finite). Start NED: n={self.start_n:.2f} e={self.start_e:.2f} d={self.start_d:.2f}"
                )

    def timer_callback(self):
        if not self.have_start:
            # Need a valid start point before commanding a height-only takeoff.
            now_sec = int(self.get_clock().now().nanoseconds / 1e9)
            if now_sec != self.last_wait_log_sec:
                self.last_wait_log_sec = now_sec
                self.get_logger().warn("Waiting for /fmu/out/vehicle_odometry (finite N/E/D) ... mission not started yet.")
            return

        timestamp = int(self.get_clock().now().nanoseconds / 1000)  # PX4 expects usec

        # Step 1: Publish OffboardControlMode
        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = timestamp
        offboard_mode.position = True
        self.offboard_control_mode_pub.publish(offboard_mode)

        # Step 2: Publish TrajectorySetpoint
        # Keep x/y fixed at takeoff point; only change height.
        # NED frame: z is Down. Ascend by takeoff_height => z decreases by takeoff_height.
        target_d = self.start_d - float(self.takeoff_height)
        traj = TrajectorySetpoint()
        traj.timestamp = timestamp
        traj.position = [self.start_n, self.start_e, target_d]
        traj.yaw = 0.0
        self.trajectory_setpoint_pub.publish(traj)

        now_sec = int(self.get_clock().now().nanoseconds / 1e9)
        if now_sec != self.last_sp_log_sec:
            self.last_sp_log_sec = now_sec
            self.get_logger().info(
                f"SP NED: n={self.start_n:.2f} e={self.start_e:.2f} d={target_d:.2f} (takeoff_height={float(self.takeoff_height):.2f})"
            )


        # Step 3: Send setpoints first; after 10 cycles, switch to Offboard and Arm
        if self.offboard_setpoint_counter == 10:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,   # base mode = custom
                param2=6.0    # custom mode = Offboard
            )
            self.get_logger().info("✅ Offboard mode command sent")

            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0    # Arm
            )
            self.get_logger().info("✅ Arm command sent")

            # Record the takeoff time
            self.takeoff_time = time.time()

        # Step 4: After hover_time seconds, send Land command
        if self.takeoff_time and not self.landed:
            if time.time() - self.takeoff_time > self.hover_time:
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_NAV_LAND,
                    param1=0.0
                )
                self.get_logger().info(
                    f"🛬 Land command sent after {self.hover_time} sec hover at {self.takeoff_height}m"
                )
                self.landed = True

                # Shut down the node 3 seconds after landing command
                self.create_timer(3.0, self.shutdown_node)

        self.offboard_setpoint_counter += 1

    def publish_vehicle_command(self, command, **kwargs):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(kwargs.get('param1', 0.0))
        msg.param2 = float(kwargs.get('param2', 0.0))
        msg.param7 = float(kwargs.get('param7', 0.0))
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def shutdown_node(self):
        self.get_logger().info("👋 Node shutting down after landing")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = OffboardTakeoffLand()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
