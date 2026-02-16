import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint, VehicleOdometry
import time


class OffboardTakeoffLand(Node):
    def __init__(self):
        super().__init__('offboard_takeoff_land')

        # Parameters
        self.declare_parameter('hover_time', 10.0)      # Hover time (seconds)
        self.declare_parameter('takeoff_height', 5.0)   # Takeoff height (meters)

        self.hover_time = self.get_parameter('hover_time').value
        self.takeoff_height = self.get_parameter('takeoff_height').value

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)
        self.odometry_pub = self.create_publisher(
            VehicleOdometry, '/fmu/in/vehicle_odometry', 10)

        # Timer (20Hz)
        self.timer = self.create_timer(0.05, self.timer_callback)
        self.offboard_setpoint_counter = 0
        self.takeoff_time = None
        self.landed = False

    def timer_callback(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)  # PX4 expects usec

        # Step 1: Publish OffboardControlMode
        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = timestamp
        offboard_mode.position = True
        self.offboard_control_mode_pub.publish(offboard_mode)

        # Step 2: Publish TrajectorySetpoint (target altitude)
        traj = TrajectorySetpoint()
        traj.timestamp = timestamp
        traj.position = [0.0, 0.0, -self.takeoff_height]  # NED frame: z = -h means ascending to h meters
        traj.yaw = 0.0
        self.trajectory_setpoint_pub.publish(traj)


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
