import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint
import time


class OffboardSquareMission(Node):
    def __init__(self):
        super().__init__('offboard_square_mission')

        self.declare_parameter('altitude', 10.0)
        self.altitude = self.get_parameter('altitude').value

        self.offboard_control_mode_pub = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)
        self.step = 0
        self.start_time = None

    def timer_callback(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

            # Must continuously publish OffboardControlMode
        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = timestamp
        offboard_mode.position = True
        self.offboard_control_mode_pub.publish(offboard_mode)

            # Mission: square path
        square = [
            (0.0, 0.0, -self.altitude),   # Takeoff point
            (10.0, 0.0, -self.altitude),  # Forward
            (10.0, 10.0, -self.altitude), # Right
            (0.0, 10.0, -self.altitude),  # Back
            (0.0, 0.0, -self.altitude)    # Start point
        ]

            # Step 0: Arm + Offboard
        if self.step == 0:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.get_logger().info("✅ Armed and switched to Offboard")
            self.start_time = time.time()
            self.step = 1

            # Step 1-n: Follow square points
        elif 1 <= self.step <= len(square):
            traj = TrajectorySetpoint()
            traj.timestamp = timestamp
            traj.position = list(square[self.step - 1])
            traj.yaw = 0.0
            self.trajectory_setpoint_pub.publish(traj)

                # Stay at each waypoint for 5 seconds
            if time.time() - self.start_time > 5:
                self.step += 1
                self.start_time = time.time()
                self.get_logger().info(f"📍 Reached waypoint {self.step-1}")

            # Final step: Land
        elif self.step == len(square) + 1:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info("🛬 Landing...")
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
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
