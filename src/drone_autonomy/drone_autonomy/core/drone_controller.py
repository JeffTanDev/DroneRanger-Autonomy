"""
Base drone controller providing fundamental flight operations.

This module handles:
- Basic flight commands (arm, disarm, takeoff, land)
- Offboard mode control with PX4
- Position and velocity control
- Safety monitoring
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
import time
from typing import Tuple, Optional

from px4_msgs.msg import (
    VehicleCommand, OffboardControlMode, TrajectorySetpoint,
    VehicleStatus, VehicleOdometry
)

from .coordinate_transforms import enu_to_ned, ned_to_enu


class DroneController(Node):
    """Base drone controller for PX4 offboard control."""

    def __init__(self, node_name: str = 'drone_controller'):
        super().__init__(node_name)

        # QoS profile for PX4 compatibility
        qos_profile = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_profile)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_profile)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_profile)

        # Subscribers
        self.vehicle_status_sub = self.create_subscription(
            VehicleStatus, '/fmu/out/vehicle_status_v1',
            self.vehicle_status_callback, qos_profile)
        self.vehicle_odometry_sub = self.create_subscription(
            VehicleOdometry, '/fmu/out/vehicle_odometry',
            self.vehicle_odometry_callback, qos_profile)

        # State variables
        self.vehicle_status = None
        self.vehicle_odometry = None
        self.current_position_ned = (0.0, 0.0, 0.0)
        self.current_position_enu = (0.0, 0.0, 0.0)
        self.is_armed = False
        self.flight_mode = None

        # Control parameters
        self.offboard_setpoint_counter = 0
        self.arm_counter = 0

        # Timer for control loop
        self.timer = self.create_timer(0.1, self.control_loop_callback)

        self.get_logger().info('Drone Controller initialized')

    def vehicle_status_callback(self, msg):
        """Handle vehicle status updates."""
        self.vehicle_status = msg
        self.is_armed = (msg.arming_state == VehicleStatus.ARMING_STATE_ARMED)
        self.flight_mode = msg.nav_state

    def vehicle_odometry_callback(self, msg):
        """Handle vehicle odometry updates."""
        self.vehicle_odometry = msg
        # Update current position (NED from PX4)
        self.current_position_ned = (msg.position[0], msg.position[1], msg.position[2])
        # Convert to ENU for easier use in planning
        self.current_position_enu = ned_to_enu(*self.current_position_ned)

    def control_loop_callback(self):
        """Main control loop - override in subclasses."""
        self.publish_offboard_control_mode()

    def publish_offboard_control_mode(self, position=True, velocity=False, acceleration=False):
        """Publish offboard control mode."""
        msg = OffboardControlMode()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = acceleration
        self.offboard_control_mode_pub.publish(msg)

    def publish_trajectory_setpoint(self, position_ned: Tuple[float, float, float],
                                  yaw: float = 0.0, velocity_ned: Optional[Tuple[float, float, float]] = None):
        """Publish trajectory setpoint in NED coordinates."""
        msg = TrajectorySetpoint()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.position = list(position_ned)
        msg.yaw = yaw

        if velocity_ned:
            msg.velocity = list(velocity_ned)

        self.trajectory_setpoint_pub.publish(msg)

    def publish_vehicle_command(self, command: int, **params):
        """Publish vehicle command."""
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(params.get('param1', 0.0))
        msg.param2 = float(params.get('param2', 0.0))
        msg.param3 = float(params.get('param3', 0.0))
        msg.param4 = float(params.get('param4', 0.0))
        msg.param5 = float(params.get('param5', 0.0))
        msg.param6 = float(params.get('param6', 0.0))
        msg.param7 = float(params.get('param7', 0.0))
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def arm(self, force=True):
        """Arm the vehicle."""
        # param2=21196.0 forces arming (bypasses safety checks)
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            param1=1.0,
            param2=21196.0 if force else 0.0
        )
        self.get_logger().info('Arm command sent (force={})'.format(force))

    def disarm(self):
        """Disarm the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=0.0)
        self.get_logger().info('Disarm command sent')

    def set_offboard_mode(self):
        """Switch to offboard mode."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            param1=1.0,  # base mode = custom
            param2=6.0   # custom mode = offboard
        )
        self.get_logger().info('Offboard mode command sent')

    def takeoff(self, altitude: float = 5.0):
        """Takeoff to specified altitude."""
        self.publish_vehicle_command(
            VehicleCommand.VEHICLE_CMD_NAV_TAKEOFF,
            param7=altitude
        )
        self.get_logger().info(f'Takeoff command sent (altitude: {altitude}m)')

    def land(self):
        """Land the vehicle."""
        self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
        self.get_logger().info('Land command sent')

    def goto_position_enu(self, x: float, y: float, z: float, yaw: float = 0.0):
        """Go to position in ENU coordinates."""
        position_ned = enu_to_ned(x, y, z)
        self.publish_trajectory_setpoint(position_ned, yaw)

    def goto_position_ned(self, x: float, y: float, z: float, yaw: float = 0.0):
        """Go to position in NED coordinates."""
        self.publish_trajectory_setpoint((x, y, z), yaw)

    def is_at_position(self, target_pos_enu: Tuple[float, float, float], tolerance: float = 0.5) -> bool:
        """Check if drone is at target position (ENU coordinates)."""
        if not self.current_position_enu:
            return False

        distance = ((self.current_position_enu[0] - target_pos_enu[0])**2 +
                   (self.current_position_enu[1] - target_pos_enu[1])**2 +
                   (self.current_position_enu[2] - target_pos_enu[2])**2)**0.5

        return distance < tolerance

    def wait_for_arm(self, timeout: float = 10.0) -> bool:
        """Wait for vehicle to arm."""
        start_time = time.time()
        while not self.is_armed and (time.time() - start_time) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
        return self.is_armed


def main(args=None):
    """Main function for running standalone drone controller."""
    rclpy.init(args=args)

    # Create simple hover test
    controller = DroneController('test_controller')

    try:
        rclpy.spin(controller)
    except KeyboardInterrupt:
        pass
    finally:
        controller.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()