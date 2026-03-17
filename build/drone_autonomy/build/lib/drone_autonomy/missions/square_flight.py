"""
Square flight mission - flies a square pattern.

This mission demonstrates:
1. Waypoint navigation
2. Position hold at each corner
3. Autonomous square pattern flight
"""

import rclpy
import time
from rclpy.parameter import Parameter

from ..core.drone_controller import DroneController


class SquareFlightMission(DroneController):
    """Flies a square pattern at specified altitude."""

    def __init__(self):
        super().__init__('square_flight_mission')

        # Declare parameters
        self.declare_parameter('altitude', 5.0)
        self.declare_parameter('side_length', 10.0)
        self.declare_parameter('corner_pause', 2.0)

        # Get parameters
        self.altitude = self.get_parameter('altitude').get_parameter_value().double_value
        self.side_length = self.get_parameter('side_length').get_parameter_value().double_value
        self.corner_pause = self.get_parameter('corner_pause').get_parameter_value().double_value

        # Define square waypoints (NED coordinates)
        half_side = self.side_length / 2
        self.waypoints = [
            (half_side, half_side, -self.altitude),    # Corner 1 (NE)
            (half_side, -half_side, -self.altitude),   # Corner 2 (SE)
            (-half_side, -half_side, -self.altitude),  # Corner 3 (SW)
            (-half_side, half_side, -self.altitude),   # Corner 4 (NW)
            (0.0, 0.0, -self.altitude),                # Return to center
        ]

        # Mission state
        self.mission_state = 'INIT'
        self.state_start_time = None
        self.setpoint_counter = 0
        self.current_waypoint = 0

        self.get_logger().info(f'Square Flight Mission initialized')
        self.get_logger().info(f'Altitude: {self.altitude}m, Side: {self.side_length}m')

    def control_loop_callback(self):
        """State machine for square flight."""
        self.publish_offboard_control_mode()

        if self.mission_state == 'INIT':
            self.init_state()

        elif self.mission_state == 'PRE_ARM':
            self.pre_arm_state()

        elif self.mission_state == 'ARM_AND_OFFBOARD':
            self.arm_and_offboard_state()

        elif self.mission_state == 'TAKEOFF':
            self.takeoff_state()

        elif self.mission_state == 'NAVIGATE':
            self.navigate_state()

        elif self.mission_state == 'HOVER':
            self.hover_state()

        elif self.mission_state == 'LAND':
            self.land_state()

        elif self.mission_state == 'COMPLETE':
            self.complete_state()

    def init_state(self):
        """Initialize mission."""
        self.get_logger().info('🚁 Starting square flight mission...')
        self.mission_state = 'PRE_ARM'
        self.state_start_time = time.time()

    def pre_arm_state(self):
        """Send setpoints before arming."""
        target_pos_ned = (0.0, 0.0, -self.altitude)
        self.publish_trajectory_setpoint(target_pos_ned)
        self.setpoint_counter += 1

        if self.setpoint_counter >= 10:
            self.mission_state = 'ARM_AND_OFFBOARD'
            self.state_start_time = time.time()
            self.get_logger().info('Pre-arm complete')

    def arm_and_offboard_state(self):
        """Arm and switch to offboard."""
        if not self.is_armed:
            self.set_offboard_mode()
            self.arm()

        target_pos_ned = (0.0, 0.0, -self.altitude)
        self.publish_trajectory_setpoint(target_pos_ned)

        if self.is_armed:
            self.get_logger().info('✅ Armed, starting takeoff...')
            self.mission_state = 'TAKEOFF'
            self.state_start_time = time.time()

    def takeoff_state(self):
        """Takeoff to initial altitude."""
        target_pos_ned = (0.0, 0.0, -self.altitude)
        self.publish_trajectory_setpoint(target_pos_ned)

        target_pos_enu = (0.0, 0.0, self.altitude)
        if self.is_at_position(target_pos_enu, tolerance=0.5):
            self.get_logger().info(f'✅ Reached altitude {self.altitude}m, starting square pattern...')
            self.mission_state = 'NAVIGATE'
            self.state_start_time = time.time()
            self.current_waypoint = 0

    def navigate_state(self):
        """Navigate to current waypoint."""
        if self.current_waypoint >= len(self.waypoints):
            self.get_logger().info('✅ Square pattern complete, landing...')
            self.mission_state = 'LAND'
            self.state_start_time = time.time()
            return

        # Get current waypoint
        target_ned = self.waypoints[self.current_waypoint]
        self.publish_trajectory_setpoint(target_ned)

        # Convert NED to ENU for position check
        target_enu = (target_ned[1], target_ned[0], -target_ned[2])

        if self.is_at_position(target_enu, tolerance=0.5):
            if self.current_waypoint < 4:
                self.get_logger().info(f'✅ Reached corner {self.current_waypoint + 1}')
            else:
                self.get_logger().info(f'✅ Returned to center')

            self.mission_state = 'HOVER'
            self.state_start_time = time.time()

    def hover_state(self):
        """Hover at current waypoint."""
        target_ned = self.waypoints[self.current_waypoint]
        self.publish_trajectory_setpoint(target_ned)

        elapsed = time.time() - self.state_start_time
        if elapsed >= self.corner_pause:
            self.current_waypoint += 1
            self.mission_state = 'NAVIGATE'
            self.state_start_time = time.time()

    def land_state(self):
        """Land at current position."""
        self.land()
        self.get_logger().info('🛬 Landing...')
        self.mission_state = 'COMPLETE'
        self.state_start_time = time.time()

    def complete_state(self):
        """Mission complete."""
        if time.time() - self.state_start_time > 3.0:
            self.get_logger().info('✅ Mission completed!')
            self.create_timer(1.0, lambda: rclpy.shutdown())


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    try:
        mission = SquareFlightMission()
        rclpy.spin(mission)
    except KeyboardInterrupt:
        print('Mission interrupted by user')
    finally:
        try:
            mission.destroy_node()
        except:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
