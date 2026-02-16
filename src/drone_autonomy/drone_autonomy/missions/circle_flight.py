"""
Circle flight mission - flies in a circular pattern.

This mission demonstrates:
1. Smooth circular trajectory
2. Continuous waypoint generation
3. Constant velocity control
"""

import rclpy
import time
import math
from rclpy.parameter import Parameter

from ..core.drone_controller import DroneController


class CircleFlightMission(DroneController):
    """Flies in a circular pattern at specified altitude."""

    def __init__(self):
        super().__init__('circle_flight_mission')

        # Declare parameters
        self.declare_parameter('altitude', 5.0)
        self.declare_parameter('radius', 8.0)
        self.declare_parameter('num_circles', 2)
        self.declare_parameter('angular_velocity', 0.3)  # rad/s

        # Get parameters
        self.altitude = self.get_parameter('altitude').get_parameter_value().double_value
        self.radius = self.get_parameter('radius').get_parameter_value().double_value
        self.num_circles = self.get_parameter('num_circles').get_parameter_value().integer_value
        self.angular_velocity = self.get_parameter('angular_velocity').get_parameter_value().double_value

        # Mission state
        self.mission_state = 'INIT'
        self.state_start_time = None
        self.setpoint_counter = 0
        self.circle_start_time = None
        self.total_angle = 0.0

        self.get_logger().info(f'Circle Flight Mission initialized')
        self.get_logger().info(f'Altitude: {self.altitude}m, Radius: {self.radius}m, Circles: {self.num_circles}')

    def control_loop_callback(self):
        """State machine for circle flight."""
        self.publish_offboard_control_mode()

        if self.mission_state == 'INIT':
            self.init_state()

        elif self.mission_state == 'PRE_ARM':
            self.pre_arm_state()

        elif self.mission_state == 'ARM_AND_OFFBOARD':
            self.arm_and_offboard_state()

        elif self.mission_state == 'TAKEOFF':
            self.takeoff_state()

        elif self.mission_state == 'MOVE_TO_START':
            self.move_to_start_state()

        elif self.mission_state == 'CIRCLE':
            self.circle_state()

        elif self.mission_state == 'RETURN_CENTER':
            self.return_center_state()

        elif self.mission_state == 'LAND':
            self.land_state()

        elif self.mission_state == 'COMPLETE':
            self.complete_state()

    def init_state(self):
        """Initialize mission."""
        self.get_logger().info('🚁 Starting circle flight mission...')
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

    def arm_and_offboard_state(self):
        """Arm and switch to offboard."""
        if not self.is_armed:
            self.set_offboard_mode()
            self.arm()

        target_pos_ned = (0.0, 0.0, -self.altitude)
        self.publish_trajectory_setpoint(target_pos_ned)

        if self.is_armed:
            self.get_logger().info('✅ Armed, taking off...')
            self.mission_state = 'TAKEOFF'
            self.state_start_time = time.time()

    def takeoff_state(self):
        """Takeoff to altitude."""
        target_pos_ned = (0.0, 0.0, -self.altitude)
        self.publish_trajectory_setpoint(target_pos_ned)

        target_pos_enu = (0.0, 0.0, self.altitude)
        if self.is_at_position(target_pos_enu, tolerance=0.5):
            self.get_logger().info(f'✅ At altitude, moving to circle start position...')
            self.mission_state = 'MOVE_TO_START'
            self.state_start_time = time.time()

    def move_to_start_state(self):
        """Move to circle start position."""
        # Start at (radius, 0)
        target_pos_ned = (self.radius, 0.0, -self.altitude)
        self.publish_trajectory_setpoint(target_pos_ned)

        target_pos_enu = (0.0, self.radius, self.altitude)
        if self.is_at_position(target_pos_enu, tolerance=0.5):
            self.get_logger().info(f'✅ At start position, beginning circles...')
            self.mission_state = 'CIRCLE'
            self.circle_start_time = time.time()
            self.total_angle = 0.0

    def circle_state(self):
        """Fly in circular pattern."""
        elapsed = time.time() - self.circle_start_time
        self.total_angle = self.angular_velocity * elapsed

        # Calculate position on circle (NED frame)
        # X (North) = radius * cos(angle)
        # Y (East) = radius * sin(angle)
        x = self.radius * math.cos(self.total_angle)
        y = self.radius * math.sin(self.total_angle)

        target_pos_ned = (x, y, -self.altitude)

        # Calculate yaw to face forward along circle
        yaw = self.total_angle + math.pi / 2

        self.publish_trajectory_setpoint(target_pos_ned, yaw=yaw)

        # Check if completed required number of circles
        circles_completed = self.total_angle / (2 * math.pi)

        # Log progress every half circle
        if int(circles_completed * 2) != getattr(self, '_last_log_circles', -1):
            self.get_logger().info(f'🔄 Circles completed: {circles_completed:.1f}/{self.num_circles}')
            self._last_log_circles = int(circles_completed * 2)

        if circles_completed >= self.num_circles:
            self.get_logger().info(f'✅ Completed {self.num_circles} circles, returning to center...')
            self.mission_state = 'RETURN_CENTER'
            self.state_start_time = time.time()

    def return_center_state(self):
        """Return to center position."""
        target_pos_ned = (0.0, 0.0, -self.altitude)
        self.publish_trajectory_setpoint(target_pos_ned)

        target_pos_enu = (0.0, 0.0, self.altitude)
        if self.is_at_position(target_pos_enu, tolerance=0.5):
            self.get_logger().info('✅ Returned to center, landing...')
            self.mission_state = 'LAND'
            self.state_start_time = time.time()

    def land_state(self):
        """Land at center."""
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
        mission = CircleFlightMission()
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
