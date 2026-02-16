"""
Figure-eight flight mission - flies a figure-8 pattern.

This mission demonstrates:
1. Complex trajectory following
2. Smooth transitions between curves
3. Advanced path planning
"""

import rclpy
import time
import math
from rclpy.parameter import Parameter

from ..core.drone_controller import DroneController


class FigureEightMission(DroneController):
    """Flies a figure-8 pattern at specified altitude."""

    def __init__(self):
        super().__init__('figure_eight_mission')

        # Declare parameters
        self.declare_parameter('altitude', 5.0)
        self.declare_parameter('radius', 6.0)
        self.declare_parameter('num_loops', 2)
        self.declare_parameter('angular_velocity', 0.4)

        # Get parameters
        self.altitude = self.get_parameter('altitude').get_parameter_value().double_value
        self.radius = self.get_parameter('radius').get_parameter_value().double_value
        self.num_loops = self.get_parameter('num_loops').get_parameter_value().integer_value
        self.angular_velocity = self.get_parameter('angular_velocity').get_parameter_value().double_value

        # Mission state
        self.mission_state = 'INIT'
        self.state_start_time = None
        self.setpoint_counter = 0
        self.pattern_start_time = None
        self.total_angle = 0.0

        self.get_logger().info(f'Figure-8 Flight Mission initialized')
        self.get_logger().info(f'Altitude: {self.altitude}m, Radius: {self.radius}m, Loops: {self.num_loops}')

    def control_loop_callback(self):
        """State machine for figure-8 flight."""
        self.publish_offboard_control_mode()

        if self.mission_state == 'INIT':
            self.init_state()

        elif self.mission_state == 'PRE_ARM':
            self.pre_arm_state()

        elif self.mission_state == 'ARM_AND_OFFBOARD':
            self.arm_and_offboard_state()

        elif self.mission_state == 'TAKEOFF':
            self.takeoff_state()

        elif self.mission_state == 'FIGURE_EIGHT':
            self.figure_eight_state()

        elif self.mission_state == 'RETURN_CENTER':
            self.return_center_state()

        elif self.mission_state == 'LAND':
            self.land_state()

        elif self.mission_state == 'COMPLETE':
            self.complete_state()

    def init_state(self):
        """Initialize mission."""
        self.get_logger().info('🚁 Starting figure-8 flight mission...')
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
            self.get_logger().info(f'✅ At altitude, starting figure-8 pattern...')
            self.mission_state = 'FIGURE_EIGHT'
            self.pattern_start_time = time.time()
            self.total_angle = 0.0

    def figure_eight_state(self):
        """Fly figure-8 pattern using Lissajous curve."""
        elapsed = time.time() - self.pattern_start_time
        self.total_angle = self.angular_velocity * elapsed

        # Figure-8 parametric equations (Lissajous curve)
        # x(t) = A * sin(t)
        # y(t) = B * sin(2t)
        t = self.total_angle

        x = self.radius * math.sin(t)
        y = self.radius * math.sin(2 * t) / 2  # Divide by 2 to make symmetric

        target_pos_ned = (x, y, -self.altitude)

        # Calculate yaw tangent to path
        dx = self.radius * math.cos(t)
        dy = self.radius * math.cos(2 * t)
        yaw = math.atan2(dy, dx)

        self.publish_trajectory_setpoint(target_pos_ned, yaw=yaw)

        # Check if completed required number of loops
        loops_completed = self.total_angle / (2 * math.pi)

        # Log progress
        if int(loops_completed * 4) != getattr(self, '_last_log_loops', -1):
            self.get_logger().info(f'∞ Figure-8 loops: {loops_completed:.1f}/{self.num_loops}')
            self._last_log_loops = int(loops_completed * 4)

        if loops_completed >= self.num_loops:
            self.get_logger().info(f'✅ Completed {self.num_loops} figure-8 loops, returning...')
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
        mission = FigureEightMission()
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
