"""
Orbit point mission - orbits around a point of interest while facing it.

This mission demonstrates:
1. Camera pointing control
2. Circular orbit with center tracking
3. Inspection-like behavior
"""

import rclpy
import time
import math
from rclpy.parameter import Parameter

from ..core.drone_controller import DroneController


class OrbitPointMission(DroneController):
    """Orbits around a point while always facing it."""

    def __init__(self):
        super().__init__('orbit_point_mission')

        # Declare parameters
        self.declare_parameter('altitude', 8.0)
        self.declare_parameter('orbit_radius', 10.0)
        self.declare_parameter('target_x', 0.0)
        self.declare_parameter('target_y', 0.0)
        self.declare_parameter('num_orbits', 2)
        self.declare_parameter('angular_velocity', 0.25)

        # Get parameters
        self.altitude = self.get_parameter('altitude').get_parameter_value().double_value
        self.orbit_radius = self.get_parameter('orbit_radius').get_parameter_value().double_value
        self.target_x = self.get_parameter('target_x').get_parameter_value().double_value
        self.target_y = self.get_parameter('target_y').get_parameter_value().double_value
        self.num_orbits = self.get_parameter('num_orbits').get_parameter_value().integer_value
        self.angular_velocity = self.get_parameter('angular_velocity').get_parameter_value().double_value

        # Mission state
        self.mission_state = 'INIT'
        self.state_start_time = None
        self.setpoint_counter = 0
        self.orbit_start_time = None
        self.total_angle = 0.0

        self.get_logger().info(f'Orbit Point Mission initialized')
        self.get_logger().info(f'Target: ({self.target_x}, {self.target_y}), Radius: {self.orbit_radius}m, Alt: {self.altitude}m')

    def control_loop_callback(self):
        """State machine for orbit mission."""
        self.publish_offboard_control_mode()

        if self.mission_state == 'INIT':
            self.init_state()

        elif self.mission_state == 'PRE_ARM':
            self.pre_arm_state()

        elif self.mission_state == 'ARM_AND_OFFBOARD':
            self.arm_and_offboard_state()

        elif self.mission_state == 'TAKEOFF':
            self.takeoff_state()

        elif self.mission_state == 'MOVE_TO_ORBIT':
            self.move_to_orbit_state()

        elif self.mission_state == 'ORBIT':
            self.orbit_state()

        elif self.mission_state == 'RETURN_HOME':
            self.return_home_state()

        elif self.mission_state == 'LAND':
            self.land_state()

        elif self.mission_state == 'COMPLETE':
            self.complete_state()

    def init_state(self):
        """Initialize mission."""
        self.get_logger().info('🚁 Starting orbit inspection mission...')
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
            self.get_logger().info(f'✅ At altitude, moving to orbit start...')
            self.mission_state = 'MOVE_TO_ORBIT'
            self.state_start_time = time.time()

    def move_to_orbit_state(self):
        """Move to orbit starting position."""
        # Start position: target + radius in X direction
        start_x = self.target_x + self.orbit_radius
        start_y = self.target_y

        target_pos_ned = (start_x, start_y, -self.altitude)
        self.publish_trajectory_setpoint(target_pos_ned)

        target_pos_enu = (start_y, start_x, self.altitude)
        if self.is_at_position(target_pos_enu, tolerance=0.8):
            self.get_logger().info(f'✅ At orbit start, beginning inspection orbit...')
            self.mission_state = 'ORBIT'
            self.orbit_start_time = time.time()
            self.total_angle = 0.0

    def orbit_state(self):
        """Orbit around target point while facing it."""
        elapsed = time.time() - self.orbit_start_time
        self.total_angle = self.angular_velocity * elapsed

        # Calculate position on orbit (NED frame, centered on target)
        x = self.target_x + self.orbit_radius * math.cos(self.total_angle)
        y = self.target_y + self.orbit_radius * math.sin(self.total_angle)

        target_pos_ned = (x, y, -self.altitude)

        # Calculate yaw to always face the target point
        # Yaw points from current position toward target
        dx = self.target_x - x
        dy = self.target_y - y
        yaw = math.atan2(dy, dx)

        self.publish_trajectory_setpoint(target_pos_ned, yaw=yaw)

        # Check progress
        orbits_completed = self.total_angle / (2 * math.pi)

        # Log every quarter orbit
        if int(orbits_completed * 4) != getattr(self, '_last_log_orbits', -1):
            self.get_logger().info(f'📷 Orbiting target... {orbits_completed:.1f}/{self.num_orbits} orbits')
            self._last_log_orbits = int(orbits_completed * 4)

        if orbits_completed >= self.num_orbits:
            self.get_logger().info(f'✅ Inspection complete, returning home...')
            self.mission_state = 'RETURN_HOME'
            self.state_start_time = time.time()

    def return_home_state(self):
        """Return to home position."""
        target_pos_ned = (0.0, 0.0, -self.altitude)
        self.publish_trajectory_setpoint(target_pos_ned)

        target_pos_enu = (0.0, 0.0, self.altitude)
        if self.is_at_position(target_pos_enu, tolerance=0.5):
            self.get_logger().info('✅ Returned home, landing...')
            self.mission_state = 'LAND'
            self.state_start_time = time.time()

    def land_state(self):
        """Land at home."""
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
        mission = OrbitPointMission()
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
