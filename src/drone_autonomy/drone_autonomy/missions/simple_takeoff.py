"""
Simple takeoff mission - basic example of drone operation.

This mission demonstrates:
1. Arming the drone
2. Switching to offboard mode
3. Taking off to a specified altitude
4. Hovering for a specified time
5. Landing safely
"""

import rclpy
import time
from rclpy.parameter import Parameter

from ..core.drone_controller import DroneController


class SimpleTakeoffMission(DroneController):
    """Simple takeoff and hover mission."""

    def __init__(self):
        super().__init__('simple_takeoff_mission')

        # Declare parameters
        self.declare_parameter('takeoff_altitude', 3.0)
        self.declare_parameter('hover_duration', 10.0)
        self.declare_parameter('safety_timeout', 30.0)

        # Get parameters
        self.takeoff_altitude = self.get_parameter('takeoff_altitude').get_parameter_value().double_value
        self.hover_duration = self.get_parameter('hover_duration').get_parameter_value().double_value
        self.safety_timeout = self.get_parameter('safety_timeout').get_parameter_value().double_value

        # Mission state machine
        self.mission_state = 'INIT'
        self.state_start_time = None
        self.setpoint_counter = 0

        self.get_logger().info(f'Simple Takeoff Mission initialized')
        self.get_logger().info(f'Target altitude: {self.takeoff_altitude}m, Hover duration: {self.hover_duration}s')

    def control_loop_callback(self):
        """State machine for takeoff mission."""
        # Always publish offboard control mode
        self.publish_offboard_control_mode()

        current_time = time.time()

        if self.mission_state == 'INIT':
            self.init_state()

        elif self.mission_state == 'PRE_ARM':
            self.pre_arm_state()

        elif self.mission_state == 'ARM_AND_OFFBOARD':
            self.arm_and_offboard_state()

        elif self.mission_state == 'TAKEOFF':
            self.takeoff_state()

        elif self.mission_state == 'HOVER':
            self.hover_state()

        elif self.mission_state == 'LAND':
            self.land_state()

        elif self.mission_state == 'COMPLETE':
            self.complete_state()

        # Safety timeout check
        if (self.state_start_time and
            (current_time - self.state_start_time) > self.safety_timeout):
            self.get_logger().error(f'Safety timeout in state: {self.mission_state}')
            self.emergency_land()

    def init_state(self):
        """Initialize mission."""
        self.get_logger().info('Mission started - Initializing...')
        self.mission_state = 'PRE_ARM'
        self.state_start_time = time.time()

    def pre_arm_state(self):
        """Send setpoints before arming (PX4 requirement)."""
        # Send position setpoints at current position for a few cycles
        target_pos_ned = (0.0, 0.0, -self.takeoff_altitude)
        self.publish_trajectory_setpoint(target_pos_ned)

        self.setpoint_counter += 1

        # After 10 setpoints, proceed to arm and switch to offboard
        if self.setpoint_counter >= 10:
            self.mission_state = 'ARM_AND_OFFBOARD'
            self.state_start_time = time.time()
            self.get_logger().info('Pre-arm phase complete, proceeding to arm...')

    def arm_and_offboard_state(self):
        """Arm drone and switch to offboard mode."""
        if not self.is_armed:
            self.set_offboard_mode()
            self.arm()

        # Continue sending setpoints
        target_pos_ned = (0.0, 0.0, -self.takeoff_altitude)
        self.publish_trajectory_setpoint(target_pos_ned)

        # Wait for successful arming
        if self.is_armed:
            self.get_logger().info('✅ Armed and in offboard mode, starting takeoff...')
            self.mission_state = 'TAKEOFF'
            self.state_start_time = time.time()

    def takeoff_state(self):
        """Execute takeoff to target altitude."""
        target_pos_ned = (0.0, 0.0, -self.takeoff_altitude)
        self.publish_trajectory_setpoint(target_pos_ned)

        # Check if we've reached target altitude
        target_pos_enu = (0.0, 0.0, self.takeoff_altitude)
        if self.is_at_position(target_pos_enu, tolerance=0.3):
            self.get_logger().info(f'✅ Reached target altitude: {self.takeoff_altitude}m')
            self.mission_state = 'HOVER'
            self.state_start_time = time.time()

    def hover_state(self):
        """Hover at target altitude for specified duration."""
        target_pos_ned = (0.0, 0.0, -self.takeoff_altitude)
        self.publish_trajectory_setpoint(target_pos_ned)

        elapsed_time = time.time() - self.state_start_time
        remaining_time = self.hover_duration - elapsed_time

        if remaining_time <= 0:
            self.get_logger().info(f'✅ Hover complete ({self.hover_duration}s), initiating landing...')
            self.mission_state = 'LAND'
            self.state_start_time = time.time()
        else:
            # Log remaining hover time every 2 seconds
            if int(elapsed_time) % 2 == 0 and int(elapsed_time) != getattr(self, '_last_log_time', -1):
                self.get_logger().info(f'🚁 Hovering... {remaining_time:.1f}s remaining')
                self._last_log_time = int(elapsed_time)

    def land_state(self):
        """Execute landing."""
        self.land()
        self.get_logger().info('🛬 Landing command sent')
        self.mission_state = 'COMPLETE'
        self.state_start_time = time.time()

    def complete_state(self):
        """Mission complete."""
        # Wait a bit then shutdown
        if time.time() - self.state_start_time > 3.0:
            self.get_logger().info('✅ Mission completed successfully!')
            # Gracefully shutdown ROS
            self.create_timer(1.0, lambda: rclpy.shutdown())

    def emergency_land(self):
        """Emergency landing procedure."""
        self.get_logger().error('⚠️ Emergency landing initiated!')
        self.land()
        self.mission_state = 'COMPLETE'
        self.state_start_time = time.time()


def main(args=None):
    """Main function."""
    rclpy.init(args=args)

    try:
        mission = SimpleTakeoffMission()
        rclpy.spin(mission)
    except KeyboardInterrupt:
        print('Mission interrupted by user')
    finally:
        try:
            mission.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()