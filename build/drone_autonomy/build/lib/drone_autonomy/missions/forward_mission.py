"""
Forward 10m mission - takeoff -> fly forward ~10m -> hover

Assumes DroneController provides:
- publish_offboard_control_mode()
- publish_trajectory_setpoint((n, e, d))
- set_offboard_mode(), arm(), land()
- is_armed (bool)
- is_at_position((x_enu, y_enu, z_enu), tolerance=...)
"""

import rclpy
import time
from ..core.drone_controller import DroneController


class Forward10mHoverMission(DroneController):
    def __init__(self):
        super().__init__('forward_10m_hover_mission')

        # params
        self.declare_parameter('altitude', 5.0)     # meters (up)
        self.declare_parameter('forward_m', 10.0)   # meters (north)
        self.declare_parameter('warmup_setpoints', 10)
        self.declare_parameter('pos_tol', 0.6)

        self.altitude = float(self.get_parameter('altitude').value)
        self.forward_m = float(self.get_parameter('forward_m').value)
        self.warmup_setpoints = int(self.get_parameter('warmup_setpoints').value)
        self.pos_tol = float(self.get_parameter('pos_tol').value)

        # mission state
        self.state = 'INIT'
        self.state_t0 = time.time()
        self.sp_count = 0

        # store start point in NED (set when we first get valid position)
        self.start_n = None
        self.start_e = None

        self.get_logger().info(
            f'Forward10mHoverMission ready: altitude={self.altitude}m forward={self.forward_m}m'
        )

    def control_loop_callback(self):
        # offboard heartbeat every tick
        self.publish_offboard_control_mode()

        if self.state == 'INIT':
            self.state = 'WARMUP'
            self.state_t0 = time.time()
            self.get_logger().info('🚁 INIT -> WARMUP')
            return

        if self.state == 'WARMUP':
            # send a few setpoints before arming/offboard (PX4 requirement)
            target_ned = (0.0, 0.0, -self.altitude)
            self.publish_trajectory_setpoint(target_ned)
            self.sp_count += 1
            if self.sp_count >= self.warmup_setpoints:
                self.state = 'ARM_AND_OFFBOARD'
                self.state_t0 = time.time()
                self.get_logger().info('✅ WARMUP -> ARM_AND_OFFBOARD')
            return

        if self.state == 'ARM_AND_OFFBOARD':
            if not self.is_armed:
                self.set_offboard_mode()
                self.arm()

            # keep holding takeoff point
            target_ned = (0.0, 0.0, -self.altitude)
            self.publish_trajectory_setpoint(target_ned)

            if self.is_armed:
                self.state = 'TAKEOFF'
                self.state_t0 = time.time()
                self.get_logger().info('✅ Armed -> TAKEOFF')
            return

        if self.state == 'TAKEOFF':
            # hold at (0,0,-alt)
            target_ned = (0.0, 0.0, -self.altitude)
            self.publish_trajectory_setpoint(target_ned)

            # check position in ENU
            target_enu = (0.0, 0.0, self.altitude)
            if self.is_at_position(target_enu, tolerance=self.pos_tol):
                # record start point as "origin" for forward move
                # If your DroneController exposes local position, replace below with actual values.
                # Otherwise we assume (0,0) is the takeoff origin.
                self.start_n, self.start_e = 0.0, 0.0

                self.state = 'FORWARD'
                self.state_t0 = time.time()
                self.get_logger().info('✅ TAKEOFF complete -> FORWARD')
            return

        if self.state == 'FORWARD':
            # fly forward along +North by forward_m
            goal_n = self.start_n + self.forward_m
            goal_e = self.start_e
            goal_d = -self.altitude
            target_ned = (goal_n, goal_e, goal_d)
            self.publish_trajectory_setpoint(target_ned)

            # NED -> ENU for check: (E, N, -D)
            target_enu = (goal_e, goal_n, -goal_d)

            if self.is_at_position(target_enu, tolerance=self.pos_tol):
                self.state = 'HOVER'
                self.state_t0 = time.time()
                self.get_logger().info('✅ Reached forward point -> HOVER')
            return

        if self.state == 'HOVER':
            # keep publishing the same setpoint to hold position
            goal_n = self.start_n + self.forward_m
            goal_e = self.start_e
            goal_d = -self.altitude
            self.publish_trajectory_setpoint((goal_n, goal_e, goal_d))
            # stay here forever (or add a param to land after X seconds)
            return


def main(args=None):
    rclpy.init(args=args)
    try:
        node = Forward10mHoverMission()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
