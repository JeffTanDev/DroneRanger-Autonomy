#!/usr/bin/env python3
import math
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy
from sensor_msgs.msg import CameraInfo, Image
from nav_msgs.msg import Odometry
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


def enu_to_ned(x_enu, y_enu, z_enu):
    """
    ENU: x=East, y=North, z=Up
    NED: x=North, y=East, z=Down
    """
    return (y_enu, x_enu, -z_enu)


class UnityHouseAvoidance(Node):
    def __init__(self):
        super().__init__('unity_house_avoidance')

        # Mission parameters (ENU)
        self.declare_parameter('altitude', 5.0)
        self.declare_parameter('start_x', 1.0)
        self.declare_parameter('start_y', 1.0)
        self.declare_parameter('forward_distance', 40.0)
        self.declare_parameter('hold_time_s', 6.0)
        self.declare_parameter('waypoint_stay_s', 5.0)  # 到达航点后原地停留时间(秒)

        # Avoidance parameters (camera-based)
        self.declare_parameter('depth_topic', '/oak/depth/image_rect_raw')
        self.declare_parameter('depth_info_topic', '/oak/depth/camera_info')
        self.declare_parameter('rgb_topic', '/oak/rgb/image_raw')
        self.declare_parameter('front_distance', 0.8)
        self.declare_parameter('roi_ratio', 0.25)
        self.declare_parameter('roi_center_x_ratio', 0.5)
        self.declare_parameter('roi_center_y_ratio', 0.4)
        self.declare_parameter('depth_percentile', 5.0)
        self.declare_parameter('min_valid_depth', 0.2)
        self.declare_parameter('max_valid_depth', 50.0)
        self.declare_parameter('depth_log_interval_s', 1.0)
        self.declare_parameter('avoid_step', 1.0)
        self.declare_parameter('avoid_max', 60.0)
        self.declare_parameter('depth_timeout_s', 1.0)
        self.declare_parameter('fail_safe_blocked', False)
        self.declare_parameter('max_step_per_cycle', 0.5)
        self.declare_parameter('avoid_log_interval_s', 1.0)

        altitude = float(self.get_parameter('altitude').value)
        start_x = float(self.get_parameter('start_x').value)
        start_y = float(self.get_parameter('start_y').value)
        forward_distance = float(self.get_parameter('forward_distance').value)
        self.hold_time_s = float(self.get_parameter('hold_time_s').value)
        self.waypoint_stay_s = float(self.get_parameter('waypoint_stay_s').value)

        waypoints_enu = [
            (start_x, start_y, altitude),  # Takeoff to start
            (start_x - 5.0, start_y, altitude),  # Left 5m (ENU -X / West)
            (start_x - 5.0, start_y + forward_distance, altitude),  # Forward to north (ENU +Y)
            (start_x, start_y, altitude),  # Return to start
        ]
        self.waypoints = [enu_to_ned(*p) for p in waypoints_enu]

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        # Subscribers (camera + position)
        depth_topic = str(self.get_parameter('depth_topic').value)
        depth_info_topic = str(self.get_parameter('depth_info_topic').value)
        rgb_topic = str(self.get_parameter('rgb_topic').value)
        self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.create_subscription(CameraInfo, depth_info_topic, self.depth_info_callback, 10)
        self.create_subscription(Image, rgb_topic, self.rgb_callback, 10)
        odom_qos = QoSProfile(
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
        )
        self.create_subscription(
            Odometry,
            '/drone/odom',
            self.odom_callback,
            odom_qos,
        )

        self.current_position = None
        self.depth_image = None
        self.depth_encoding = None
        self.depth_stamp_s = None
        self.depth_width = None
        self.depth_height = None
        self.rgb_stamp_s = None
        self.last_depth_warn_s = 0.0
        self.last_depth_log_s = 0.0
        self.last_nearest_depth = None
        self.last_avoid_log_s = 0.0
        self.last_pos_log_s = 0.0

        self.avoid_offset = 0.0
        self.blocked_hold_pos = None
        self.last_forward = (1.0, 0.0)
        self.command_pos = None
        self.preflight_setpoints_sent = 0
        self.preflight_setpoints_required = 15
        self.step = 0
        self.start_time = None
        self.waypoint_reached_at = None  # 到达当前航点的时间，用于原地停留计时

        self.timer = self.create_timer(0.1, self.timer_callback)

    def depth_callback(self, msg: Image):
        self.depth_encoding = msg.encoding
        self.depth_width = msg.width
        self.depth_height = msg.height
        self.depth_stamp_s = time.time()

        if msg.encoding == '32FC1':
            depth = np.frombuffer(msg.data, dtype=np.float32)
        elif msg.encoding == '16UC1':
            depth = np.frombuffer(msg.data, dtype=np.uint16).astype(np.float32) / 1000.0
        else:
            self.get_logger().warn(f'Unsupported depth encoding: {msg.encoding}')
            self.depth_image = None
            return

        if depth.size != msg.width * msg.height:
            self.get_logger().warn('Depth image size mismatch')
            self.depth_image = None
            return

        self.depth_image = depth.reshape((msg.height, msg.width))

    def depth_info_callback(self, msg: CameraInfo):
        self.depth_width = msg.width
        self.depth_height = msg.height

    def rgb_callback(self, msg: Image):
        self.rgb_stamp_s = time.time()

    def odom_callback(self, msg: Odometry):
        self.current_position = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            msg.pose.pose.position.z,
        )

    def timer_callback(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = timestamp
        offboard_mode.position = True
        self.offboard_control_mode_pub.publish(offboard_mode)

        if self.step == 0:
            # Send a few setpoints before switching to Offboard (PX4 requirement)
            target = self.waypoints[0]
            if self.current_position is not None:
                target = (self.current_position[0], self.current_position[1], target[2])

            traj = TrajectorySetpoint()
            traj.timestamp = timestamp
            traj.position = list(target)
            traj.yaw = 0.0
            self.trajectory_setpoint_pub.publish(traj)

            self.preflight_setpoints_sent += 1
            if self.preflight_setpoints_sent >= self.preflight_setpoints_required:
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
                self.publish_vehicle_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
                self.get_logger().info('Armed and switched to Offboard')
                self.start_time = time.time()
                self.step = 1
            return

        if 1 <= self.step <= len(self.waypoints):
            target = self.waypoints[self.step - 1]
            current_pos = self.current_position if self.current_position else target

            forward, right = self._compute_forward_right(current_pos, target)
            # Ignore depth during takeoff (first waypoint)
            if self.step == 1:
                blocked = False
            else:
                blocked = self._front_blocked()
            self._update_avoid_offset(blocked)
            self._log_avoid(blocked, self.avoid_offset)

            if blocked and current_pos is not None:
                if self.blocked_hold_pos is None:
                    hold_from = current_pos if current_pos is not None else self.command_pos
                    if hold_from is None:
                        hold_from = target
                    self.blocked_hold_pos = (hold_from[0], hold_from[1], target[2])
                target_base = self.blocked_hold_pos
                forward = self.last_forward
                # Don't advance waypoint timer while blocked
                self.start_time = time.time()
            else:
                self.blocked_hold_pos = None
                target_base = target
                self.last_forward = forward
            self._log_position(current_pos)

            target_avoid = (
                target_base[0] + right[0] * self.avoid_offset,
                target_base[1] + right[1] * self.avoid_offset,
                target_base[2],
            )

            target_avoid = self._apply_speed_limit(current_pos, target_avoid)
            self.command_pos = target_avoid

            traj = TrajectorySetpoint()
            traj.timestamp = timestamp
            traj.position = list(target_avoid)
            traj.yaw = self._compute_yaw(forward)
            self.trajectory_setpoint_pub.publish(traj)

            if time.time() - self.start_time > self.hold_time_s:
                if self.waypoint_reached_at is None:
                    self.waypoint_reached_at = time.time()
                    self.get_logger().info(
                        f"Reached waypoint {self.step}/{len(self.waypoints)} NED {target}, staying {self.waypoint_stay_s}s")
                elif time.time() - self.waypoint_reached_at >= self.waypoint_stay_s:
                    self.get_logger().info(
                        f"Waypoint {self.step} hold finished, starting next")
                    self.step += 1
                    self.start_time = time.time()
                    self.waypoint_reached_at = None
            return

        if self.step == len(self.waypoints) + 1:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info('Landing...')
            rclpy.shutdown()

    def _compute_forward_right(self, current_pos, target):
        dx = target[0] - current_pos[0]
        dy = target[1] - current_pos[1]
        norm = math.hypot(dx, dy)
        if norm < 1e-3:
            forward = (1.0, 0.0)
        else:
            forward = (dx / norm, dy / norm)
        right = (-forward[1], forward[0])
        return forward, right

    def _compute_yaw(self, forward):
        # PX4 yaw in NED: 0 = North, +pi/2 = East
        return math.atan2(forward[1], forward[0])

    def _front_blocked(self):
        if self.depth_image is None:
            return self._maybe_fail_safe('Depth not available')

        now = time.time()
        timeout = float(self.get_parameter('depth_timeout_s').value)
        if self.depth_stamp_s is None or now - self.depth_stamp_s > timeout:
            return self._maybe_fail_safe('Depth image stale')

        front_dist = float(self.get_parameter('front_distance').value)
        roi_ratio = float(self.get_parameter('roi_ratio').value)
        cx_ratio = float(self.get_parameter('roi_center_x_ratio').value)
        cy_ratio = float(self.get_parameter('roi_center_y_ratio').value)
        depth_percentile = float(self.get_parameter('depth_percentile').value)
        min_valid_depth = float(self.get_parameter('min_valid_depth').value)
        max_valid_depth = float(self.get_parameter('max_valid_depth').value)

        height, width = self.depth_image.shape
        roi_w = max(1, int(width * roi_ratio))
        roi_h = max(1, int(height * roi_ratio))
        cx = int(width * cx_ratio)
        cy = int(height * cy_ratio)
        x0 = min(max(0, cx - roi_w // 2), width - roi_w)
        y0 = min(max(0, cy - roi_h // 2), height - roi_h)
        roi = self.depth_image[y0:y0 + roi_h, x0:x0 + roi_w]

        roi = roi[np.isfinite(roi)]
        roi = roi[(roi >= min_valid_depth) & (roi <= max_valid_depth)]
        if roi.size == 0:
            return self._maybe_fail_safe('Depth ROI empty')

        nearest = float(np.percentile(roi, depth_percentile))
        self._log_depth(nearest, front_dist)
        return nearest < front_dist

    def _log_depth(self, nearest, front_dist):
        now = time.time()
        interval = float(self.get_parameter('depth_log_interval_s').value)
        if now - self.last_depth_log_s < interval:
            return
        self.last_depth_log_s = now
        self.last_nearest_depth = nearest
        self.get_logger().info(
            f'Depth nearest={nearest:.2f}m front_dist={front_dist:.2f}m')

    def _maybe_fail_safe(self, reason):
        fail_safe = bool(self.get_parameter('fail_safe_blocked').value)
        now = time.time()
        if now - self.last_depth_warn_s > 2.0:
            self.get_logger().warn(f'Obstacle check skipped: {reason}')
            self.last_depth_warn_s = now
        return fail_safe

    def _update_avoid_offset(self, blocked):
        step = float(self.get_parameter('avoid_step').value)
        max_offset = float(self.get_parameter('avoid_max').value)

        if blocked:
            self.avoid_offset = min(max_offset, self.avoid_offset + step)
            return

        if self.avoid_offset > 0.0:
            self.avoid_offset = max(0.0, self.avoid_offset - step)

    def _log_avoid(self, blocked, offset):
        now = time.time()
        interval = float(self.get_parameter('avoid_log_interval_s').value)
        if now - self.last_avoid_log_s < interval:
            return
        self.last_avoid_log_s = now
        self.get_logger().info(f'Avoid blocked={blocked} offset={offset:.2f}m')

    def _log_position(self, current_pos):
        if current_pos is None:
            return
        now = time.time()
        if now - self.last_pos_log_s < 1.0:
            return
        self.last_pos_log_s = now
        self.get_logger().info(
            f'Position NED x={current_pos[0]:.2f} y={current_pos[1]:.2f} z={current_pos[2]:.2f}')


    def _apply_speed_limit(self, current_pos, target):
        max_step = float(self.get_parameter('max_step_per_cycle').value)
        if max_step <= 0.0:
            return target

        base_pos = self.command_pos if self.command_pos is not None else current_pos
        if base_pos is None:
            return target

        dx = target[0] - base_pos[0]
        dy = target[1] - base_pos[1]
        dz = target[2] - base_pos[2]
        dist = math.sqrt(dx * dx + dy * dy + dz * dz)
        if dist <= max_step:
            return target

        scale = max_step / dist
        return (
            base_pos[0] + dx * scale,
            base_pos[1] + dy * scale,
            base_pos[2] + dz * scale,
        )

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
    node = UnityHouseAvoidance()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
