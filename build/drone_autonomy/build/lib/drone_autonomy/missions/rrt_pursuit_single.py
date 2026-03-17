
#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Pursuit with Depth-Based Obstacle Hold — stable takeoff + GPS target + depth-based avoidance

Key points
- Publishes OffboardControlMode + TrajectorySetpoint every cycle with timestamps
- Uses a conservative takeoff state machine before entering pursuit
- Converts GPS target (lat, lon, alt) -> local NED using a fixed home GPS
- Computes yaw consistently in NED: yaw = atan2(vE, vN)
- Uses depth-based front check to hold position when blocked

PX4 topics (uXRCE-DDS, ROS 2):
  pub: /fmu/in/offboard_control_mode     (px4_msgs/OffboardControlMode)
  pub: /fmu/in/trajectory_setpoint       (px4_msgs/TrajectorySetpoint)
  pub: /fmu/in/vehicle_command           (px4_msgs/VehicleCommand)
  sub: /fmu/out/vehicle_local_position_v1 (px4_msgs/VehicleLocalPosition)
  sub: /drone/gps                        (sensor_msgs/NavSatFix)           -- current GPS (used to set home once)
  sub: /static_cam/target_gps            (sensor_msgs/NavSatFix)            -- GPS target
  sub: /oak/depth/image_rect_raw         (sensor_msgs/Image)                -- depth for avoidance
  sub: /oak/depth/camera_info            (sensor_msgs/CameraInfo)

Author: Modified for depth-based avoidance
"""

import math
import time
from enum import Enum

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CameraInfo, Image, NavSatFix
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)

# ---------------------------- Utilities ----------------------------

def radians_wrap(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle

def gps_to_ned(lat_deg: float, lon_deg: float, alt_m: float,
               home_lat_deg: float, home_lon_deg: float, home_alt_m: float):
    """Convert GPS (lat/lon/alt) to local NED using a home reference.

    NOTE: North sign is flipped to match Unity Z-forward = South convention.
    """
    earth_radius_m = 6378137.0
    lat_rad = math.radians(lat_deg)
    home_lat_rad = math.radians(home_lat_deg)
    d_lat = lat_rad - home_lat_rad
    d_lon = math.radians(lon_deg - home_lon_deg)

    north = -d_lat * earth_radius_m
    east = d_lon * earth_radius_m * math.cos(home_lat_rad)
    down = -(alt_m - home_alt_m)
    return north, east, down

def micros(node: Node) -> int:
    """Current time in microseconds for PX4 timestamps."""
    return node.get_clock().now().nanoseconds // 1000

# ---------------------------- State Machine ----------------------------

class Stage(Enum):
    WARMUP = 0
    ARMING = 1
    TAKEOFF = 2
    HOVER_STABLE = 3
    PURSUIT = 4

# ---------------------------- Main Node ----------------------------

class RRTPursuitFixed(Node):
    def __init__(self):
        super().__init__('rrt_pursuit_fixed')

        # ---------- Parameters ----------
        self.declare_parameter('hz', 30.0)
        self.declare_parameter('takeoff_alt_m', 5.0)       # "up" in meters; will convert to zD=-takeoff_alt_m
        self.declare_parameter('min_nav_alt_m', 2.0)       # must be above this to start pursuit
        self.declare_parameter('stable_hover_s', 1.0)      # time to hold after takeoff reaches altitude
        self.declare_parameter('target_gps_topic', '/static_cam/target_gps')
        self.declare_parameter('step_size_m', 4.0)                # step size in meters for pursuit
        # Camera-based avoidance parameters (from unity_house_avoidance.py)
        self.declare_parameter('depth_topic', '/oak/depth/image_rect_raw')
        self.declare_parameter('depth_info_topic', '/oak/depth/camera_info')
        self.declare_parameter('front_distance', 0.8)
        self.declare_parameter('roi_ratio', 0.25)
        self.declare_parameter('roi_center_x_ratio', 0.5)
        self.declare_parameter('roi_center_y_ratio', 0.4)
        self.declare_parameter('depth_percentile', 5.0)
        self.declare_parameter('min_valid_depth', 0.2)
        self.declare_parameter('max_valid_depth', 50.0)
        self.declare_parameter('depth_log_interval_s', 1.0)
        self.declare_parameter('depth_timeout_s', 1.0)
        self.declare_parameter('fail_safe_blocked', False)
        self.declare_parameter('status_log_interval_s', 1.0)
        self.declare_parameter('avoid_stop_s', 2.0)
        self.declare_parameter('avoid_yaw_step_deg', 10.0)
        self.declare_parameter('avoid_rotate_hz', 2.0)
        self.declare_parameter('avoid_forward_m', 2.0)
        self.declare_parameter('avoid_clear_extra_deg', 10.0)
        self.declare_parameter('alt_align_tolerance_m', 0.3)
        self.declare_parameter('yaw_align_threshold_deg', 8.0)

        hz = float(self.get_parameter('hz').value)
        self.dt = 1.0 / max(hz, 1.0)

        # ---------- Runtime Vars ----------
        self.stage = Stage.WARMUP
        self.stage_enter_ts = self.get_clock().now().nanoseconds
        self.armed = False
        self.offboard_ok = False

        # local position validity
        self.xy_valid = False
        self.z_valid  = False

        # latest local position (NED/FRD PX4 frame)
        self.px = 0.0  # north
        self.py = 0.0  # east
        self.pz = 0.0  # down (positive is downward)
        self.heading = 0.0  # yaw in NED

        # target in GPS
        self.tgt_lat = None
        self.tgt_lon = None
        self.tgt_alt = None

        # current vehicle GPS
        self.cur_lat = None
        self.cur_lon = None
        self.cur_alt = None
        self.cur_fix_type = None

        # fixed home GPS reference
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None

        # no exploration state (obstacle => hold position)

        # depth-based obstacle detection state
        self.depth_image = None
        self.depth_encoding = None
        self.depth_stamp_s = None
        self.depth_width = None
        self.depth_height = None
        self.last_depth_warn_s = 0.0
        self.last_depth_log_s = 0.0
        self.last_nearest_depth = None
        self.last_status_log_s = 0.0

        # simple avoid state (blocked -> stop -> rotate right until clear -> move -> resume)
        self.avoid_state = None  # None | 'STOP' | 'ROTATE_RIGHT' | 'MOVE_RIGHT'
        self.avoid_heading = None
        self.avoid_stop_start_s = None
        self.avoid_move_start_n = None
        self.avoid_move_start_e = None
        self.avoid_last_rotate_s = None

        # pursue alignment state
        self.aligning_to_target = False
        self.aligning_altitude = False
        self.align_last_rotate_s = None

        # ---------- Publishers ----------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        self.pub_offboard = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_sp       = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_cmd      = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)

        # ---------- Subscribers ----------
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.on_local_pos,
            qos,
        )
        self.create_subscription(
            NavSatFix,
            '/drone/gps',
            self.on_gps_pos,
            qos,
        )
        target_gps_topic = str(self.get_parameter('target_gps_topic').value)
        self.create_subscription(NavSatFix, target_gps_topic, self.on_target_gps, 10)

        depth_topic = str(self.get_parameter('depth_topic').value)
        depth_info_topic = str(self.get_parameter('depth_info_topic').value)
        self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.create_subscription(CameraInfo, depth_info_topic, self.depth_info_callback, 10)

        # ---------- Timer Loop ----------
        self.timer = self.create_timer(self.dt, self.loop)

        # log banner
        self.get_logger().info(
            f"[node] up | hz={hz:.1f} | takeoff={self.get_parameter('takeoff_alt_m').value:.2f}m "
            f"| min_nav_alt={self.get_parameter('min_nav_alt_m').value:.2f}m "
            f"| step={self.get_parameter('step_size_m').value:.2f}m (safe: 2.5m) "
            f"| target_gps={target_gps_topic} | depth={depth_topic} "
        )

    # ---------------- Callbacks ----------------

    def on_local_pos(self, msg: VehicleLocalPosition):
        self.xy_valid = bool(msg.xy_valid)
        self.z_valid  = bool(msg.z_valid)
        self.px = msg.x  # North (m)
        self.py = msg.y  # East  (m)
        self.pz = msg.z  # Down  (m, positive is down)
        self.heading = msg.heading  # rad, NED frame

    def on_gps_pos(self, msg: NavSatFix):
        # NavSatFix: lat/lon in degrees, altitude in meters.
        self.cur_lat = float(msg.latitude)
        self.cur_lon = float(msg.longitude)
        self.cur_alt = float(msg.altitude)
        self.cur_fix_type = int(msg.status.status)

        if self.home_lat is None:
            self.home_lat = self.cur_lat
            self.home_lon = self.cur_lon
            self.home_alt = self.cur_alt

    def on_target_gps(self, msg: NavSatFix):
        self.tgt_lat = float(msg.latitude)
        self.tgt_lon = float(msg.longitude)
        self.tgt_alt = float(msg.altitude)

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

    # ---------------- PX4 Helpers ----------------

    def publish_offboard_mode(self):
        m = OffboardControlMode()
        m.timestamp = micros(self)
        # We control position in NED (x,y,z), not using velocity/accel/body-rate outputs here
        m.position = True
        m.velocity = False
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        self.pub_offboard.publish(m)

    def publish_sp(self, xN=None, yE=None, zD=None, yaw=None):
        sp = TrajectorySetpoint()
        sp.timestamp = micros(self)

        # Keep last setpoint components if None
        # For simplicity, always set all components explicitly here.
        sp.position = [float(xN if xN is not None else self.px),
                       float(yE if yE is not None else self.py),
                       float(zD if zD is not None else self.pz)]
        sp.yaw = float(yaw if yaw is not None else self.heading)

        self.pub_sp.publish(sp)

    def cmd(self, command: int, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = micros(self)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.pub_cmd.publish(msg)

    def arm(self):
        self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0, 0.0)
        self.armed = True

    def disarm(self):
        self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 0.0, 0.0)
        self.armed = False

    def set_offboard(self):
        self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)  # PX4 custom: base=1 (custom), sub=6 (OFFBOARD)
        self.offboard_ok = True

    # ---------------- Depth-Based Avoidance Helpers ----------------

    def _front_blocked(self) -> bool:
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

    def _handle_simple_avoid(self, zD_goal: float) -> bool:
        """Handle simple avoidance. Returns True if handled (published)."""
        if self.avoid_state is None:
            return False

        now = time.time()
        blocked = self._front_blocked()

        if self.avoid_state == 'STOP':
            stop_s = float(self.get_parameter('avoid_stop_s').value)
            self.publish_sp(self.px, self.py, zD_goal, self.heading)
            if now - self.avoid_stop_start_s >= stop_s:
                self.avoid_state = 'ROTATE_RIGHT'
                self.avoid_heading = self.heading
                self.avoid_last_rotate_s = now
            return True

        if self.avoid_state == 'ROTATE_RIGHT':
            # Rotate right until front is clear
            if blocked:
                rotate_hz = float(self.get_parameter('avoid_rotate_hz').value)
                rotate_interval = 1.0 / max(rotate_hz, 0.1)
                if self.avoid_last_rotate_s is None or (now - self.avoid_last_rotate_s) >= rotate_interval:
                    step_deg = float(self.get_parameter('avoid_yaw_step_deg').value)
                    self.avoid_heading = radians_wrap(self.avoid_heading + math.radians(step_deg))
                    self.avoid_last_rotate_s = now
                self.publish_sp(self.px, self.py, zD_goal, self.avoid_heading)
                return True

            # Clear: move forward in the new heading
            self.avoid_state = 'MOVE_RIGHT'
            extra_deg = float(self.get_parameter('avoid_clear_extra_deg').value)
            self.avoid_heading = radians_wrap(self.avoid_heading + math.radians(extra_deg))
            self.avoid_move_start_n = self.px
            self.avoid_move_start_e = self.py
            return True

        if self.avoid_state == 'MOVE_RIGHT':
            # If blocked again during move, restart avoid logic
            if blocked:
                self.avoid_state = 'STOP'
                self.avoid_stop_start_s = now
                self.publish_sp(self.px, self.py, zD_goal, self.heading)
                return True

            move_m = float(self.get_parameter('avoid_forward_m').value)
            dirN = math.cos(self.avoid_heading)
            dirE = math.sin(self.avoid_heading)
            target_n = self.avoid_move_start_n + dirN * move_m
            target_e = self.avoid_move_start_e + dirE * move_m
            self.publish_sp(target_n, target_e, zD_goal, self.avoid_heading)

            moved = math.hypot(self.px - self.avoid_move_start_n, self.py - self.avoid_move_start_e)
            if moved >= move_m:
                self.avoid_state = None
                self.avoid_heading = None
                self.avoid_stop_start_s = None
                self.avoid_move_start_n = None
                self.avoid_move_start_e = None
                self.avoid_last_rotate_s = None
                # Re-enter pursuit alignment before continuing
                self.aligning_to_target = True
                self.aligning_altitude = False
                self.align_last_rotate_s = None
            return True

        return False

    def _log_status(self, stage, goal_ned=None, blocked=None, zD_goal=None):
        now = time.time()
        interval = float(self.get_parameter('status_log_interval_s').value)
        if now - self.last_status_log_s < interval:
            return
        self.last_status_log_s = now

        cur_pos = f"drone cur NED=({self.px:.2f},{self.py:.2f},{self.pz:.2f})"
        cur = "drone GPS=unset"
        if self.cur_lat is not None and self.cur_lon is not None and self.cur_alt is not None:
            cur = f"drone GPS=({self.cur_lat:.7f},{self.cur_lon:.7f},{self.cur_alt:.1f})"
        home = "original GPS=unset"
        if self.home_lat is not None and self.home_lon is not None and self.home_alt is not None:
            home = f"original GPS=({self.home_lat:.7f},{self.home_lon:.7f},{self.home_alt:.1f})"
        tgt = "target GPS=unset"
        if self.tgt_lat is not None and self.tgt_lon is not None:
            alt = self.tgt_alt if self.tgt_alt is not None else float('nan')
            tgt = f"target GPS=({self.tgt_lat:.7f},{self.tgt_lon:.7f},{alt:.1f})"
        goal = "goal=unset"
        if goal_ned is not None:
            goal = f"goalNED=({goal_ned[0]:.2f},{goal_ned[1]:.2f},{goal_ned[2]:.2f})"
        avoid = "blocked=unknown"
        if blocked is not None:
            avoid = f"blocked={blocked}"
        depth = "depth=none"
        if self.last_nearest_depth is not None:
            depth = f"depth={self.last_nearest_depth:.2f}m"
        zgoal = "zD_goal=unset"
        if zD_goal is not None:
            zgoal = f"zD_goal={zD_goal:.2f}m"

        self.get_logger().info(
            f"[status] stage={stage} {cur_pos} {cur} {home} {tgt} {goal} {zgoal} {avoid} {depth}"
        )

    # ---------------- Planning Helpers ----------------

    def desired_yaw_towards(self, xN: float, yE: float, goalN: float, goalE: float) -> float:
        """Yaw in NED to face (goal - current). yaw = atan2(vE, vN)."""
        vN = goalN - xN
        vE = goalE - yE
        return radians_wrap(math.atan2(vE, vN))

    def rrt_next_step(self, curN: float, curE: float, goalN: float, goalE: float, step: float = 1.0):
        """Very small 'RRT-like' step: go straight toward the goal limited by 'step'."""
        dN = goalN - curN
        dE = goalE - curE
        dist = math.hypot(dN, dE)
        if dist < 1e-3:
            return goalN, goalE
        scale = min(1.0, step / dist)
        return curN + dN * scale, curE + dE * scale

    # ---------------- Main Loop ----------------

    def loop(self):
        # Heartbeat messages every cycle
        self.publish_offboard_mode()

        t_now_ns = self.get_clock().now().nanoseconds

        # Do not run avoidance before PURSUIT
        if self.stage != Stage.PURSUIT and self.avoid_state is not None:
            self.avoid_state = None
            self.avoid_heading = None
            self.avoid_stop_start_s = None
            self.avoid_move_start_n = None
            self.avoid_move_start_e = None
            self.avoid_last_rotate_s = None

        # State machine
        if self.stage == Stage.WARMUP:
            # continuously publish a hold setpoint at current (px,py), target zD = -takeoff_alt
            takeoff_alt_m = float(self.get_parameter('takeoff_alt_m').value)
            zD_goal = -abs(takeoff_alt_m)  # down is positive, so up = negative
            yaw = self.heading
            self.publish_sp(self.px, self.py, zD_goal, yaw)
            self._log_status("WARMUP", zD_goal=zD_goal)

            # After >= 0.5s of warmup, arm + offboard
            if (t_now_ns - self.stage_enter_ts) * 1e-9 >= 0.5:
                self.arm()
                self.set_offboard()
                self.stage = Stage.ARMING
                self.stage_enter_ts = t_now_ns
                self.get_logger().info("[stage] WARMUP -> ARMING")
                return

        elif self.stage == Stage.ARMING:
            # keep sending the same takeoff setpoint until we see z_valid and begin moving
            takeoff_alt_m = float(self.get_parameter('takeoff_alt_m').value)
            zD_goal = -abs(takeoff_alt_m)
            self.publish_sp(self.px, self.py, zD_goal, self.heading)
            self._log_status("ARMING", zD_goal=zD_goal)

            # move to TAKEOFF once z_valid (height feedback) is available
            if self.z_valid:
                self.stage = Stage.TAKEOFF
                self.stage_enter_ts = t_now_ns
                self.get_logger().info("[stage] ARMING -> TAKEOFF")
                return

        elif self.stage == Stage.TAKEOFF:
            # keep commanding the takeoff altitude until reached (within tolerance)
            takeoff_alt_m = float(self.get_parameter('takeoff_alt_m').value)
            zD_goal = -abs(takeoff_alt_m)
            self.publish_sp(self.px, self.py, zD_goal, self.heading)
            self._log_status("TAKEOFF", zD_goal=zD_goal)

            alt_err = abs(self.pz - zD_goal)  # both in Down
            if alt_err <= 0.25:  # within 25cm
                self.stage = Stage.HOVER_STABLE
                self.stage_enter_ts = t_now_ns
                self.get_logger().info("[stage] TAKEOFF -> HOVER_STABLE")
                return

        elif self.stage == Stage.HOVER_STABLE:
            # hold altitude for some time to stabilize, wait for valid xy too
            takeoff_alt_m = float(self.get_parameter('takeoff_alt_m').value)
            zD_goal = -abs(takeoff_alt_m)
            self.publish_sp(self.px, self.py, zD_goal, self.heading)
            self._log_status("HOVER_STABLE", zD_goal=zD_goal)

            stable_hover_s = float(self.get_parameter('stable_hover_s').value)
            min_nav_alt_m   = float(self.get_parameter('min_nav_alt_m').value)
            if (t_now_ns - self.stage_enter_ts) * 1e-9 >= stable_hover_s and self.xy_valid and self.z_valid:
                # ensure we're above min_nav_alt_m (i.e., |zD| >= min_nav_alt_m)
                if abs(self.pz) >= min_nav_alt_m:
                    self.stage = Stage.PURSUIT
                    self.stage_enter_ts = t_now_ns
                    self.aligning_to_target = True
                    self.aligning_altitude = False
                    self.align_last_rotate_s = None
                    self.get_logger().info("[stage] HOVER_STABLE -> PURSUIT")
                    return

        elif self.stage == Stage.PURSUIT:
            # Convert target GPS to local NED goal (if target + current GPS exist), else hold
            takeoff_alt_m = float(self.get_parameter('takeoff_alt_m').value)

            if self.home_lat is None or self.home_lon is None or self.home_alt is None:
                # No home reference yet; hold position
                zD_goal = -abs(takeoff_alt_m)
                self.publish_sp(self.px, self.py, zD_goal, self.heading)
                self._log_status("PURSUIT", zD_goal=zD_goal)
                return

            if (self.tgt_lat is None or self.tgt_lon is None or
                    not math.isfinite(self.tgt_lat) or not math.isfinite(self.tgt_lon)):
                # No target yet; hold position at current x/y and altitude
                zD_goal = -abs(takeoff_alt_m)
                self.publish_sp(self.px, self.py, zD_goal, self.heading)
                self._log_status("PURSUIT", zD_goal=zD_goal)
                return

            tgt_alt = self.tgt_alt if (self.tgt_alt is not None and math.isfinite(self.tgt_alt)) else self.home_alt
            goalN, goalE, goalD = gps_to_ned(
                self.tgt_lat,
                self.tgt_lon,
                tgt_alt,
                self.home_lat,
                self.home_lon,
                self.home_alt,
            )

            # Use target altitude (fallback to takeoff altitude)
            zD_goal = goalD if math.isfinite(goalD) else -abs(takeoff_alt_m)

            # First, align yaw to face target before moving
            if self.aligning_to_target:
                desired_yaw = self.desired_yaw_towards(self.px, self.py, goalN, goalE)
                yaw_err = radians_wrap(desired_yaw - self.heading)
                yaw_thresh = math.radians(float(self.get_parameter('yaw_align_threshold_deg').value))
                rotate_hz = float(self.get_parameter('avoid_rotate_hz').value)
                rotate_interval = 1.0 / max(rotate_hz, 0.1)
                if self.align_last_rotate_s is None or (time.time() - self.align_last_rotate_s) >= rotate_interval:
                    step_deg = float(self.get_parameter('avoid_yaw_step_deg').value)
                    step_rad = math.radians(step_deg)
                    if yaw_err > 0:
                        desired_step = min(step_rad, yaw_err)
                    else:
                        desired_step = max(-step_rad, yaw_err)
                    self.align_last_rotate_s = time.time()
                    desired_yaw = radians_wrap(self.heading + desired_step)
                # Hold position and current altitude while rotating
                self.publish_sp(self.px, self.py, self.pz, desired_yaw)
                if abs(yaw_err) <= yaw_thresh:
                    self.aligning_to_target = False
                    self.aligning_altitude = True
                    self.align_last_rotate_s = None
                self._log_status("PURSUIT", goal_ned=(goalN, goalE, zD_goal), blocked=None, zD_goal=zD_goal)
                return

            # Then, align altitude before moving
            if self.aligning_altitude:
                alt_tol = float(self.get_parameter('alt_align_tolerance_m').value)
                self.publish_sp(self.px, self.py, zD_goal, self.heading)
                if abs(self.pz - zD_goal) <= alt_tol:
                    self.aligning_altitude = False
                self._log_status("PURSUIT", goal_ned=(goalN, goalE, zD_goal), blocked=None, zD_goal=zD_goal)
                return

            # Use configurable step toward goal
            step_xy = float(self.get_parameter('step_size_m').value)

            # If in avoid sequence, handle it and return
            if self._handle_simple_avoid(zD_goal):
                self._log_status("PURSUIT", goal_ned=(goalN, goalE, zD_goal), blocked=None, zD_goal=zD_goal)
                return

            blocked = self._front_blocked()

            if blocked:
                # Start simple avoid: stop, then rotate right until clear
                self.avoid_state = 'STOP'
                self.avoid_stop_start_s = time.time()
                self.publish_sp(self.px, self.py, zD_goal, self.heading)
                self._log_status("PURSUIT", goal_ned=(goalN, goalE, zD_goal), blocked=blocked, zD_goal=zD_goal)
                return

            # Direct pursuit
            nxtN, nxtE = self.rrt_next_step(self.px, self.py, goalN, goalE, step=step_xy)
            yaw = self.desired_yaw_towards(self.px, self.py, goalN, goalE)

            # Publish the setpoint at target altitude
            self.publish_sp(nxtN, nxtE, zD_goal, yaw)
            self._log_status("PURSUIT", goal_ned=(goalN, goalE, zD_goal), blocked=blocked, zD_goal=zD_goal)

        else:
            # Failsafe: publish hold
            self.publish_sp(self.px, self.py, self.pz, self.heading)

def main():
    rclpy.init()
    node = RRTPursuitFixed()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
