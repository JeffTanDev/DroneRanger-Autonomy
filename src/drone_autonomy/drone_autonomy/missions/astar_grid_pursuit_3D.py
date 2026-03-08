#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
3D grid A* pursuit: depth -> local 3D occupancy grid (NED) -> 3D A* path -> setpoints.

- Grid: local 3D voxel grid around drone (configurable size and resolution in N/E/D).
- Depth: full image (upper + lower half) used to mark obstacles in 3D grid.
- All coordinates in NED; path is (n, e, d) waypoints.

Topics:
  sub: /fmu/out/vehicle_local_position_v1  (px4_msgs/VehicleLocalPosition) - NED position & heading
  sub: /fmu/out/vehicle_attitude            (px4_msgs/VehicleAttitude, optional) - roll, pitch, yaw
  sub: /drone/gps                           (sensor_msgs/NavSatFix) - used to set home once
  sub: /static_cam/target_gps                (sensor_msgs/NavSatFix) - target in NED via gps_to_ned
  sub: /oak/depth/image_rect_raw             (sensor_msgs/Image)
  sub: /oak/depth/camera_info                (sensor_msgs/CameraInfo)
  pub: /fmu/in/offboard_control_mode, /fmu/in/trajectory_setpoint, /fmu/in/vehicle_command
  pub: /planning/path (geometry_msgs/PoseArray) - NED path for Unity visualization
"""

import math
import os
import time
from enum import Enum
import heapq

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import CameraInfo, Image, NavSatFix
from geometry_msgs.msg import Point, Pose, PoseArray, Quaternion
from std_msgs.msg import Header
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)
try:
    from px4_msgs.msg import VehicleAttitude
    _HAS_VEHICLE_ATTITUDE = True
except ImportError:
    _HAS_VEHICLE_ATTITUDE = False

# ---------------------------- Utilities ----------------------------

def radians_wrap(angle: float) -> float:
    """Wrap angle to [-pi, pi]."""
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def rotation_body_to_ned(roll: float, pitch: float, yaw: float) -> np.ndarray:
    """
    Rotation matrix R_wb from body (FRD) to NED: v_ned = R_wb @ v_body.
    R_wb = Rz(yaw) @ Ry(pitch) @ Rx(roll). Angles in radians.
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]], dtype=np.float64)
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]], dtype=np.float64)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]], dtype=np.float64)
    return Rz @ Ry @ Rx


def gps_to_ned(lat_deg: float, lon_deg: float, alt_m: float,
               home_lat_deg: float, home_lon_deg: float, home_alt_m: float):
    """Convert GPS (lat/lon/alt) to local NED using a home reference."""
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
    return node.get_clock().now().nanoseconds // 1000


# ---------------------------- 3D Grid ----------------------------

def world_to_grid_3d(n: float, e: float, d: float,
                    center_n: float, center_e: float, center_d: float,
                    half_n: float, half_e: float, half_d: float,
                    res_xy: float, res_z: float,
                    size_n: int, size_e: int, size_d: int):
    """Map NED (n, e, d) to 3D grid (i, j, k). Returns (i, j, k) or None if outside."""
    gi = int((n - center_n + half_n) / res_xy)
    gj = int((e - center_e + half_e) / res_xy)
    gk = int((d - center_d + half_d) / res_z)
    if 0 <= gi < size_n and 0 <= gj < size_e and 0 <= gk < size_d:
        return (gi, gj, gk)
    return None


def grid_to_world_3d(i: int, j: int, k: int,
                    center_n: float, center_e: float, center_d: float,
                    half_n: float, half_e: float, half_d: float,
                    res_xy: float, res_z: float):
    """3D grid cell (i, j, k) center in NED (n, e, d)."""
    n = center_n - half_n + (i + 0.5) * res_xy
    e = center_e - half_e + (j + 0.5) * res_xy
    d = center_d - half_d + (k + 0.5) * res_z
    return (n, e, d)


def is_point_free_in_grid(n: float, e: float, d: float, grid, center_n: float, center_e: float, center_d: float,
                          half_n: float, half_e: float, half_d: float, res_xy: float, res_z: float,
                          size_n: int, size_e: int, size_d: int) -> bool:
    """True if NED point (n,e,d) lies in grid bounds and the cell is not obstacle."""
    ijk = world_to_grid_3d(n, e, d, center_n, center_e, center_d, half_n, half_e, half_d,
                           res_xy, res_z, size_n, size_e, size_d)
    if ijk is None:
        return False
    i, j, k = ijk
    return grid[i, j, k] != CELL_OBSTACLE


def is_waypoint_ahead(wn: float, we: float, wd: float,
                      px: float, py: float, pz: float,
                      goal_n: float, goal_e: float, goal_d: float) -> bool:
    """True if waypoint (wn,we,wd) is ahead of current (px,py,pz) toward goal (dot product > 0)."""
    dn = wn - px
    de = we - py
    dd = wd - pz
    gn = goal_n - px
    ge = goal_e - py
    gd = goal_d - pz
    return (dn * gn + de * ge + dd * gd) > 0.0


# Grid cell values: 0=free, 1=obstacle (blocked), 2=buffer (flyable but high cost, avoid when possible)
CELL_FREE = 0
CELL_OBSTACLE = 1
CELL_BUFFER = 2


# ---------------------------- A* ----------------------------

def astar_3d(grid: np.ndarray, start_ijk: tuple, goal_ijk: tuple, allow_diagonal_3d: bool = True,
             buffer_cost: float = 1.0):
    """
    A* on 3D grid. grid[i,j,k]: 0=free, 1=obstacle (blocked), 2=buffer (traversable with cost buffer_cost).
    start_ijk, goal_ijk are (i, j, k). Returns list of (i, j, k) from start to goal, or [] if no path.
    buffer_cost: cost multiplier for stepping into a buffer cell (e.g. 5.0 to prefer avoiding buffer).
    """
    sn, se, sd = grid.shape
    si, sj, sk = start_ijk
    gi, gj, gk = goal_ijk

    if grid[si, sj, sk] == CELL_OBSTACLE:
        return []

    if allow_diagonal_3d:
        neighbors = []
        for di in (-1, 0, 1):
            for dj in (-1, 0, 1):
                for dk in (-1, 0, 1):
                    if (di, dj, dk) == (0, 0, 0):
                        continue
                    neighbors.append((di, dj, dk))
    else:
        neighbors = [(-1,0,0),(1,0,0),(0,-1,0),(0,1,0),(0,0,-1),(0,0,1)]

    def heuristic(a, b):
        ai, aj, ak = a
        bi, bj, bk = b
        di, dj, dk = abs(ai - bi), abs(aj - bj), abs(ak - bk)
        if allow_diagonal_3d:
            # Octile-like in 3D: max + (sqrt3-1)*min_mid + (sqrt2-1)*min_min
            d_max = max(di, dj, dk)
            d_min = min(di, dj, dk)
            d_mid = di + dj + dk - d_max - d_min
            return d_max + (math.sqrt(3) - 1) * d_mid + (math.sqrt(2) - 1) * d_min
        return di + dj + dk

    open_heap = []
    counter = 0
    g_score = {start_ijk: 0}
    parent = {}
    heapq.heappush(open_heap, (heuristic(start_ijk, goal_ijk), counter, start_ijk))
    counter += 1

    while open_heap:
        _, _, curr = heapq.heappop(open_heap)
        if curr == goal_ijk:
            path = []
            cur = curr
            while cur in parent:
                path.append(cur)
                cur = parent[cur]
            path.append(start_ijk)
            path.reverse()
            return path

        ci, cj, ck = curr
        for di, dj, dk in neighbors:
            ni, nj, nk = ci + di, cj + dj, ck + dk
            if ni < 0 or ni >= sn or nj < 0 or nj >= se or nk < 0 or nk >= sd:
                continue
            cell = grid[ni, nj, nk]
            if cell == CELL_OBSTACLE:
                continue
            step = math.sqrt(di*di + dj*dj + dk*dk)
            step_cost = step * (buffer_cost if cell == CELL_BUFFER else 1.0)
            tg = g_score[curr] + step_cost
            nkey = (ni, nj, nk)
            if nkey not in g_score or tg < g_score[nkey]:
                g_score[nkey] = tg
                parent[nkey] = curr
                f = tg + heuristic(nkey, goal_ijk)
                heapq.heappush(open_heap, (f, counter, nkey))
                counter += 1

    return []


# ---------------------------- State Machine ----------------------------

class Stage(Enum):
    WARMUP = 0
    ARMING = 1
    TAKEOFF = 2
    HOVER_STABLE = 3
    PLAN_FOLLOW = 4


# ---------------------------- Main Node ----------------------------

class AStarGridPursuit(Node):
    def __init__(self):
        super().__init__('astar_grid_pursuit')

        # ---------- Parameters ----------
        self.declare_parameter('hz', 30.0)
        self.declare_parameter('takeoff_alt_m', 5.0)
        self.declare_parameter('stable_hover_s', 1.0)
        self.declare_parameter('min_nav_alt_m', 3.5)
        self.declare_parameter('target_gps_topic', '/static_cam/target_gps')
        self.declare_parameter('depth_topic', '/oak/depth/image_rect_raw')
        self.declare_parameter('depth_info_topic', '/oak/depth/camera_info')
        # Local 3D planning window (centered on drone)
        self.declare_parameter('grid_half_size_m', 20.0)
        self.declare_parameter('grid_resolution_m', 0.5)
        self.declare_parameter('grid_half_height_m', 5.0)   # ±half in D (altitude)
        self.declare_parameter('grid_resolution_z_m', 0.5)
        # Depth: camera may output 0~1 normalized (1 = max range). depth_scale_m converts to real meters (e.g. 1.0 -> 3 m).
        self.declare_parameter('depth_scale_m', 10.0)   # real_depth_m = raw_depth * depth_scale_m (e.g. raw 1.0 = 3 m)
        self.declare_parameter('depth_max_m', 9.9)    # max range in real meters to consider for obstacles
        self.declare_parameter('depth_fov_deg', 120.0)
        self.declare_parameter('min_valid_depth_m', 0.2)  # ignore depth < this in real meters (noise)
        self.declare_parameter('cruise_speed_m_s', 1.0)  # reserved; actual speed from PX4 MPC (e.g. MPC_XY_CRUISE)
        self.declare_parameter('waypoint_reach_radius_m', 0.4)
        self.declare_parameter('replan_interval_s', 2.0)
        self.declare_parameter('inflation_cells', 2)  # obstacle inflation (blocked)
        self.declare_parameter('buffer_cells', 1)     # extra layer around inflation: flyable but high cost (avoids delay overshoot)
        self.declare_parameter('buffer_cost', 5.0)   # A* cost multiplier for buffer cells (prefer paths outside buffer)
        self.declare_parameter('debug_obstacle', True)  # log depth/grid/path diagnostics to find obstacle issues
        self.declare_parameter('obstacle_encounter_log_dir', '')  # e.g. /path/to/missions/log; empty = disable encounter reports
        self.declare_parameter('obstacle_encounter_throttle_s', 3.0)  # min seconds between writing encounter reports
        self.declare_parameter('reuse_waypoints', True)  # persist committed waypoints; prefer unchanged points on replan for smoother flight

        hz = float(self.get_parameter('hz').value)
        self.dt = 1.0 / max(hz, 1.0)

        # ---------- State ----------
        self.stage = Stage.WARMUP
        self.stage_enter_ts = self.get_clock().now().nanoseconds
        self.armed = False
        self.offboard_ok = False
        self.xy_valid = False
        self.z_valid = False
        self.px = 0.0   # NED North
        self.py = 0.0   # NED East
        self.pz = 0.0   # NED Down
        self.roll = 0.0   # rad, body roll
        self.pitch = 0.0  # rad, body pitch
        self.heading = 0.0  # rad NED yaw

        self.home_lat = None
        self.home_lon = None
        self.home_alt = None
        self.tgt_lat = None
        self.tgt_lon = None
        self.tgt_alt = None

        self.depth_image = None
        self.depth_stamp_s = None
        self.depth_width = None
        self.depth_height = None
        # Default intrinsics match Unity OakLitePublisher: 1280x720, ~120° FOV → fx=465, fy=614, cx=320, cy=320
        self.cam_fx = 465
        self.cam_fy = 614
        self.cam_cx = 320.0
        self.cam_cy = 320.0

        self.path_ned = []           # list of (n, e, d) in NED (3D waypoints)
        self.committed_path_ned = [] # last committed path for reuse: on replan, keep waypoints that stay free
        self.waypoint_idx = 0
        self.last_replan_s = 0.0
        self._depth_diag_ts = 0.0    # throttle depth callback diagnostics
        self._last_encounter_report_ts = 0.0  # throttle obstacle encounter reports

        # ---------- Publishers ----------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub_offboard = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', qos)
        self.pub_sp = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', qos)
        self.pub_path = self.create_publisher(PoseArray, '/planning/path', 2)

        # ---------- Subscribers ----------
        self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position_v1', self.on_local_pos, qos)
        if _HAS_VEHICLE_ATTITUDE:
            self.create_subscription(VehicleAttitude, '/fmu/out/vehicle_attitude', self.on_attitude, qos)
        self.create_subscription(NavSatFix, '/drone/gps', self.on_gps, qos)
        target_topic = str(self.get_parameter('target_gps_topic').value)
        self.create_subscription(NavSatFix, target_topic, self.on_target_gps, 10)
        depth_topic = str(self.get_parameter('depth_topic').value)
        depth_info_topic = str(self.get_parameter('depth_info_topic').value)
        self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.create_subscription(CameraInfo, depth_info_topic, self.depth_info_callback, 10)

        self.timer = self.create_timer(self.dt, self.loop)
        self.get_logger().info(
            f"[astar_grid] hz={hz} grid_half={self.get_parameter('grid_half_size_m').value}m "
            f"grid_half_height={self.get_parameter('grid_half_height_m').value}m "
            f"depth_max={self.get_parameter('depth_max_m').value}m fov={self.get_parameter('depth_fov_deg').value}deg "
            f"cruise_speed={self.get_parameter('cruise_speed_m_s').value}m/s | "
            f"stages: WARMUP->ARMING->TAKEOFF->HOVER_STABLE->PLAN_FOLLOW (3D A* replan in PLAN_FOLLOW)"
        )

    def on_local_pos(self, msg: VehicleLocalPosition):
        self.xy_valid = bool(msg.xy_valid)
        self.z_valid = bool(msg.z_valid)
        self.px = float(msg.x)   # North
        self.py = float(msg.y)   # East
        self.pz = float(msg.z)  # Down
        self.heading = float(msg.heading)

    def on_attitude(self, msg):
        """Update roll, pitch, yaw from VehicleAttitude quaternion (q: w, x, y, z)."""
        if not _HAS_VEHICLE_ATTITUDE:
            return
        q = msg.q
        w, x, y, z = float(q[0]), float(q[1]), float(q[2]), float(q[3])
        self.roll = math.atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
        self.pitch = math.asin(max(-1.0, min(1.0, 2.0 * (w * y - z * x))))
        self.heading = math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))

    def on_gps(self, msg: NavSatFix):
        if self.home_lat is None:
            self.home_lat = float(msg.latitude)
            self.home_lon = float(msg.longitude)
            self.home_alt = float(msg.altitude)

    def on_target_gps(self, msg: NavSatFix):
        self.tgt_lat = float(msg.latitude)
        self.tgt_lon = float(msg.longitude)
        self.tgt_alt = float(msg.altitude)

    def depth_callback(self, msg: Image):
        self.depth_stamp_s = time.time()
        self.depth_width = msg.width
        self.depth_height = msg.height
        if msg.encoding == '32FC1':
            depth = np.frombuffer(msg.data, dtype=np.float32)
        elif msg.encoding == '16UC1':
            depth = np.frombuffer(msg.data, dtype=np.uint16).astype(np.float32) / 1000.0
        else:
            if self.get_parameter('debug_obstacle').value:
                self.get_logger().warn(
                    f"[depth] unsupported encoding='{msg.encoding}', need 32FC1 or 16UC1"
                )
            self.depth_image = None
            return
        if depth.size != msg.width * msg.height:
            if self.get_parameter('debug_obstacle').value:
                self.get_logger().warn(
                    f"[depth] size mismatch: {depth.size} vs {msg.width * msg.height}"
                )
            self.depth_image = None
            return
        self.depth_image = depth.reshape((msg.height, msg.width))
        # Throttled diagnostics (use depth_scale: raw -> real m, so valid and range are in meters)
        if self.get_parameter('debug_obstacle').value:
            t = time.time()
            if t - self._depth_diag_ts >= 2.0:
                self._depth_diag_ts = t
                depth_scale = float(self.get_parameter('depth_scale_m').value)
                min_d = float(self.get_parameter('min_valid_depth_m').value)
                depth_max = float(self.get_parameter('depth_max_m').value)
                raw = self.depth_image.astype(np.float64)
                finite = np.isfinite(raw)
                real_m = raw * depth_scale
                valid = (finite & (real_m >= min_d) & (real_m < depth_max))
                n_valid = int(np.sum(valid))
                d_min = float(np.min(real_m[finite])) if np.any(finite) else float('nan')
                d_max = float(np.max(real_m[finite])) if np.any(finite) else float('nan')
                self.get_logger().info(
                    f"[depth] {msg.width}x{msg.height} {msg.encoding} scale={depth_scale} | "
                    f"valid({min_d}~{depth_max}m)={n_valid} | range=[{d_min:.2f}, {d_max:.2f}]m"
                )

    def depth_info_callback(self, msg: CameraInfo):
        self.depth_width = msg.width
        self.depth_height = msg.height
        if len(msg.k) >= 6:
            self.cam_fx = float(msg.k[0])
            self.cam_fy = float(msg.k[4])
            self.cam_cx = float(msg.k[2])
            self.cam_cy = float(msg.k[5])

    def _write_obstacle_encounter_report(
        self,
        depth_summary: dict,
        steps: list,
        result: dict,
        obstacle_ijk: np.ndarray,
        center_n: float, center_e: float, center_d: float,
        half_xy: float, half_d: float, res_xy: float, res_z: float,
    ):
        """Write one encounter report to log dir: depth summary, processing steps, result, obstacle bearings/distances."""
        log_dir = str(self.get_parameter('obstacle_encounter_log_dir').value).strip()
        if not log_dir:
            return
        throttle_s = float(self.get_parameter('obstacle_encounter_throttle_s').value)
        now = self.get_clock().now().nanoseconds * 1e-9
        if now - self._last_encounter_report_ts < throttle_s:
            return
        self._last_encounter_report_ts = now
        try:
            from datetime import datetime
            os.makedirs(log_dir, exist_ok=True)
            ts = datetime.now().strftime('%Y-%m-%dT%H-%M-%S')
            report_path = os.path.join(log_dir, 'encounter_%s.txt' % ts)
            depth_npy_path = os.path.join(log_dir, 'encounter_depth_%s.npy' % ts)

            lines = []
            lines.append('=' * 60)
            lines.append('OBSTACLE ENCOUNTER REPORT @ %s' % datetime.now().isoformat())
            lines.append('=' * 60)
            lines.append('')
            lines.append('--- 1. DEPTH CAMERA INPUT ---')
            for k, v in depth_summary.items():
                lines.append('  %s: %s' % (k, v))
            lines.append('')
            lines.append('--- 2. PROCESSING STEPS ---')
            for i, step in enumerate(steps, 1):
                lines.append('  Step %d: %s' % (i, step))
            lines.append('')
            lines.append('--- 3. RESULT ---')
            for k, v in result.items():
                lines.append('  %s: %s' % (k, v))
            lines.append('')

            n_obs = obstacle_ijk.shape[0]
            if n_obs == 0:
                lines.append('--- 4. OBSTACLE RELATIVE TO DRONE --- (no obstacle cells)')
            else:
                offsets_n = np.empty(n_obs)
                offsets_e = np.empty(n_obs)
                offsets_d = np.empty(n_obs)
                for idx in range(n_obs):
                    i, j, k = int(obstacle_ijk[idx, 0]), int(obstacle_ijk[idx, 1]), int(obstacle_ijk[idx, 2])
                    n, e, d = grid_to_world_3d(i, j, k, center_n, center_e, center_d,
                                               half_xy, half_xy, half_d, res_xy, res_z)
                    offsets_n[idx] = n - center_n
                    offsets_e[idx] = e - center_e
                    offsets_d[idx] = d - center_d
                dist = np.sqrt(offsets_n**2 + offsets_e**2 + offsets_d**2)
                bearing_rad = np.arctan2(offsets_e, offsets_n)
                bearing_deg = np.degrees(bearing_rad)
                lines.append('--- 4. OBSTACLE RELATIVE TO DRONE (NED) ---')
                lines.append('  Drone position (NED): n=%.3f e=%.3f d=%.3f' % (center_n, center_e, center_d))
                lines.append('  Obstacle cells count: %d' % n_obs)
                lines.append('  Distance (m): min=%.3f max=%.3f mean=%.3f' % (float(np.min(dist)), float(np.max(dist)), float(np.mean(dist))))
                lines.append('  Bearing (deg from North): min=%.1f max=%.1f' % (float(np.min(bearing_deg)), float(np.max(bearing_deg))))
                lines.append('  Offset North (m): min=%.3f max=%.3f' % (float(np.min(offsets_n)), float(np.max(offsets_n))))
                lines.append('  Offset East (m): min=%.3f max=%.3f' % (float(np.min(offsets_e)), float(np.max(offsets_e))))
                lines.append('  Offset Down (m): min=%.3f max=%.3f' % (float(np.min(offsets_d)), float(np.max(offsets_d))))
                idx_min = int(np.argmin(dist))
                lines.append('  Closest obstacle: %.3f m at bearing %.1f deg (N=%.3f E=%.3f D=%.3f m)' % (
                    float(dist[idx_min]),
                    float(bearing_deg[idx_min]),
                    float(offsets_n[idx_min]), float(offsets_e[idx_min]), float(offsets_d[idx_min]),
                ))
            lines.append('')
            lines.append('=' * 60)

            with open(report_path, 'w', encoding='utf-8') as f:
                f.write('\n'.join(lines) + '\n')

            if self.depth_image is not None and self.depth_image.size > 0:
                np.save(depth_npy_path, self.depth_image.astype(np.float32))

            self.get_logger().info('[encounter] report written: %s' % report_path)
        except OSError as e:
            self.get_logger().warn('[encounter] failed to write report: %s' % str(e))

    def build_3d_grid_ned(self, inflation_override=None) -> tuple:
        """
        Build 3D occupancy grid (NED: north, east, down) centered at (px, py, pz).
        Uses full attitude (roll, pitch, yaw) and R_wb = Rz(yaw) Ry(pitch) Rx(roll) for camera->body->NED.
        Camera: Z forward, X right, Y down. Vectorized depth projection.
        Returns (grid_3d, size_n, size_e, size_d, res_xy, res_z, half_n, half_e, half_d).
        """
        half_xy = float(self.get_parameter('grid_half_size_m').value)
        half_d = float(self.get_parameter('grid_half_height_m').value)
        res_xy = float(self.get_parameter('grid_resolution_m').value)
        res_z = float(self.get_parameter('grid_resolution_z_m').value)
        depth_scale = float(self.get_parameter('depth_scale_m').value)
        depth_max = float(self.get_parameter('depth_max_m').value)
        min_d = float(self.get_parameter('min_valid_depth_m').value)
        inflate = int(inflation_override if inflation_override is not None else self.get_parameter('inflation_cells').value)

        size_n = int(2 * half_xy / res_xy + 1)
        size_e = int(2 * half_xy / res_xy + 1)
        size_d = int(2 * half_d / res_z + 1)
        size_n = max(10, min(200, size_n))
        size_e = max(10, min(200, size_e))
        size_d = max(5, min(100, size_d))
        res_xy_actual = 2 * half_xy / (size_n - 1) if size_n > 1 else res_xy
        res_z_actual = 2 * half_d / (size_d - 1) if size_d > 1 else res_z

        grid = np.zeros((size_n, size_e, size_d), dtype=np.uint8)
        center_n, center_e, center_d = self.px, self.py, self.pz
        fx, fy = self.cam_fx, self.cam_fy
        cx, cy = self.cam_cx, self.cam_cy

        if self.depth_image is None or self.depth_image.size == 0:
            if self.get_parameter('debug_obstacle').value:
                self._last_grid_diag = {
                    'depth_used': False, 'n_raw': 0, 'n_after': 0,
                    'grid_size': (size_n, size_e, size_d), 'res_xy': res_xy_actual, 'res_z': res_z_actual,
                }
            return grid, size_n, size_e, size_d, res_xy_actual, res_z_actual, half_xy, half_xy, half_d

        depth_summary = {}
        steps = []
        h, w = self.depth_image.shape
        depth_flat_all = self.depth_image.ravel().astype(np.float64)
        finite = np.isfinite(depth_flat_all)
        depth_summary['shape'] = '%dx%d' % (h, w)
        depth_summary['total_pixels'] = int(depth_flat_all.size)
        depth_summary['finite_count'] = int(np.sum(finite))
        if np.any(finite):
            raw_min, raw_max = float(np.min(depth_flat_all[finite])), float(np.max(depth_flat_all[finite]))
            depth_summary['depth_range_raw'] = '[%.3f, %.3f]' % (raw_min, raw_max)
            depth_summary['depth_range_real_m'] = '[%.3f, %.3f]' % (raw_min * depth_scale, raw_max * depth_scale)
            depth_summary['depth_mean_real_m'] = float(np.mean(depth_flat_all[finite])) * depth_scale
        else:
            depth_summary['depth_range_raw'] = 'N/A'
            depth_summary['depth_range_real_m'] = 'N/A'
            depth_summary['depth_mean_real_m'] = 'N/A'
        depth_summary['depth_scale_m'] = depth_scale
        depth_summary['min_valid_m'] = min_d
        depth_summary['depth_max_m'] = depth_max
        depth_summary['attitude_rad'] = 'roll=%.3f pitch=%.3f yaw=%.3f' % (self.roll, self.pitch, self.heading)

        step = 2
        uu, vv = np.meshgrid(
            np.arange(0, w, step, dtype=np.int32),
            np.arange(0, h, step, dtype=np.int32),
            indexing='xy',
        )
        u_flat = uu.ravel()
        v_flat = vv.ravel()
        d_flat = self.depth_image[v_flat, u_flat].astype(np.float64)
        d_real = d_flat * depth_scale

        valid = (
            np.isfinite(d_flat)
            & (d_real >= min_d)
            & (d_real < depth_max)
        )
        n_valid = int(np.sum(valid))
        depth_summary['valid_in_range_count'] = n_valid
        steps.append('Scale: raw * %.1f -> real m. Filter valid: [%.2f, %.2f) m -> n_valid=%d' % (depth_scale, min_d, depth_max, n_valid))
        if n_valid == 0:
            return grid, size_n, size_e, size_d, res_xy_actual, res_z_actual, half_xy, half_xy, half_d
        near_thresh = depth_max - 0.05
        near_mask = d_real < near_thresh
        near_count = int(np.sum(valid & near_mask))
        depth_summary['near_count_d_real_lt_%.2f' % near_thresh] = near_count
        steps.append('Near-depth count (real < %.2f m): %d' % (near_thresh, near_count))
        # Only skip when sensor is clearly at max range (almost no near depth). Do NOT require
        # near_count >= 1% of valid - that skipped the first tree when it was small in the image.
        if near_count < 10:
            steps.append('Skip: near_count < 10 -> no obstacle marking')
            if self.get_parameter('debug_obstacle').value:
                self._last_grid_diag = {
                    'depth_used': True, 'n_raw': 0, 'n_after': 0,
                    'grid_size': (size_n, size_e, size_d), 'res_xy': res_xy_actual, 'res_z': res_z_actual,
                }
            return grid, size_n, size_e, size_d, res_xy_actual, res_z_actual, half_xy, half_xy, half_d

        steps.append('Project to camera (x_c,y_c,z_c) then body (forward,right,down) then NED with R_wb(roll,pitch,yaw)')
        u_flat = u_flat[valid]
        v_flat = v_flat[valid]
        d_real = d_real[valid]

        x_c = (u_flat - cx) * d_real / fx  # camera right (real m)
        y_c = (v_flat - cy) * d_real / fy  # camera down (real m)
        z_c = d_real                         # camera forward (real m)
        # Body FRD: Forward=z_c, Right=x_c, Down=y_c -> v_body = (forward, right, down)
        P_body = np.column_stack((z_c, x_c, y_c))

        R_wb = rotation_body_to_ned(self.roll, self.pitch, self.heading)
        P_ned = (R_wb @ P_body.T).T
        P_ned[:, 0] += center_n
        P_ned[:, 1] += center_e
        P_ned[:, 2] += center_d

        gi = ((P_ned[:, 0] - center_n + half_xy) / res_xy_actual).astype(np.int32)
        gj = ((P_ned[:, 1] - center_e + half_xy) / res_xy_actual).astype(np.int32)
        gk = ((P_ned[:, 2] - center_d + half_d) / res_z_actual).astype(np.int32)

        in_bounds = (
            (gi >= 0) & (gi < size_n)
            & (gj >= 0) & (gj < size_e)
            & (gk >= 0) & (gk < size_d)
        )
        n_in_bounds = int(np.sum(in_bounds))
        steps.append('Grid indices in bounds: %d / %d' % (n_in_bounds, int(np.sum(valid))))
        gi = gi[in_bounds]
        gj = gj[in_bounds]
        gk = gk[in_bounds]
        grid[gi, gj, gk] = CELL_OBSTACLE

        n_raw = int(np.sum(grid == CELL_OBSTACLE))
        steps.append('Mark obstacle cells: n_raw=%d' % n_raw)
        if inflate > 0:
            occupied = np.where(grid == CELL_OBSTACLE)
            oi, oj, ok = occupied[0], occupied[1], occupied[2]
            inf_lo, inf_hi = -inflate, inflate + 1
            for di in range(inf_lo, inf_hi):
                for dj in range(inf_lo, inf_hi):
                    for dk in range(inf_lo, inf_hi):
                        ni = np.clip(oi + di, 0, size_n - 1)
                        nj = np.clip(oj + dj, 0, size_e - 1)
                        nk = np.clip(ok + dk, 0, size_d - 1)
                        grid[ni, nj, nk] = CELL_OBSTACLE
        buffer_cells = int(self.get_parameter('buffer_cells').value)
        if buffer_cells > 0:
            obst = np.where(grid == CELL_OBSTACLE)
            oi, oj, ok = obst[0], obst[1], obst[2]
            buf_lo, buf_hi = -buffer_cells, buffer_cells + 1
            for di in range(buf_lo, buf_hi):
                for dj in range(buf_lo, buf_hi):
                    for dk in range(buf_lo, buf_hi):
                        ni = np.clip(oi + di, 0, size_n - 1)
                        nj = np.clip(oj + dj, 0, size_e - 1)
                        nk = np.clip(ok + dk, 0, size_d - 1)
                        mask = grid[ni, nj, nk] == CELL_FREE
                        grid[ni[mask], nj[mask], nk[mask]] = CELL_BUFFER
        n_after = int(np.sum(grid == CELL_OBSTACLE))
        steps.append('Inflation: inflate_cells=%d -> n_after=%d' % (inflate, n_after))
        steps.append('Buffer: buffer_cells=%d (free cells near obstacle marked as buffer)' % buffer_cells)
        result = {
            'n_raw': n_raw,
            'n_after_inflation': n_after,
            'grid_shape': '(%d, %d, %d)' % (size_n, size_e, size_d),
            'res_xy_m': res_xy_actual,
            'res_z_m': res_z_actual,
        }
        if n_after > 0:
            obst = np.where(grid == CELL_OBSTACLE)
            oi, oj, ok = obst[0], obst[1], obst[2]
            n_obs = len(oi)
            sample = min(n_obs, 500)
            idx = np.linspace(0, n_obs - 1, sample, dtype=np.int32) if n_obs > 0 else np.array([], dtype=np.int32)
            if len(idx) > 0:
                obstacle_ijk = np.column_stack((oi[idx], oj[idx], ok[idx]))
                self._write_obstacle_encounter_report(
                    depth_summary, steps, result, obstacle_ijk,
                    center_n, center_e, center_d, half_xy, half_d, res_xy_actual, res_z_actual,
                )
        if self.get_parameter('debug_obstacle').value:
            self._last_grid_diag = {
                'depth_used': True, 'n_raw': n_raw, 'n_after': n_after,
                'grid_size': (size_n, size_e, size_d), 'res_xy': res_xy_actual, 'res_z': res_z_actual,
            }
        return grid, size_n, size_e, size_d, res_xy_actual, res_z_actual, half_xy, half_xy, half_d

    def plan_path_ned(self) -> list:
        """
        Run 3D A* on local 3D occupancy grid. Start = drone (px, py, pz), goal = target NED (n, e, d).
        Returns list of (n, e, d) waypoints in NED.
        """
        if self.home_lat is None or self.tgt_lat is None:
            if self.get_parameter('debug_obstacle').value:
                t = getattr(self, '_last_plan_skip_log', 0.0)
                now = self.get_clock().now().nanoseconds * 1e-9
                if now - t >= 5.0:
                    self._last_plan_skip_log = now
                    self.get_logger().info(
                        "[plan] skipped: home=%s target=%s — need /drone/gps and target_gps"
                        % ("ok" if self.home_lat is not None else "None",
                           "ok" if self.tgt_lat is not None else "None")
                    )
            return []
        tgt_alt = self.tgt_alt if self.tgt_alt is not None else self.home_alt
        goal_n, goal_e, goal_d = gps_to_ned(
            self.tgt_lat, self.tgt_lon, tgt_alt,
            self.home_lat, self.home_lon, self.home_alt,
        )
        # gps_to_ned returns (north, east, down); goal_d is NED down component

        grid, size_n, size_e, size_d, res_xy, res_z, half_n, half_e, half_d = self.build_3d_grid_ned()

        center_n, center_e, center_d = self.px, self.py, self.pz
        current_ijk = (size_n // 2, size_e // 2, size_d // 2)
        goal_ijk = world_to_grid_3d(goal_n, goal_e, goal_d, center_n, center_e, center_d,
                                    half_n, half_e, half_d, res_xy, res_z,
                                    size_n, size_e, size_d)
        if goal_ijk is None:
            gi = int((goal_n - center_n + half_n) / res_xy)
            gj = int((goal_e - center_e + half_e) / res_xy)
            gk = int((goal_d - center_d + half_d) / res_z)
            gi = max(0, min(size_n - 1, gi))
            gj = max(0, min(size_e - 1, gj))
            gk = max(0, min(size_d - 1, gk))
            goal_ijk = (gi, gj, gk)

        # Mark current position as obstacle so the path never includes it (avoids "step backward" to self)
        si, sj, sk = current_ijk
        grid[si, sj, sk] = 1
        # A* start = free neighbor of current cell that is closest to goal (path starts from first step toward goal)
        gi, gj, gk = goal_ijk
        start_ijk = None
        best_dist = float('inf')
        for di in range(-1, 2):
            for dj in range(-1, 2):
                for dk in range(-1, 2):
                    if (di, dj, dk) == (0, 0, 0):
                        continue
                    ni, nj, nk = si + di, sj + dj, sk + dk
                    if 0 <= ni < size_n and 0 <= nj < size_e and 0 <= nk < size_d and grid[ni, nj, nk] != CELL_OBSTACLE:
                        d = abs(ni - gi) + abs(nj - gj) + abs(nk - gk)
                        if d < best_dist:
                            best_dist = d
                            start_ijk = (ni, nj, nk)
        if start_ijk is None:
            if self.get_parameter('debug_obstacle').value:
                self.get_logger().info("[plan] no free neighbor of current cell, NO PATH")
            return []
        buffer_cost = float(self.get_parameter('buffer_cost').value)
        path_ijk = astar_3d(grid, start_ijk, goal_ijk, allow_diagonal_3d=True, buffer_cost=buffer_cost)

        if self.get_parameter('debug_obstacle').value:
            diag = getattr(self, '_last_grid_diag', None)
            if diag:
                gs = diag.get('grid_size', (0, 0, 0))
                self.get_logger().info(
                    f"[grid] depth_used={diag['depth_used']} grid={gs} "
                    f"res_xy={diag.get('res_xy', 0):.3f}m res_z={diag.get('res_z', 0):.3f}m | "
                    f"obstacles raw={diag['n_raw']} after_inflate={diag['n_after']}"
                )
            self.get_logger().info(
                f"[plan] start_ijk={start_ijk} goal_ijk={goal_ijk} | "
                f"3D A* {'path_len=' + str(len(path_ijk)) if path_ijk else 'NO PATH'}"
            )

        if not path_ijk:
            return []

        new_path = []
        for (i, j, k) in path_ijk:
            n, e, d = grid_to_world_3d(i, j, k, center_n, center_e, center_d,
                                       half_n, half_e, half_d, res_xy, res_z)
            new_path.append((n, e, d))

        reuse = self.get_parameter('reuse_waypoints').value
        old_path = getattr(self, 'committed_path_ned', []) or []

        if reuse and old_path:
            def free(w):
                return is_point_free_in_grid(
                    w[0], w[1], w[2], grid, center_n, center_e, center_d,
                    half_n, half_e, half_d, res_xy, res_z, size_n, size_e, size_d)
            def ahead(w):
                return is_waypoint_ahead(w[0], w[1], w[2], center_n, center_e, center_d, goal_n, goal_e, goal_d)

            valid_old = []
            for w in old_path:
                if not ahead(w):
                    continue
                if not free(w):
                    break
                valid_old.append(w)

            if valid_old:
                last_n, last_e, last_d = valid_old[-1]
                last_ijk = world_to_grid_3d(last_n, last_e, last_d, center_n, center_e, center_d,
                                            half_n, half_e, half_d, res_xy, res_z, size_n, size_e, size_d)
                if last_ijk is not None:
                    tail_ijk = astar_3d(grid, last_ijk, goal_ijk, allow_diagonal_3d=True, buffer_cost=buffer_cost)
                    if tail_ijk:
                        tail_ned = []
                        for (i, j, k) in tail_ijk:
                            n, e, d = grid_to_world_3d(i, j, k, center_n, center_e, center_d,
                                                       half_n, half_e, half_d, res_xy, res_z)
                            tail_ned.append((n, e, d))
                        merged = valid_old + tail_ned[1:]
                        self.committed_path_ned = list(merged)
                        if self.get_parameter('debug_obstacle').value:
                            self.get_logger().info(
                                f"[plan] reuse: valid_old={len(valid_old)} tail={len(tail_ned)} merged={len(merged)}"
                            )
                        return merged

        self.committed_path_ned = list(new_path)
        return new_path

    def publish_offboard_mode(self):
        m = OffboardControlMode()
        m.timestamp = micros(self)
        m.position = True
        m.velocity = False
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        self.pub_offboard.publish(m)

    def publish_path_ned(self, path_ned: list):
        """Publish path as PoseArray (NED: x=north, y=east, z=down). path_ned: list of (n, e, d)."""
        if not path_ned:
            return
        msg = PoseArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        for (n, e, d) in path_ned:
            p = Pose()
            p.position = Point(x=float(n), y=float(e), z=float(d))
            p.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
            msg.poses.append(p)
        self.pub_path.publish(msg)

    def publish_sp(self, xN=None, yE=None, zD=None, yaw=None):
        sp = TrajectorySetpoint()
        sp.timestamp = micros(self)
        sp.position = [
            float(xN if xN is not None else self.px),
            float(yE if yE is not None else self.py),
            float(zD if zD is not None else self.pz),
        ]
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

    def set_offboard(self):
        self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.offboard_ok = True

    def loop(self):
        self.publish_offboard_mode()
        t_ns = self.get_clock().now().nanoseconds
        t_s = t_ns * 1e-9

        takeoff_alt = float(self.get_parameter('takeoff_alt_m').value)
        zD_goal = -abs(takeoff_alt)

        if self.stage == Stage.WARMUP:
            self.publish_sp(self.px, self.py, zD_goal, self.heading)
            if (t_ns - self.stage_enter_ts) * 1e-9 >= 0.5:
                self.arm()
                self.set_offboard()
                self.stage = Stage.ARMING
                self.stage_enter_ts = t_ns
                self.get_logger().info("[stage] WARMUP -> ARMING")
            return

        if self.stage == Stage.ARMING:
            self.publish_sp(self.px, self.py, zD_goal, self.heading)
            if self.z_valid:
                self.stage = Stage.TAKEOFF
                self.stage_enter_ts = t_ns
                self.get_logger().info("[stage] ARMING -> TAKEOFF")
            return

        if self.stage == Stage.TAKEOFF:
            self.publish_sp(self.px, self.py, zD_goal, self.heading)
            if abs(self.pz - zD_goal) <= 0.25:
                self.stage = Stage.HOVER_STABLE
                self.stage_enter_ts = t_ns
                self.get_logger().info("[stage] TAKEOFF -> HOVER_STABLE")
            return

        if self.stage == Stage.HOVER_STABLE:
            self.publish_sp(self.px, self.py, zD_goal, self.heading)
            stable_s = float(self.get_parameter('stable_hover_s').value)
            min_nav = float(self.get_parameter('min_nav_alt_m').value)
            if (t_ns - self.stage_enter_ts) * 1e-9 >= stable_s and self.xy_valid and self.z_valid:
                if abs(self.pz) >= min_nav:
                    self.stage = Stage.PLAN_FOLLOW
                    self.stage_enter_ts = t_ns
                    self.path_ned = []
                    self.committed_path_ned = []
                    self.waypoint_idx = 0
                    self.last_replan_s = t_s
                    self.get_logger().info("[stage] HOVER_STABLE -> PLAN_FOLLOW")
            return

        if self.stage == Stage.PLAN_FOLLOW:
            if self.home_lat is None or self.tgt_lat is None:
                self.publish_sp(self.px, self.py, zD_goal, self.heading)
                if self.get_parameter('debug_obstacle').value:
                    t = getattr(self, '_last_plan_follow_wait_log', 0.0)
                    if (t_ns * 1e-9 - t) >= 5.0:
                        self._last_plan_follow_wait_log = t_ns * 1e-9
                        self.get_logger().info(
                            "[plan] PLAN_FOLLOW waiting: home=%s target=%s (need /drone/gps + target_gps)"
                            % ("ok" if self.home_lat is not None else "None",
                               "ok" if self.tgt_lat is not None else "None")
                        )
                return

            replan_interval = float(self.get_parameter('replan_interval_s').value)
            reach_radius = float(self.get_parameter('waypoint_reach_radius_m').value)

            # Replan when no path or interval elapsed
            if not self.path_ned or (t_s - self.last_replan_s) >= replan_interval:
                self.path_ned = self.plan_path_ned()
                # Start from 2nd waypoint (index 1): latency often puts drone past 1st, avoids flying backward
                self.waypoint_idx = 1 if len(self.path_ned) >= 2 else 0
                self.last_replan_s = t_s
                if self.path_ned:
                    self.get_logger().info(f"[plan] 3D A* path length={len(self.path_ned)}")
                    self.publish_path_ned(self.path_ned)

            if not self.path_ned:
                self.publish_sp(self.px, self.py, zD_goal, self.heading)
                return

            # Advance waypoint if close enough (3D distance)
            while self.waypoint_idx < len(self.path_ned):
                wn, we, wd = self.path_ned[self.waypoint_idx]
                dist = math.sqrt((wn - self.px)**2 + (we - self.py)**2 + (wd - self.pz)**2)
                if dist <= reach_radius and self.waypoint_idx < len(self.path_ned) - 1:
                    self.waypoint_idx += 1
                else:
                    break

            if self.waypoint_idx >= len(self.path_ned):
                self.publish_sp(self.px, self.py, self.pz, self.heading)
                return

            wn, we, wd = self.path_ned[self.waypoint_idx]
            yaw = radians_wrap(math.atan2(we - self.py, wn - self.px))
            self.publish_sp(wn, we, wd, yaw)
            return

        self.publish_sp(self.px, self.py, self.pz, self.heading)


def main():
    rclpy.init()
    node = AStarGridPursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
