#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
2D grid A* pursuit: depth → local 2D occupancy grid (NED) → A* path → setpoints.

- Grid: 20 m around drone (40 m × 40 m), configurable resolution.
- Depth: 1.0 m range (configurable), 120° FOV (used to mark obstacles in grid).
- All coordinates in NED: current position from vehicle_local_position (or GPS),
  target from target GPS with home reference; grid and path in NED.
- Cruise speed as parameter (default 1 m/s).

Topics:
  sub: /fmu/out/vehicle_local_position_v1  (px4_msgs/VehicleLocalPosition) — NED position & heading
  sub: /drone/gps                           (sensor_msgs/NavSatFix) — used to set home once
  sub: /static_cam/target_gps                (sensor_msgs/NavSatFix) — target in NED via gps_to_ned
  sub: /oak/depth/image_rect_raw             (sensor_msgs/Image)
  sub: /oak/depth/camera_info                (sensor_msgs/CameraInfo)
  pub: /fmu/in/offboard_control_mode, /fmu/in/trajectory_setpoint, /fmu/in/vehicle_command
  pub: /planning/path (geometry_msgs/PoseArray) — NED path for Unity visualization
"""

import math
import os
import time
from enum import Enum
import heapq

import numpy as np

# Log files written under this directory
_MISSIONS_DIR = "/home/jefft/drone_ws/src/drone_autonomy/drone_autonomy/missions/log"
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


def _save_depth_preview_png(depth: np.ndarray, depth_max_m: float, png_path: str):
    """Save depth array as a viewable PNG (grayscale: near=dark, far=bright). No matplotlib to avoid NumPy 2.x issues."""
    try:
        from PIL import Image
        depth_show = np.nan_to_num(np.asarray(depth, dtype=np.float64), nan=0.0, posinf=0.0, neginf=0.0)
        depth_show = np.clip(depth_show, 0.0, float(depth_max_m))
        scale = max(1e-6, float(depth_max_m))
        gray = (depth_show / scale * 255).astype(np.uint8)
        Image.fromarray(gray, mode='L').save(png_path)
    except Exception:
        pass


# ---------------------------- 2D Grid ----------------------------

def world_to_grid(n: float, e: float, center_n: float, center_e: float,
                  half_size: float, res: float, size: int):
    """Map NED (n, e) to grid (i, j). Returns (i, j) or None if outside."""
    gx = (n - center_n + half_size) / res
    gy = (e - center_e + half_size) / res
    i, j = int(gx), int(gy)
    if 0 <= i < size and 0 <= j < size:
        return (i, j)
    return None


def grid_to_world(i: int, j: int, center_n: float, center_e: float,
                  half_size: float, res: float):
    """Grid cell (i, j) center in NED."""
    n = center_n - half_size + (i + 0.5) * res
    e = center_e - half_size + (j + 0.5) * res
    return (n, e)


def world_ned_to_world_grid(n: float, e: float, center_n: float, center_e: float,
                            world_half: float, res: float, size_w: int):
    """Map NED (n, e) to world grid (i, j). Grid is centered at (center_n, center_e). Returns (i, j) or None if outside."""
    gi = int(round((n - center_n + world_half) / res))
    gj = int(round((e - center_e + world_half) / res))
    if 0 <= gi < size_w and 0 <= gj < size_w:
        return (gi, gj)
    return None


# ---------------------------- A* ----------------------------

def astar_2d(grid: np.ndarray, start_ij: tuple, goal_ij: tuple, allow_diagonal: bool = False):
    """
    A* on 2D grid. grid[i,j] == 1 is obstacle, 0 is free.
    start_ij, goal_ij are (i, j). Returns list of (i, j) from start to goal, or [] if no path.
    """
    rows, cols = grid.shape
    si, sj = start_ij
    gi, gj = goal_ij

    if grid[si, sj] != 0:
        return []
    if grid[gi, gj] != 0:
        # goal in obstacle: try nearest free (optional); here we still run A* and may fail
        pass

    if allow_diagonal:
        neighbors = [(-1,-1),(-1,0),(-1,1),(0,-1),(0,1),(1,-1),(1,0),(1,1)]
    else:
        neighbors = [(0, 1), (1, 0), (0, -1), (-1, 0)]

    def heuristic(a, b):
        di, dj = abs(a[0] - b[0]), abs(a[1] - b[1])
        if allow_diagonal:
            return max(di, dj) + (math.sqrt(2) - 1) * min(di, dj)
        return di + dj

    # (f, counter, (i, j), parent)
    open_heap = []
    counter = 0
    g_score = {(si, sj): 0}
    parent = {}
    heapq.heappush(open_heap, (heuristic((si, sj), (gi, gj)), counter, (si, sj)))
    counter += 1

    while open_heap:
        _, _, (ci, cj) = heapq.heappop(open_heap)
        if (ci, cj) == (gi, gj):
            path = []
            cur = (ci, cj)
            while cur in parent:
                path.append(cur)
                cur = parent[cur]
            path.append((si, sj))
            path.reverse()
            return path

        for di, dj in neighbors:
            ni, nj = ci + di, cj + dj
            if ni < 0 or ni >= rows or nj < 0 or nj >= cols:
                continue
            if grid[ni, nj] != 0:
                continue
            step = 1.0 if di == 0 or dj == 0 else math.sqrt(2)
            tg = g_score[(ci, cj)] + step
            if (ni, nj) not in g_score or tg < g_score[(ni, nj)]:
                g_score[(ni, nj)] = tg
                parent[(ni, nj)] = (ci, cj)
                f = tg + heuristic((ni, nj), (gi, gj))
                heapq.heappush(open_heap, (f, counter, (ni, nj)))
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
        # Local planning window (centered on drone); world grid is persistent and larger
        self.declare_parameter('grid_half_size_m', 20.0)
        self.declare_parameter('grid_resolution_m', 0.3)
        self.declare_parameter('world_grid_half_size_m', 30.0)  # persistent map: ±half from midpoint(home,target)
        # Depth: 1 m range, 120° FOV
        self.declare_parameter('depth_max_m', 1.0)
        self.declare_parameter('depth_fov_deg', 120.0)
        self.declare_parameter('min_valid_depth_m', 0.2)  # ignore depth < this (noise); 1.0 would ignore obstacles < 1m
        self.declare_parameter('cruise_speed_m_s', 1.0)  # reserved; actual speed from PX4 MPC (e.g. MPC_XY_CRUISE)
        self.declare_parameter('waypoint_reach_radius_m', 0.4)
        self.declare_parameter('replan_interval_s', 2.5)
        self.declare_parameter('inflation_cells', 2)  # keep inflation so drone avoids narrow gaps; grid size must allow detour
        self.declare_parameter('debug_obstacle', True)  # log depth/grid/path diagnostics to find obstacle issues
        self.declare_parameter('grid_log_file', os.path.join(_MISSIONS_DIR, 'astar_grid_no_path.log'))
        self.declare_parameter('world_grid_file', os.path.join(_MISSIONS_DIR, 'world_grid.log'))
        self.declare_parameter('obstacle_log_file', os.path.join(_MISSIONS_DIR, 'obstacles_detected.log'))

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
        self.cam_fx = 465.0
        self.cam_fy = 614.0
        self.cam_cx = 320.0
        self.cam_cy = 320.0

        self.path_ned = []           # list of (n, e) in NED
        self.waypoint_idx = 0
        self.last_replan_s = 0.0
        self._depth_diag_ts = 0.0    # throttle depth callback diagnostics
        self.world_grid = None       # persistent occupancy grid in NED (origin=home), filled by depth over time

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
        self.create_subscription(NavSatFix, '/drone/gps', self.on_gps, qos)
        target_topic = str(self.get_parameter('target_gps_topic').value)
        self.create_subscription(NavSatFix, target_topic, self.on_target_gps, 10)
        depth_topic = str(self.get_parameter('depth_topic').value)
        depth_info_topic = str(self.get_parameter('depth_info_topic').value)
        self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.create_subscription(CameraInfo, depth_info_topic, self.depth_info_callback, 10)

        self.timer = self.create_timer(self.dt, self.loop)
        self.get_logger().info(
            "[astar_grid] _MISSIONS_DIR=%s" % _MISSIONS_DIR
        )
        self.get_logger().info(
            "[astar_grid] grid_log_file=%s | world_grid_file=%s"
            % (self.get_parameter('grid_log_file').value, self.get_parameter('world_grid_file').value)
        )
        self.get_logger().info(
            f"[astar_grid] hz={hz} grid_half={self.get_parameter('grid_half_size_m').value}m "
            f"depth_max={self.get_parameter('depth_max_m').value}m fov={self.get_parameter('depth_fov_deg').value}deg "
            f"cruise_speed={self.get_parameter('cruise_speed_m_s').value}m/s | "
            f"stages: WARMUP->ARMING->TAKEOFF->HOVER_STABLE->PLAN_FOLLOW (world_grid/replan only in PLAN_FOLLOW)"
        )

    def on_local_pos(self, msg: VehicleLocalPosition):
        self.xy_valid = bool(msg.xy_valid)
        self.z_valid = bool(msg.z_valid)
        self.px = float(msg.x)   # North
        self.py = float(msg.y)  # East
        self.pz = float(msg.z)  # Down
        self.heading = float(msg.heading)

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
        # Throttled diagnostics
        if self.get_parameter('debug_obstacle').value:
            t = time.time()
            if t - self._depth_diag_ts >= 2.0:
                self._depth_diag_ts = t
                min_d = float(self.get_parameter('min_valid_depth_m').value)
                depth_max = float(self.get_parameter('depth_max_m').value)
                valid = (np.isfinite(self.depth_image) & (self.depth_image >= min_d)
                         & (self.depth_image <= depth_max))
                n_valid = int(np.sum(valid))
                finite = np.isfinite(self.depth_image)
                d_min = float(np.min(self.depth_image[finite])) if np.any(finite) else float('nan')
                d_max = float(np.max(self.depth_image[finite])) if np.any(finite) else float('nan')
                self.get_logger().info(
                    f"[depth] {msg.width}x{msg.height} {msg.encoding} | "
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

    def build_2d_grid_ned(self, inflation_override=None) -> np.ndarray:
        """
        Build 2D occupancy grid (NED North-East plane) centered at (px, py).
        Obstacles from depth: depth_max_m range, 120° FOV; camera forward = body forward (heading).
        Returns grid (size x size), 0 = free, 1 = occupied.
        inflation_override: if set, use this instead of parameter (for NO_PATH retry with 0).
        """
        half = float(self.get_parameter('grid_half_size_m').value)
        res = float(self.get_parameter('grid_resolution_m').value)
        depth_max = float(self.get_parameter('depth_max_m').value)
        min_d = float(self.get_parameter('min_valid_depth_m').value)
        inflate = int(inflation_override if inflation_override is not None else self.get_parameter('inflation_cells').value)
        size = int(2 * half / res + 1)
        size = max(10, min(500, size))
        res_actual = 2 * half / (size - 1) if size > 1 else res
        grid = np.zeros((size, size), dtype=np.uint8)

        center_n, center_e = self.px, self.py
        heading = self.heading
        fx, fy = self.cam_fx, self.cam_fy
        cx, cy = self.cam_cx, self.cam_cy

        if self.depth_image is None or self.depth_image.size == 0:
            if self.get_parameter('debug_obstacle').value:
                self._last_grid_diag = {
                    'depth_used': False, 'n_raw': 0, 'n_after': 0,
                    'grid_size': size, 'res_actual': res_actual,
                }
            return grid

        h, w = self.depth_image.shape
        # Only use upper half of image to ignore ground (lower half sees floor)
        v_max = max(2, h // 2)
        valid_depths = []
        for v in range(0, v_max, 2):
            for u in range(0, w, 2):
                d = self.depth_image[v, u]
                if np.isfinite(d) and min_d <= d < depth_max:
                    valid_depths.append(d)
        if valid_depths and np.std(np.array(valid_depths, dtype=np.float64)) < 0.05:
            # Nearly constant depth (e.g. all 1.0 = no obstacle) -> no obstacles
            if self.get_parameter('debug_obstacle').value:
                self._last_grid_diag = {
                    'depth_used': True, 'n_raw': 0, 'n_after': 0,
                    'grid_size': size, 'res_actual': res_actual,
                }
            return grid
        for v in range(0, v_max, 2):
            for u in range(0, w, 2):
                d = self.depth_image[v, u]
                if not np.isfinite(d) or d < min_d or d >= depth_max:
                    continue
                # Camera: x right, y down, z forward. pinhole: x = (u-cx)*d/fx, y = (v-cy)*d/fy, z = d
                x_c = (u - cx) * d / fx
                y_c = (v - cy) * d / fy
                z_c = d
                # Body NED offset: forward = (cos(h), sin(h)), right = (-sin(h), cos(h))
                # N = z*cos(h) - x*sin(h), E = z*sin(h) + x*cos(h)
                dn = z_c * math.cos(heading) - x_c * math.sin(heading)
                de = z_c * math.sin(heading) + x_c * math.cos(heading)
                n_w = center_n + dn
                e_w = center_e + de
                ij = world_to_grid(n_w, e_w, center_n, center_e, half, res_actual, size)
                if ij is not None:
                    grid[ij[0], ij[1]] = 1

        n_raw = int(np.sum(grid == 1))

        # Inflation: mark neighbors within inflate cells as occupied
        if inflate > 0:
            occupied = np.where(grid == 1)
            for k in range(len(occupied[0])):
                i0, j0 = occupied[0][k], occupied[1][k]
                for di in range(-inflate, inflate + 1):
                    for dj in range(-inflate, inflate + 1):
                        ni, nj = i0 + di, j0 + dj
                        if 0 <= ni < size and 0 <= nj < size:
                            grid[ni, nj] = 1
        n_after = int(np.sum(grid == 1))
        if self.get_parameter('debug_obstacle').value:
            self._last_grid_diag = {
                'depth_used': True, 'n_raw': n_raw, 'n_after': n_after,
                'grid_size': size, 'res_actual': res_actual,
            }
        return grid

    def _ensure_world_grid(self, center_n: float, center_e: float):
        """Allocate persistent world grid centered at (center_n, center_e) if not yet created.
        center_n, center_e = midpoint between home and target (NED)."""
        if self.world_grid is not None:
            return
        res = float(self.get_parameter('grid_resolution_m').value)
        world_half = float(self.get_parameter('world_grid_half_size_m').value)
        size_w = int(2 * world_half / res + 1)
        size_w = max(50, min(1000, size_w))
        self.world_grid = np.zeros((size_w, size_w), dtype=np.uint8)
        self._world_grid_size = size_w
        self._world_grid_half = world_half
        self._world_grid_res = res
        self._world_grid_center_n = center_n
        self._world_grid_center_e = center_e
        self.get_logger().info(
            "[world_grid] persistent map %dm x %dm (res=%.2fm) center=(%.1f,%.1f) -> %dx%d"
            % (2 * world_half, 2 * world_half, res, center_n, center_e, size_w, size_w)
        )

    def update_world_grid_from_depth(self):
        """
        Merge current depth into the persistent world grid (no clear).
        World grid is centered at midpoint(home, target); obstacles accumulate as the drone moves.
        """
        res = self._world_grid_res
        world_half = self._world_grid_half
        size_w = self._world_grid_size
        wc_n, wc_e = self._world_grid_center_n, self._world_grid_center_e
        depth_max = float(self.get_parameter('depth_max_m').value)
        min_d = float(self.get_parameter('min_valid_depth_m').value)
        inflate = int(self.get_parameter('inflation_cells').value)
        center_n, center_e = self.px, self.py
        heading = self.heading
        fx, fy = self.cam_fx, self.cam_fy
        cx, cy = self.cam_cx, self.cam_cy

        if self.depth_image is None or self.depth_image.size == 0:
            return

        h, w = self.depth_image.shape
        # Only use upper half of image to ignore ground (lower half sees floor)
        v_max = max(2, h // 2)
        # Collect valid depths in the sampled region to detect "flat" (no real obstacle) frames
        valid_depths = []
        for v in range(0, v_max, 2):
            for u in range(0, w, 2):
                d = self.depth_image[v, u]
                if np.isfinite(d) and min_d <= d < depth_max:
                    valid_depths.append(d)
        if not valid_depths:
            return
        # If depth is nearly constant (e.g. all 1.0 = "no data" from sensor), skip to avoid fake obstacles
        valid_arr = np.array(valid_depths, dtype=np.float64)
        if np.std(valid_arr) < 0.05:
            return

        new_cells = []
        for v in range(0, v_max, 2):
            for u in range(0, w, 2):
                d = self.depth_image[v, u]
                if not np.isfinite(d) or d < min_d or d >= depth_max:
                    continue
                x_c = (u - cx) * d / fx
                y_c = (v - cy) * d / fy
                z_c = d
                dn = z_c * math.cos(heading) - x_c * math.sin(heading)
                de = z_c * math.sin(heading) + x_c * math.cos(heading)
                n_w = center_n + dn
                e_w = center_e + de
                ij = world_ned_to_world_grid(n_w, e_w, wc_n, wc_e, world_half, res, size_w)
                if ij is not None:
                    self.world_grid[ij[0], ij[1]] = 1
                    new_cells.append(ij)

        if inflate > 0 and new_cells:
            for (i0, j0) in new_cells:
                for di in range(-inflate, inflate + 1):
                    for dj in range(-inflate, inflate + 1):
                        ni, nj = i0 + di, j0 + dj
                        if 0 <= ni < size_w and 0 <= nj < size_w:
                            self.world_grid[ni, nj] = 1

    def get_local_grid(self, center_n: float, center_e: float) -> tuple:
        """
        Extract local planning window from persistent world grid.
        Returns (grid, size, res_actual) for the window centered at (center_n, center_e).
        """
        half = float(self.get_parameter('grid_half_size_m').value)
        res = self._world_grid_res
        world_half = self._world_grid_half
        size_w = self._world_grid_size
        wc_n, wc_e = self._world_grid_center_n, self._world_grid_center_e
        size = int(2 * half / res + 1)
        size = max(10, min(500, size))
        res_actual = 2 * half / (size - 1) if size > 1 else res
        grid = np.zeros((size, size), dtype=np.uint8)
        for i in range(size):
            for j in range(size):
                n = center_n - half + (i + 0.5) * res_actual
                e = center_e - half + (j + 0.5) * res_actual
                wi = int(round((n - wc_n + world_half) / res))
                wj = int(round((e - wc_e + world_half) / res))
                if 0 <= wi < size_w and 0 <= wj < size_w:
                    grid[i, j] = self.world_grid[wi, wj]
        n_obs = int(np.sum(grid == 1))
        if self.get_parameter('debug_obstacle').value:
            self._last_grid_diag = {
                'depth_used': True, 'n_raw': n_obs, 'n_after': n_obs,
                'grid_size': size, 'res_actual': res_actual,
            }
        return grid, size, res_actual

    def _format_grid_lines(self, grid: np.ndarray, size: int, start_ij: tuple, goal_ij: tuple):
        """Return list of text lines for the grid. . = free, # = obstacle, S = start, G = goal."""
        si, sj = start_ij
        gi, gj = goal_ij
        lines = []
        lines.append("--- occupancy grid (i=row=North, j=col=East) ---")
        for i in range(size):
            row = []
            for j in range(size):
                if (i, j) == (si, sj) and (i, j) == (gi, gj):
                    row.append('X')
                elif (i, j) == (si, sj):
                    row.append('S')
                elif (i, j) == (gi, gj):
                    row.append('G')
                else:
                    row.append('#' if grid[i, j] == 1 else '.')
            lines.append("  " + "".join(row))
        lines.append("--- .=free #=obst S=start G=goal ---")
        return lines

    def _write_grid_to_log(self, grid: np.ndarray, size: int, start_ij: tuple, goal_ij: tuple, path_ij: list):
        """Append current grid to log file (every plan). path_ij empty = NO PATH."""
        log_path = str(self.get_parameter('grid_log_file').value).strip()
        if not log_path:
            return
        lines = self._format_grid_lines(grid, size, start_ij, goal_ij)
        try:
            from datetime import datetime
            import os
            with open(log_path, 'a', encoding='utf-8') as f:
                f.write("\n" + "=" * 60 + "\n")
                f.write("%s @ %s\n" % (
                    "NO PATH" if not path_ij else "path_len=%d" % len(path_ij),
                    datetime.now().isoformat(),
                ))
                f.write("start_ij=%s goal_ij=%s\n" % (start_ij, goal_ij))
                f.write("\n".join(lines) + "\n")
            # Log absolute path once so user knows where the file is
            if not getattr(self, '_grid_log_path_logged', False):
                self._grid_log_path_logged = True
                abs_path = os.path.abspath(log_path)
                self.get_logger().info("grid log file: %s" % abs_path)
        except OSError as e:
            self.get_logger().warn("failed to write grid log %s: %s" % (log_path, e))

    def _log_grid_no_path(self, grid: np.ndarray, size: int, start_ij: tuple, goal_ij: tuple):
        """When A* returns NO PATH, print grid to console."""
        for line in self._format_grid_lines(grid, size, start_ij, goal_ij):
            self.get_logger().info("[grid NO PATH] " + line)

    def _write_world_grid_to_file(self, goal_n: float, goal_e: float):
        """Overwrite world grid to file (each replan). . = free, # = obstacle, H = home, T = target."""
        import os
        log_path = str(self.get_parameter('world_grid_file').value).strip()
        if not log_path:
            if not getattr(self, '_world_grid_file_empty_logged', False):
                self._world_grid_file_empty_logged = True
                self.get_logger().warn("world_grid_file is empty, not writing world grid (set param to e.g. world_grid.log)")
            return
        if self.world_grid is None:
            return
        abs_path = os.path.abspath(log_path)
        try:
            from datetime import datetime
            size_w = self.world_grid.shape[0]
            res = getattr(self, '_world_grid_res', 0.0)
            world_half = getattr(self, '_world_grid_half', 0.0)
            wc_n = getattr(self, '_world_grid_center_n', 0.0)
            wc_e = getattr(self, '_world_grid_center_e', 0.0)
            home_ij = world_ned_to_world_grid(0.0, 0.0, wc_n, wc_e, world_half, res, size_w)
            target_ij = world_ned_to_world_grid(goal_n, goal_e, wc_n, wc_e, world_half, res, size_w)
            lines = []
            lines.append("# world_grid %dx%d res=%.3fm center_n=%.2f center_e=%.2f @ %s" % (
                size_w, size_w, res, wc_n, wc_e, datetime.now().isoformat(),
            ))
            lines.append("# home NED=(0,0) target NED=(%.2f,%.2f) | .=free #=obst H=home T=target" % (goal_n, goal_e))
            for i in range(size_w):
                row = []
                for j in range(size_w):
                    if (i, j) == home_ij:
                        row.append('H')
                    elif (i, j) == target_ij:
                        row.append('T')
                    else:
                        row.append('#' if self.world_grid[i, j] == 1 else '.')
                lines.append("".join(row))
            with open(abs_path, 'w', encoding='utf-8') as f:
                f.write("\n".join(lines) + "\n")
            self.get_logger().info("world grid written to %s" % abs_path)
        except OSError as e:
            self.get_logger().warn("failed to write world grid %s: %s" % (abs_path, e))

    def _write_obstacle_detection_log(self):
        """
        When obstacles are detected (world_grid has obstacle cells), append to log file:
        drone position (NED), heading, depth summary, and save /oak/depth/image_rect_raw
        raw data to a .npy file (referenced in the log).
        """
        log_path = str(self.get_parameter('obstacle_log_file').value).strip()
        if not log_path or self.world_grid is None:
            return
        n_obs = int(np.sum(self.world_grid == 1))
        if n_obs == 0:
            return
        if self.depth_image is None or self.depth_image.size == 0:
            return
        depth_shape = self.depth_image.shape
        min_d = float(self.get_parameter('min_valid_depth_m').value)
        d_max = float(self.get_parameter('depth_max_m').value)
        valid = np.isfinite(self.depth_image) & (self.depth_image >= min_d) & (self.depth_image <= d_max)
        depth_valid = int(np.sum(valid))
        depth_min = depth_max_val = depth_mean = float('nan')
        if depth_valid > 0:
            depth_min = float(np.min(self.depth_image[valid]))
            depth_max_val = float(np.max(self.depth_image[valid]))
            depth_mean = float(np.mean(self.depth_image[valid]))
        try:
            from datetime import datetime
            ts = datetime.now().isoformat().replace(':', '-').replace('.', '-')
            log_abs = os.path.abspath(log_path)
            log_dir = os.path.dirname(log_abs)
            depth_basename = "obstacles_depth_%s.npy" % ts
            depth_npy_path = os.path.join(log_dir, depth_basename)
            np.save(depth_npy_path, self.depth_image.astype(np.float32))
            png_basename = "obstacles_depth_%s.png" % ts
            png_path = os.path.join(log_dir, png_basename)
            _save_depth_preview_png(self.depth_image, d_max, png_path)
            with open(log_abs, 'a', encoding='utf-8') as f:
                f.write("\n" + "=" * 60 + "\n")
                f.write("obstacles_detected @ %s | n_obstacles=%d\n" % (datetime.now().isoformat(), n_obs))
                f.write("drone_ned: px=%.3f py=%.3f pz=%.3f heading_rad=%.4f\n" % (
                    self.px, self.py, self.pz, self.heading))
                f.write("depth: shape=%s valid_pixels=%d range=[%.3f, %.3f]m mean=%.3fm\n" % (
                    depth_shape, depth_valid, depth_min, depth_max_val, depth_mean))
                f.write("depth_image_file: %s\n" % depth_basename)
                f.write("depth_preview_png: %s\n" % png_basename)
        except OSError as e:
            self.get_logger().warn("failed to write obstacle log %s: %s" % (log_path, e))

    def plan_path_ned(self) -> list:
        """
        Run A* on local window of persistent world grid. Start = drone, goal = target NED.
        World grid is only updated (merged) with new depth, never cleared; local window moves with drone.
        """
        if self.home_lat is None or self.tgt_lat is None:
            if self.get_parameter('debug_obstacle').value:
                t = getattr(self, '_last_plan_skip_log', 0.0)
                now = self.get_clock().now().nanoseconds * 1e-9
                if now - t >= 5.0:
                    self._last_plan_skip_log = now
                    self.get_logger().info(
                        "[plan] skipped (no world_grid): home=%s target=%s — need /drone/gps and target_gps"
                        % ("ok" if self.home_lat is not None else "None",
                           "ok" if self.tgt_lat is not None else "None")
                    )
            return []
        tgt_alt = self.tgt_alt if self.tgt_alt is not None else self.home_alt
        goal_n, goal_e, _ = gps_to_ned(
            self.tgt_lat, self.tgt_lon, tgt_alt,
            self.home_lat, self.home_lon, self.home_alt,
        )
        center_n, center_e = self.px, self.py
        mid_n, mid_e = goal_n * 0.5, goal_e * 0.5  # midpoint(home, target); home NED = (0,0)

        self._ensure_world_grid(mid_n, mid_e)
        # Only merge depth into world grid when at or above min_nav_alt (avoid marking ground as obstacle at low altitude)
        min_nav = float(self.get_parameter('min_nav_alt_m').value)
        if abs(self.pz) >= min_nav:
            self.update_world_grid_from_depth()
            self._write_world_grid_to_file(goal_n, goal_e)
            self._write_obstacle_detection_log()
        # Extract local planning window centered on drone
        grid, size, res_actual = self.get_local_grid(center_n, center_e)
        half = float(self.get_parameter('grid_half_size_m').value)

        start_ij = (size // 2, size // 2)  # drone at center of local window
        goal_ij = world_to_grid(goal_n, goal_e, center_n, center_e, half, res_actual, size)
        if goal_ij is None:
            gi = int((goal_n - center_n + half) / res_actual)
            gj = int((goal_e - center_e + half) / res_actual)
            gi = max(0, min(size - 1, gi))
            gj = max(0, min(size - 1, gj))
            goal_ij = (gi, gj)

        # Ensure start (and neighbors) are free so the drone can move e.g. backward
        si, sj = start_ij
        for di in range(-1, 2):
            for dj in range(-1, 2):
                ni, nj = si + di, sj + dj
                if 0 <= ni < grid.shape[0] and 0 <= nj < grid.shape[1]:
                    grid[ni, nj] = 0
        path_ij = astar_2d(grid, start_ij, goal_ij, allow_diagonal=True)

        if self.get_parameter('debug_obstacle').value:
            diag = getattr(self, '_last_grid_diag', None)
            if diag:
                self.get_logger().info(
                    f"[grid] depth_used={diag['depth_used']} grid={diag['grid_size']}x{diag['grid_size']} "
                    f"res={diag['res_actual']:.3f}m | obstacles raw={diag['n_raw']} after_inflate={diag['n_after']}"
                )
            self.get_logger().info(
                f"[plan] start_ij={start_ij} goal_ij={goal_ij} | "
                f"A* {'path_len=' + str(len(path_ij)) if path_ij else 'NO PATH'}"
            )

        # Always append current grid to log file (every replan)
        self._write_grid_to_log(grid, size, start_ij, goal_ij, path_ij if path_ij else [])

        if not path_ij:
            if self.get_parameter('debug_obstacle').value:
                self._log_grid_no_path(grid, size, start_ij, goal_ij)
            return []

        path_ned = []
        for (i, j) in path_ij:
            n, e = grid_to_world(i, j, center_n, center_e, half, res_actual)
            path_ned.append((n, e))
        return path_ned

    def publish_offboard_mode(self):
        m = OffboardControlMode()
        m.timestamp = micros(self)
        m.position = True
        m.velocity = False
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        self.pub_offboard.publish(m)

    def publish_path_ned(self, path_ned: list, zD: float):
        """Publish path as PoseArray (NED: x=north, y=east, z=down) for Unity visualization."""
        if not path_ned:
            return
        msg = PoseArray()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'world'
        for (n, e) in path_ned:
            p = Pose()
            p.position = Point(x=float(n), y=float(e), z=float(zD))
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
                self.waypoint_idx = 0
                self.last_replan_s = t_s
                if self.path_ned:
                    self.get_logger().info(f"[plan] A* path length={len(self.path_ned)}")
                    self.publish_path_ned(self.path_ned, zD_goal)

            if not self.path_ned:
                self.publish_sp(self.px, self.py, zD_goal, self.heading)
                return

            # Advance waypoint if close enough
            while self.waypoint_idx < len(self.path_ned):
                wn, we = self.path_ned[self.waypoint_idx]
                dist = math.hypot(wn - self.px, we - self.py)
                if dist <= reach_radius and self.waypoint_idx < len(self.path_ned) - 1:
                    self.waypoint_idx += 1
                else:
                    break

            if self.waypoint_idx >= len(self.path_ned):
                self.publish_sp(self.px, self.py, zD_goal, self.heading)
                return

            wn, we = self.path_ned[self.waypoint_idx]
            # Send current waypoint as setpoint so the drone actually flies there (PX4 limits speed via MPC).
            # Previously we sent setpoint only step_max ahead each cycle, which made the drone mostly turn.
            yaw = radians_wrap(math.atan2(we - self.py, wn - self.px))
            self.publish_sp(wn, we, zD_goal, yaw)
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
