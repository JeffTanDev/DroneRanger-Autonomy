#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Haoran Unity Pursuit / Avoidance Template

这个文件是专门给你和同学在 Unity 模拟环境下开发 / 测试避障&飞控算法用的。

特点：
- 已经帮你把 PX4 Offboard、Unity 相关话题、状态机等“框架”搭好；
- 你只需要在标记好的“算法区域”里改代码，就能在 Unity 场景中直接测试。

接口（与现有节点保持一致，方便复用）：
- pub: /fmu/in/offboard_control_mode  (px4_msgs/OffboardControlMode)
- pub: /fmu/in/trajectory_setpoint    (px4_msgs/TrajectorySetpoint)
- pub: /fmu/in/vehicle_command        (px4_msgs/VehicleCommand)
- sub: /fmu/out/vehicle_local_position_v1 (px4_msgs/VehicleLocalPosition)
- sub: /drone/gps                        (sensor_msgs/NavSatFix)        -- current GPS, 用来设 home
- sub: /static_cam/target_gps            (sensor_msgs/NavSatFix)        -- Unity 静态相机生成的目标点
- sub: /oak/depth/image_rect_raw         (sensor_msgs/Image)            -- Unity OAK 深度图，用于避障
- sub: /oak/depth/camera_info            (sensor_msgs/CameraInfo)

你主要会修改的方法：
- _compute_goal_ned(...)：从 GPS 计算目标 NED（如果你要自定义目标策略，可以改这里）
- _compute_setpoint_from_goal(...)：核心“算法区域”，从当前状态 + 目标位置 + 深度图，算出下一步 setpoint。
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


def gps_to_ned(
    lat_deg: float,
    lon_deg: float,
    alt_m: float,
    home_lat_deg: float,
    home_lon_deg: float,
    home_alt_m: float,
):
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
    """Current time in microseconds for PX4 timestamps."""
    return node.get_clock().now().nanoseconds // 1000


# ---------------------------- State Machine ----------------------------


class Stage(Enum):
    WARMUP = 0
    ARMING = 1
    TAKEOFF = 2
    HOVER_STABLE = 3
    EXECUTE = 4  # 你的算法真正开始工作的阶段


# ---------------------------- Main Node ----------------------------


class HaoranUnityPursuit(Node):
    def __init__(self):
        super().__init__("haoran_unity_pursuit")

        # ---------- Parameters ----------
        self.declare_parameter("hz", 30.0)
        self.declare_parameter("takeoff_alt_m", 5.0)
        self.declare_parameter("min_nav_alt_m", 2.0)
        self.declare_parameter("stable_hover_s", 1.0)
        self.declare_parameter("target_gps_topic", "/static_cam/target_gps")

        # Depth / avoidance 相关参数（可以按需增加）
        self.declare_parameter("depth_topic", "/oak/depth/image_rect_raw")
        self.declare_parameter("depth_info_topic", "/oak/depth/camera_info")
        self.declare_parameter("front_distance_m", 1.0)
        self.declare_parameter("roi_ratio", 0.25)
        self.declare_parameter("roi_center_x_ratio", 0.5)
        self.declare_parameter("roi_center_y_ratio", 0.4)
        self.declare_parameter("depth_percentile", 5.0)
        self.declare_parameter("min_valid_depth_m", 0.2)
        self.declare_parameter("max_valid_depth_m", 50.0)
        self.declare_parameter("depth_timeout_s", 1.0)
        self.declare_parameter("depth_log_interval_s", 1.0)
        # VFH+ / 扇区直方图 + 速度门控 + yaw 扫描
        self.declare_parameter("hfov_deg", 80.0)              # 相机水平视场角（度）
        self.declare_parameter("num_sectors", 9)              # 在 [-hfov/2, +hfov/2] 之间划分的扇区数
        self.declare_parameter("min_clear_dist_m", 1.5)       # 被认为“安全”的最小清障距离
        self.declare_parameter("stop_dist_m", 0.8)            # 小于此距离强制停车
        self.declare_parameter("max_speed_m_s", 3.0)          # 最大平面速度（用于 gating）
        self.declare_parameter("min_speed_m_s", 0.5)          # 最小前进速度（安全但不太慢）
        self.declare_parameter("yaw_scan_speed_deg_s", 30.0)  # 扫描时的 yaw 角速度（度/秒）
        # 当深度不可用/无法选出安全扇区时，是否退化为“直线追目标”（方便先跑通测试）
        self.declare_parameter("fallback_to_goal_when_no_depth", True)
        self.declare_parameter("fallback_speed_m_s", 1.0)
        self.declare_parameter("debug_log_interval_s", 1.0)

        hz = float(self.get_parameter("hz").value)
        self.dt = 1.0 / max(hz, 1.0)

        # ---------- Runtime State ----------
        self.stage = Stage.WARMUP
        self.stage_enter_ts = self.get_clock().now().nanoseconds

        self.armed = False
        self.offboard_ok = False

        self.xy_valid = False
        self.z_valid = False

        # latest local position (NED)
        self.px = 0.0
        self.py = 0.0
        self.pz = 0.0
        self.heading = 0.0

        # GPS
        self.home_lat = None
        self.home_lon = None
        self.home_alt = None
        self.tgt_lat = None
        self.tgt_lon = None
        self.tgt_alt = None

        # Depth image state
        self.depth_image = None
        self.depth_stamp_s = None
        self.depth_width = None
        self.depth_height = None
        self.last_depth_log_s = 0.0
        self.last_debug_log_s = 0.0

        # ---------- Publishers ----------
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.pub_offboard = self.create_publisher(
            OffboardControlMode, "/fmu/in/offboard_control_mode", qos
        )
        self.pub_sp = self.create_publisher(
            TrajectorySetpoint, "/fmu/in/trajectory_setpoint", qos
        )
        self.pub_cmd = self.create_publisher(
            VehicleCommand, "/fmu/in/vehicle_command", qos
        )

        # ---------- Subscribers ----------
        self.create_subscription(
            VehicleLocalPosition,
            "/fmu/out/vehicle_local_position_v1",
            self.on_local_pos,
            qos,
        )
        self.create_subscription(NavSatFix, "/drone/gps", self.on_gps, qos)

        target_topic = str(self.get_parameter("target_gps_topic").value)
        self.create_subscription(NavSatFix, target_topic, self.on_target_gps, 10)

        depth_topic = str(self.get_parameter("depth_topic").value)
        depth_info_topic = str(self.get_parameter("depth_info_topic").value)
        self.create_subscription(Image, depth_topic, self.depth_callback, 10)
        self.create_subscription(CameraInfo, depth_info_topic, self.depth_info_callback, 10)

        # ---------- Timer Loop ----------
        self.timer = self.create_timer(self.dt, self.loop)

        self.get_logger().info(
            f"[haoran_unity_pursuit] up | hz={hz:.1f} | takeoff={self.get_parameter('takeoff_alt_m').value:.2f}m "
            f"| min_nav_alt={self.get_parameter('min_nav_alt_m').value:.2f}m "
            f"| target_gps={target_topic} | depth={depth_topic}"
        )

    # ---------------- ROS Callbacks ----------------

    def on_local_pos(self, msg: VehicleLocalPosition):
        self.xy_valid = bool(msg.xy_valid)
        self.z_valid = bool(msg.z_valid)
        self.px = float(msg.x)
        self.py = float(msg.y)
        self.pz = float(msg.z)
        self.heading = float(msg.heading)

    def on_gps(self, msg: NavSatFix):
        lat = float(msg.latitude)
        lon = float(msg.longitude)
        alt = float(msg.altitude)
        if self.home_lat is None:
            self.home_lat = lat
            self.home_lon = lon
            self.home_alt = alt
        # 如果需要用当前 GPS，也可以在这里存 cur_lat / cur_lon / cur_alt

    def on_target_gps(self, msg: NavSatFix):
        self.tgt_lat = float(msg.latitude)
        self.tgt_lon = float(msg.longitude)
        self.tgt_alt = float(msg.altitude)

    def depth_callback(self, msg: Image):
        self.depth_stamp_s = time.time()
        self.depth_width = msg.width
        self.depth_height = msg.height

        if msg.encoding == "32FC1":
            depth = np.frombuffer(msg.data, dtype=np.float32)
        elif msg.encoding == "16UC1":
            depth = (
                np.frombuffer(msg.data, dtype=np.uint16).astype(np.float32) / 1000.0
            )
        else:
            self.get_logger().warn(f"[depth] unsupported encoding: {msg.encoding}")
            self.depth_image = None
            return

        if depth.size != msg.width * msg.height:
            self.get_logger().warn("[depth] image size mismatch")
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
        m.position = True
        m.velocity = False
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        self.pub_offboard.publish(m)

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

    # ---------------- Depth helpers ----------------

    def _front_nearest_depth(self):
        """计算前方 ROI 的最近距离（米），如果不可用返回 None。"""
        if self.depth_image is None:
            return None
        now = time.time()
        timeout = float(self.get_parameter("depth_timeout_s").value)
        if self.depth_stamp_s is None or now - self.depth_stamp_s > timeout:
            return None

        front_dist = float(self.get_parameter("front_distance_m").value)
        roi_ratio = float(self.get_parameter("roi_ratio").value)
        cx_ratio = float(self.get_parameter("roi_center_x_ratio").value)
        cy_ratio = float(self.get_parameter("roi_center_y_ratio").value)
        depth_percentile = float(self.get_parameter("depth_percentile").value)
        min_valid = float(self.get_parameter("min_valid_depth_m").value)
        max_valid = float(self.get_parameter("max_valid_depth_m").value)

        h, w = self.depth_image.shape
        roi_w = max(1, int(w * roi_ratio))
        roi_h = max(1, int(h * roi_ratio))
        cx = int(w * cx_ratio)
        cy = int(h * cy_ratio)
        x0 = min(max(0, cx - roi_w // 2), w - roi_w)
        y0 = min(max(0, cy - roi_h // 2), h - roi_h)
        roi = self.depth_image[y0 : y0 + roi_h, x0 : x0 + roi_w]

        roi = roi[np.isfinite(roi)]
        roi = roi[(roi >= min_valid) & (roi <= max_valid)]
        if roi.size == 0:
            return None

        nearest = float(np.percentile(roi, depth_percentile))

        t = time.time()
        log_interval = float(self.get_parameter("depth_log_interval_s").value)
        if t - self.last_depth_log_s >= log_interval:
            self.last_depth_log_s = t
            self.get_logger().info(
                f"[depth] nearest={nearest:.2f}m threshold={front_dist:.2f}m"
            )
        return nearest

    def _depth_sectors(self):
        """
        将当前深度图在水平 FOV 内划分为若干扇区，返回：
        - angles: 每个扇区中心相对于相机前向的角度（弧度，左负右正）
        - d_min:  每个扇区的最近障碍距离（米，如无有效数据则为 None）
        """
        if self.depth_image is None:
            return None, None

        now = time.time()
        timeout = float(self.get_parameter("depth_timeout_s").value)
        if self.depth_stamp_s is None or now - self.depth_stamp_s > timeout:
            return None, None

        h, w = self.depth_image.shape
        num = int(self.get_parameter("num_sectors").value)
        num = max(3, num)

        hfov_deg = float(self.get_parameter("hfov_deg").value)
        hfov_rad = math.radians(hfov_deg)

        min_valid = float(self.get_parameter("min_valid_depth_m").value)
        max_valid = float(self.get_parameter("max_valid_depth_m").value)
        depth_percentile = float(self.get_parameter("depth_percentile").value)

        # 只用中间一部分竖直区域，减少地面/天空干扰
        roi_h_ratio = float(self.get_parameter("roi_ratio").value)
        cy_ratio = float(self.get_parameter("roi_center_y_ratio").value)
        roi_h = max(1, int(h * roi_h_ratio))
        cy = int(h * cy_ratio)
        y0 = min(max(0, cy - roi_h // 2), h - roi_h)
        y1 = y0 + roi_h

        sector_width = w / num
        angles = []
        d_min = []
        for k in range(num):
            x0 = int(k * sector_width)
            x1 = int((k + 1) * sector_width)
            x1 = min(x1, w)
            if x1 <= x0:
                angles.append(0.0)
                d_min.append(None)
                continue

            roi = self.depth_image[y0:y1, x0:x1]
            roi = roi[np.isfinite(roi)]
            roi = roi[(roi >= min_valid) & (roi <= max_valid)]
            if roi.size == 0:
                angles.append(0.0)
                d_min.append(None)
                continue

            nearest = float(np.percentile(roi, depth_percentile))

            # 扇区中心对应的水平角度（简单线性映射）
            center_col = (x0 + x1) * 0.5
            rel = (center_col / w - 0.5)  # [-0.5, 0.5]
            ang = rel * hfov_rad         # [-hfov/2, +hfov/2]

            angles.append(ang)
            d_min.append(nearest)

        return angles, d_min

    def _vfh_choose_direction(self, goal_yaw: float):
        """
        简化版 VFH+：
        - 输入：全局目标航向 goal_yaw（弧度，NED yaw）
        - 使用深度扇区的最近距离作为代价，挑选一个“安全且朝向目标”的扇区。
        返回：
        - best_yaw: 选定的全局 yaw（若无可行扇区则为 None）
        - best_dist: 该扇区最近障碍距离（米，或 None）
        """
        angles, d_min = self._depth_sectors()
        if angles is None or d_min is None:
            return None, None

        min_clear = float(self.get_parameter("min_clear_dist_m").value)
        stop_dist = float(self.get_parameter("stop_dist_m").value)

        best_yaw = None
        best_score = None
        best_dist = None

        for ang, dist in zip(angles, d_min):
            if dist is None:
                continue
            if dist < stop_dist:
                # 明显太近，认为完全不可行
                continue

            # 扇区对应的全局 yaw
            cand_yaw = radians_wrap(self.heading + ang)

            # 角度代价：偏离目标越大，代价越高
            yaw_diff = abs(radians_wrap(cand_yaw - goal_yaw))

            # 距离代价：距离越小，代价越高；>= min_clear 视为“足够安全”
            if dist >= min_clear:
                dist_cost = 0.0
            else:
                dist_cost = (min_clear - dist) / max(min_clear, 1e-3)

            # 总代价（可以以后改权重）
            score = yaw_diff + 2.0 * dist_cost

            if best_score is None or score < best_score:
                best_score = score
                best_yaw = cand_yaw
                best_dist = dist

        return best_yaw, best_dist

    # ---------------- Algorithm Hooks ----------------

    def _compute_goal_ned(self):
        """从 home 和 target GPS 计算目标 NED，返回 (goalN, goalE, goalD) 或 None。"""
        if (
            self.home_lat is None
            or self.home_lon is None
            or self.home_alt is None
            or self.tgt_lat is None
            or self.tgt_lon is None
        ):
            return None

        tgt_alt = self.tgt_alt if self.tgt_alt is not None else self.home_alt
        goalN, goalE, goalD = gps_to_ned(
            self.tgt_lat,
            self.tgt_lon,
            tgt_alt,
            self.home_lat,
            self.home_lon,
            self.home_alt,
        )
        return goalN, goalE, goalD

    def _compute_setpoint_from_goal(self, goalN, goalE, goalD):
        """
        ====== 你主要要改的“算法区域”在这里 ======

        输入：
        - 当前无人机状态：self.px, self.py, self.pz, self.heading （NED）
        - 目标位置：goalN, goalE, goalD （NED）
        - 深度图：self.depth_image（米），可以用 _front_nearest_depth() 做简单前方避障

        输出：
        - 返回一个 tuple: (spN, spE, spD, spYaw)
          表示下一帧要发给 PX4 的 TrajectorySetpoint（位置 + yaw）。

        下面实现的是一个“简化 VFH+ + 速度门控 + 主动 yaw 扫描”的版本：
        - 用深度图在水平方向划分扇区，估计每个扇区的最近障碍距离；
        - 基于目标方向和障碍距离选一个最佳扇区（类似 VFH+）；
        - 用最近距离做速度门控（distance 越大，允许速度越高）；
        - 如果所有扇区都不好，则原地做 yaw 扫描，主动寻找新的安全方向。
        """
        # 1) 计算全局“理想”目标航向
        vN = goalN - self.px
        vE = goalE - self.py
        goal_yaw = radians_wrap(math.atan2(vE, vN))
        goal_dist_xy = math.hypot(vN, vE)

        # 2) 用 VFH+ 风格选择一个安全方向
        best_yaw, best_dist = self._vfh_choose_direction(goal_yaw)

        zD_goal = goalD

        # 3) 如果深度不可用/没有任何可行扇区：
        #    - 默认退化为“直线追目标”（先验证会动、能追目标）
        #    - 如关闭 fallback，则保持原来的 yaw 扫描策略
        if best_yaw is None or best_dist is None:
            fallback = bool(self.get_parameter("fallback_to_goal_when_no_depth").value)
            if fallback:
                fallback_speed = float(self.get_parameter("fallback_speed_m_s").value)
                dt = self.dt
                step_xy = max(0.0, fallback_speed) * dt
                if goal_dist_xy > 1e-3:
                    dirN = vN / goal_dist_xy
                    dirE = vE / goal_dist_xy
                    spN = self.px + dirN * step_xy
                    spE = self.py + dirE * step_xy
                else:
                    spN = self.px
                    spE = self.py
                spD = zD_goal
                spYaw = goal_yaw
                self._debug_throttled(
                    f"[algo] no_depth_or_no_sector -> fallback_to_goal | dist_xy={goal_dist_xy:.2f}m "
                    f"| step={step_xy:.2f}m | yaw={spYaw:.2f}"
                )
                return spN, spE, spD, spYaw

            yaw_scan_speed_deg_s = float(self.get_parameter("yaw_scan_speed_deg_s").value)
            yaw_scan_speed = math.radians(yaw_scan_speed_deg_s)
            dt = self.dt
            spYaw = radians_wrap(self.heading + yaw_scan_speed * dt)
            spN = self.px
            spE = self.py
            spD = zD_goal
            self._debug_throttled("[algo] no_depth_or_no_sector -> yaw_scan_hold")
            return spN, spE, spD, spYaw

        # 4) 根据最近障碍距离和与目标的距离，决定“这一次要给 PX4 的 waypoint”
        #    思路对齐 astar_grid_pursuit：直接给一个位置 setpoint，让 PX4 用 MPC_XY_CRUISE 控制速度，
        #    而不是我们自己每帧只走一个很小的步长。
        max_step_ahead_m = float(self.get_parameter("max_speed_m_s").value) * 3.0
        min_step_ahead_m = float(self.get_parameter("min_speed_m_s").value)
        min_clear = float(self.get_parameter("min_clear_dist_m").value)
        stop_dist = float(self.get_parameter("stop_dist_m").value)

        # 如果前面太近有障碍，直接停在原地（高度仍跟随目标高度）
        if best_dist <= stop_dist:
            spN = self.px
            spE = self.py
            spD = zD_goal
            spYaw = best_yaw
            self._debug_throttled(
                f"[algo] stop | best_dist={best_dist:.2f}m <= stop_dist={stop_dist:.2f}m"
            )
            return spN, spE, spD, spYaw

        # 根据障碍距离和目标距离，确定这次 waypoint 距离当前机体多远
        # 1) 先按障碍距离 gate：离障碍越近，step 越小
        if best_dist < min_clear:
            ratio = (best_dist - stop_dist) / max(min_clear - stop_dist, 1e-3)
            step_from_obstacle = min_step_ahead_m + (max_step_ahead_m - min_step_ahead_m) * max(
                0.0, min(1.0, ratio)
            )
        else:
            step_from_obstacle = max_step_ahead_m

        # 2) 再按目标距离 gate：不能比到目标距离还远
        step_xy = max(min_step_ahead_m, min(step_from_obstacle, goal_dist_xy))

        # 5) 沿着选中的方向给出一个“前方 step_xy 米”的 waypoint，高度跟随目标高度
        dirN = math.cos(best_yaw)
        dirE = math.sin(best_yaw)

        spN = self.px + dirN * step_xy
        spE = self.py + dirE * step_xy
        spD = zD_goal
        spYaw = best_yaw
        self._debug_throttled(
            f"[algo] move | goal_dist_xy={goal_dist_xy:.2f}m | best_dist={best_dist:.2f}m | step_xy={step_xy:.2f}m"
        )
        return spN, spE, spD, spYaw

    def _debug_throttled(self, msg: str):
        """节流输出，避免刷屏。"""
        interval = float(self.get_parameter("debug_log_interval_s").value)
        now = time.time()
        if now - self.last_debug_log_s >= max(0.1, interval):
            self.last_debug_log_s = now
            self.get_logger().info(msg)

    # ---------------- Main Loop ----------------

    def loop(self):
        self.publish_offboard_mode()
        t_ns = self.get_clock().now().nanoseconds

        takeoff_alt = float(self.get_parameter("takeoff_alt_m").value)
        zD_takeoff = -abs(takeoff_alt)

        # --- WARMUP ---
        if self.stage == Stage.WARMUP:
            self.publish_sp(self.px, self.py, zD_takeoff, self.heading)
            if (t_ns - self.stage_enter_ts) * 1e-9 >= 0.5:
                self.arm()
                self.set_offboard()
                self.stage = Stage.ARMING
                self.stage_enter_ts = t_ns
                self.get_logger().info("[stage] WARMUP -> ARMING")
            return

        # --- ARMING ---
        if self.stage == Stage.ARMING:
            self.publish_sp(self.px, self.py, zD_takeoff, self.heading)
            if self.z_valid:
                self.stage = Stage.TAKEOFF
                self.stage_enter_ts = t_ns
                self.get_logger().info("[stage] ARMING -> TAKEOFF")
            return

        # --- TAKEOFF ---
        if self.stage == Stage.TAKEOFF:
            self.publish_sp(self.px, self.py, zD_takeoff, self.heading)
            if abs(self.pz - zD_takeoff) <= 0.25:
                self.stage = Stage.HOVER_STABLE
                self.stage_enter_ts = t_ns
                self.get_logger().info("[stage] TAKEOFF -> HOVER_STABLE")
            return

        # --- HOVER_STABLE ---
        if self.stage == Stage.HOVER_STABLE:
            self.publish_sp(self.px, self.py, zD_takeoff, self.heading)
            stable_s = float(self.get_parameter("stable_hover_s").value)
            min_nav = float(self.get_parameter("min_nav_alt_m").value)
            if (t_ns - self.stage_enter_ts) * 1e-9 >= stable_s and self.xy_valid and self.z_valid:
                if abs(self.pz) >= min_nav:
                    self.stage = Stage.EXECUTE
                    self.stage_enter_ts = t_ns
                    self.get_logger().info("[stage] HOVER_STABLE -> EXECUTE (algorithm on)")
            return

        # --- EXECUTE：你的算法在这里工作 ---
        if self.stage == Stage.EXECUTE:
            goal = self._compute_goal_ned()
            if goal is None:
                # 还没拿到 home / target，就先在当前高度悬停
                self.publish_sp(self.px, self.py, zD_takeoff, self.heading)
                return

            goalN, goalE, goalD = goal
            spN, spE, spD, spYaw = self._compute_setpoint_from_goal(goalN, goalE, goalD)
            self.publish_sp(spN, spE, spD, spYaw)
            return

        # Fallback：保持当前位置
        self.publish_sp(self.px, self.py, self.pz, self.heading)


def main():
    rclpy.init()
    node = HaoranUnityPursuit()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

