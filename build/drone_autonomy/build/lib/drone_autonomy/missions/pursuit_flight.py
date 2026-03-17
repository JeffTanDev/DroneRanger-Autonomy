#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Minimal PX4 offboard script:
- Warm up (stream setpoints)
- Arm + switch to OFFBOARD
- Take off to a fixed altitude and hold at (N=0,E=0)
- When a /testTarget (PoseStamped, ENU) arrives, fly above it and hold

Topics (PX4 uXRCE-DDS, ROS 2):
  pub: /fmu/in/offboard_control_mode     (px4_msgs/OffboardControlMode)
  pub: /fmu/in/trajectory_setpoint       (px4_msgs/TrajectorySetpoint)
  pub: /fmu/in/vehicle_command           (px4_msgs/VehicleCommand)
  sub: /testTarget                       (geometry_msgs/PoseStamped)  -- ENU
  sub: /fmu/out/vehicle_local_position   (px4_msgs/VehicleLocalPosition) -- for validity

Author: you
"""

import math, time
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
)


def quat_to_yaw(q):
    """Extract yaw in ENU (around +Z)."""
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                      1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SimpleTakeoffThenGoto(Node):
    """
    Ultra-simple offboard node:
      warmup -> arm_offboard -> takeoff_hold -> goto_target_hold
    No continuous tracking; just fly over the first received target and hold.
    """

    def __init__(self):
        super().__init__('simple_takeoff_then_goto')

        # ---------- parameters (tune or override via --ros-args) ----------
        self.declare_parameter('target_topic', '/testTarget')
        self.declare_parameter('publish_hz', 40.0)
        self.declare_parameter('takeoff_alt_m', 2.5)      # Up (positive)
        self.declare_parameter('above_target_m', 3.0)     # stay this high above target
        self.declare_parameter('fixed_yaw_deg', 0.0)      # keep a fixed yaw
        self.declare_parameter('warmup_s', 1.0)           # pre-offboard streaming
        self.declare_parameter('climb_s', 3.0)            # time to climb before moving
        self.declare_parameter('target_timeout_s', 5.0)   # if no target within this, keep holding

        gp = self.get_parameter
        self.target_topic     = gp('target_topic').get_parameter_value().string_value
        self.publish_hz       = float(gp('publish_hz').value)
        self.takeoff_alt_m    = float(gp('takeoff_alt_m').value)
        self.above_target_m   = float(gp('above_target_m').value)
        self.fixed_yaw_deg    = float(gp('fixed_yaw_deg').value)
        self.warmup_s         = float(gp('warmup_s').value)
        self.climb_s          = float(gp('climb_s').value)
        self.target_timeout_s = float(gp('target_timeout_s').value)

        # ---------- state ----------
        self.dt = 1.0 / max(1.0, self.publish_hz)
        self.stage = 'warmup'            # warmup -> arm_offboard -> takeoff_hold -> goto_target
        self.t0 = time.time()
        self.last_target = None
        self.last_target_time = 0.0
        self.have_local_pos = False

        # ---------- pubs/subs ----------
        self.sub_target = self.create_subscription(PoseStamped, self.target_topic, self.on_target, 10)
        self.sub_lpos   = self.create_subscription(VehicleLocalPosition, '/fmu/out/vehicle_local_position', self.on_lpos, 10)

        self.pub_off = self.create_publisher(OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.pub_sp  = self.create_publisher(TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.pub_cmd = self.create_publisher(VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.timer = self.create_timer(self.dt, self.on_timer)

        self.get_logger().info(
            f"[node] up | hz={self.publish_hz} | takeoff_alt={self.takeoff_alt_m} | target={self.target_topic}"
        )

    # ---------- helpers ----------
    def micros(self) -> int:
        """PX4 expects timestamps in microseconds."""
        return int(self.get_clock().now().nanoseconds // 1000)

    def publish_offboard_mode(self):
        m = OffboardControlMode()
        m.timestamp = self.micros()
        # position-only control
        m.position = True
        m.velocity = False
        m.acceleration = False
        m.attitude = False
        m.body_rate = False
        self.pub_off.publish(m)

    def publish_sp(self, sp: TrajectorySetpoint):
        sp.timestamp = self.micros()
        self.pub_sp.publish(sp)

    def cmd(self, command, p1=0.0, p2=0.0):
        c = VehicleCommand()
        c.timestamp = self.micros()
        c.command = command
        c.param1 = float(p1); c.param2 = float(p2)
        c.target_system = 1; c.target_component = 1
        c.source_system = 1; c.source_component = 1
        c.from_external = True
        self.pub_cmd.publish(c)

    def arm(self):
        self.cmd(VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0)
        self.get_logger().info("[ARM] sent")

    def offboard(self):
        # base=1, custom=6 (OFFBOARD)
        self.cmd(VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0)
        self.get_logger().info("[MODE] OFFBOARD sent")

    # ---------- callbacks ----------
    def on_target(self, msg: PoseStamped):
        self.last_target = msg
        self.last_target_time = time.time()

    def on_lpos(self, msg: VehicleLocalPosition):
        # require local position validity for safety
        self.have_local_pos = bool(msg.xy_valid) and bool(msg.z_valid)

    # ---------- setpoint factories ----------
    def sp_hold_takeoff(self) -> TrajectorySetpoint:
        """Hold at (N=0,E=0,Up=takeoff_alt) -> D = -Up."""
        sp = TrajectorySetpoint()
        sp.position = [0.0, 0.0, -float(self.takeoff_alt_m)]  # N, E, D
        sp.yaw = math.radians(self.fixed_yaw_deg)
        return sp

    def sp_over_target(self, tgt: PoseStamped) -> TrajectorySetpoint:
        """
        Convert ENU target (E = x, N = y, U = z) -> NED setpoint:
          xN = N, yE = E, zD = -(U + above_target_m)
        Keep yaw fixed (simplest).
        """
        E = float(tgt.pose.position.x)
        N = float(tgt.pose.position.y)
        U = float(tgt.pose.position.z)

        xN, yE, zD = N, E, -(U + self.above_target_m)

        sp = TrajectorySetpoint()
        sp.position = [xN, yE, zD]
        sp.yaw = math.radians(self.fixed_yaw_deg)
        return sp

    # ---------- main loop ----------
    def on_timer(self):
        now = time.time()

        # 1) always stream OffboardControlMode + some setpoint
        self.publish_offboard_mode()

        if self.stage == 'warmup':
            # stream neutral hold, helps PX4 accept OFFBOARD
            self.publish_sp(self.sp_hold_takeoff())
            # wait a bit and ensure local position valid if available
            if (now - self.t0) >= self.warmup_s and (self.have_local_pos or (now - self.t0) > self.warmup_s + 2.0):
                self.stage = 'arm_offboard'
                self.t0 = now
                self.get_logger().info("[stage] warmup -> arm_offboard")

        elif self.stage == 'arm_offboard':
            self.arm()
            self.offboard()
            self.stage = 'takeoff_hold'
            self.t0 = now
            self.get_logger().info("[stage] -> takeoff_hold")

        elif self.stage == 'takeoff_hold':
            # climb and hold at takeoff altitude for a short time
            self.publish_sp(self.sp_hold_takeoff())
            if (now - self.t0) >= self.climb_s:
                self.stage = 'goto_target'
                self.t0 = now
                self.get_logger().info("[stage] -> goto_target")

        elif self.stage == 'goto_target':
            # if a fresh target exists, go above it; otherwise keep holding
            fresh = (self.last_target is not None) and ((now - self.last_target_time) <= self.target_timeout_s)
            if fresh:
                self.publish_sp(self.sp_over_target(self.last_target))
            else:
                self.publish_sp(self.sp_hold_takeoff())
                # optional: log at low rate
                if int(now) % 2 == 0:
                    self.get_logger().warn("[goto_target] no fresh /testTarget yet; holding")

def main(args=None):
    rclpy.init(args=args)
    node = SimpleTakeoffThenGoto()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()