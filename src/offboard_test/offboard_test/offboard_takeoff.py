import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleOdometry,
)


class OffboardTakeoff(Node):
    """
    持续发布 Offboard 位置设定点；订阅 /fmu/out/vehicle_odometry。
    首次有效里程计时锁定参考点 (n0,e0,d0)；固定目标 NED：
      [n0+北偏, e0+东偏, d0 - takeoff_height]（升高 = Down 减小）。
    可选：先发若干周期设定点后自动切 Offboard 并解锁（见参数 auto_arm、arm_after_cycles）。

    TrajectorySetpoint 里未用的速度/加速度须为 NaN（勿默认 0），否则 PX4 可能把速度前馈当 0，表现为不爬升。
    """

    _NAN3 = (math.nan, math.nan, math.nan)

    def __init__(self):
        super().__init__('offboard_takeoff')

        qos_be = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=10,
        )

        self.declare_parameter('takeoff_height', 15.0)
        self.declare_parameter('odom_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('takeoff_offset_east_m', 0.0)
        self.declare_parameter('takeoff_offset_north_m', 0.0)
        self.declare_parameter('odom_out_log_interval_sec', 3.0)
        self.declare_parameter('auto_arm', True)
        self.declare_parameter('arm_after_cycles', 10)

        self.takeoff_height = float(self.get_parameter('takeoff_height').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.takeoff_offset_east_m = float(self.get_parameter('takeoff_offset_east_m').value)
        self.takeoff_offset_north_m = float(self.get_parameter('takeoff_offset_north_m').value)
        self._odom_log_interval = float(self.get_parameter('odom_out_log_interval_sec').value)
        self.auto_arm = bool(self.get_parameter('auto_arm').value)
        self.arm_after_cycles = int(self.get_parameter('arm_after_cycles').value)

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', qos_be)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', qos_be)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', qos_be)

        self.timer = self.create_timer(0.05, self.timer_callback)
        self.offboard_setpoint_counter = 0
        self.arm_sent = False

        self.have_odom = False
        self.n = 0.0
        self.e = 0.0
        self.d = 0.0
        self._have_ref = False
        self._ref_n = 0.0
        self._ref_e = 0.0
        self._ref_d = 0.0
        self._last_odom_msg = None

        self.create_subscription(
            VehicleOdometry,
            self.odom_topic,
            self.on_odom,
            qos_be,
        )

        self.odom_log_timer = self.create_timer(
            self._odom_log_interval, self.odom_log_callback
        )

        self.get_logger().info(
            'offboard_takeoff: 20Hz setpoints, auto_arm=%s (after %d cyc), odom_log=%.1fs'
            % (self.auto_arm, self.arm_after_cycles, self._odom_log_interval)
        )

    def odom_log_callback(self):
        if not self.have_odom or self._last_odom_msg is None:
            return
        m = self._last_odom_msg
        self.get_logger().info(
            '%s: ts=%s ts_sample=%s pose_frame=%s vel_frame=%s '
            'pos_NED=[%.3f, %.3f, %.3f] vel=[%.3f, %.3f, %.3f] '
            'q_wxyz=[%.4f, %.4f, %.4f, %.4f] reset=%s quality=%s' % (
                self.odom_topic,
                m.timestamp,
                m.timestamp_sample,
                m.pose_frame,
                m.velocity_frame,
                float(m.position[0]),
                float(m.position[1]),
                float(m.position[2]),
                float(m.velocity[0]),
                float(m.velocity[1]),
                float(m.velocity[2]),
                float(m.q[0]),
                float(m.q[1]),
                float(m.q[2]),
                float(m.q[3]),
                m.reset_counter,
                m.quality,
            )
        )

    def timer_callback(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = timestamp
        offboard_mode.position = True
        self.offboard_control_mode_pub.publish(offboard_mode)

        traj = TrajectorySetpoint()
        traj.timestamp = timestamp
        traj.velocity = list(self._NAN3)
        traj.acceleration = list(self._NAN3)
        traj.jerk = list(self._NAN3)
        traj.yawspeed = math.nan
        if not self.have_odom:
            traj.position = [0.0, 0.0, -self.takeoff_height]
            # self.get_logger().info('No odom received, using takeoff height')
        else:
            target_n = self._ref_n + self.takeoff_offset_north_m
            target_e = self._ref_e + self.takeoff_offset_east_m
            target_d = self._ref_d - self.takeoff_height
            traj.position = [float(target_n), float(target_e), float(target_d)]
            # self.get_logger().info(f'Target position: {traj.position}')
            # self.get_logger().info(f'Target height (Down, fixed ref): {target_d}')
        traj.yaw = 0.0
        self.trajectory_setpoint_pub.publish(traj)

        if (
            self.auto_arm
            and self.have_odom
            and self._have_ref
            and not self.arm_sent
            and self.offboard_setpoint_counter >= self.arm_after_cycles
        ):
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=6.0,
            )
            self.get_logger().info('VehicleCommand: Offboard mode')
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0,
            )
            self.get_logger().info('VehicleCommand: ARM')
            self.arm_sent = True

        self.offboard_setpoint_counter += 1

    def publish_vehicle_command(self, command, **kwargs):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(kwargs.get('param1', 0.0))
        msg.param2 = float(kwargs.get('param2', 0.0))
        msg.param7 = float(kwargs.get('param7', 0.0))
        msg.command = command
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)

    def on_odom(self, msg: VehicleOdometry):
        try:
            self.n = float(msg.position[0])
            self.e = float(msg.position[1])
            self.d = float(msg.position[2])
            self.have_odom = True
        except Exception:
            self.have_odom = False
            return

        if not self._have_ref:
            self._ref_n = self.n
            self._ref_e = self.e
            self._ref_d = self.d
            self._have_ref = True
            self.get_logger().info(
                'Locked NED reference [n,e,d]=[%.4f, %.4f, %.4f]; '
                'fixed target Down=%.4f (%.2f m above ref in NED sense)'
                % (
                    self._ref_n,
                    self._ref_e,
                    self._ref_d,
                    self._ref_d - self.takeoff_height,
                    self.takeoff_height,
                )
            )

        self._last_odom_msg = msg


def main(args=None):
    rclpy.init(args=args)
    node = OffboardTakeoff()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
