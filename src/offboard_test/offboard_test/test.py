import time

import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint


class OffboardSquareMission(Node):
    def __init__(self):
        super().__init__('offboard_square_mission')

        self.declare_parameter('altitude', 3.0)
        self.altitude = float(self.get_parameter('altitude').value)

        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.step = 0
        self.start_time = None
        self.offboard_setpoint_counter = 0
        self.offboard_sent = False
        self.arm_sent = False

        self.square = [
            (0.0, 0.0, -self.altitude),
            (5.0, 0.0, -self.altitude),
            (5.0, 5.0, -self.altitude),
            (0.0, 5.0, -self.altitude),
            (0.0, 0.0, -self.altitude),
        ]

    def timer_callback(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # 1) 持续发送 OffboardControlMode
        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = timestamp
        offboard_mode.position = True
        offboard_mode.velocity = False
        offboard_mode.acceleration = False
        offboard_mode.attitude = False
        offboard_mode.body_rate = False
        self.offboard_control_mode_pub.publish(offboard_mode)

        # 2) 持续发送当前位置/目标位置 setpoint
        traj = TrajectorySetpoint()
        traj.timestamp = timestamp
        traj.yaw = 0.0

        if self.step == 0:
            # 先给一个起飞点，作为 Offboard 预热 setpoint
            traj.position = [0.0, 0.0, -self.altitude]
        elif 1 <= self.step <= len(self.square):
            traj.position = list(self.square[self.step - 1])
        else:
            traj.position = [0.0, 0.0, -self.altitude]

        self.trajectory_setpoint_pub.publish(traj)

        # 3) 先连续发送一段时间 setpoint，再切 Offboard
        if self.offboard_setpoint_counter < 20:
            self.offboard_setpoint_counter += 1
            if self.offboard_setpoint_counter == 1:
                self.get_logger().info('Pre-sending offboard setpoints...')
            return

        # 4) 切 Offboard（只发一次）
        if not self.offboard_sent:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=6.0
            )
            self.offboard_sent = True
            self.get_logger().info('Switched to Offboard mode')
            return

        # 5) Arm（只发一次）
        if not self.arm_sent:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0
            )
            self.arm_sent = True
            self.start_time = time.time()
            self.step = 1
            self.get_logger().info('Vehicle armed')
            return

        # 6) 方形航点，每个点停 5 秒
        if 1 <= self.step <= len(self.square):
            if time.time() - self.start_time > 5.0:
                self.get_logger().info(f'Reached waypoint {self.step}')
                self.step += 1
                self.start_time = time.time()

        # 7) 最后降落
        elif self.step == len(self.square) + 1:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info('Landing...')
            self.step += 1

    def publish_vehicle_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.timestamp = int(self.get_clock().now().nanoseconds / 1000)
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.command = int(command)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        self.vehicle_command_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = OffboardSquareMission()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()