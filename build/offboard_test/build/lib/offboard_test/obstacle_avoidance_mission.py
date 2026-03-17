#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from px4_msgs.msg import VehicleCommand, OffboardControlMode, TrajectorySetpoint
import time


def enu_to_ned(x_enu, y_enu, z_enu):
    """
    ENU (Gazebo) → NED (PX4) coordinate transformation
    ENU: x=East, y=North, z=Up
    NED: x=North, y=East, z=Down
    """
    x_ned = y_enu
    y_ned = x_enu
    z_ned = -z_enu
    return (x_ned, y_ned, z_ned)


class OffboardHouseMission(Node):
    def __init__(self):
        super().__init__('offboard_house_mission')

        # === Mission Parameters (ENU) ===
        altitude = 3.0   # meters (above the roof to avoid collision)
        hx, hy = 10.0, 0.0   # House center (East, North)
        half, margin = 2.5, 3.0   # Half side length of the house + safety margin

        # Obstacles
        obs1 = (15.0, 5.0, 0.5)
        obs2 = (18.0, 3.0, 0.5)

        # Create a "corridor" to bypass the east side of the obstacles (SE→NE)
        x_max = max(obs1[0], obs2[0])         # 18
        clearance = 2.0                       # Horizontal safety distance from obstacles
        x_detour = x_max + clearance          # 20
        y_min = min(obs1[1], obs2[1]) - 1.0   # 2
        y_max = max(obs1[1], obs2[1]) + 1.0   # 6
        # Define ENU waypoints
        waypoints_enu = [
            (hx - half - margin, hy - half - margin, altitude),  # SW
            (hx + half + margin, hy - half - margin, altitude),  # SE
            (x_detour, y_min, 2.0),  # First go east to the bottom of the corridor
            (x_detour, y_max, 2.0),  # Go north along the corridor
            (hx + half + margin, hy + half + margin, altitude),  # NE
            (hx - half - margin, hy + half + margin, altitude),  # NW
            (hx - half - margin, hy - half - margin, altitude),  # Back to SW
            (1.0, 0.98, altitude),  # Return to starting point (1,1,0)
        ]

        self.waypoints = [enu_to_ned(*p) for p in waypoints_enu]

        # ROS2 Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10)
        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10)
        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10)

    # Timer
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.step = 0
        self.start_time = None

    def timer_callback(self):
        timestamp = int(self.get_clock().now().nanoseconds / 1000)

        # OffboardControlMode must be sent continuously
        offboard_mode = OffboardControlMode()
        offboard_mode.timestamp = timestamp
        offboard_mode.position = True
        self.offboard_control_mode_pub.publish(offboard_mode)

        # Step 0: Arm + switch to Offboard mode
        if self.step == 0:
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, param1=1.0, param2=6.0)
            self.publish_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, param1=1.0)
            self.get_logger().info("✅ Armed and switched to Offboard")
            self.start_time = time.time()
            self.step = 1

        # Step 1-n: Fly to target waypoints
        elif 1 <= self.step <= len(self.waypoints):
            target = self.waypoints[self.step - 1]
            traj = TrajectorySetpoint()
            traj.timestamp = timestamp
            traj.position = list(target)
            traj.yaw = 0.0
            self.trajectory_setpoint_pub.publish(traj)

            if time.time() - self.start_time > 6:  # Stay at each waypoint for 6 seconds
                self.get_logger().info(
                    f"📍 Reached waypoint {self.step}/{len(self.waypoints)} (NED {target})")
                self.step += 1
                self.start_time = time.time()

        # Final step: Land
        elif self.step == len(self.waypoints) + 1:
            self.publish_vehicle_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.get_logger().info("🛬 Landing...")
            rclpy.shutdown()

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
    node = OffboardHouseMission()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
