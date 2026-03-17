import math
import time

import rclpy
from nav_msgs.msg import Odometry
from px4_msgs.msg import VehicleOdometry
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy


class OdomMonitor(Node):
    def __init__(self):
        super().__init__("odom_monitor")

        self.declare_parameter("log_interval_s", 0.5)
        self.declare_parameter("min_delta_m", 0.02)
        self.declare_parameter("log_first_msg", True)
        self.declare_parameter("enable_px4_odom", True)

        self.log_interval_s = float(self.get_parameter("log_interval_s").value)
        self.min_delta_m = float(self.get_parameter("min_delta_m").value)
        self.log_first_msg = bool(self.get_parameter("log_first_msg").value)
        self.enable_px4_odom = bool(self.get_parameter("enable_px4_odom").value)

        qos_best_effort = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)

        self.create_subscription(Odometry, "/drone/odom", self._on_world_odom, 10)
        if self.enable_px4_odom:
            self.create_subscription(
                VehicleOdometry,
                "/fmu/out/vehicle_odometry",
                self._on_px4_odom,
                qos_best_effort,
            )

        self._last_world = None
        self._last_px4 = None
        self._last_log_world_s = 0.0
        self._last_log_px4_s = 0.0
        self._world_count = 0
        self._px4_count = 0

    @staticmethod
    def _norm3(dx: float, dy: float, dz: float) -> float:
        return math.sqrt(dx * dx + dy * dy + dz * dz)

    def _should_log(self, last_log_s: float, delta_m: float) -> bool:
        now = time.time()
        if self.log_interval_s > 0.0 and (now - last_log_s) >= self.log_interval_s:
            return True
        return delta_m >= self.min_delta_m

    def _on_world_odom(self, msg: Odometry):
        self._world_count += 1
        p = msg.pose.pose.position
        curr = (float(p.x), float(p.y), float(p.z))

        if self._last_world is None:
            self._last_world = curr
            if self.log_first_msg:
                self.get_logger().info(
                    f"/drone/odom first: x={curr[0]:.3f} y={curr[1]:.3f} z={curr[2]:.3f}"
                )
            return

        dx = curr[0] - self._last_world[0]
        dy = curr[1] - self._last_world[1]
        dz = curr[2] - self._last_world[2]
        d = self._norm3(dx, dy, dz)

        if self._should_log(self._last_log_world_s, d):
            self._last_log_world_s = time.time()
            self.get_logger().info(
                f"/drone/odom #{self._world_count}: "
                f"x={curr[0]:.3f} y={curr[1]:.3f} z={curr[2]:.3f} | "
                f"Δ={d:.3f}m (dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f})"
            )

        self._last_world = curr

    def _on_px4_odom(self, msg: VehicleOdometry):
        self._px4_count += 1
        curr = (float(msg.position[0]), float(msg.position[1]), float(msg.position[2]))

        if self._last_px4 is None:
            self._last_px4 = curr
            if self.log_first_msg:
                self.get_logger().info(
                    f"/fmu/out/vehicle_odometry first: x={curr[0]:.3f} y={curr[1]:.3f} z={curr[2]:.3f}"
                )
            return

        dx = curr[0] - self._last_px4[0]
        dy = curr[1] - self._last_px4[1]
        dz = curr[2] - self._last_px4[2]
        d = self._norm3(dx, dy, dz)

        if self._should_log(self._last_log_px4_s, d):
            self._last_log_px4_s = time.time()
            self.get_logger().info(
                f"/fmu/out/vehicle_odometry #{self._px4_count}: "
                f"x={curr[0]:.3f} y={curr[1]:.3f} z={curr[2]:.3f} | "
                f"Δ={d:.3f}m (dx={dx:.3f}, dy={dy:.3f}, dz={dz:.3f})"
            )

        self._last_px4 = curr


def main(args=None):
    rclpy.init(args=args)
    node = OdomMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

