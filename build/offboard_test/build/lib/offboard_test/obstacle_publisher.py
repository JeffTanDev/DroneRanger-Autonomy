#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Point


class ObstaclePublisher(Node):
    def __init__(self):
        super().__init__('obstacle_publisher')
        self.pub = self.create_publisher(Point, '/detected_obstacle', 10)

        # Timer: publish every 10 seconds
        self.timer = self.create_timer(10.0, self.publish_obstacle)
        self.obstacle_count = 0

    def publish_obstacle(self):
        msg = Point()

        if self.obstacle_count == 0:
            # First obstacle near (15,5,-5)
            msg.x, msg.y, msg.z = 15.0, 5.0, -5.0
            self.get_logger().info(f"Publishing obstacle 1 at ({msg.x}, {msg.y}, {msg.z})")
            self.pub.publish(msg)

        elif self.obstacle_count == 1:
            # Second obstacle near (18,3,-5)
            msg.x, msg.y, msg.z = 18.0, 3.0, -5.0
            self.get_logger().info(f"Publishing obstacle 2 at ({msg.x}, {msg.y}, {msg.z})")
            self.pub.publish(msg)

            # Stop after publishing two obstacles
            self.timer.cancel()
            self.get_logger().info("All obstacles published, stopping publisher.")

        self.obstacle_count += 1


def main(args=None):
    rclpy.init(args=args)
    node = ObstaclePublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
