import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np


class SimpleTracker(Node):

    def __init__(self):
        super().__init__('simple_tracker')

        self.bridge = CvBridge()

        self.sub = self.create_subscription(
            Image,
            '/oak/rgb/image_raw',
            self.image_callback,
            10)

        self.pub_overlay = self.create_publisher(
            Image,
            '/tracking/overlay',
            10)

        self.get_logger().info("Simple Tracker Started")

    def image_callback(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        h, w, _ = frame.shape

        # ===== 假 tracking bbox（先验证链路）=====
        cx = int(w * 0.5)
        cy = int(h * 0.5)
        bw = int(w * 0.3)
        bh = int(h * 0.3)

        x1 = int(cx - bw/2)
        y1 = int(cy - bh/2)
        x2 = int(cx + bw/2)
        y2 = int(cy + bh/2)

        cv2.rectangle(frame, (x1, y1), (x2, y2), (0,255,0), 3)
        cv2.putText(frame, "Target ID: 1", (x1, y1-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0,255,0), 2)

        overlay_msg = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        overlay_msg.header = msg.header
        self.pub_overlay.publish(overlay_msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimpleTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
