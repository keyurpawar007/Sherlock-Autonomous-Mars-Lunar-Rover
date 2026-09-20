#!/usr/bin/env python3

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PolygonStamped, Point32


class RedZoneDetectorNode(Node):
    """
    Red / Contaminated Zone Detector.
    Processes RGB images, segments red-colored regions,
    and publishes no-go polygon boundaries to /perception/no_go for costmap inflation.
    """

    def __init__(self):
        super().__init__('red_zone_detector')

        self.declare_parameter('color_topic', '/d435i/color/image_raw')
        self.color_topic = self.get_parameter('color_topic').value

        # Subscriptions
        self.create_subscription(Image, self.color_topic, self.image_cb, 10)

        # Publisher
        self.nogo_pub = self.create_publisher(PolygonStamped, '/perception/no_go', 10)

        self.get_logger().info("Red / Contaminated Zone Detector started")

    def image_cb(self, msg: Image):
        # Convert ROS Image to OpenCV BGR
        try:
            if msg.encoding == 'bgr8':
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            elif msg.encoding == 'rgb8':
                frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
            else:
                return
        except Exception:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Dual HSV range for Red hue wrapping around 0/180
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([170, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)

        # Morphological cleaning
        kernel = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        red_mask = cv2.morphologyEx(red_mask, cv2.MORPH_OPEN, kernel)

        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 1000:  # Significant red region
                polygon_msg = PolygonStamped()
                polygon_msg.header = msg.header
                polygon_msg.header.frame_id = 'base_link'

                # Approximate contour to polygon
                approx = cv2.approxPolyDP(cnt, 0.02 * cv2.arcLength(cnt, True), True)
                for pt in approx:
                    p = Point32()
                    # Simplified projection assuming forward distance
                    p.x = float(1.5 + (pt[0][1] / msg.height))
                    p.y = float((pt[0][0] - (msg.width / 2.0)) / (msg.width / 2.0))
                    p.z = 0.0
                    polygon_msg.polygon.points.append(p)

                self.nogo_pub.publish(polygon_msg)
                self.get_logger().info(f"RED NO-GO ZONE DETECTED! Polygon Area: {area:.0f}px")


def main(args=None):
    rclpy.init(args=args)
    node = RedZoneDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
