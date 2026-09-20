#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from rover_interfaces.msg import CliffAlert
from geometry_msgs.msg import Point


class CliffDetectorNode(Node):
    """
    Cliff / Vertical Drop Safety Detector.
    Monitors camera depth rays projected at a downward angle.
    If measured depth suddenly jumps beyond expected ground surface, alerts Safety Manager.
    """

    def __init__(self):
        super().__init__('cliff_detector')

        self.declare_parameter('depth_topic', '/d435i/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/d435i/depth/camera_info')
        self.declare_parameter('cam_mount_height', 0.35)      # 35 cm from ground
        self.declare_parameter('cam_pitch_deg', 15.0)         # downward pitch angle
        self.declare_parameter('drop_threshold_m', 0.35)      # vertical drop sensitivity

        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.cam_height = self.get_parameter('cam_mount_height').value
        self.pitch_deg = self.get_parameter('cam_pitch_deg').value
        self.drop_thresh = self.get_parameter('drop_threshold_m').value

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        # Subscriptions
        self.create_subscription(CameraInfo, self.camera_info_topic, self.info_cb, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)

        # Publisher
        self.cliff_pub = self.create_publisher(CliffAlert, '/perception/cliffs', 10)

        self.get_logger().info("Cliff / Drop Safety Detector started (Camera Height: 0.35m)")

    def info_cb(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def depth_cb(self, msg: Image):
        if self.fx is None:
            return

        # Convert ROS Image to OpenCV depth matrix in meters
        if msg.encoding in ('16UC1', 'mono16'):
            depth_m = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32) * 0.001
        elif msg.encoding == '32FC1':
            depth_m = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        else:
            return

        h, w = depth_m.shape

        # Analyze lower third ROI of depth image where ground rays hit
        lower_roi = depth_m[int(h * 0.6):int(h * 0.95), :]

        valid_mask = np.isfinite(lower_roi) & (lower_roi > 0.2) & (lower_roi < 3.0)

        if not np.any(valid_mask):
            return

        # Expected ground distance based on camera pitch
        pitch_rad = math.radians(self.pitch_deg)
        expected_ground_dist = self.cam_height / max(math.sin(pitch_rad), 0.05)

        # Identify pixels where depth exceeds expected ground distance by drop_thresh
        cliff_mask = valid_mask & (lower_roi > (expected_ground_dist + self.drop_thresh))

        cliff_pixel_count = np.count_nonzero(cliff_mask)

        alert = CliffAlert()
        alert.header = msg.header

        if cliff_pixel_count > (h * w * 0.02):  # >2% of image indicates cliff
            alert.cliff_detected = True
            alert.distance_to_drop = float(expected_ground_dist)
            alert.drop_depth_meters = float(np.mean(lower_roi[cliff_mask]) - expected_ground_dist)
            alert.cliff_position = Point(x=expected_ground_dist, y=0.0, z=-self.cam_height)

            self.get_logger().error(f"CLIFF DETECTED! Drop distance: {alert.distance_to_drop:.2f}m, Depth: {alert.drop_depth_meters:.2f}m")
        else:
            alert.cliff_detected = False
            alert.distance_to_drop = 0.0
            alert.drop_depth_meters = 0.0

        self.cliff_pub.publish(alert)


def main(args=None):
    rclpy.init(args=args)
    node = CliffDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
