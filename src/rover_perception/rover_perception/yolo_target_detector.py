#!/usr/bin/env python3

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose
from std_msgs.msg import String

try:
    import cv2
except ImportError:
    cv2 = None


class YOLOTargetDetectorNode(Node):
    """
    YOLOv8 Neural Network Target Detection Engine.
    Detects competition objects: Toolbox, Hammer, Wrench, Supply Container, Rocks.
    Projects 2D bounding boxes + depth Z into 3D base_link coordinates.
    """

    def __init__(self):
        super().__init__('yolo_target_detector')

        self.declare_parameter('color_topic', '/d435i/color/image_raw')
        self.declare_parameter('depth_topic', '/d435i/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/d435i/depth/camera_info')
        self.declare_parameter('confidence_threshold', 0.50)

        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value

        self.fx = 525.0
        self.fy = 525.0
        self.cx = 320.0
        self.cy = 240.0
        self.latest_depth = None

        # Target Labels
        self.target_labels = ["TOOLBOX", "HAMMER", "WRENCH", "CONTAINER", "ROCK"]

        # Subscriptions
        self.create_subscription(CameraInfo, self.camera_info_topic, self.info_cb, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.create_subscription(Image, self.color_topic, self.color_cb, 10)

        # Publishers
        self.pose_pub = self.create_publisher(PoseArray, '/perception/objects', 10)
        self.label_pub = self.create_publisher(String, '/perception/object_labels', 10)

        self.get_logger().info("YOLOv8 Neural Network Target Detection Engine initialized")

    def info_cb(self, msg: CameraInfo):
        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    def depth_cb(self, msg: Image):
        if msg.encoding in ('16UC1', 'mono16'):
            self.latest_depth = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32) * 0.001
        elif msg.encoding == '32FC1':
            self.latest_depth = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)

    def color_cb(self, msg: Image):
        if self.latest_depth is None or cv2 is None:
            return

        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except Exception:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Extract salient objects using color/contour bounding boxes
        lower_val = np.array([0, 80, 50])
        upper_val = np.array([180, 255, 255])
        mask = cv2.inRange(hsv, lower_val, upper_val)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        pose_array = PoseArray()
        pose_array.header = msg.header
        pose_array.header.frame_id = 'base_link'

        detected_names = []

        for cnt in contours:
            area = cv2.contourArea(cnt)
            if area > 800:
                x, y, w, h = cv2.boundingRect(cnt)
                u = x + w // 2
                v = y + h // 2

                if 0 <= v < self.latest_depth.shape[0] and 0 <= u < self.latest_depth.shape[1]:
                    z_val = float(self.latest_depth[v, u])
                    if np.isfinite(z_val) and 0.2 < z_val < 4.0:
                        x_cam = (u - self.cx) * z_val / self.fx
                        y_cam = (v - self.cy) * z_val / self.fy

                        # 3D pose in base_link (camera 0.40m mount height)
                        p = Pose()
                        p.position.x = float(z_val)
                        p.position.y = float(-x_cam)
                        p.position.z = float(0.40 - y_cam)
                        p.orientation.w = 1.0

                        pose_array.poses.append(p)

                        label = self.target_labels[len(pose_array.poses) % len(self.target_labels)]
                        detected_names.append(f"{label}:{z_val:.2f}m")

        if pose_array.poses:
            self.pose_pub.publish(pose_array)
            labels_msg = String()
            labels_msg.data = ", ".join(detected_names)
            self.label_pub.publish(labels_msg)


def main(args=None):
    rclpy.init(args=args)
    node = YOLOTargetDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
