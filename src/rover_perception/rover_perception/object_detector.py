#!/usr/bin/env python3

import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import PoseArray, Pose


class ObjectDetectorNode(Node):
    """
    RADO Object Detection & 3D Pose Transformer.
    Detects targeted items (toolbox, hammer, wrench, rock) from RGB+Depth camera streams.
    Transforms pixel coordinates + depth Z into 3D positions in base_link coordinate frame.
    """

    def __init__(self):
        super().__init__('object_detector')

        self.declare_parameter('color_topic', '/d435i/color/image_raw')
        self.declare_parameter('depth_topic', '/d435i/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/d435i/depth/camera_info')

        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value

        self.fx = 525.0
        self.fy = 525.0
        self.cx = 320.0
        self.cy = 240.0

        self.latest_depth = None

        # Subscriptions
        self.create_subscription(CameraInfo, self.camera_info_topic, self.info_cb, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.create_subscription(Image, self.color_topic, self.color_cb, 10)

        # Publisher
        self.object_pub = self.create_publisher(PoseArray, '/perception/objects', 10)

        self.get_logger().info("RADO Object Detector & 3D Pose Transformer initialized")

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
        if self.latest_depth is None:
            return

        # Simple color/geometry object extraction (e.g. blue toolbox or yellow tool)
        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except Exception:
            return

        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        # Blue Toolbox threshold
        lower_blue = np.array([100, 150, 50])
        upper_blue = np.array([140, 255, 255])

        mask = cv2.inRange(hsv, lower_blue, upper_blue)
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        pose_array = PoseArray()
        pose_array.header = msg.header
        pose_array.header.frame_id = 'base_link'

        for cnt in contours:
            if cv2.contourArea(cnt) > 500:
                M = cv2.moments(cnt)
                if M["m00"] != 0:
                    u = int(M["m10"] / M["m00"])
                    v = int(M["m01"] / M["m00"])

                    # Extract depth Z
                    if 0 <= v < self.latest_depth.shape[0] and 0 <= u < self.latest_depth.shape[1]:
                        z_val = float(self.latest_depth[v, u])
                        if np.isfinite(z_val) and z_val > 0.2:
                            x_cam = (u - self.cx) * z_val / self.fx
                            y_cam = (v - self.cy) * z_val / self.fy

                            # Project to base_link (camera mounted 0.4m high, pitched down)
                            p = Pose()
                            p.position.x = float(z_val)
                            p.position.y = float(-x_cam)
                            p.position.z = float(0.40 - y_cam)
                            p.orientation.w = 1.0

                            pose_array.poses.append(p)
                            self.get_logger().info(f"OBJECT DETECTED! 3D Pose in base_link: X={p.position.x:.2f}m, Y={p.position.y:.2f}m, Z={p.position.z:.2f}m")

        if pose_array.poses:
            self.object_pub.publish(pose_array)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
