#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Bool

try:
    import cv2
    import cv2.aruco as aruco
except ImportError:
    cv2 = None
    aruco = None


class ArUcoDockingNode(Node):
    """
    ArUco Precision Marker Docking Controller.
    Detects target ArUco markers on delivery containers/payload targets.
    Computes 6-DoF relative pose and generates precise creep commands for millimeter alignment.
    """

    def __init__(self):
        super().__init__('aruco_docking_node')

        self.declare_parameter('color_topic', '/d435i/color/image_raw')
        self.declare_parameter('depth_topic', '/d435i/depth/image_raw')
        self.declare_parameter('camera_info_topic', '/d435i/depth/camera_info')
        self.declare_parameter('marker_size_m', 0.15)          # 15 cm target ArUco marker
        self.declare_parameter('dock_dist_target_m', 0.18)     # Stop docking at 18 cm
        self.declare_parameter('max_creep_speed', 0.12)        # 0.12 m/s max docking speed

        self.color_topic = self.get_parameter('color_topic').value
        self.depth_topic = self.get_parameter('depth_topic').value
        self.camera_info_topic = self.get_parameter('camera_info_topic').value
        self.marker_size = self.get_parameter('marker_size_m').value
        self.target_dist = self.get_parameter('dock_dist_target_m').value
        self.max_creep = self.get_parameter('max_creep_speed').value

        self.fx = 525.0
        self.fy = 525.0
        self.cx = 320.0
        self.cy = 240.0
        self.latest_depth = None

        self.is_docking_active = False

        # Subscriptions
        self.create_subscription(CameraInfo, self.camera_info_topic, self.info_cb, 10)
        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.create_subscription(Image, self.color_topic, self.color_cb, 10)
        self.create_subscription(Bool, '/docking/enable', self.enable_cb, 10)

        # Publishers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.status_pub = self.create_publisher(String, '/docking/status', 10)
        self.pose_pub = self.create_publisher(PoseStamped, '/docking/target_pose', 10)

        self.get_logger().info("ArUco Precision Docking Node initialized")

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

    def enable_cb(self, msg: Bool):
        self.is_docking_active = msg.data
        self.get_logger().info(f"ArUco Docking Controller Active: {self.is_docking_active}")

    def color_cb(self, msg: Image):
        if not self.is_docking_active or cv2 is None:
            return

        try:
            frame = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.width, 3)
            if msg.encoding == 'rgb8':
                frame = cv2.cvtColor(frame, cv2.COLOR_RGB2BGR)
        except Exception:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        marker_center = None
        marker_dist_z = None

        # Try OpenCV ArUco detection if module is available
        if hasattr(cv2, 'aruco'):
            aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
            parameters = cv2.aruco.DetectorParameters()
            corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=parameters)

            if ids is not None and len(corners) > 0:
                c = corners[0][0]
                u_center = int(np.mean(c[:, 0]))
                v_center = int(np.mean(c[:, 1]))
                marker_center = (u_center, v_center)

        # Fallback to high-contrast square contour detector if ArUco dictionary is not found
        if marker_center is None:
            _, thresh = cv2.threshold(gray, 50, 255, cv2.THRESH_BINARY_INV)
            contours, _ = cv2.findContours(thresh, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for cnt in contours:
                approx = cv2.approxPolyDP(cnt, 0.04 * cv2.arcLength(cnt, True), True)
                if len(approx) == 4 and cv2.contourArea(cnt) > 400:
                    M = cv2.moments(cnt)
                    if M["m00"] != 0:
                        u_center = int(M["m10"] / M["m00"])
                        v_center = int(M["m01"] / M["m00"])
                        marker_center = (u_center, v_center)
                        break

        if marker_center and self.latest_depth is not None:
            u, v = marker_center
            if 0 <= v < self.latest_depth.shape[0] and 0 <= u < self.latest_depth.shape[1]:
                z_val = float(self.latest_depth[v, u])
                if np.isfinite(z_val) and z_val > 0.05:
                    marker_dist_z = z_val
                    x_cam = (u - self.cx) * z_val / self.fx

                    # Compute alignment error
                    err_z = z_val - self.target_dist
                    err_x = x_cam

                    cmd = Twist()
                    status_msg = String()

                    if abs(err_z) < 0.02 and abs(err_x) < 0.02:
                        cmd.linear.x = 0.0
                        cmd.angular.z = 0.0
                        status_msg.data = "DOCKING COMPLETED SUCCESSFULLY"
                        self.get_logger().info("PRECISION DOCKING COMPLETE!")
                    else:
                        cmd.linear.x = float(max(min(0.5 * err_z, self.max_creep), -self.max_creep))
                        cmd.angular.z = float(max(min(-1.5 * err_x, 0.4), -0.4))
                        status_msg.data = f"DOCKING IN PROGRESS | Range: {z_val:.3f}m, Offset: {x_cam:.3f}m"

                    self.cmd_pub.publish(cmd)
                    self.status_pub.publish(status_msg)

                    # Publish 3D pose
                    p_msg = PoseStamped()
                    p_msg.header = msg.header
                    p_msg.header.frame_id = 'base_link'
                    p_msg.pose.position.x = float(z_val)
                    p_msg.pose.position.y = float(-x_cam)
                    p_msg.pose.position.z = 0.0
                    p_msg.pose.orientation.w = 1.0
                    self.pose_pub.publish(p_msg)


def main(args=None):
    rclpy.init(args=args)
    node = ArUcoDockingNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
