#!/usr/bin/env python3

from collections import deque
from pathlib import Path

import cv2
import numpy as np
from ultralytics import YOLO

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo

from rover_interfaces.msg import ToolboxGoal


CONFIDENCE = 0.50
MIN_DEPTH = 0.30
MAX_DEPTH = 5.0
DEPTH_ROI_RATIO = 0.30
SMOOTHING_FRAMES = 5
GOAL_REACHED_DISTANCE = 0.35


class ToolboxNode(Node):

    def __init__(self):
        super().__init__('toolbox_perception')

        self.declare_parameter(
            'rgb_topic',
            '/d435i/image_raw'
        )

        self.declare_parameter(
            'depth_topic',
            '/d435i/depth/image_raw'
        )

        self.declare_parameter(
            'camera_info_topic',
            '/d435i/depth/camera_info'
        )

        rgb_topic = self.get_parameter('rgb_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        info_topic = self.get_parameter('camera_info_topic').value

        # ------------------------------------------------------
        # YOLO model
        # ------------------------------------------------------
        model_path = (
            Path.home()
            / 'ros2_ws'
            / 'src'
            / 'rover_perception'
            / 'models'
            / 'best.pt'
        )

        self.model = YOLO(str(model_path))

        # ------------------------------------------------------
        # Camera state
        # ------------------------------------------------------
        self.latest_depth = None

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.depth_buffer = deque(
            maxlen=SMOOTHING_FRAMES
        )

        # ------------------------------------------------------
        # ROS
        # ------------------------------------------------------
        self.pub = self.create_publisher(
            ToolboxGoal,
            '/perception/toolbox_goal',
            10
        )

        self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            10
        )

        self.create_subscription(
            CameraInfo,
            info_topic,
            self.camera_info_callback,
            10
        )

        self.create_subscription(
            Image,
            rgb_topic,
            self.rgb_callback,
            10
        )

        self.get_logger().info(
            'Continuous toolbox YOLO perception started'
        )

    # ==========================================================
    # Camera intrinsics
    # ==========================================================

    def camera_info_callback(self, msg):

        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    # ==========================================================
    # Depth
    # ==========================================================

    def depth_callback(self, msg):

        expected = msg.width * msg.height

        if msg.encoding == '32FC1':

            dtype = (
                np.dtype('>f4')
                if msg.is_bigendian
                else np.dtype('<f4')
            )

            arr = np.frombuffer(
                msg.data,
                dtype=dtype
            )

            if arr.size < expected:
                return

            self.latest_depth = (
                arr[:expected]
                .reshape(
                    msg.height,
                    msg.width
                )
                .astype(np.float32)
            )

        elif msg.encoding in ('16UC1', 'mono16'):

            dtype = (
                np.dtype('>u2')
                if msg.is_bigendian
                else np.dtype('<u2')
            )

            arr = np.frombuffer(
                msg.data,
                dtype=dtype
            )

            if arr.size < expected:
                return

            self.latest_depth = (
                arr[:expected]
                .reshape(
                    msg.height,
                    msg.width
                )
                .astype(np.float32)
                * 0.001
            )

    # ==========================================================
    # RGB
    # ==========================================================

    def rgb_callback(self, msg):

        if self.latest_depth is None:
            return

        if self.fx is None:
            return

        if msg.encoding not in ('rgb8', 'bgr8'):
            return

        channels = 3

        img = np.frombuffer(
            msg.data,
            dtype=np.uint8
        )

        expected = (
            msg.height
            * msg.width
            * channels
        )

        if img.size < expected:
            return

        img = img[:expected].reshape(
            msg.height,
            msg.width,
            channels
        )

        if msg.encoding == 'rgb8':
            img = cv2.cvtColor(
                img,
                cv2.COLOR_RGB2BGR
            )

        results = self.model(
            img,
            conf=CONFIDENCE,
            verbose=False
        )

        best_box = None
        best_conf = 0.0

        for result in results:

            if result.boxes is None:
                continue

            for box in result.boxes:

                cls_id = int(
                    box.cls[0]
                )

                class_name = (
                    self.model.names[
                        cls_id
                    ]
                )

                conf = float(
                    box.conf[0]
                )

                if (
                    class_name.lower() == 'toolbox'
                    and
                    conf > best_conf
                ):
                    best_conf = conf
                    best_box = box

        # ------------------------------------------------------
        # Nothing detected
        # ------------------------------------------------------

        if best_box is None:

            self.depth_buffer.clear()

            out = ToolboxGoal()
            out.header = msg.header
            out.detected = False
            out.confidence = 0.0
            out.distance_m = 0.0
            out.goal_reached = False
            out.status = 'TOOLBOX NOT DETECTED'

            self.pub.publish(out)
            return

        # ------------------------------------------------------
        # Bounding box
        # ------------------------------------------------------

        x1, y1, x2, y2 = (
            best_box.xyxy[0]
            .cpu()
            .numpy()
            .astype(int)
        )

        center_x = int(
            (x1 + x2) / 2
        )

        center_y = int(
            (y1 + y2) / 2
        )

        box_w = max(
            x2 - x1,
            1
        )

        box_h = max(
            y2 - y1,
            1
        )

        roi_w = max(
            int(box_w * DEPTH_ROI_RATIO),
            2
        )

        roi_h = max(
            int(box_h * DEPTH_ROI_RATIO),
            2
        )

        depth_h, depth_w = (
            self.latest_depth.shape
        )

        # Scale RGB pixel into depth resolution if required.
        depth_u = int(
            center_x
            * depth_w
            / msg.width
        )

        depth_v = int(
            center_y
            * depth_h
            / msg.height
        )

        roi_w = int(
            roi_w
            * depth_w
            / msg.width
        )

        roi_h = int(
            roi_h
            * depth_h
            / msg.height
        )

        dx1 = max(
            depth_u - roi_w // 2,
            0
        )

        dx2 = min(
            depth_u + roi_w // 2,
            depth_w
        )

        dy1 = max(
            depth_v - roi_h // 2,
            0
        )

        dy2 = min(
            depth_v + roi_h // 2,
            depth_h
        )

        roi = self.latest_depth[
            dy1:dy2,
            dx1:dx2
        ]

        valid = roi[
            np.isfinite(roi)
            &
            (roi >= MIN_DEPTH)
            &
            (roi <= MAX_DEPTH)
        ]

        if valid.size == 0:
            return

        depth = float(
            np.median(valid)
        )

        self.depth_buffer.append(
            depth
        )

        smooth_depth = float(
            np.median(
                self.depth_buffer
            )
        )

        # ------------------------------------------------------
        # 3D point:
        # same RealSense deprojection mathematics
        # X = (u-cx)Z/fx
        # Y = (v-cy)Z/fy
        # ------------------------------------------------------

        X = (
            (depth_u - self.cx)
            * smooth_depth
            / self.fx
        )

        Y = (
            (depth_v - self.cy)
            * smooth_depth
            / self.fy
        )

        Z = smooth_depth

        out = ToolboxGoal()

        out.header = msg.header

        out.detected = True
        out.confidence = best_conf
        out.distance_m = smooth_depth

        out.position.x = float(X)
        out.position.y = float(Y)
        out.position.z = float(Z)

        out.goal_reached = (
            smooth_depth
            <= GOAL_REACHED_DISTANCE
        )

        if out.goal_reached:
            out.status = 'GOAL REACHED'
        else:
            out.status = 'GOAL AHEAD'

        self.pub.publish(out)


def main(args=None):

    rclpy.init(args=args)

    node = ToolboxNode()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
