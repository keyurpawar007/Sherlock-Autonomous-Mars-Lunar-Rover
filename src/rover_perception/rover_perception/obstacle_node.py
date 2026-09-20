#!/usr/bin/env python3

import math
from collections import deque

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import CameraInfo, Image
from std_msgs.msg import String

from rover_interfaces.msg import ObstacleInfo


# ==============================================================
# PERCEPTION TEAM PARAMETERS
# Kept from rover_perception_test.py
# ==============================================================

CAM_MOUNT_HEIGHT = 0.40
CAM_PITCH_DEG = 15.0

STOP_SAFETY_DIST = 0.40
MAX_CLIMB_HEIGHT = 0.50
CHASSIS_CLR_HGHT = 0.35
TRACK_SAFE_WIDTH = 0.40

GROUND_MARGIN = 0.03
MIN_DIST_M = 0.30
MAX_DIST_M = 2.50

CONFIDENCE_FRAME_THRESHOLD = 5


# ==============================================================
# ORIGINAL DECISION LOGIC
# ==============================================================

def evaluate_decision(dist_m, width_m, height_m):

    if dist_m < STOP_SAFETY_DIST:
        return "EMERGENCY STOP (TOO CLOSE)", 4

    if height_m <= MAX_CLIMB_HEIGHT:

        if (
            height_m <= CHASSIS_CLR_HGHT
            and width_m <= 0.30
        ):
            return "STRADDLE (UNDER-BELLY)", 1

        return "CLIMB / CROSS", 2

    if width_m <= TRACK_SAFE_WIDTH:
        return "DETOUR (NARROW OBSTACLE)", 3

    return "PATH BLOCKED (FULL STOP)", 4


class ObstaclePerceptionNode(Node):

    def __init__(self):

        super().__init__('obstacle_perception')

        # ------------------------------------------------------
        # Topics
        # ------------------------------------------------------

        self.declare_parameter(
            'depth_topic',
            '/d435i/depth/image_raw'
        )

        self.declare_parameter(
            'camera_info_topic',
            '/d435i/depth/camera_info'
        )

        # Physical-camera original = 800.
        # Gazebo 424x240 needs a lower pixel count.
        self.declare_parameter(
            'min_cluster_px',
            250
        )

        # Camera ROI.
        # Defaults below are tuned for the Gazebo D435i.
        # Real D435i values are supplied by perception_real.yaml.
        self.declare_parameter('roi_x_min', 0.50)
        self.declare_parameter('roi_x_max', 0.65)
        self.declare_parameter('roi_y_min', 0.20)
        self.declare_parameter('roi_y_max', 0.35)

        self.depth_topic = self.get_parameter(
            'depth_topic'
        ).value

        self.camera_info_topic = self.get_parameter(
            'camera_info_topic'
        ).value

        self.min_cluster_px = int(
            self.get_parameter(
                'min_cluster_px'
            ).value
        )

        self.roi_x_min = float(
            self.get_parameter('roi_x_min').value
        )

        self.roi_x_max = float(
            self.get_parameter('roi_x_max').value
        )

        self.roi_y_min = float(
            self.get_parameter('roi_y_min').value
        )

        self.roi_y_max = float(
            self.get_parameter('roi_y_max').value
        )

        # ------------------------------------------------------
        # Team algorithm state
        # ------------------------------------------------------

        pitch_rad = math.radians(
            CAM_PITCH_DEG
        )

        self.cos_pitch = math.cos(
            pitch_rad
        )

        self.sin_pitch = math.sin(
            pitch_rad
        )

        self.dist_buffer = deque(
            maxlen=5
        )

        self.width_buffer = deque(
            maxlen=5
        )

        self.height_buffer = deque(
            maxlen=5
        )

        self.lateral_buffer = deque(
            maxlen=5
        )

        self.consecutive_frames = 0

        # ------------------------------------------------------
        # Camera intrinsics
        # ------------------------------------------------------

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None

        self.grid_shape = None
        self.u_grid = None
        self.v_grid = None

        # ------------------------------------------------------
        # ROS publishers
        # ------------------------------------------------------

        self.info_pub = self.create_publisher(
            ObstacleInfo,
            '/perception/obstacle_info',
            10
        )

        # Compatibility with our frozen navigation stack
        self.status_pub = self.create_publisher(
            String,
            '/obstacle_status',
            10
        )

        self.avoidance_pub = self.create_publisher(
            Twist,
            '/avoidance_cmd',
            10
        )

        # ------------------------------------------------------
        # Continuous subscriptions
        # ------------------------------------------------------

        self.create_subscription(
            CameraInfo,
            self.camera_info_topic,
            self.camera_info_callback,
            10
        )

        self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )

        self.last_state = None

        self.get_logger().info(
            'Continuous D435i obstacle perception started'
        )

    # ==========================================================
    # CameraInfo
    # ==========================================================

    def camera_info_callback(self, msg):

        self.fx = float(msg.k[0])
        self.fy = float(msg.k[4])
        self.cx = float(msg.k[2])
        self.cy = float(msg.k[5])

    # ==========================================================
    # ROS depth -> metres
    # ==========================================================

    def depth_to_metres(self, msg):

        expected = msg.width * msg.height

        if msg.encoding == '32FC1':

            dtype = (
                np.dtype('>f4')
                if msg.is_bigendian
                else np.dtype('<f4')
            )

            depth = np.frombuffer(
                msg.data,
                dtype=dtype
            )

            if depth.size < expected:
                return None

            return depth[:expected].reshape(
                msg.height,
                msg.width
            ).astype(
                np.float32,
                copy=False
            )

        if msg.encoding in ('16UC1', 'mono16'):

            dtype = (
                np.dtype('>u2')
                if msg.is_bigendian
                else np.dtype('<u2')
            )

            depth = np.frombuffer(
                msg.data,
                dtype=dtype
            )

            if depth.size < expected:
                return None

            # RealSense depth normally millimetres in 16UC1
            return (
                depth[:expected]
                .reshape(
                    msg.height,
                    msg.width
                )
                .astype(np.float32)
                * 0.001
            )

        return None

    # ==========================================================
    # Continuous frame processing
    # ==========================================================

    def depth_callback(self, msg):

        if self.fx is None:
            return

        depth_m = self.depth_to_metres(
            msg
        )

        if depth_m is None:
            return

        h, w = depth_m.shape

        # Original code used np.meshgrid for 640x480.
        # Same calculation, but based on incoming resolution.
        if self.grid_shape != (h, w):

            self.u_grid, self.v_grid = np.meshgrid(
                np.arange(w),
                np.arange(h)
            )

            self.grid_shape = (
                h,
                w
            )

        # ------------------------------------------------------
        # ORIGINAL CAMERA GEOMETRY
        # ------------------------------------------------------

        x_cam = (
            (self.u_grid - self.cx)
            * depth_m
            / self.fx
        )

        y_cam = (
            (self.v_grid - self.cy)
            * depth_m
            / self.fy
        )

        z_cam = depth_m

        y_world = (
            y_cam * self.cos_pitch
            +
            z_cam * self.sin_pitch
        )

        z_world = (
            -y_cam * self.sin_pitch
            +
            z_cam * self.cos_pitch
        )

        height_above_ground = (
            CAM_MOUNT_HEIGHT
            -
            y_world
        )

        # ------------------------------------------------------
        # ORIGINAL FRACTIONAL ROI
        # ------------------------------------------------------

        roi_x1 = int(w * self.roi_x_min)
        roi_x2 = int(w * self.roi_x_max)

        roi_y1 = int(h * self.roi_y_min)
        roi_y2 = int(h * self.roi_y_max)

        roi_mask = (
            (self.u_grid >= roi_x1)
            &
            (self.u_grid < roi_x2)
            &
            (self.v_grid >= roi_y1)
            &
            (self.v_grid < roi_y2)
        )

        obstacle_mask = (
            roi_mask
            &
            np.isfinite(depth_m)
            &
            (depth_m > MIN_DIST_M)
            &
            (depth_m < MAX_DIST_M)
            &
            (
                height_above_ground
                >
                GROUND_MARGIN
            )
        )

        # ------------------------------------------------------
        # ORIGINAL CONNECTED COMPONENT PROCESSING
        # ------------------------------------------------------

        binary_obs = (
            obstacle_mask * 255
        ).astype(np.uint8)

        kernel = np.ones(
            (5, 5),
            np.uint8
        )

        cleaned_obs = cv2.morphologyEx(
            binary_obs,
            cv2.MORPH_OPEN,
            kernel
        )

        (
            num_labels,
            labels,
            stats,
            _
        ) = cv2.connectedComponentsWithStats(
            cleaned_obs
        )

        raw_detected = False
        label_idx_to_use = -1
        max_area = 0

        for idx in range(
            1,
            num_labels
        ):

            area = stats[
                idx,
                cv2.CC_STAT_AREA
            ]

            if (
                area >= self.min_cluster_px
                and
                area > max_area
            ):
                max_area = area
                label_idx_to_use = idx
                raw_detected = True

        # ------------------------------------------------------
        # ORIGINAL 5-FRAME CONFIDENCE
        # ------------------------------------------------------

        if raw_detected:

            self.consecutive_frames += 1

        else:

            self.consecutive_frames = 0

            self.dist_buffer.clear()
            self.width_buffer.clear()
            self.height_buffer.clear()
            self.lateral_buffer.clear()

        confidence_pct = min(
            100,
            int(
                (
                    self.consecutive_frames
                    /
                    CONFIDENCE_FRAME_THRESHOLD
                )
                * 100
            )
        )

        is_sure = (
            self.consecutive_frames
            >=
            CONFIDENCE_FRAME_THRESHOLD
        )

        payload = {
            'obstacle_detected': False,
            'surety': False,
            'confidence_pct': confidence_pct,
            'distance_m': 0.0,
            'forward_m': 0.0,
            'lateral_m': 0.0,
            'width_m': 0.0,
            'height_m': 0.0,
            'decision': 'PATH CLEAR',
            'action_code': 0,
        }

        # ------------------------------------------------------
        # ORIGINAL METRIC EXTRACTION
        # ------------------------------------------------------

        if (
            is_sure
            and
            label_idx_to_use != -1
        ):

            component_mask = (
                labels == label_idx_to_use
            )

            z_cls = z_world[
                component_mask
            ]

            x_cls = x_cam[
                component_mask
            ]

            h_cls = height_above_ground[
                component_mask
            ]

            valid = (
                np.isfinite(z_cls)
                &
                np.isfinite(x_cls)
                &
                np.isfinite(h_cls)
            )

            z_cls = z_cls[valid]
            x_cls = x_cls[valid]
            h_cls = h_cls[valid]

            if z_cls.size >= 20:

                inst_dist = float(
                    np.percentile(
                        z_cls,
                        5
                    )
                )

                inst_width = float(
                    np.percentile(
                        x_cls,
                        95
                    )
                    -
                    np.percentile(
                        x_cls,
                        5
                    )
                )

                inst_height = float(
                    np.percentile(
                        h_cls,
                        95
                    )
                )

                # Added only for A* obstacle positioning.
                inst_lateral = float(
                    np.median(
                        x_cls
                    )
                )

                self.dist_buffer.append(
                    inst_dist
                )

                self.width_buffer.append(
                    inst_width
                )

                self.height_buffer.append(
                    inst_height
                )

                self.lateral_buffer.append(
                    inst_lateral
                )

                dist_m = float(
                    np.mean(
                        self.dist_buffer
                    )
                )

                width_m = max(
                    float(
                        np.mean(
                            self.width_buffer
                        )
                    ),
                    0.05
                )

                height_m = max(
                    float(
                        np.mean(
                            self.height_buffer
                        )
                    ),
                    0.02
                )

                lateral_m = float(
                    np.mean(
                        self.lateral_buffer
                    )
                )

                decision, action_code = (
                    evaluate_decision(
                        dist_m,
                        width_m,
                        height_m
                    )
                )

                payload = {
                    'obstacle_detected': True,
                    'surety': True,
                    'confidence_pct': 100,

                    'distance_m': dist_m,
                    'forward_m': dist_m,
                    'lateral_m': lateral_m,

                    'width_m': width_m,
                    'height_m': height_m,

                    'decision': decision,
                    'action_code': action_code,
                }

        self.publish_result(
            msg,
            payload
        )

    # ==========================================================
    # ROS output
    # ==========================================================

    def publish_result(
        self,
        depth_msg,
        payload
    ):

        info = ObstacleInfo()

        info.header = depth_msg.header

        info.detected = bool(
            payload[
                'obstacle_detected'
            ]
        )

        info.surety = bool(
            payload[
                'surety'
            ]
        )

        info.confidence_pct = int(
            payload[
                'confidence_pct'
            ]
        )

        info.distance_m = float(
            payload[
                'distance_m'
            ]
        )

        info.forward_m = float(
            payload[
                'forward_m'
            ]
        )

        info.lateral_m = float(
            payload[
                'lateral_m'
            ]
        )

        info.width_m = float(
            payload[
                'width_m'
            ]
        )

        info.height_m = float(
            payload[
                'height_m'
            ]
        )

        info.action_code = int(
            payload[
                'action_code'
            ]
        )

        info.decision = str(
            payload[
                'decision'
            ]
        )

        self.info_pub.publish(
            info
        )

        # ------------------------------------------------------
        # Bridge into our existing navigation
        #
        # No greedy LEFT / RIGHT commands.
        # ------------------------------------------------------

        cmd = Twist()
        status = String()

        action = info.action_code

        if (
            not info.detected
            or
            not info.surety
        ):

            state = 'CLEAR'

            cmd.linear.x = 1.0
            cmd.angular.z = 0.0

        elif action in (3, 4):

            # DETOUR / BLOCKED
            # A* chooses the route.
            state = 'BLOCKED_REPLAN'

            cmd.linear.x = 0.0
            cmd.angular.z = 0.0

        elif action in (1, 2):

            # Keep team classification.
            # Conservative speed for integration stage.
            state = 'CAUTION_SLOW'

            cmd.linear.x = 0.35
            cmd.angular.z = 0.0

        else:

            state = 'CLEAR'

            cmd.linear.x = 1.0
            cmd.angular.z = 0.0

        self.avoidance_pub.publish(
            cmd
        )

        status.data = (
            f'{state} '
            f'distance={info.distance_m:.2f}m '
            f'width={info.width_m:.2f}m '
            f'height={info.height_m:.2f}m '
            f'lateral={info.lateral_m:.2f}m '
            f'action={info.action_code} '
            f'decision="{info.decision}"'
        )

        self.status_pub.publish(
            status
        )

        if state != self.last_state:

            self.get_logger().info(
                status.data
            )

            self.last_state = state


def main(args=None):

    rclpy.init(
        args=args
    )

    node = ObstaclePerceptionNode()

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
