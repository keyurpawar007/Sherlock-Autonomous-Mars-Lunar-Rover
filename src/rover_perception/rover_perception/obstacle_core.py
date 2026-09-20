import math
from collections import deque

import cv2
import numpy as np


class ObstacleDetector:
    """
    Pure obstacle-perception algorithm.

    No ROS.
    No RealSense pipeline.

    Input:
        depth image in metres
        camera intrinsics

    Output:
        dictionary containing detection metrics and action decision
    """

    def __init__(
        self,
        cam_mount_height=0.40,
        cam_pitch_deg=15.0,
        stop_safety_dist=0.40,
        max_climb_height=0.50,
        chassis_clear_height=0.35,
        track_safe_width=0.40,
        ground_margin=0.03,
        min_dist_m=0.30,
        max_dist_m=2.50,
        min_cluster_px=250,
        confidence_frames=5,
    ):

        self.cam_mount_height = cam_mount_height
        self.cam_pitch_deg = cam_pitch_deg

        self.stop_safety_dist = stop_safety_dist
        self.max_climb_height = max_climb_height
        self.chassis_clear_height = chassis_clear_height
        self.track_safe_width = track_safe_width

        self.ground_margin = ground_margin
        self.min_dist_m = min_dist_m
        self.max_dist_m = max_dist_m
        self.min_cluster_px = min_cluster_px

        self.confidence_frames = confidence_frames

        pitch_rad = math.radians(
            self.cam_pitch_deg
        )

        self.cos_pitch = math.cos(
            pitch_rad
        )

        self.sin_pitch = math.sin(
            pitch_rad
        )

        self.consecutive_frames = 0

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

        self.cached_shape = None
        self.u_grid = None
        self.v_grid = None

    # ============================================================
    # DECISION LOGIC
    # ============================================================

    def evaluate_decision(
        self,
        dist_m,
        width_m,
        height_m
    ):
        """
        Action codes preserved from teammate perception code:

        0 = PATH CLEAR
        1 = STRADDLE
        2 = CLIMB / CROSS
        3 = DETOUR
        4 = STOP / PATH BLOCKED
        """

        if dist_m < self.stop_safety_dist:

            return (
                "EMERGENCY STOP (TOO CLOSE)",
                4
            )

        if height_m <= self.max_climb_height:

            if (
                height_m <= self.chassis_clear_height
                and
                width_m <= 0.30
            ):

                return (
                    "STRADDLE (UNDER-BELLY)",
                    1
                )

            return (
                "CLIMB / CROSS",
                2
            )

        if width_m <= self.track_safe_width:

            return (
                "DETOUR (NARROW OBSTACLE)",
                3
            )

        return (
            "PATH BLOCKED (FULL STOP)",
            4
        )

    # ============================================================
    # GRID CACHE
    # ============================================================

    def ensure_pixel_grid(
        self,
        height,
        width
    ):

        shape = (
            height,
            width
        )

        if shape == self.cached_shape:
            return

        self.u_grid, self.v_grid = (
            np.meshgrid(
                np.arange(width),
                np.arange(height)
            )
        )

        self.cached_shape = shape

    # ============================================================
    # MAIN PROCESSING
    # ============================================================

    def process(
        self,
        depth_m,
        fx,
        fy,
        cx,
        cy
    ):

        if depth_m is None:
            return self.clear_payload()

        if depth_m.ndim != 2:
            return self.clear_payload()

        height, width = depth_m.shape

        self.ensure_pixel_grid(
            height,
            width
        )

        # --------------------------------------------------------
        # Camera XYZ
        # --------------------------------------------------------

        x_cam = (
            (self.u_grid - cx)
            * depth_m
            / fx
        )

        y_cam = (
            (self.v_grid - cy)
            * depth_m
            / fy
        )

        z_cam = depth_m

        # --------------------------------------------------------
        # Correct for camera pitch
        # --------------------------------------------------------

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
            self.cam_mount_height
            -
            y_world
        )

        # --------------------------------------------------------
        # Dynamic ROI
        #
        # Same fractional ROI as original 640×480 implementation.
        # --------------------------------------------------------

        roi_x1 = int(
            width * 0.20
        )

        roi_x2 = int(
            width * 0.80
        )

        roi_y1 = int(
            height * 0.10
        )

        roi_y2 = int(
            height * 0.95
        )

        roi_mask = (
            (self.u_grid >= roi_x1)
            &
            (self.u_grid < roi_x2)
            &
            (self.v_grid >= roi_y1)
            &
            (self.v_grid < roi_y2)
        )

        valid_depth = (
            np.isfinite(depth_m)
            &
            (depth_m > self.min_dist_m)
            &
            (depth_m < self.max_dist_m)
        )

        obstacle_mask = (
            roi_mask
            &
            valid_depth
            &
            (
                height_above_ground
                >
                self.ground_margin
            )
        )

        # --------------------------------------------------------
        # Morphological filtering
        # --------------------------------------------------------

        binary_obs = (
            obstacle_mask.astype(
                np.uint8
            )
            * 255
        )

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

        # --------------------------------------------------------
        # Select largest valid obstacle
        # --------------------------------------------------------

        raw_detected = False
        selected_label = -1
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
                selected_label = idx
                raw_detected = True

        # --------------------------------------------------------
        # Temporal confidence
        # --------------------------------------------------------

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
                    self.confidence_frames
                )
                * 100
            )
        )

        is_sure = (
            self.consecutive_frames
            >=
            self.confidence_frames
        )

        if (
            not is_sure
            or
            selected_label == -1
        ):

            payload = self.clear_payload()

            payload[
                'confidence_pct'
            ] = confidence_pct

            payload[
                'raw_detected'
            ] = raw_detected

            return payload

        # --------------------------------------------------------
        # Obstacle metrics
        # --------------------------------------------------------

        component_mask = (
            labels == selected_label
        )

        z_cls = z_world[
            component_mask
        ]

        x_cls = x_cam[
            component_mask
        ]

        h_cls = (
            height_above_ground[
                component_mask
            ]
        )

        finite = (
            np.isfinite(z_cls)
            &
            np.isfinite(x_cls)
            &
            np.isfinite(h_cls)
        )

        z_cls = z_cls[finite]
        x_cls = x_cls[finite]
        h_cls = h_cls[finite]

        if z_cls.size < 20:

            return self.clear_payload()

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

        # Median X gives obstacle centre relative to camera.
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
            self.evaluate_decision(
                dist_m,
                width_m,
                height_m
            )
        )

        return {
            'obstacle_detected': True,
            'raw_detected': True,
            'surety': True,
            'confidence_pct': 100,

            'distance_m': round(
                dist_m,
                3
            ),

            'forward_m': round(
                dist_m,
                3
            ),

            'lateral_m': round(
                lateral_m,
                3
            ),

            'width_m': round(
                width_m,
                3
            ),

            'height_m': round(
                height_m,
                3
            ),

            'decision': decision,
            'action_code': action_code,
        }

    # ============================================================
    # CLEAR PAYLOAD
    # ============================================================

    @staticmethod
    def clear_payload():

        return {
            'obstacle_detected': False,
            'raw_detected': False,
            'surety': False,
            'confidence_pct': 0,

            'distance_m': 0.0,
            'forward_m': 0.0,
            'lateral_m': 0.0,

            'width_m': 0.0,
            'height_m': 0.0,

            'decision': 'PATH CLEAR',
            'action_code': 0,
        }
