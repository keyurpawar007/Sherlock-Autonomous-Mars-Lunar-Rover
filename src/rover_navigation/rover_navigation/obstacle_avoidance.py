#!/usr/bin/env python3

import math
import numpy as np

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image
from std_msgs.msg import String


class ObstacleAvoidance(Node):

    def __init__(self):
        super().__init__('obstacle_avoidance')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------
        self.declare_parameter(
            'depth_topic',
            '/d435i/depth/image_raw'
        )

        self.declare_parameter(
            'avoidance_topic',
            '/avoidance_cmd'
        )

        self.declare_parameter(
            'status_topic',
            '/obstacle_status'
        )

        # Required rover behavior
        self.declare_parameter('stop_distance', 2.00)
        self.declare_parameter('caution_distance', 2.30)

        self.depth_topic = (
            self.get_parameter('depth_topic')
            .get_parameter_value().string_value
        )

        self.avoidance_topic = (
            self.get_parameter('avoidance_topic')
            .get_parameter_value().string_value
        )

        self.status_topic = (
            self.get_parameter('status_topic')
            .get_parameter_value().string_value
        )

        self.stop_distance = (
            self.get_parameter('stop_distance')
            .get_parameter_value().double_value
        )

        self.caution_distance = (
            self.get_parameter('caution_distance')
            .get_parameter_value().double_value
        )

        # ---------------------------------------------------------
        # ROS interfaces
        # ---------------------------------------------------------
        self.create_subscription(
            Image,
            self.depth_topic,
            self.depth_callback,
            10
        )

        self.avoidance_pub = self.create_publisher(
            Twist,
            self.avoidance_topic,
            10
        )

        self.status_pub = self.create_publisher(
            String,
            self.status_topic,
            10
        )

        self.last_status = None

        self.get_logger().info(
            'Depth Obstacle Avoidance started'
        )
        self.get_logger().info(
            f'Depth image     : {self.depth_topic}'
        )
        self.get_logger().info(
            f'Avoidance cmd   : {self.avoidance_topic}'
        )
        self.get_logger().info(
            f'Obstacle status : {self.status_topic}'
        )
        self.get_logger().info(
            f'Stop distance   : {self.stop_distance:.2f} m'
        )
        self.get_logger().info(
            f'Caution distance: {self.caution_distance:.2f} m'
        )

    # -------------------------------------------------------------
    # Depth helpers
    # -------------------------------------------------------------
    @staticmethod
    def region_distance(region):

        valid = region[
            np.isfinite(region)
            & (region >= 0.20)
            & (region <= 6.00)
        ]

        if valid.size < 20:
            return math.inf

        # Do NOT use the absolute minimum.
        # 10th percentile ignores isolated bad/noisy depth pixels.
        return float(
            np.percentile(valid, 5)
        )

    # -------------------------------------------------------------
    # Depth callback
    # -------------------------------------------------------------
    def depth_callback(self, msg):

        if msg.encoding != '32FC1':
            self.get_logger().warning(
                f'Expected 32FC1 depth, got {msg.encoding}'
            )
            return

        dtype = (
            np.dtype('>f4')
            if msg.is_bigendian
            else np.dtype('<f4')
        )

        depth = np.frombuffer(
            msg.data,
            dtype=dtype
        )

        expected = msg.height * msg.width

        if depth.size < expected:
            return

        depth = depth[:expected].reshape(
            msg.height,
            msg.width
        )

        h = msg.height
        w = msg.width

        # ---------------------------------------------------------
        # Central forward ROI
        #
        # For 424 x 240:
        # x ≈ 127 ... 297
        # y ≈ 48  ... 132
        #
        # We intentionally ignore the lower part of the image
        # so the floor / rover body does not trigger STOP.
        # ---------------------------------------------------------
        x1 = int(w * 0.50)
        x2 = int(w * 0.65)

        y1 = int(h * 0.20)
        y2 = int(h * 0.35)

        roi = depth[
            y1:y2,
            x1:x2
        ]

        mid = roi.shape[1] // 2

        left_roi = roi[:, :mid]
        right_roi = roi[:, mid:]

        left_distance = self.region_distance(
            left_roi
        )

        right_distance = self.region_distance(
            right_roi
        )

        # Use the nearest forward sector as the effective
        # obstacle distance. This prevents a side obstacle
        # from being diluted by far pixels in the other half.
        front_distance = min(
            left_distance,
            right_distance
        )

        command = Twist()

        # ---------------------------------------------------------
        # BLOCKED: <= 1.5 m
        # ---------------------------------------------------------
        if front_distance <= self.stop_distance:

            # Safety layer only.
            # Stop translation and allow A* to choose the new route.
            command.linear.x = 0.0
            command.angular.z = 0.0
            state = 'BLOCKED_REPLAN'

        # ---------------------------------------------------------
        # CAUTION
        # ---------------------------------------------------------
        elif front_distance <= self.caution_distance:

            # Slow down, but do NOT choose left/right here.
            # Route direction belongs to the A* planner.
            command.linear.x = 0.25
            command.angular.z = 0.0
            state = 'CAUTION_SLOW'

        # ---------------------------------------------------------
        # CLEAR
        # ---------------------------------------------------------
        else:

            command.linear.x = 1.0
            command.angular.z = 0.0
            state = 'CLEAR'

        self.avoidance_pub.publish(
            command
        )

        status = String()

        status.data = (
            f'{state} '
            f'front={front_distance:.2f}m '
            f'left={left_distance:.2f}m '
            f'right={right_distance:.2f}m'
        )

        self.status_pub.publish(
            status
        )

        if state != self.last_status:

            self.get_logger().info(
                status.data
            )

            self.last_status = state


def main(args=None):

    rclpy.init(args=args)

    node = ObstacleAvoidance()

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
