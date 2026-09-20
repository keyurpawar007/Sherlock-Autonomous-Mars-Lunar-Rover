#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from tf2_ros import Buffer, TransformListener, TransformException

from rover_interfaces.msg import ToolboxGoal


class ToolboxGoalBridge(Node):

    def __init__(self):

        super().__init__(
            'toolbox_goal_bridge'
        )

        self.declare_parameter(
            'map_frame',
            'map'
        )

        self.declare_parameter(
            'camera_frame',
            'camera_optical_frame'
        )

        self.declare_parameter(
            'stop_offset',
            0.45
        )

        self.map_frame = self.get_parameter(
            'map_frame'
        ).value

        self.camera_frame = self.get_parameter(
            'camera_frame'
        ).value

        self.stop_offset = float(
            self.get_parameter(
                'stop_offset'
            ).value
        )

        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

        self.create_subscription(
            ToolboxGoal,
            '/perception/toolbox_goal',
            self.callback,
            10
        )

        self.last_goal = None

        self.get_logger().info(
            'Toolbox temporary-goal bridge started'
        )

    def callback(self, msg):

        if not msg.detected:
            return

        if msg.goal_reached:

            self.get_logger().info(
                'TOOLBOX GOAL REACHED'
            )

            return

        # Camera optical:
        # X = horizontal
        # Z = forward
        lateral = msg.position.x
        forward = msg.position.z

        distance = math.hypot(
            forward,
            lateral
        )

        if distance <= self.stop_offset:
            return

        # Stop slightly before the toolbox.
        scale = (
            distance - self.stop_offset
        ) / distance

        forward *= scale
        lateral *= scale

        try:

            tf = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.camera_frame,
                Time()
            )

        except TransformException:
            return

        # Camera point in optical coordinates:
        # x = lateral
        # y = 0
        # z = forward
        #
        # Rotate using transform quaternion.

        q = tf.transform.rotation

        px = lateral
        py = 0.0
        pz = forward

        # Quaternion vector rotation.
        # v' = q * v * q^-1
        ux, uy, uz = q.x, q.y, q.z
        s = q.w

        dot_uv = (
            ux * px +
            uy * py +
            uz * pz
        )

        dot_uu = (
            ux * ux +
            uy * uy +
            uz * uz
        )

        cross_x = (
            uy * pz -
            uz * py
        )

        cross_y = (
            uz * px -
            ux * pz
        )

        cross_z = (
            ux * py -
            uy * px
        )

        rx = (
            2.0 * dot_uv * ux
            +
            (s * s - dot_uu) * px
            +
            2.0 * s * cross_x
        )

        ry = (
            2.0 * dot_uv * uy
            +
            (s * s - dot_uu) * py
            +
            2.0 * s * cross_y
        )

        goal = PoseStamped()

        goal.header.frame_id = (
            self.map_frame
        )

        goal.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        goal.pose.position.x = (
            tf.transform.translation.x
            + rx
        )

        goal.pose.position.y = (
            tf.transform.translation.y
            + ry
        )

        goal.pose.orientation.w = 1.0

        # Don't spam identical goals every camera frame.
        current = (
            goal.pose.position.x,
            goal.pose.position.y
        )

        if self.last_goal is not None:

            change = math.hypot(
                current[0] - self.last_goal[0],
                current[1] - self.last_goal[1]
            )

            if change < 0.15:
                return

        self.last_goal = current

        self.goal_pub.publish(goal)

        self.get_logger().info(
            'Toolbox goal published: '
            f'x={current[0]:.2f}, '
            f'y={current[1]:.2f}, '
            f'distance={msg.distance_m:.2f}m'
        )


def main(args=None):

    rclpy.init(args=args)

    node = ToolboxGoalBridge()

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
