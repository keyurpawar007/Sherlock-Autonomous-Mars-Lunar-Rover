#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Path
from std_msgs.msg import Float64MultiArray, Bool

from tf2_ros import Buffer, TransformListener, TransformException


class DirectPathFollower(Node):

    def __init__(self):
        super().__init__('direct_path_follower')

        self.robot_frame = 'base_footprint'

        self.base_wheel_speed = 2.0
        self.max_turn_component = 1.6
        self.heading_gain = 1.5

        self.lookahead_distance = 0.45
        self.final_lock_distance = 0.45
        self.final_tolerance = 0.10

        self.path = None
        self.progress_index = 0

        self.goal_reached = False
        self.avoidance_cmd = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        self.create_subscription(
            Path,
            '/global_path',
            self.path_callback,
            10
        )

        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.new_goal_callback,
            10
        )

        self.create_subscription(
            Twist,
            '/avoidance_cmd',
            self.avoidance_callback,
            10
        )

        self.create_subscription(
            Bool,
            '/goal_reached',
            self.goal_reached_callback,
            10
        )

        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            '/nav_wheel_commands',
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'DIRECT PATH FOLLOWER STARTED'
        )

    def path_callback(self, msg):

        if not msg.poses:
            return

        self.path = msg
        self.progress_index = 0
        self.goal_reached = False

        final = msg.poses[-1].pose.position

        self.get_logger().info(
            f'New global path: {len(msg.poses)} points | '
            f'final=({final.x:.2f}, {final.y:.2f})'
        )

    def new_goal_callback(self, msg):

        # Stop using any previous path until the global planner
        # publishes a fresh path for this new goal.
        self.path = None
        self.progress_index = 0
        self.goal_reached = False
        self.stop()

    def avoidance_callback(self, msg):
        self.avoidance_cmd = msg

    def goal_reached_callback(self, msg):

        self.goal_reached = msg.data

        if self.goal_reached:
            self.stop()

    def get_robot_pose(self, frame):

        try:
            transform = self.tf_buffer.lookup_transform(
                frame,
                self.robot_frame,
                Time()
            )

        except TransformException:
            return None

        x = transform.transform.translation.x
        y = transform.transform.translation.y

        q = transform.transform.rotation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        return x, y, yaw

    @staticmethod
    def clamp(value, low, high):
        return max(low, min(high, value))

    def publish_wheels(self, right, left):

        msg = Float64MultiArray()

        # FR MR BR FL ML BL
        msg.data = [
            float(right),
            float(right),
            float(right),
            float(left),
            float(left),
            float(left)
        ]

        self.wheel_pub.publish(msg)

    def stop(self):
        self.publish_wheels(0.0, 0.0)

    def control_loop(self):

        if self.goal_reached:
            self.stop()
            return

        if self.path is None or not self.path.poses:
            self.stop()
            return

        frame = self.path.header.frame_id or 'map'

        robot = self.get_robot_pose(frame)

        if robot is None:
            self.stop()
            return

        rx, ry, yaw = robot

        final = self.path.poses[-1].pose.position

        final_distance = math.hypot(
            final.x - rx,
            final.y - ry
        )

        # Exact final point reached.
        if final_distance <= self.final_tolerance:
            self.stop()
            return

        # -----------------------------------------------------
        # Find nearest point ahead on the path
        # -----------------------------------------------------

        nearest_index = self.progress_index
        nearest_distance = float('inf')

        for i in range(
            self.progress_index,
            len(self.path.poses)
        ):

            p = self.path.poses[i].pose.position

            d = math.hypot(
                p.x - rx,
                p.y - ry
            )

            if d < nearest_distance:
                nearest_distance = d
                nearest_index = i

        self.progress_index = max(
            self.progress_index,
            nearest_index
        )

        # -----------------------------------------------------
        # Target selection
        #
        # Far away: use path lookahead.
        # Near goal: aim DIRECTLY at exact final point.
        # -----------------------------------------------------

        if final_distance <= self.final_lock_distance:

            target = final

        else:

            target_index = nearest_index
            travelled = 0.0

            for i in range(
                nearest_index,
                len(self.path.poses) - 1
            ):

                p1 = self.path.poses[i].pose.position
                p2 = self.path.poses[i + 1].pose.position

                travelled += math.hypot(
                    p2.x - p1.x,
                    p2.y - p1.y
                )

                target_index = i + 1

                if travelled >= self.lookahead_distance:
                    break

            target = (
                self.path.poses[target_index]
                .pose.position
            )

        # -----------------------------------------------------
        # Heading to target
        # -----------------------------------------------------

        dx = target.x - rx
        dy = target.y - ry

        base_x = (
            math.cos(yaw) * dx +
            math.sin(yaw) * dy
        )

        base_y = (
            -math.sin(yaw) * dx +
            math.cos(yaw) * dy
        )

        # Rover physical forward = base -Y.
        heading_error = math.atan2(
            base_x,
            -base_y
        )

        steering = self.clamp(
            self.heading_gain * heading_error,
            -1.0,
            1.0
        )

        # -----------------------------------------------------
        # Motion control
        #
        # Normal path:
        #   smooth arc following.
        #
        # Final 0.8 m:
        #   align first, then drive slowly to the exact point.
        # -----------------------------------------------------

        angle = abs(math.degrees(heading_error))

        if final_distance <= self.final_lock_distance:

            # Final approach:
            # never stop just because a small turn is needed.
            # Keep moving slowly while steering toward exact goal.

            if angle > 25.0:
                forward_speed = self.base_wheel_speed * 0.25

                if steering >= 0.0:
                    steering = max(steering, 0.80)
                else:
                    steering = min(steering, -0.80)

            elif angle > 7.0:
                forward_speed = self.base_wheel_speed * 0.35

                if steering >= 0.0:
                    steering = max(steering, 0.50)
                else:
                    steering = min(steering, -0.50)

            else:
                forward_speed = self.base_wheel_speed * 0.30

            # Slow down near the point, but never almost-zero.
            if final_distance < 0.20:
                forward_speed = min(
                    forward_speed,
                    self.base_wheel_speed * 0.25
                )

        else:

            # Normal path following.
            if angle > 60.0:
                speed_scale = 0.25
            elif angle > 35.0:
                speed_scale = 0.40
            elif angle > 15.0:
                speed_scale = 0.65
            else:
                speed_scale = 1.0

            forward_speed = (
                self.base_wheel_speed *
                speed_scale
            )

        # -----------------------------------------------------
        # Obstacle avoidance
        # -----------------------------------------------------

        avoidance_linear = 1.0
        avoidance_turn = 0.0

        if self.avoidance_cmd is not None:

            avoidance_linear = self.clamp(
                self.avoidance_cmd.linear.x,
                0.0,
                1.0
            )

            avoidance_turn = self.clamp(
                self.avoidance_cmd.angular.z,
                -1.0,
                1.0
            )

        forward_speed *= avoidance_linear

        combined_turn = self.clamp(
            steering + avoidance_turn,
            -1.0,
            1.0
        )

        # BLOCKED: obstacle avoidance gets full priority.
        if (
            avoidance_linear <= 0.01
            and
            abs(avoidance_turn) > 0.01
        ):

            forward_speed = 0.0
            combined_turn = avoidance_turn

        turn_component = (
            self.max_turn_component *
            combined_turn
        )

        right_speed = (
            forward_speed +
            turn_component
        )

        left_speed = (
            forward_speed -
            turn_component
        )

        self.publish_wheels(
            right_speed,
            left_speed
        )


def main(args=None):

    rclpy.init(args=args)

    node = DirectPathFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.stop()
    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
