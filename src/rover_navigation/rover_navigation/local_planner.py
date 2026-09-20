#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from tf2_ros import Buffer, TransformListener, TransformException


class LocalPlanner(Node):

    def __init__(self):
        super().__init__('local_planner')

        self.declare_parameter('lookahead_distance', 0.50)
        self.declare_parameter('robot_frame', 'base_footprint')

        self.lookahead_distance = (
            self.get_parameter('lookahead_distance')
            .get_parameter_value().double_value
        )

        self.robot_frame = (
            self.get_parameter('robot_frame')
            .get_parameter_value().string_value
        )

        self.global_path = None
        self.last_target_index = None
        self.progress_index = 0

        # TF
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # Subscribe to global A* path
        self.create_subscription(
            Path,
            '/global_path',
            self.path_callback,
            10
        )

        # Publish local waypoint
        self.local_goal_pub = self.create_publisher(
            PoseStamped,
            '/local_goal',
            10
        )

        # Re-evaluate local goal as rover moves
        self.create_timer(
            0.2,
            self.update_local_goal
        )

        self.get_logger().info('Local Planner started')
        self.get_logger().info('Global path : /global_path')
        self.get_logger().info('Local goal  : /local_goal')
        self.get_logger().info(
            f'Lookahead   : {self.lookahead_distance:.2f} m'
        )

    def path_callback(self, msg):

        if not msg.poses:
            self.global_path = None
            self.last_target_index = None
            self.progress_index = 0

            self.get_logger().warning(
                'Received empty global path'
            )
            return

        self.global_path = msg
        self.last_target_index = None
        self.progress_index = 0

        self.get_logger().info(
            f'Global path received: {len(msg.poses)} poses'
        )

    def get_robot_position(self, frame):

        try:
            transform = self.tf_buffer.lookup_transform(
                frame,
                self.robot_frame,
                Time()
            )

            return (
                transform.transform.translation.x,
                transform.transform.translation.y
            )

        except TransformException as exc:

            self.get_logger().warning(
                f'Cannot get rover pose: {exc}'
            )

            return None

    def update_local_goal(self):

        if self.global_path is None:
            return

        if not self.global_path.poses:
            return

        frame = self.global_path.header.frame_id

        if not frame:
            frame = 'map'

        robot_position = self.get_robot_position(frame)

        if robot_position is None:
            return

        rx, ry = robot_position

        # -----------------------------------------------------
        # Find closest global-path point to rover
        # -----------------------------------------------------
        nearest_index = self.progress_index
        nearest_distance = float('inf')

        # Search only from the furthest confirmed progress point
        # onward. This prevents the local target from jumping
        # backward along the global path.
        for i in range(
            self.progress_index,
            len(self.global_path.poses)
        ):

            pose_stamped = self.global_path.poses[i]

            px = pose_stamped.pose.position.x
            py = pose_stamped.pose.position.y

            distance = math.hypot(
                px - rx,
                py - ry
            )

            if distance < nearest_distance:
                nearest_distance = distance
                nearest_index = i

        # Progress is monotonic for a single global path.
        self.progress_index = max(
            self.progress_index,
            nearest_index
        )

        # -----------------------------------------------------
        # Walk forward along path until lookahead distance
        # -----------------------------------------------------
        target_index = nearest_index
        travelled = 0.0

        for i in range(
            nearest_index,
            len(self.global_path.poses) - 1
        ):

            p1 = (
                self.global_path
                .poses[i]
                .pose.position
            )

            p2 = (
                self.global_path
                .poses[i + 1]
                .pose.position
            )

            travelled += math.hypot(
                p2.x - p1.x,
                p2.y - p1.y
            )

            target_index = i + 1

            if travelled >= self.lookahead_distance:
                break

        # -----------------------------------------------------
        # Publish selected local waypoint
        # -----------------------------------------------------
        if self.last_target_index is not None:
            target_index = max(
                target_index,
                self.last_target_index
            )

        # -----------------------------------------------------
        # Final-goal lock
        #
        # Once the rover is close to the end of the global path,
        # stop chasing lookahead points and aim directly at the
        # exact final X/Y goal.
        # -----------------------------------------------------
        final_pose = self.global_path.poses[-1].pose.position

        distance_to_final = math.hypot(
            final_pose.x - rx,
            final_pose.y - ry
        )

        if distance_to_final <= 0.75:
            target_index = len(self.global_path.poses) - 1

        selected = self.global_path.poses[target_index]

        local_goal = PoseStamped()

        local_goal.header.frame_id = frame
        local_goal.header.stamp = (
            self.get_clock().now().to_msg()
        )

        local_goal.pose = selected.pose

        self.local_goal_pub.publish(local_goal)

        if target_index != self.last_target_index:

            self.get_logger().info(
                'Local target selected: '
                f'index={target_index}, '
                f'x={local_goal.pose.position.x:.2f}, '
                f'y={local_goal.pose.position.y:.2f}, '
                f'lookahead={travelled:.2f} m'
            )

            self.last_target_index = target_index


def main(args=None):

    rclpy.init(args=args)

    node = LocalPlanner()

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
