#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


class GoalManagerNode(Node):
    """
    Goal Waypoint & Speed-Adaptive Approach Manager.
    Receives navigation goals, tracks current distance to goal,
    and dynamically shapes linear/angular velocity scaling for safe docking.
    """

    def __init__(self):
        super().__init__('goal_manager')

        self.current_pose = None
        self.active_goal = None

        # Subscriptions
        self.create_subscription(Odometry, '/odometry/fused', self.odom_cb, 10)
        self.create_subscription(PoseStamped, '/navigation/goal', self.goal_cb, 10)

        # Publisher for shaped commands
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.create_timer(0.05, self.control_loop)  # 20 Hz
        self.get_logger().info("Goal & Adaptive Speed Manager Node initialized")

    def odom_cb(self, msg: Odometry):
        self.current_pose = msg.pose.pose

    def goal_cb(self, msg: PoseStamped):
        self.active_goal = msg.pose
        self.get_logger().info(f"New Navigation Goal Received! Target X: {self.active_goal.position.x:.2f}m, Y: {self.active_goal.position.y:.2f}m")

    def control_loop(self):
        if self.current_pose is None or self.active_goal is None:
            return

        dx = self.active_goal.position.x - self.current_pose.position.x
        dy = self.active_goal.position.y - self.current_pose.position.y
        dist = math.sqrt(dx * dx + dy * dy)

        # Target heading
        target_yaw = math.atan2(dy, dx)

        # Current yaw from orientation quaternion
        qz = self.current_pose.orientation.z
        qw = self.current_pose.orientation.w
        current_yaw = 2.0 * math.atan2(qz, qw)

        yaw_error = target_yaw - current_yaw
        # Normalize angle error to [-pi, pi]
        yaw_error = math.atan2(math.sin(yaw_error), math.cos(yaw_error))

        cmd = Twist()

        if dist < 0.2:  # Reached target
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
            self.active_goal = None
            self.get_logger().info("GOAL REACHED SUCCESSFULLY!")
        else:
            # Distance-based speed profile
            if dist > 2.0:
                v_max = 1.0     # Fast speed
            elif dist > 0.5:
                v_max = 0.4     # Medium speed
            else:
                v_max = 0.15    # Creep speed

            # Angular scaling
            cmd.angular.z = float(max(min(1.2 * yaw_error, 0.8), -0.8))

            # Reduce forward speed if angle error is large
            if abs(yaw_error) > 0.5:
                cmd.linear.x = 0.0
            else:
                cmd.linear.x = float(v_max * math.cos(yaw_error))

        self.cmd_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = GoalManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
