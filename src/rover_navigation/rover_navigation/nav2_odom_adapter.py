#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class Nav2OdomAdapter(Node):

    def __init__(self):
        super().__init__('nav2_odom_adapter')

        self.sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            20
        )

        self.pub = self.create_publisher(
            Odometry,
            '/odom_nav',
            20
        )

        self.get_logger().info(
            'Nav2 odom adapter ready: '
            '/odom(base_footprint) -> /odom_nav(nav_base_link)'
        )

    def odom_callback(self, msg):

        out = Odometry()

        out.header = msg.header
        out.child_frame_id = 'nav_base_link'

        # nav_base_link has zero translation from base_footprint,
        # so position stays identical.
        out.pose.pose.position = msg.pose.pose.position

        # Existing /odom orientation describes base_footprint.
        # nav_base_link is rotated -90 degrees from base_footprint.
        q = msg.pose.pose.orientation

        base_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        nav_yaw = base_yaw - math.pi / 2.0

        out.pose.pose.orientation.x = 0.0
        out.pose.pose.orientation.y = 0.0
        out.pose.pose.orientation.z = math.sin(nav_yaw / 2.0)
        out.pose.pose.orientation.w = math.cos(nav_yaw / 2.0)

        out.pose.covariance = msg.pose.covariance

        # Convert velocity from base_footprint coordinates
        # into nav_base_link coordinates.
        #
        # nav +X = physical rover forward = base -Y
        base_vx = msg.twist.twist.linear.x
        base_vy = msg.twist.twist.linear.y

        out.twist.twist.linear.x = -base_vy
        out.twist.twist.linear.y = base_vx
        out.twist.twist.linear.z = msg.twist.twist.linear.z

        out.twist.twist.angular.x = msg.twist.twist.angular.x
        out.twist.twist.angular.y = msg.twist.twist.angular.y
        out.twist.twist.angular.z = msg.twist.twist.angular.z

        out.twist.covariance = msg.twist.covariance

        self.pub.publish(out)


def main(args=None):

    rclpy.init(args=args)
    node = Nav2OdomAdapter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
