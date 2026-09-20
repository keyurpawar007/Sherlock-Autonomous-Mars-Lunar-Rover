#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomTFBridge(Node):

    def __init__(self):
        super().__init__('odom_tf_bridge')

        self.br = TransformBroadcaster(self)

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info(
            'Publishing odom -> base_footprint TF from /odom'
        )

    def odom_callback(self, msg):

        tf = TransformStamped()

        tf.header.stamp = msg.header.stamp
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'

        tf.transform.translation.x = msg.pose.pose.position.x
        tf.transform.translation.y = msg.pose.pose.position.y
        tf.transform.translation.z = 0.0

        tf.transform.rotation = msg.pose.pose.orientation

        self.br.sendTransform(tf)


def main(args=None):

    rclpy.init(args=args)

    node = OdomTFBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
