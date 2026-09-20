#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState


class JointStateRetimestamp(Node):

    def __init__(self):
        super().__init__('joint_state_retimestamp')

        self.pub = self.create_publisher(
            JointState,
            '/joint_states_synced',
            10
        )

        self.create_subscription(
            JointState,
            '/joint_states',
            self.callback,
            10
        )

        self.get_logger().info(
            '/joint_states -> /joint_states_synced (system timestamps)'
        )

    def callback(self, msg):

        out = JointState()

        out.header = msg.header
        out.header.stamp = self.get_clock().now().to_msg()

        out.name = list(msg.name)
        out.position = list(msg.position)
        out.velocity = list(msg.velocity)
        out.effort = list(msg.effort)

        self.pub.publish(out)


def main(args=None):

    rclpy.init(args=args)

    node = JointStateRetimestamp()

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
