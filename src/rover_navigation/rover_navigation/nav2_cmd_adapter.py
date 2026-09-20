#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class Nav2CmdAdapter(Node):

    def __init__(self):
        super().__init__('nav2_cmd_adapter')

        self.sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_callback,
            10
        )

        self.pub = self.create_publisher(
            Twist,
            '/sim_cmd_vel',
            10
        )

        self.get_logger().info(
            'Nav2 command adapter ready: /cmd_vel -> /sim_cmd_vel'
        )

    def cmd_callback(self, msg):

        out = Twist()

        # Nav2:
        # +X = forward
        #
        # Our Gazebo rover:
        # -Y = physical forward

        out.linear.x = 0.0
        out.linear.y = -msg.linear.x
        out.linear.z = 0.0

        out.angular.x = 0.0
        out.angular.y = 0.0
        out.angular.z = msg.angular.z

        self.pub.publish(out)


def main(args=None):

    rclpy.init(args=args)

    node = Nav2CmdAdapter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
