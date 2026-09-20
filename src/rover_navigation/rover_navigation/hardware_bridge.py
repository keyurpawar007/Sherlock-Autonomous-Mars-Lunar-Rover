#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import Bool, Float64MultiArray


class HardwareBridge(Node):

    def __init__(self):
        super().__init__('hardware_bridge')

        self.declare_parameter('max_wheel_speed', 4.0)

        self.max_wheel_speed = float(
            self.get_parameter('max_wheel_speed').value
        )

        self.estop = False

        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/nav_wheel_commands',
            self.command_callback,
            10
        )

        self.estop_sub = self.create_subscription(
            Bool,
            '/emergency_stop',
            self.estop_callback,
            10
        )

        self.hw_pub = self.create_publisher(
            Float64MultiArray,
            '/hardware/wheel_velocity_commands',
            10
        )

        self.get_logger().info(
            f'Hardware bridge ready | '
            f'max wheel speed={self.max_wheel_speed:.2f}'
        )

    def clamp(self, value):
        return max(
            -self.max_wheel_speed,
            min(self.max_wheel_speed, value)
        )

    def publish_stop(self):
        msg = Float64MultiArray()
        msg.data = [0.0] * 6
        self.hw_pub.publish(msg)

    def estop_callback(self, msg):

        self.estop = bool(msg.data)

        if self.estop:
            self.publish_stop()
            self.get_logger().warning(
                'EMERGENCY STOP ACTIVE'
            )
        else:
            self.get_logger().info(
                'Emergency stop released'
            )

    def command_callback(self, msg):

        if self.estop:
            self.publish_stop()
            return

        if len(msg.data) != 6:
            self.get_logger().error(
                'Expected exactly 6 wheel commands'
            )
            self.publish_stop()
            return

        out = Float64MultiArray()

        out.data = [
            self.clamp(float(v))
            for v in msg.data
        ]

        self.hw_pub.publish(out)


def main(args=None):

    rclpy.init(args=args)

    node = HardwareBridge()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.publish_stop()
    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
