#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
    DurabilityPolicy,
    qos_profile_sensor_data,
)

from sensor_msgs.msg import LaserScan


class ScanRetimestamp(Node):

    def __init__(self):
        super().__init__('scan_retimestamp')

        # Gazebo scan uses simulator-relative timestamps.
        self.create_subscription(
            LaserScan,
            '/scan_raw',
            self.callback,
            qos_profile_sensor_data
        )

        # Publish system-time scan for SLAM/RViz/navigation.
        output_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        self.publisher = self.create_publisher(
            LaserScan,
            '/scan',
            output_qos
        )

        self.get_logger().info(
            'LaserScan timestamp synchronizer started'
        )

        self.get_logger().info(
            '/scan_raw -> /scan (system time)'
        )

    def callback(self, msg):

        # Backdate slightly so the corresponding TF is already
        # available in Nav2's transform buffer.
        now = self.get_clock().now()
        msg.header.stamp = (
            now - Duration(seconds=0.20)
        ).to_msg()

        self.publisher.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = ScanRetimestamp()

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
