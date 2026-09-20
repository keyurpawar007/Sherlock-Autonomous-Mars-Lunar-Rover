#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from nav_msgs.msg import OccupancyGrid


class SlamMonitor(Node):

    def __init__(self):
        super().__init__('rover_slam_node')

        self.scan_received = False
        self.odom_received = False
        self.map_received = False

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.timer = self.create_timer(
            2.0,
            self.status_callback
        )

        self.get_logger().info(
            'Rover SLAM monitor started'
        )

        self.get_logger().info(
            'Waiting for /scan, /odom and /map'
        )

    def scan_callback(self, msg):
        self.scan_received = True

    def odom_callback(self, msg):
        self.odom_received = True

    def map_callback(self, msg):

        first_map = not self.map_received
        self.map_received = True

        if first_map:
            self.get_logger().info(
                'SLAM MAP RECEIVED SUCCESSFULLY'
            )

            self.get_logger().info(
                f'Map size: {msg.info.width} x {msg.info.height}'
            )

            self.get_logger().info(
                f'Map resolution: '
                f'{msg.info.resolution:.3f} m/cell'
            )

    def status_callback(self):

        scan_status = 'OK' if self.scan_received else 'WAITING'
        odom_status = 'OK' if self.odom_received else 'WAITING'
        map_status = 'OK' if self.map_received else 'WAITING'

        self.get_logger().info(
            'SLAM STATUS | '
            f'/scan: {scan_status} | '
            f'/odom: {odom_status} | '
            f'/map: {map_status}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = SlamMonitor()

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

