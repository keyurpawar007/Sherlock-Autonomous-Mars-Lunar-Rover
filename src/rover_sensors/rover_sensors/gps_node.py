#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import NavSatFix, NavSatStatus


class GPSNode(Node):
    """
    GPS Driver Node.
    Publishes /sensors/gps (sensor_msgs/msg/NavSatFix).
    Supports physical NMEA GPS reading and synthetic mock generation around test field coordinates.
    """

    def __init__(self):
        super().__init__('gps_node')

        self.declare_parameter('use_mock', True)
        self.declare_parameter('latitude_init', 18.531675228344593)   # Exact test field GPS coordinates
        self.declare_parameter('longitude_init', 73.86529303900305)
        self.declare_parameter('altitude_init', 560.0)

        self.use_mock = self.get_parameter('use_mock').value
        self.lat = self.get_parameter('latitude_init').value
        self.lon = self.get_parameter('longitude_init').value
        self.alt = self.get_parameter('altitude_init').value

        self.gps_pub = self.create_publisher(NavSatFix, '/sensors/gps', 10)
        self.timer = self.create_timer(0.2, self.publish_gps)  # 5 Hz GPS

        self.get_logger().info(f"GPS Node initialized (Lat: {self.lat}, Lon: {self.lon})")

    def publish_gps(self):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gps_link'

        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS

        if self.use_mock:
            # Add subtle Gaussian noise to mock real GPS wobble
            msg.latitude = self.lat + (math.sin(self.get_clock().now().nanoseconds / 1e9) * 0.000005)
            msg.longitude = self.lon + (math.cos(self.get_clock().now().nanoseconds / 1e9) * 0.000005)
            msg.altitude = self.alt
        else:
            msg.latitude = self.lat
            msg.longitude = self.lon
            msg.altitude = self.alt

        # Covariance matrix (1.5m variance)
        msg.position_covariance = [1.5, 0.0, 0.0,
                                    0.0, 1.5, 0.0,
                                    0.0, 0.0, 3.0]
        msg.position_covariance_type = NavSatFix.COVARIANCE_TYPE_DIAGONAL_KNOWN

        self.gps_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = GPSNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
