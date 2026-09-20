#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Bool


class LocalizationMonitorNode(Node):
    """
    Localization Health & Covariance Monitor.
    Tracks state estimation uncertainty from /odometry/fused.
    If covariance trace exceeds safety threshold, flags localization loss to Safety Manager.
    """

    def __init__(self):
        super().__init__('localization_monitor_node')

        self.declare_parameter('max_allowed_variance', 4.0)  # max variance threshold in m^2
        self.max_variance = self.get_parameter('max_allowed_variance').value

        # Subscribers
        self.create_subscription(Odometry, '/odometry/fused', self.odom_cb, 10)

        # Publishers
        self.variance_pub = self.create_publisher(Float32, '/localization/variance', 10)
        self.loss_pub = self.create_publisher(Bool, '/localization/lost', 10)

        self.get_logger().info("Localization Monitor Node initialized")

    def odom_cb(self, msg: Odometry):
        # Extract covariance for x and y position
        var_x = msg.pose.covariance[0]
        var_y = msg.pose.covariance[7]
        total_variance = math.sqrt(abs(var_x) + abs(var_y))

        var_msg = Float32()
        var_msg.data = float(total_variance)
        self.variance_pub.publish(var_msg)

        lost_msg = Bool()
        if total_variance > self.max_variance:
            lost_msg.data = True
            self.get_logger().warn(f"HIGH LOCALIZATION UNCERTAINTY! Total Variance: {total_variance:.2f} > {self.max_variance}")
        else:
            lost_msg.data = False

        self.loss_pub.publish(lost_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LocalizationMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
