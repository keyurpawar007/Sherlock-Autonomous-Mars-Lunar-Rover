#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class IMUNode(Node):
    """
    IMU Driver Node.
    Publishes /sensors/imu (sensor_msgs/msg/Imu).
    Publishes orientation quaternion, angular velocities, and linear accelerations.
    """

    def __init__(self):
        super().__init__('imu_node')

        self.declare_parameter('use_mock', True)
        self.use_mock = self.get_parameter('use_mock').value

        self.imu_pub = self.create_publisher(Imu, '/sensors/imu', 10)
        self.timer = self.create_timer(0.02, self.publish_imu)  # 50 Hz IMU

        self.yaw = 0.0
        self.get_logger().info(f"IMU Node initialized (Mock: {self.use_mock})")

    def publish_imu(self):
        msg = Imu()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'imu_link'

        # Orientation quaternion
        qx = 0.0
        qy = 0.0
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        msg.orientation.x = qx
        msg.orientation.y = qy
        msg.orientation.z = qz
        msg.orientation.w = qw

        # Covariance
        msg.orientation_covariance = [0.01, 0.0, 0.0,
                                      0.0, 0.01, 0.0,
                                      0.0, 0.0, 0.01]

        # Angular Velocity
        msg.angular_velocity.x = 0.0
        msg.angular_velocity.y = 0.0
        msg.angular_velocity.z = 0.0
        msg.angular_velocity_covariance = [0.001, 0.0, 0.0,
                                           0.0, 0.001, 0.0,
                                           0.0, 0.0, 0.001]

        # Linear Acceleration (Gravity on Z)
        msg.linear_acceleration.x = 0.0
        msg.linear_acceleration.y = 0.0
        msg.linear_acceleration.z = 9.81
        msg.linear_acceleration_covariance = [0.01, 0.0, 0.0,
                                              0.0, 0.01, 0.0,
                                              0.0, 0.0, 0.01]

        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = IMUNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
