#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from rover_interfaces.msg import WheelRPM
from tf2_ros import TransformBroadcaster


class KinematicsNode(Node):
    """
    Kinematics Node for 6-Wheel Skid-Steer/Differential Drive Rover.
    - Converts /cmd_vel (v, w) into 6-wheel RPMs published to /drive/rpm_command.
    - Receives /drive/rpm_feedback, computes dead-reckoning odometry, and publishes /odom + TF.
    """

    def __init__(self):
        super().__init__('kinematics_node')

        # Parameters based on exact physical dimensions:
        # Wheel Diameter 22 cm => Radius = 0.11 m
        # Track Width (Wheel to Wheel) = 1.0 m
        self.declare_parameter('wheel_radius', 0.11)
        self.declare_parameter('track_width', 1.00)
        self.declare_parameter('publish_tf', True)
        self.declare_parameter('max_rpm', 300.0)

        self.r = self.get_parameter('wheel_radius').value
        self.L = self.get_parameter('track_width').value
        self.publish_tf = self.get_parameter('publish_tf').value
        self.max_rpm = self.get_parameter('max_rpm').value

        # Odometry state
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.last_time = self.get_clock().now()

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Subscribers
        self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.create_subscription(WheelRPM, '/drive/rpm_feedback', self.rpm_feedback_callback, 10)

        # Publishers
        self.rpm_cmd_pub = self.create_publisher(WheelRPM, '/drive/rpm_command', 10)
        self.odom_pub = self.create_publisher(Odometry, '/odom', 10)

        self.get_logger().info(f"Kinematics Node initialized (Wheel Radius: {self.r}m, Track Width: {self.L}m)")

    def cmd_vel_callback(self, msg: Twist):
        v = msg.linear.x
        w = msg.angular.z

        # Differential drive velocity equations
        v_left = v - (w * self.L / 2.0)
        v_right = v + (w * self.L / 2.0)

        # Convert linear velocity to RPM: RPM = (v * 60) / (2 * pi * r)
        rpm_left = (v_left * 60.0) / (2.0 * math.pi * self.r)
        rpm_right = (v_right * 60.0) / (2.0 * math.pi * self.r)

        # Clamp RPMs to max hardware limits
        rpm_left = max(min(rpm_left, self.max_rpm), -self.max_rpm)
        rpm_right = max(min(rpm_right, self.max_rpm), -self.max_rpm)

        rpm_msg = WheelRPM()
        rpm_msg.header.stamp = self.get_clock().now().to_msg()
        # Assign same RPM to all 3 wheels on the left side
        rpm_msg.fl = float(rpm_left)
        rpm_msg.ml = float(rpm_left)
        rpm_msg.bl = float(rpm_left)
        # Assign same RPM to all 3 wheels on the right side
        rpm_msg.fr = float(rpm_right)
        rpm_msg.mr = float(rpm_right)
        rpm_msg.br = float(rpm_right)
        self.rpm_cmd_pub.publish(rpm_msg)

    def rpm_feedback_callback(self, msg: WheelRPM):
        current_time = self.get_clock().now()
        dt = (current_time - self.last_time).nanoseconds / 1e9
        self.last_time = current_time

        if dt <= 0.0:
            return

        # Average the RPM feedback for each side
        avg_left_rpm = (msg.fl + msg.ml + msg.bl) / 3.0
        avg_right_rpm = (msg.fr + msg.mr + msg.br) / 3.0

        # Measured wheel linear velocities from feedback RPMs
        v_left_meas = (avg_left_rpm * 2.0 * math.pi * self.r) / 60.0
        v_right_meas = (avg_right_rpm * 2.0 * math.pi * self.r) / 60.0

        v_linear = (v_right_meas + v_left_meas) / 2.0
        w_angular = (v_right_meas - v_left_meas) / self.L

        # Dead-reckoning integration
        delta_x = v_linear * math.cos(self.yaw) * dt
        delta_y = v_linear * math.sin(self.yaw) * dt
        delta_yaw = w_angular * dt

        self.x += delta_x
        self.y += delta_y
        self.yaw += delta_yaw

        # Quaternion from Yaw
        qx = 0.0
        qy = 0.0
        qz = math.sin(self.yaw / 2.0)
        qw = math.cos(self.yaw / 2.0)

        # Publish Odometry msg
        odom_msg = Odometry()
        odom_msg.header.stamp = current_time.to_msg()
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'

        odom_msg.pose.pose.position.x = self.x
        odom_msg.pose.pose.position.y = self.y
        odom_msg.pose.pose.position.z = 0.0
        odom_msg.pose.pose.orientation.x = qx
        odom_msg.pose.pose.orientation.y = qy
        odom_msg.pose.pose.orientation.z = qz
        odom_msg.pose.pose.orientation.w = qw

        odom_msg.twist.twist.linear.x = v_linear
        odom_msg.twist.twist.angular.z = w_angular

        self.odom_pub.publish(odom_msg)

        # Publish TF transform odom -> base_link
        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = current_time.to_msg()
            t.header.frame_id = 'odom'
            t.child_frame_id = 'base_link'

            t.transform.translation.x = self.x
            t.transform.translation.y = self.y
            t.transform.translation.z = 0.0
            t.transform.rotation.x = qx
            t.transform.rotation.y = qy
            t.transform.rotation.z = qz
            t.transform.rotation.w = qw

            self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
