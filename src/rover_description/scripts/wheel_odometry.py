#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class WheelOdometry(Node):

    def __init__(self):
        super().__init__('wheel_odometry')

        # Rover dimensions
        self.declare_parameter('wheel_radius', 0.09)
        self.declare_parameter('track_width', 1.003787)

        self.wheel_radius = (
            self.get_parameter('wheel_radius')
            .get_parameter_value()
            .double_value
        )

        self.track_width = (
            self.get_parameter('track_width')
            .get_parameter_value()
            .double_value
        )

        # Wheel joint names
        self.right_wheels = [
            'FR_wheel',
            'MR_wheel',
            'BR_wheel',
        ]

        self.left_wheels = [
            'FL_wheel',
            'ML_wheel',
            'BL_wheel',
        ]

        # Odometry state
        self.x = 0.0
        self.y = 0.0

        # psi represents yaw of base_footprint in odom.
        self.psi = 0.0

        self.last_time = None

        # Publisher
        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        # TF broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

        # Joint-state subscriber
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info(
            'Wheel odometry started: '
            f'radius={self.wheel_radius:.4f} m, '
            f'track={self.track_width:.4f} m'
        )

    def joint_state_callback(self, msg):

        # Ensure velocity data exists
        if not msg.velocity:
            return

        velocity_map = {
            name: velocity
            for name, velocity in zip(msg.name, msg.velocity)
        }

        required_wheels = (
            self.right_wheels +
            self.left_wheels
        )

        # Wait until all six wheels are present
        if not all(
            wheel in velocity_map
            for wheel in required_wheels
        ):
            return

        # Simulation timestamp from joint_states
        current_time = (
            msg.header.stamp.sec +
            msg.header.stamp.nanosec * 1e-9
        )

        if self.last_time is None:
            self.last_time = current_time
            return

        dt = current_time - self.last_time
        self.last_time = current_time

        if dt <= 0.0 or dt > 1.0:
            return

        # Average angular velocity on each rover side
        omega_right = sum(
            velocity_map[j]
            for j in self.right_wheels
        ) / 3.0

        omega_left = sum(
            velocity_map[j]
            for j in self.left_wheels
        ) / 3.0

        # Convert wheel angular velocity [rad/s]
        # to linear velocity [m/s]
        v_right = self.wheel_radius * omega_right
        v_left = self.wheel_radius * omega_left

        # Equivalent skid-steer / differential-drive model
        linear_velocity = (
            v_right + v_left
        ) / 2.0

        angular_velocity = (
            v_right - v_left
        ) / self.track_width

        # Current CAD/base frame has physical forward along -Y.
        #
        # If psi is the orientation of base_footprint +X,
        # physical forward heading is psi - 90 degrees.
        forward_heading = self.psi - math.pi / 2.0

        # Integrate position
        self.x += (
            linear_velocity *
            math.cos(forward_heading) *
            dt
        )

        self.y += (
            linear_velocity *
            math.sin(forward_heading) *
            dt
        )

        self.psi += angular_velocity * dt

        # Keep yaw bounded
        self.psi = math.atan2(
            math.sin(self.psi),
            math.cos(self.psi)
        )

        # Quaternion for yaw
        qz = math.sin(self.psi / 2.0)
        qw = math.cos(self.psi / 2.0)

        # --------------------------------------------------
        # nav_msgs/Odometry
        # --------------------------------------------------

        odom = Odometry()

        odom.header.stamp = msg.header.stamp
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Twist must be expressed in child frame.
        # Physical forward is -Y of the current CAD base frame.
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.linear.y = -linear_velocity
        odom.twist.twist.linear.z = 0.0

        odom.twist.twist.angular.x = 0.0
        odom.twist.twist.angular.y = 0.0
        odom.twist.twist.angular.z = angular_velocity

        self.odom_pub.publish(odom)

        # --------------------------------------------------
        # TF: odom -> base_footprint
        # --------------------------------------------------

        transform = TransformStamped()

        transform.header.stamp = msg.header.stamp
        transform.header.frame_id = 'odom'
        transform.child_frame_id = 'base_footprint'

        transform.transform.translation.x = self.x
        transform.transform.translation.y = self.y
        transform.transform.translation.z = 0.0

        transform.transform.rotation.x = 0.0
        transform.transform.rotation.y = 0.0
        transform.transform.rotation.z = qz
        transform.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(transform)


def main(args=None):
    rclpy.init(args=args)

    node = WheelOdometry()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

