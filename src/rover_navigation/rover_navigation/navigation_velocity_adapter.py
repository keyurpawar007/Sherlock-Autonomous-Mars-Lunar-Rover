#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from std_msgs.msg import Float32MultiArray


class NavigationVelocityAdapter(Node):

    def __init__(self):
        super().__init__('navigation_velocity_adapter')

        # Same rover geometry currently used by sim_drive_adapter.
        self.declare_parameter('wheel_radius', 0.09)
        self.declare_parameter('track_width', 1.003787)

        self.wheel_radius = float(
            self.get_parameter('wheel_radius').value
        )

        self.track_width = float(
            self.get_parameter('track_width').value
        )

        # Input from our existing path follower.
        self.create_subscription(
            Float64MultiArray,
            '/nav_wheel_commands',
            self.navigation_callback,
            10
        )

        # Output expected by the kinematics team.
        self.velocity_pub = self.create_publisher(
            Float32MultiArray,
            '/navigation/velocity_data',
            10
        )

        self.get_logger().info(
            'Navigation velocity adapter started'
        )

        self.get_logger().info(
            '/nav_wheel_commands -> /navigation/velocity_data'
        )

    def navigation_callback(self, msg):

        if len(msg.data) < 6:
            self.get_logger().warning(
                'Expected 6 wheel velocities '
                '[FR, MR, BR, FL, ML, BL]'
            )
            return

        # Wheel order used by our navigation stack:
        #
        # FR, MR, BR, FL, ML, BL
        fr = float(msg.data[0])
        mr = float(msg.data[1])
        br = float(msg.data[2])

        fl = float(msg.data[3])
        ml = float(msg.data[4])
        bl = float(msg.data[5])

        # Average angular velocity of each side.
        omega_right = (
            fr + mr + br
        ) / 3.0

        omega_left = (
            fl + ml + bl
        ) / 3.0

        # Wheel angular velocity [rad/s]
        # -> wheel linear velocity [m/s]
        v_right = (
            self.wheel_radius
            * omega_right
        )

        v_left = (
            self.wheel_radius
            * omega_left
        )

        # Differential/skid-steer rover velocity.
        linear_velocity = (
            v_right + v_left
        ) / 2.0

        angular_velocity = (
            v_right - v_left
        ) / self.track_width

        # Turning radius R = v / omega.
        #
        # For straight motion omega ~= 0, use 0.0 in the
        # communication interface instead of infinity.
        if abs(angular_velocity) > 1e-4:
            turning_radius = (
                linear_velocity
                / angular_velocity
            )
        else:
            turning_radius = 0.0

        # Current rover configuration does not use steering angle.
        # Reserved for the final wheel/steering design.
        turning_angle = 0.0

        output = Float32MultiArray()

        output.data = [
            float(linear_velocity),
            float(angular_velocity),
            float(turning_radius),
            float(turning_angle),
        ]

        self.velocity_pub.publish(output)


def main(args=None):

    rclpy.init(args=args)

    node = NavigationVelocityAdapter()

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
