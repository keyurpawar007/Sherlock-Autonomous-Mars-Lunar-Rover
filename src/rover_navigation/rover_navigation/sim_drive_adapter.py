#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import Float64MultiArray
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelStates
from tf2_ros import TransformBroadcaster


def normalize_angle(a):
    return math.atan2(math.sin(a), math.cos(a))


def yaw_from_quaternion(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


class SimDriveAdapter(Node):

    def __init__(self):
        super().__init__('sim_drive_adapter')

        # -----------------------------------------------------
        # Rover parameters
        # -----------------------------------------------------
        self.wheel_radius = 0.09
        self.track_width = 1.003787
        self.timeout = 0.35
        self.model_name = 'rover'

        self.last_cmd_time = None

        # Initial Gazebo pose used as odom origin
        self.have_origin = False
        self.origin_x = 0.0
        self.origin_y = 0.0
        self.origin_yaw = 0.0

        # -----------------------------------------------------
        # Navigation wheel command input
        # Controller order:
        # FR, MR, BR, FL, ML, BL
        # -----------------------------------------------------
        self.cmd_sub = self.create_subscription(
            Float64MultiArray,
            '/rover_velocity_controller/commands',
            self.command_callback,
            10
        )

        # -----------------------------------------------------
        # Actual Gazebo rover pose
        # -----------------------------------------------------
        self.model_states_sub = self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )

        # -----------------------------------------------------
        # Outputs
        # -----------------------------------------------------
        self.sim_cmd_pub = self.create_publisher(
            Twist,
            '/sim_cmd_vel',
            10
        )

        self.odom_pub = self.create_publisher(
            Odometry,
            '/odom',
            10
        )

        self.tf_broadcaster = TransformBroadcaster(self)

        # Watchdog for motion commands only
        self.timer = self.create_timer(
            0.05,
            self.watchdog_callback
        )

        self.get_logger().info(
            'SimDriveAdapter started: '
            'wheel commands -> /sim_cmd_vel, '
            'Gazebo model pose -> /odom'
        )

    # =========================================================
    # DRIVE COMMAND
    # =========================================================

    def command_callback(self, msg):

        if len(msg.data) < 6:
            self.get_logger().warning(
                'Expected 6 wheel commands'
            )
            return

        # Right side: FR MR BR
        omega_right = (
            msg.data[0] +
            msg.data[1] +
            msg.data[2]
        ) / 3.0

        # Left side: FL ML BL
        omega_left = (
            msg.data[3] +
            msg.data[4] +
            msg.data[5]
        ) / 3.0

        v_right = self.wheel_radius * omega_right
        v_left = self.wheel_radius * omega_left

        forward = (v_right + v_left) / 2.0

        yaw_rate = (
            v_right - v_left
        ) / self.track_width

        cmd = Twist()

        # Rover physical forward direction is base -Y
        cmd.linear.x = 0.0
        cmd.linear.y = -forward
        cmd.linear.z = 0.0

        cmd.angular.x = 0.0
        cmd.angular.y = 0.0
        cmd.angular.z = yaw_rate

        self.sim_cmd_pub.publish(cmd)

        self.last_cmd_time = self.get_clock().now()

    # =========================================================
    # ACTUAL GAZEBO ODOMETRY
    # =========================================================

    def model_states_callback(self, msg):

        try:
            index = msg.name.index(self.model_name)
        except ValueError:
            return

        pose = msg.pose[index]
        twist = msg.twist[index]

        world_x = pose.position.x
        world_y = pose.position.y
        world_yaw = yaw_from_quaternion(
            pose.orientation
        )

        # -----------------------------------------------------
        # Capture starting pose as odom origin
        # -----------------------------------------------------
        if not self.have_origin:
            self.origin_x = world_x
            self.origin_y = world_y
            self.origin_yaw = world_yaw
            self.have_origin = True

            self.get_logger().info(
                f'Odom origin captured from Gazebo: '
                f'x={world_x:.3f}, '
                f'y={world_y:.3f}, '
                f'yaw={math.degrees(world_yaw):.2f} deg'
            )

        dx = world_x - self.origin_x
        dy = world_y - self.origin_y

        c0 = math.cos(self.origin_yaw)
        s0 = math.sin(self.origin_yaw)

        # World displacement expressed in odom frame
        odom_x = c0 * dx + s0 * dy
        odom_y = -s0 * dx + c0 * dy

        odom_yaw = normalize_angle(
            world_yaw - self.origin_yaw
        )

        qz = math.sin(odom_yaw / 2.0)
        qw = math.cos(odom_yaw / 2.0)

        now = self.get_clock().now().to_msg()

        # -----------------------------------------------------
        # /odom message
        # -----------------------------------------------------
        odom = Odometry()

        odom.header.stamp = now
        odom.header.frame_id = 'odom'
        odom.child_frame_id = 'base_footprint'

        odom.pose.pose.position.x = odom_x
        odom.pose.pose.position.y = odom_y
        odom.pose.pose.position.z = 0.0

        odom.pose.pose.orientation.x = 0.0
        odom.pose.pose.orientation.y = 0.0
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw

        # Gazebo velocity is expressed in world coordinates.
        # Convert it into rover/body coordinates.
        cy = math.cos(world_yaw)
        sy = math.sin(world_yaw)

        body_vx = (
            cy * twist.linear.x +
            sy * twist.linear.y
        )

        body_vy = (
            -sy * twist.linear.x +
            cy * twist.linear.y
        )

        odom.twist.twist.linear.x = body_vx
        odom.twist.twist.linear.y = body_vy
        odom.twist.twist.angular.z = twist.angular.z

        self.odom_pub.publish(odom)

        # -----------------------------------------------------
        # odom -> base_footprint TF
        # -----------------------------------------------------
        tf = TransformStamped()

        tf.header.stamp = now
        tf.header.frame_id = 'odom'
        tf.child_frame_id = 'base_footprint'

        tf.transform.translation.x = odom_x
        tf.transform.translation.y = odom_y
        tf.transform.translation.z = 0.0

        tf.transform.rotation.x = 0.0
        tf.transform.rotation.y = 0.0
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw

        self.tf_broadcaster.sendTransform(tf)

    # =========================================================
    # COMMAND WATCHDOG
    # =========================================================

    def watchdog_callback(self):

        if self.last_cmd_time is None:
            return

        elapsed = (
            self.get_clock().now() -
            self.last_cmd_time
        ).nanoseconds / 1e9

        if elapsed > self.timeout:

            stop = Twist()
            self.sim_cmd_pub.publish(stop)

            self.last_cmd_time = None


def main(args=None):

    rclpy.init(args=args)

    node = SimDriveAdapter()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
