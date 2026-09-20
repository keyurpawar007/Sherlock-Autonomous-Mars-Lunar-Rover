#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, Twist
from std_msgs.msg import Float64MultiArray, Bool
from tf2_ros import Buffer, TransformListener, TransformException


class PathFollower(Node):

    def __init__(self):
        super().__init__('path_follower')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------
        self.declare_parameter('local_goal_topic', '/local_goal')
        self.declare_parameter('avoidance_topic', '/avoidance_cmd')
        self.declare_parameter('goal_reached_topic', '/goal_reached')

        self.declare_parameter(
            'wheel_command_topic',
            '/rover_velocity_controller/commands'
        )

        self.declare_parameter('robot_frame', 'base_footprint')

        self.declare_parameter('base_wheel_speed', 2.8)
        self.declare_parameter('max_turn_component', 1.6)
        self.declare_parameter('max_wheel_speed', 4.0)

        self.declare_parameter('heading_gain', 1.5)
        self.declare_parameter('goal_tolerance', 0.08)

        # If heading error is large, rotate first instead of
        # driving forward while badly misaligned.
        self.declare_parameter(
            'rotate_first_angle',
            math.radians(40.0)
        )

        self.local_goal_topic = (
            self.get_parameter('local_goal_topic')
            .get_parameter_value().string_value
        )

        self.avoidance_topic = (
            self.get_parameter('avoidance_topic')
            .get_parameter_value().string_value
        )

        self.goal_reached_topic = (
            self.get_parameter('goal_reached_topic')
            .get_parameter_value().string_value
        )

        self.wheel_command_topic = (
            self.get_parameter('wheel_command_topic')
            .get_parameter_value().string_value
        )

        self.robot_frame = (
            self.get_parameter('robot_frame')
            .get_parameter_value().string_value
        )

        self.base_wheel_speed = (
            self.get_parameter('base_wheel_speed')
            .get_parameter_value().double_value
        )

        self.max_turn_component = (
            self.get_parameter('max_turn_component')
            .get_parameter_value().double_value
        )

        self.max_wheel_speed = (
            self.get_parameter('max_wheel_speed')
            .get_parameter_value().double_value
        )

        self.heading_gain = (
            self.get_parameter('heading_gain')
            .get_parameter_value().double_value
        )

        self.goal_tolerance = (
            self.get_parameter('goal_tolerance')
            .get_parameter_value().double_value
        )

        self.rotate_first_angle = (
            self.get_parameter('rotate_first_angle')
            .get_parameter_value().double_value
        )

        # ---------------------------------------------------------
        # State
        # ---------------------------------------------------------
        self.local_goal = None
        self.avoidance_cmd = None
        self.goal_reached = False
        self.last_mode = None
        self.rotate_in_place = False

        # ---------------------------------------------------------
        # TF
        # ---------------------------------------------------------
        self.tf_buffer = Buffer()

        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ---------------------------------------------------------
        # ROS interfaces
        # ---------------------------------------------------------
        self.create_subscription(
            PoseStamped,
            self.local_goal_topic,
            self.local_goal_callback,
            10
        )

        self.create_subscription(
            Twist,
            self.avoidance_topic,
            self.avoidance_callback,
            10
        )

        self.create_subscription(
            Bool,
            self.goal_reached_topic,
            self.goal_reached_callback,
            10
        )

        self.wheel_pub = self.create_publisher(
            Float64MultiArray,
            self.wheel_command_topic,
            10
        )

        self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info('Path Follower started')
        self.get_logger().info(
            f'Local goal     : {self.local_goal_topic}'
        )
        self.get_logger().info(
            f'Avoidance cmd  : {self.avoidance_topic}'
        )
        self.get_logger().info(
            f'Goal reached   : {self.goal_reached_topic}'
        )
        self.get_logger().info(
            f'Wheel commands : {self.wheel_command_topic}'
        )

    # =============================================================
    # CALLBACKS
    # =============================================================

    def local_goal_callback(self, msg):

        self.local_goal = msg

        self.get_logger().info(
            'Local goal received: '
            f'x={msg.pose.position.x:.2f}, '
            f'y={msg.pose.position.y:.2f}'
        )

    def avoidance_callback(self, msg):
        self.avoidance_cmd = msg

    def goal_reached_callback(self, msg):

        previous = self.goal_reached
        self.goal_reached = bool(msg.data)

        if self.goal_reached:

            self.stop_rover()

            if not previous:
                self.get_logger().info(
                    'FINAL GOAL REACHED -> stopping rover'
                )

        elif previous:

            # A new navigation goal has been activated.
            # Wait for a fresh local goal before driving again.
            self.local_goal = None
            self.last_mode = None

            self.get_logger().info(
                'New goal activated -> path following enabled'
            )

    # =============================================================
    # TF
    # =============================================================

    def get_robot_pose(self, frame):

        try:
            transform = self.tf_buffer.lookup_transform(
                frame,
                self.robot_frame,
                Time()
            )

            q = transform.transform.rotation

            yaw = math.atan2(
                2.0 * (
                    q.w * q.z +
                    q.x * q.y
                ),
                1.0 - 2.0 * (
                    q.y * q.y +
                    q.z * q.z
                )
            )

            return (
                transform.transform.translation.x,
                transform.transform.translation.y,
                yaw
            )

        except TransformException as exc:

            self.get_logger().warning(
                f'Cannot obtain rover pose: {exc}'
            )

            return None

    # =============================================================
    # HELPERS
    # =============================================================

    @staticmethod
    def clamp(value, low, high):
        return max(
            low,
            min(high, value)
        )

    def publish_wheels(
        self,
        right_speed,
        left_speed
    ):

        right_speed = self.clamp(
            right_speed,
            -self.max_wheel_speed,
            self.max_wheel_speed
        )

        left_speed = self.clamp(
            left_speed,
            -self.max_wheel_speed,
            self.max_wheel_speed
        )

        command = Float64MultiArray()

        # Controller order:
        # FR, MR, BR, FL, ML, BL
        command.data = [
            right_speed,
            right_speed,
            right_speed,
            left_speed,
            left_speed,
            left_speed,
        ]

        self.wheel_pub.publish(command)

    def stop_rover(self):
        self.publish_wheels(
            0.0,
            0.0
        )

    # =============================================================
    # CONTROL LOOP
    # =============================================================

    def control_loop(self):

        # Final goal reached has highest priority.
        # Ignore all local-path and avoidance commands until
        # Goal Manager activates a new goal.
        if self.goal_reached:

            self.stop_rover()

            mode = 'FINAL_GOAL_REACHED'

            if mode != self.last_mode:
                self.get_logger().info(
                    'FINAL_GOAL_REACHED | wheels stopped'
                )
                self.last_mode = mode

            return

        if self.local_goal is None:
            self.stop_rover()
            return

        # Safety:
        # Never drive until obstacle-avoidance data has arrived.
        if self.avoidance_cmd is None:
            self.stop_rover()
            return

        frame = self.local_goal.header.frame_id

        if not frame:
            frame = 'map'

        robot_pose = self.get_robot_pose(frame)

        if robot_pose is None:
            self.stop_rover()
            return

        rx, ry, robot_yaw = robot_pose

        gx = self.local_goal.pose.position.x
        gy = self.local_goal.pose.position.y

        dx = gx - rx
        dy = gy - ry

        distance = math.hypot(
            dx,
            dy
        )

        # ---------------------------------------------------------
        # Convert goal vector from map frame into rover base frame.
        # ---------------------------------------------------------

        base_x = (
            math.cos(robot_yaw) * dx +
            math.sin(robot_yaw) * dy
        )

        base_y = (
            -math.sin(robot_yaw) * dx +
            math.cos(robot_yaw) * dy
        )

        # IMPORTANT:
        # This rover's physical forward direction is base -Y.
        #
        # Therefore:
        #   forward component = -base_y
        #   left/right component = base_x
        #
        # Positive heading error means turn left.
        heading_error = math.atan2(
            base_x,
            -base_y
        )

        # ---------------------------------------------------------
        # Local goal reached
        # ---------------------------------------------------------

        if distance <= self.goal_tolerance:

            self.stop_rover()

            mode = 'LOCAL_GOAL_REACHED'

            if mode != self.last_mode:
                self.get_logger().info(
                    f'{mode} | distance={distance:.2f} m'
                )

                self.last_mode = mode

            return

        # ---------------------------------------------------------
        # Path-following steering
        # ---------------------------------------------------------

        steering = self.clamp(
            self.heading_gain * heading_error,
            -1.0,
            1.0
        )

        # Turn first for diagonal / side targets.
        # Do not drive forward while badly misaligned.
        rotate_enter_angle = math.radians(20.0)
        rotate_exit_angle = math.radians(8.0)

        # Enter rotate-in-place mode when heading error becomes large.
        if abs(heading_error) > rotate_enter_angle:
            self.rotate_in_place = True

        # Leave rotate mode only after we are well aligned.
        elif abs(heading_error) < rotate_exit_angle:
            self.rotate_in_place = False

        if self.rotate_in_place:
            forward_speed = 0.0
            mode = 'ROTATE_TO_PATH'

        else:
            # Drive normally once the rover is aligned.
            heading_scale = max(
                0.35,
                1.0 - abs(heading_error) / rotate_enter_angle
            )

            forward_speed = (
                self.base_wheel_speed *
                heading_scale
            )

            # Slow down near the target.
            if distance < 0.40:
                forward_speed = min(
                    forward_speed,
                    self.base_wheel_speed * 0.35
                )

            mode = 'FOLLOW_PATH'

        # ---------------------------------------------------------
        # Obstacle avoidance integration
        #
        # CLEAR:
        #   linear.x = 1.0, angular.z = 0
        #
        # CAUTION:
        #   linear.x = 0.4, angular.z = +/-0.5
        #
        # BLOCKED:
        #   linear.x = 0.0, angular.z = +/-1.0
        # ---------------------------------------------------------

        avoidance_linear = self.clamp(
            self.avoidance_cmd.linear.x,
            0.0,
            1.0
        )

        avoidance_turn = self.clamp(
            self.avoidance_cmd.angular.z,
            -1.0,
            1.0
        )

        # Obstacle layer controls allowed forward speed.
        forward_speed *= avoidance_linear

        combined_turn = self.clamp(
            steering + avoidance_turn,
            -1.0,
            1.0
        )

        # BLOCKED: stop translation and turn in place.
        if (
            avoidance_linear <= 0.01
            and
            abs(avoidance_turn) > 0.01
        ):
            forward_speed = 0.0
            combined_turn = avoidance_turn
            mode = 'OBSTACLE_OVERRIDE'

        turn_component = (
            self.max_turn_component *
            combined_turn
        )

        # Rocker-bogie rover needs a stronger command to overcome
        # static friction during pure in-place rotation.
        if (
            mode in ('ROTATE_TO_PATH', 'OBSTACLE_OVERRIDE')
            and abs(combined_turn) > 0.01
        ):
            turn_component = math.copysign(
                max(abs(turn_component), 3.0),
                combined_turn
            )

        # Positive turn = left:
        #
        # right wheels faster
        # left wheels slower
        #
        # Pure positive rotation:
        # + + +  - - -
        right_speed = (
            forward_speed +
            turn_component
        )

        left_speed = (
            forward_speed -
            turn_component
        )

        self.publish_wheels(
            right_speed,
            left_speed
        )

        if mode != self.last_mode:

            self.get_logger().info(
                f'{mode} | '
                f'distance={distance:.2f} m | '
                f'heading_error='
                f'{math.degrees(heading_error):.1f} deg | '
                f'R={right_speed:.2f} | '
                f'L={left_speed:.2f}'
            )

            self.last_mode = mode


def main(args=None):

    rclpy.init(args=args)

    node = PathFollower()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

    # Ctrl+C may already have shut down the ROS context.
    # Only publish the stop command while the context is valid.
        if rclpy.ok():
            try:
                node.stop_rover()
                rclpy.spin_once(
                    node,
                    timeout_sec=0.05
                )
            except Exception as exc:
                node.get_logger().warning(
                    f'Could not publish final stop command: {exc}'
                )

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()

if __name__ == '__main__':
    main()
