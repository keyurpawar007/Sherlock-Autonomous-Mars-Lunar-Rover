#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rover_interfaces.msg import WheelRPM


class PIDControllerNode(Node):
    """
    Closed-loop PID RPM Controller.
    Compares target RPM (/drive/rpm_command) vs measured RPM (/drive/rpm_feedback)
    and applies PID correction to stabilize wheel speeds.
    """

    def __init__(self):
        super().__init__('pid_controller_node')

        # Gains
        self.declare_parameter('kp', 1.2)
        self.declare_parameter('ki', 0.05)
        self.declare_parameter('kd', 0.01)

        self.kp = self.get_parameter('kp').value
        self.ki = self.get_parameter('ki').value
        self.kd = self.get_parameter('kd').value

        # Target & Measured states
        self.target_left = 0.0
        self.target_right = 0.0
        self.measured_left = 0.0
        self.measured_right = 0.0

        # Errors
        self.integral_left = 0.0
        self.integral_right = 0.0
        self.prev_error_left = 0.0
        self.prev_error_right = 0.0
        self.last_time = self.get_clock().now()

        # Subscriptions
        self.create_subscription(WheelRPM, '/drive/rpm_command', self.cmd_callback, 10)
        self.create_subscription(WheelRPM, '/drive/rpm_feedback', self.feedback_callback, 10)

        # Publisher for PID compensated command
        self.corrected_pub = self.create_publisher(WheelRPM, '/drive/rpm_corrected', 10)
        self.timer = self.create_timer(0.02, self.control_loop)  # 50 Hz loop

        self.get_logger().info("PID RPM Controller Node initialized")

    def cmd_callback(self, msg: WheelRPM):
        self.target_left = msg.left_rpm
        self.target_right = msg.right_rpm

    def feedback_callback(self, msg: WheelRPM):
        self.measured_left = msg.left_rpm
        self.measured_right = msg.right_rpm

    def control_loop(self):
        now = self.get_clock().now()
        dt = (now - self.last_time).nanoseconds / 1e9
        self.last_time = now

        if dt <= 0.0:
            return

        # Left Wheel PID
        err_left = self.target_left - self.measured_left
        self.integral_left += err_left * dt
        deriv_left = (err_left - self.prev_error_left) / dt
        out_left = (self.kp * err_left) + (self.ki * self.integral_left) + (self.kd * deriv_left)
        self.prev_error_left = err_left

        # Right Wheel PID
        err_right = self.target_right - self.measured_right
        self.integral_right += err_right * dt
        deriv_right = (err_right - self.prev_error_right) / dt
        out_right = (self.kp * err_right) + (self.ki * self.integral_right) + (self.kd * deriv_right)
        self.prev_error_right = err_right

        # Corrected RPM message
        msg = WheelRPM()
        msg.left_rpm = float(self.target_left + out_left)
        msg.right_rpm = float(self.target_right + out_right)
        self.corrected_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = PIDControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
