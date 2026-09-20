#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rover_interfaces.msg import WheelRPM


class WatchdogNode(Node):
    """
    Hardware and Telemetry Watchdog Node.
    Monitors high-rate motor command and feedback topics.
    If commands or sensor feedback drop out for > timeout_sec, publishes emergency stop.
    """

    def __init__(self):
        super().__init__('watchdog_node')

        self.declare_parameter('timeout_sec', 0.5)
        self.timeout_sec = self.get_parameter('timeout_sec').value

        self.last_cmd_time = self.get_clock().now()
        self.last_feedback_time = self.get_clock().now()

        # Subscribers
        self.create_subscription(WheelRPM, '/drive/rpm_command', self.cmd_cb, 10)
        self.create_subscription(WheelRPM, '/drive/rpm_feedback', self.feedback_cb, 10)

        # Safety Publisher
        self.stop_pub = self.create_publisher(Bool, '/safety/stop', 10)

        # Watchdog loop at 10 Hz
        self.create_timer(0.1, self.check_watchdog)
        self.get_logger().info(f"Hardware Watchdog Node initialized (Timeout: {self.timeout_sec}s)")

    def cmd_cb(self, msg: WheelRPM):
        self.last_cmd_time = self.get_clock().now()

    def feedback_cb(self, msg: WheelRPM):
        self.last_feedback_time = self.get_clock().now()

    def check_watchdog(self):
        now = self.get_clock().now()
        cmd_elapsed = (now - self.last_cmd_time).nanoseconds / 1e9
        fb_elapsed = (now - self.last_feedback_time).nanoseconds / 1e9

        stop_msg = Bool()
        if cmd_elapsed > self.timeout_sec or fb_elapsed > self.timeout_sec:
            stop_msg.data = True
            self.stop_pub.publish(stop_msg)
            self.get_logger().warn(f"WATCHDOG TIMEOUT! Cmd Elapsed: {cmd_elapsed:.2f}s, Feedback Elapsed: {fb_elapsed:.2f}s -> EMERGENCY STOP TRIGGERED")
        else:
            stop_msg.data = False
            self.stop_pub.publish(stop_msg)


def main(args=None):
    rclpy.init(args=args)
    node = WatchdogNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
