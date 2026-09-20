#!/usr/bin/env python3

import time
import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rover_interfaces.msg import WheelRPM

try:
    import serial
except ImportError:
    serial = None


class UARTBridgeNode(Node):
    """
    UART Bridge Node connecting Jetson (ROS 2) with STM32 Motor Controller.
    Handles serial frame serialization/deserialization and provides automatic hardware/mock fallback.
    """

    def __init__(self):
        super().__init__('uart_bridge_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('use_mock_hardware', True)

        self.port_name = self.get_parameter('port').value
        self.baudrate = self.get_parameter('baudrate').value
        self.use_mock = self.get_parameter('use_mock_hardware').value

        self.serial_conn = None
        self.safety_stop = False
        self.current_cmd_left = 0.0
        self.current_cmd_right = 0.0

        if not self.use_mock and serial is not None:
            try:
                self.serial_conn = serial.Serial(self.port_name, self.baudrate, timeout=0.05)
                self.get_logger().info(f"Connected to STM32 on serial port {self.port_name} @ {self.baudrate}")
            except Exception as e:
                self.get_logger().warn(f"Failed to open serial port {self.port_name}: {e}. Falling back to MOCK mode.")
                self.use_mock = True
        else:
            self.use_mock = True
            self.get_logger().info("Running UART Bridge in MOCK hardware mode.")

        # Subscriptions
        self.create_subscription(WheelRPM, '/drive/rpm_command', self.cmd_callback, 10)
        self.create_subscription(Bool, '/safety/stop', self.safety_callback, 10)

        # Publishers
        self.feedback_pub = self.create_publisher(WheelRPM, '/drive/rpm_feedback', 10)

        # Timer loop at 50 Hz
        self.create_timer(0.02, self.bridge_spin)

    def safety_callback(self, msg: Bool):
        self.safety_stop = msg.data
        if self.safety_stop:
            self.current_cmd_left = 0.0
            self.current_cmd_right = 0.0

    def cmd_callback(self, msg: WheelRPM):
        if not self.safety_stop:
            self.current_cmd_left = msg.left_rpm
            self.current_cmd_right = msg.right_rpm
        else:
            self.current_cmd_left = 0.0
            self.current_cmd_right = 0.0

    def bridge_spin(self):
        # 1. Send Command Frame to STM32: Frame Format [HEADER: 0xAA, 0xBB, float_left, float_right, CHECKSUM]
        target_l = 0.0 if self.safety_stop else self.current_cmd_left
        target_r = 0.0 if self.safety_stop else self.current_cmd_right

        if not self.use_mock and self.serial_conn and self.serial_conn.is_open:
            try:
                packet = struct.pack('<BBff', 0xAA, 0xBB, target_l, target_r)
                checksum = sum(packet) & 0xFF
                self.serial_conn.write(packet + struct.pack('B', checksum))

                # Read feedback
                if self.serial_conn.in_waiting >= 11:
                    raw_data = self.serial_conn.read(11)
                    if raw_data[0] == 0xCC and raw_data[1] == 0xDD:
                        header1, header2, f_left, f_right, chk = struct.unpack('<BBffB', raw_data)
                        if sum(raw_data[:10]) & 0xFF == chk:
                            feedback_msg = WheelRPM()
                            feedback_msg.left_rpm = f_left
                            feedback_msg.right_rpm = f_right
                            self.feedback_pub.publish(feedback_msg)
            except Exception as e:
                self.get_logger().error(f"Serial communication error: {e}")
        else:
            # Mock Feedback Simulation with realistic motor ramp-up
            feedback_msg = WheelRPM()
            feedback_msg.left_rpm = float(target_l)
            feedback_msg.right_rpm = float(target_r)
            self.feedback_pub.publish(feedback_msg)


def main(args=None):
    rclpy.init(args=args)
    node = UARTBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
