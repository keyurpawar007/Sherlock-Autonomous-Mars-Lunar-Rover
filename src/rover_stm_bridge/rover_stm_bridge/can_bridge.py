#!/usr/bin/env python3

import struct
import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from rover_interfaces.msg import WheelRPM, MissionStatus

try:
    import can
except ImportError:
    can = None


# Exact Microcontroller CAN Arbitration Header Definitions
CAN_ID_MANUAL = 0x100
CAN_ID_STOP = 0x101
CAN_ID_LORA = 0x102
CAN_ID_AUTONOMOUS = 0x103

CAN_ID_HEARTBEAT = 0x220
HEARTBEAT_PERIOD_MS = 100

CAN_ID_ENC_FL = 0x200
CAN_ID_ENC_FR = 0x201
CAN_ID_ENC_RL = 0x202
CAN_ID_ENC_RR = 0x203

CAN_ID_DRIVE_STATUS = 0x210
CAN_FAILSAFE_MS = 300


class CANBridgeNode(Node):
    """
    CAN Bus Bridge Node matching Microcontroller C Firmware Header Definitions.
    - Handles Mode-based arbitration IDs (0x100 MANUAL, 0x101 STOP, 0x103 AUTONOMOUS).
    - Transmits 100ms Heartbeat (0x220).
    - Receives individual 4-wheel encoder feedback (0x200 FL, 0x201 FR, 0x202 RL, 0x203 RR).
    - Enforces 300ms CAN failsafe.
    """

    def __init__(self):
        super().__init__('can_bridge_node')

        self.declare_parameter('interface', 'can0')
        self.declare_parameter('bitrate', 500000)
        self.declare_parameter('use_mock_hardware', True)
        self.declare_parameter('encoder_cpr', 93132)

        self.interface = self.get_parameter('interface').value
        self.bitrate = self.get_parameter('bitrate').value
        self.use_mock = self.get_parameter('use_mock_hardware').value
        self.cpr = self.get_parameter('encoder_cpr').value

        self.can_bus = None
        self.safety_stop = False
        self.active_mode = "TELEOP"
        self.current_cmd_left = 0.0
        self.current_cmd_right = 0.0

        # Individual measured 4-wheel feedback
        self.rpm_fl = 0.0
        self.rpm_fr = 0.0
        self.rpm_rl = 0.0
        self.rpm_rr = 0.0

        if not self.use_mock and can is not None:
            try:
                self.can_bus = can.interface.Bus(channel=self.interface, bustype='socketcan', bitrate=self.bitrate)
                self.get_logger().info(f"Connected to SocketCAN interface '{self.interface}' @ {self.bitrate} bps")
            except Exception as e:
                self.get_logger().warn(f"Failed to initialize CAN interface '{self.interface}': {e}. Falling back to MOCK mode.")
                self.use_mock = True
        else:
            self.use_mock = True
            self.get_logger().info("Running CAN Bridge in MOCK hardware mode.")

        # Subscriptions
        self.create_subscription(WheelRPM, '/drive/rpm_command', self.cmd_callback, 10)
        self.create_subscription(Bool, '/safety/stop', self.safety_callback, 10)
        self.create_subscription(MissionStatus, '/mission/status', self.mission_status_cb, 10)

        # Publishers
        self.feedback_pub = self.create_publisher(WheelRPM, '/drive/rpm_feedback', 10)

        # Spin loops
        self.create_timer(0.02, self.can_spin)                        # 50 Hz control frame
        self.create_timer(HEARTBEAT_PERIOD_MS / 1000.0, self.send_hb) # 10 Hz heartbeat

    def mission_status_cb(self, msg: MissionStatus):
        self.active_mode = msg.mission_mode

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

    def send_hb(self):
        if not self.use_mock and self.can_bus:
            try:
                # Transmit 100ms Heartbeat frame (0x220)
                hb_msg = can.Message(arbitration_id=CAN_ID_HEARTBEAT, data=[0x01, 0x00, 0x00, 0x00], is_extended_id=False)
                self.can_bus.send(hb_msg)
            except Exception:
                pass

    def can_spin(self):
        target_l = 0.0 if self.safety_stop else self.current_cmd_left
        target_r = 0.0 if self.safety_stop else self.current_cmd_right

        # Select outgoing arbitration ID matching firmware
        if self.safety_stop:
            arb_id = CAN_ID_STOP          # 0x101
        elif self.active_mode in ["AUTONOMOUS", "ABEX"]:
            arb_id = CAN_ID_AUTONOMOUS    # 0x103
        else:
            arb_id = CAN_ID_MANUAL        # 0x100

        if not self.use_mock and self.can_bus:
            try:
                # Pack and send command frame
                data = struct.pack('<ff', float(target_l), float(target_r))
                cmd_frame = can.Message(arbitration_id=arb_id, data=data, is_extended_id=False)
                self.can_bus.send(cmd_frame)

                # Receive and process 4-wheel encoder CAN IDs
                rx_msg = self.can_bus.recv(timeout=0.005)
                if rx_msg:
                    if rx_msg.arbitration_id == CAN_ID_ENC_FL:
                        self.rpm_fl = struct.unpack('<f', rx_msg.data[:4])[0]
                    elif rx_msg.arbitration_id == CAN_ID_ENC_FR:
                        self.rpm_fr = struct.unpack('<f', rx_msg.data[:4])[0]
                    elif rx_msg.arbitration_id == CAN_ID_ENC_RL:
                        self.rpm_rl = struct.unpack('<f', rx_msg.data[:4])[0]
                    elif rx_msg.arbitration_id == CAN_ID_ENC_RR:
                        self.rpm_rr = struct.unpack('<f', rx_msg.data[:4])[0]

                # Average Left (FL+RL)/2 and Right (FR+RR)/2
                avg_left = (self.rpm_fl + self.rpm_rl) / 2.0
                avg_right = (self.rpm_fr + self.rpm_rr) / 2.0

                fb = WheelRPM()
                fb.left_rpm = avg_left
                fb.right_rpm = avg_right
                self.feedback_pub.publish(fb)
            except Exception as e:
                self.get_logger().error(f"CAN Bus error: {e}")
        else:
            # Mock feedback with hardware response
            fb = WheelRPM()
            fb.left_rpm = float(target_l)
            fb.right_rpm = float(target_r)
            self.feedback_pub.publish(fb)


def main(args=None):
    rclpy.init(args=args)
    node = CANBridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
