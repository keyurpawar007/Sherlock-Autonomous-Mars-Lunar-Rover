#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseArray, PoseStamped
from rover_interfaces.msg import MissionStatus
from rover_interfaces.srv import SetMode


class MissionManagerNode(Node):
    """
    Top-Level Mission State Machine & Autonomous Mode Gatekeeper.
    Manages competition tasks (RADO Autonomous Delivery, ABEx Bio-exploration, IDMO Assistance).
    Enforces Software Command Lock during AUTONOMOUS operation per IRC rulebook.
    """

    def __init__(self):
        super().__init__('mission_manager')

        # States: TELEOP, AUTONOMOUS, ABEX, IDMO, EMERGENCY_STOP
        self.active_mode = "TELEOP"
        self.current_state = "IDLE"
        self.active_target = "NONE"
        self.mission_start_time = self.get_clock().now()
        self.autonomy_locked = False

        self.object_list = []

        # Service for Mode Switching
        self.create_service(SetMode, '/mission/set_mode', self.handle_set_mode)

        # Subscriptions
        self.create_subscription(PoseArray, '/perception/objects', self.objects_cb, 10)
        self.create_subscription(Bool, '/safety/stop', self.safety_cb, 10)

        # Publishers
        self.status_pub = self.create_publisher(MissionStatus, '/mission/status', 10)
        self.goal_pub = self.create_publisher(PoseStamped, '/navigation/goal', 10)

        self.create_timer(0.1, self.state_machine_loop)  # 10 Hz
        self.get_logger().info("Mission Manager Node initialized (Default Mode: TELEOP)")

    def handle_set_mode(self, request: SetMode.Request, response: SetMode.Response):
        mode = request.mode.upper()
        if mode in ["TELEOP", "AUTONOMOUS", "ABEX", "IDMO", "EMERGENCY_STOP"]:
            self.active_mode = mode
            self.autonomy_locked = (mode in ["AUTONOMOUS", "ABEX"])
            response.success = True
            response.message = f"Mission mode successfully changed to {self.active_mode}"
            self.get_logger().info(f"MODE SWITCHED: {self.active_mode} (Autonomy Lock: {self.autonomy_locked})")
        else:
            response.success = False
            response.message = f"Invalid mode requested: {mode}"

        return response

    def objects_cb(self, msg: PoseArray):
        if msg.poses:
            self.object_list = msg.poses

    def safety_cb(self, msg: Bool):
        if msg.data:
            self.current_state = "EMERGENCY_HALT"

    def state_machine_loop(self):
        now = self.get_clock().now()
        elapsed = (now - self.mission_start_time).nanoseconds / 1e9

        status_msg = MissionStatus()
        status_msg.header.stamp = now.to_msg()
        status_msg.mission_mode = self.active_mode
        status_msg.current_state = self.current_state
        status_msg.active_target = self.active_target
        status_msg.battery_level = 95.0
        status_msg.mission_time_sec = float(elapsed)
        status_msg.autonomy_locked = self.autonomy_locked

        self.status_pub.publish(status_msg)

        # RADO Autonomous Delivery Lifecycle Logic
        if self.active_mode == "AUTONOMOUS":
            if self.current_state == "IDLE":
                self.current_state = "RECONNAISSANCE"
                self.get_logger().info("Starting RADO Autonomous Reconnaissance...")

            elif self.current_state == "RECONNAISSANCE":
                if self.object_list:
                    self.current_state = "SELECT_OBJECT"

            elif self.current_state == "SELECT_OBJECT":
                if self.object_list:
                    target_pose = self.object_list[0]
                    goal_msg = PoseStamped()
                    goal_msg.header.stamp = now.to_msg()
                    goal_msg.header.frame_id = 'map'
                    goal_msg.pose = target_pose
                    self.goal_pub.publish(goal_msg)

                    self.active_target = "TOOLBOX_01"
                    self.current_state = "NAVIGATING_TO_OBJECT"
                    self.get_logger().info("Navigating to target object...")

        elif self.active_mode == "EMERGENCY_STOP":
            self.current_state = "EMERGENCY_HALT"


def main(args=None):
    rclpy.init(args=args)
    node = MissionManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
