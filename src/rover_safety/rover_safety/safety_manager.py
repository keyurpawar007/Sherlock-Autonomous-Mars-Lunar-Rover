#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
from rover_interfaces.msg import CliffAlert, ObstacleInfo, TerrainState


class SafetyManagerNode(Node):
    """
    Parallel Safety Watchdog & Emergency Override Manager.
    Aggregates cliff hazards, proximity collisions (<= 0.90m), localization losses, and steep slopes (> 35 deg).
    Triggers immediate emergency stop override (/safety/stop = True).
    """

    def __init__(self):
        super().__init__('safety_manager')

        self.declare_parameter('stop_safety_dist', 0.90)  # 90 cm emergency stop braking distance
        self.stop_dist = self.get_parameter('stop_safety_dist').value

        self.cliff_hazard = False
        self.collision_hazard = False
        self.localization_lost = False
        self.steep_slope_hazard = False

        self.last_reason = "SYSTEM OK"

        # Subscriptions
        self.create_subscription(CliffAlert, '/perception/cliffs', self.cliff_cb, 10)
        self.create_subscription(ObstacleInfo, '/perception/obstacle_info', self.obstacle_cb, 10)
        self.create_subscription(Bool, '/localization/lost', self.loc_cb, 10)
        self.create_subscription(TerrainState, '/perception/terrain', self.terrain_cb, 10)

        # Publishers
        self.stop_pub = self.create_publisher(Bool, '/safety/stop', 10)
        self.status_pub = self.create_publisher(String, '/safety/status', 10)

        self.create_timer(0.05, self.safety_loop)  # 20 Hz Watchdog
        self.get_logger().info(f"Parallel Safety Manager Node initialized (E-Stop Proximity: {self.stop_dist}m)")

    def cliff_cb(self, msg: CliffAlert):
        self.cliff_hazard = msg.cliff_detected
        if self.cliff_hazard:
            self.last_reason = f"CLIFF DETECTED ({msg.distance_to_drop:.2f}m)"

    def obstacle_cb(self, msg: ObstacleInfo):
        if msg.detected and msg.surety and msg.distance_m <= self.stop_dist:
            self.collision_hazard = True
            self.last_reason = f"PROXIMITY COLLISION ({msg.distance_m:.2f}m <= {self.stop_dist:.2f}m)"
        else:
            self.collision_hazard = False

    def loc_cb(self, msg: Bool):
        self.localization_lost = msg.data
        if self.localization_lost:
            self.last_reason = "LOCALIZATION UNCERTAINTY HIGH"

    def terrain_cb(self, msg: TerrainState):
        self.steep_slope_hazard = not msg.is_traversable
        if self.steep_slope_hazard:
            self.last_reason = f"UNSAFE SLOPE ({msg.slope_angle_deg:.1f}°)"

    def safety_loop(self):
        should_stop = (self.cliff_hazard or self.collision_hazard or self.localization_lost or self.steep_slope_hazard)

        stop_msg = Bool()
        stop_msg.data = should_stop
        self.stop_pub.publish(stop_msg)

        status_msg = String()
        status_msg.data = f"STATUS: {'EMERGENCY STOP' if should_stop else 'SAFE'} | REASON: {self.last_reason}"
        self.status_pub.publish(status_msg)

        if should_stop:
            self.get_logger().error(f"SAFETY OVERRIDE ENGAGED! Reason: {self.last_reason}")


def main(args=None):
    rclpy.init(args=args)
    node = SafetyManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
