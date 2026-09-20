#!/usr/bin/env python3

import math
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from rover_interfaces.msg import TerrainState


class TerrainDetectorNode(Node):
    """
    Terrain & Slope Angle Detector Node.
    Analyzes local point cloud surface normals to compute terrain slope angle theta
    and assess traversability (Max safe slope 35 degrees).
    """

    def __init__(self):
        super().__init__('terrain_detector')

        self.declare_parameter('depth_topic', '/d435i/depth/image_raw')
        self.declare_parameter('max_safe_slope_deg', 35.0)  # 35 degrees safe incline

        self.depth_topic = self.get_parameter('depth_topic').value
        self.max_slope_deg = self.get_parameter('max_safe_slope_deg').value

        self.create_subscription(Image, self.depth_topic, self.depth_cb, 10)
        self.terrain_pub = self.create_publisher(TerrainState, '/perception/terrain', 10)

        self.get_logger().info(f"Terrain & Slope Angle Detector started (Max Safe Slope: {self.max_slope_deg}°)")

    def depth_cb(self, msg: Image):
        # Convert depth matrix
        if msg.encoding in ('16UC1', 'mono16'):
            depth_m = np.frombuffer(msg.data, dtype=np.uint16).reshape(msg.height, msg.width).astype(np.float32) * 0.001
        elif msg.encoding == '32FC1':
            depth_m = np.frombuffer(msg.data, dtype=np.float32).reshape(msg.height, msg.width)
        else:
            return

        h, w = depth_m.shape
        roi = depth_m[int(h * 0.4):int(h * 0.8), int(w * 0.2):int(w * 0.8)]

        valid = np.isfinite(roi) & (roi > 0.3) & (roi < 3.0)
        if not np.any(valid):
            return

        # Compute vertical depth gradient dy
        dy = np.gradient(roi, axis=0)
        avg_gradient = np.mean(dy[valid])

        # Estimate inclination angle theta in degrees
        slope_angle_deg = abs(math.degrees(math.atan(avg_gradient * 5.0)))
        roughness = float(np.std(roi[valid]))

        state = TerrainState()
        state.header = msg.header
        state.slope_angle_deg = float(slope_angle_deg)
        state.roughness_score = min(1.0, roughness)

        if slope_angle_deg <= self.max_slope_deg:
            state.is_traversable = True
            state.terrain_type = "FLAT" if slope_angle_deg < 5.0 else "SLOPE"
        else:
            state.is_traversable = False
            state.terrain_type = "STEEP_SLOPE"
            self.get_logger().warn(f"STEEP SLOPE DETECTED! Angle: {slope_angle_deg:.1f}° > {self.max_slope_deg}°")

        self.terrain_pub.publish(state)


def main(args=None):
    rclpy.init(args=args)
    node = TerrainDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
