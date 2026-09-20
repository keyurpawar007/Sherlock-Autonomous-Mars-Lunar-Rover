#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CameraNode(Node):
    def __init__(self):
        super().__init__('camera_node')
        self.get_logger().info('Rover Camera Wrapper Node started.')
        self.health_pub = self.create_publisher(String, '/camera/health', 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        msg = String()
        msg.data = "OK"
        self.health_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
