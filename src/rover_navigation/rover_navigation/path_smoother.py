#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Path, OccupancyGrid
from geometry_msgs.msg import PoseStamped


class PathSmoother(Node):

    def __init__(self):
        super().__init__('path_smoother')

        self.costmap = None

        self.create_subscription(
            OccupancyGrid,
            '/costmap',
            self.costmap_callback,
            10
        )

        self.create_subscription(
            Path,
            '/global_path_raw',
            self.path_callback,
            10
        )

        self.pub = self.create_publisher(
            Path,
            '/global_path',
            10
        )

        self.get_logger().info(
            'Path smoother started: '
            '/global_path_raw -> /global_path'
        )

    def costmap_callback(self, msg):
        self.costmap = msg

    # ---------------------------------------------------------
    # World -> costmap grid
    # ---------------------------------------------------------

    def world_to_grid(self, x, y):

        if self.costmap is None:
            return None

        info = self.costmap.info

        ox = info.origin.position.x
        oy = info.origin.position.y

        q = info.origin.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )

        dx = x - ox
        dy = y - oy

        c = math.cos(-yaw)
        s = math.sin(-yaw)

        mx = c * dx - s * dy
        my = s * dx + c * dy

        gx = int(math.floor(mx / info.resolution))
        gy = int(math.floor(my / info.resolution))

        return gx, gy

    def cell_free(self, gx, gy):

        if self.costmap is None:
            return False

        width = self.costmap.info.width
        height = self.costmap.info.height

        if gx < 0 or gy < 0 or gx >= width or gy >= height:
            return False

        value = self.costmap.data[
            gy * width + gx
        ]

        # Unknown or occupied = unsafe.
        if value < 0:
            return False

        return value < 65

    # ---------------------------------------------------------
    # Bresenham line-of-sight
    # ---------------------------------------------------------

    def line_is_free(self, p1, p2):

        g1 = self.world_to_grid(p1.x, p1.y)
        g2 = self.world_to_grid(p2.x, p2.y)

        if g1 is None or g2 is None:
            return False

        x0, y0 = g1
        x1, y1 = g2

        dx = abs(x1 - x0)
        dy = abs(y1 - y0)

        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1

        err = dx - dy

        while True:

            if not self.cell_free(x0, y0):
                return False

            if x0 == x1 and y0 == y1:
                break

            e2 = 2 * err

            if e2 > -dy:
                err -= dy
                x0 += sx

            if e2 < dx:
                err += dx
                y0 += sy

        return True

    # ---------------------------------------------------------
    # Greedy string-pulling
    # ---------------------------------------------------------

    def simplify(self, poses):

        if len(poses) <= 2:
            return poses

        if self.costmap is None:
            return poses

        result = [poses[0]]

        current = 0
        last = len(poses) - 1

        while current < last:

            chosen = current + 1

            # Find farthest future point directly visible
            # without crossing an obstacle.
            for candidate in range(
                last,
                current,
                -1
            ):

                p1 = poses[current].pose.position
                p2 = poses[candidate].pose.position

                if self.line_is_free(p1, p2):
                    chosen = candidate
                    break

            result.append(poses[chosen])

            if chosen == current:
                break

            current = chosen

        # Guarantee exact original final goal.
        if result[-1] is not poses[-1]:
            result.append(poses[-1])

        return result

    def path_callback(self, msg):

        if not msg.poses:
            return

        smoothed = self.simplify(msg.poses)

        output = Path()

        output.header = msg.header
        output.header.stamp = (
            self.get_clock().now().to_msg()
        )

        output.poses = smoothed

        # Retimestamp waypoints.
        for pose in output.poses:
            pose.header.frame_id = msg.header.frame_id
            pose.header.stamp = output.header.stamp

        self.pub.publish(output)

        self.get_logger().info(
            f'Path simplified: '
            f'{len(msg.poses)} -> {len(smoothed)} waypoints'
        )


def main(args=None):

    rclpy.init(args=args)

    node = PathSmoother()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    node.destroy_node()

    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
