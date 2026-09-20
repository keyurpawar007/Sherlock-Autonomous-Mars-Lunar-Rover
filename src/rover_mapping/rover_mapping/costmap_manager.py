#!/usr/bin/env python3

import heapq
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from nav_msgs.msg import OccupancyGrid


class CostmapManager(Node):

    def __init__(self):
        super().__init__('costmap_manager')

        # Parameters
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('costmap_topic', '/costmap')

        self.declare_parameter(
            'inflation_radius',
            0.75
        )

        self.declare_parameter(
            'occupied_threshold',
            65
        )

        self.map_topic = (
            self.get_parameter('map_topic')
            .get_parameter_value()
            .string_value
        )

        self.costmap_topic = (
            self.get_parameter('costmap_topic')
            .get_parameter_value()
            .string_value
        )

        self.inflation_radius = (
            self.get_parameter('inflation_radius')
            .get_parameter_value()
            .double_value
        )

        self.occupied_threshold = (
            self.get_parameter('occupied_threshold')
            .get_parameter_value()
            .integer_value
        )

        # Map data should remain available to nodes
        # that start after the first map message.
        qos = QoSProfile(depth=1)

        qos.reliability = (
            ReliabilityPolicy.RELIABLE
        )

        qos.durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            qos
        )

        self.costmap_pub = self.create_publisher(
            OccupancyGrid,
            self.costmap_topic,
            qos
        )

        self.first_map = True

        self.get_logger().info(
            'Costmap manager started'
        )

        self.get_logger().info(
            f'Input map: {self.map_topic}'
        )

        self.get_logger().info(
            f'Output costmap: {self.costmap_topic}'
        )

        self.get_logger().info(
            f'Inflation radius: '
            f'{self.inflation_radius:.2f} m'
        )

    def map_callback(self, msg):

        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution

        if (
            width == 0
            or height == 0
            or resolution <= 0.0
        ):
            self.get_logger().warning(
                'Received invalid map metadata'
            )
            return

        inflated_data = self.inflate_map(
            msg.data,
            width,
            height,
            resolution
        )

        output = OccupancyGrid()

        output.header = msg.header
        output.info = msg.info
        output.data = inflated_data

        self.costmap_pub.publish(output)

        if self.first_map:

            self.first_map = False

            obstacle_count = sum(
                1
                for value in msg.data
                if value >= self.occupied_threshold
            )

            self.get_logger().info(
                'Costmap generated successfully'
            )

            self.get_logger().info(
                f'Map size: {width} x {height}'
            )

            self.get_logger().info(
                f'Resolution: {resolution:.3f} m/cell'
            )

            self.get_logger().info(
                f'Occupied cells: {obstacle_count}'
            )

    def inflate_map(
        self,
        map_data,
        width,
        height,
        resolution
    ):

        data = list(map_data)

        total_cells = width * height

        distances = [
            math.inf
            for _ in range(total_cells)
        ]

        queue = []

        # Every occupied cell becomes a distance source.
        for index, value in enumerate(data):

            if value >= self.occupied_threshold:

                distances[index] = 0.0

                heapq.heappush(
                    queue,
                    (0.0, index)
                )

        # 8-connected neighbours.
        neighbours = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),

            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        ]

        max_distance_cells = (
            self.inflation_radius
            / resolution
        )

        while queue:

            distance, index = heapq.heappop(
                queue
            )

            if distance != distances[index]:
                continue

            if distance > max_distance_cells:
                continue

            x = index % width
            y = index // width

            for dx, dy, step_cost in neighbours:

                nx = x + dx
                ny = y + dy

                if (
                    nx < 0
                    or nx >= width
                    or ny < 0
                    or ny >= height
                ):
                    continue

                next_index = (
                    ny * width + nx
                )

                new_distance = (
                    distance + step_cost
                )

                if (
                    new_distance
                    < distances[next_index]
                    and
                    new_distance
                    <= max_distance_cells
                ):

                    distances[next_index] = (
                        new_distance
                    )

                    heapq.heappush(
                        queue,
                        (
                            new_distance,
                            next_index
                        )
                    )

        costmap = list(data)

        for index in range(total_cells):

            original = data[index]

            # Keep real obstacles occupied.
            if original >= self.occupied_threshold:
                costmap[index] = 100
                continue

            # Preserve unknown cells.
            if original < 0:
                costmap[index] = -1
                continue

            distance_cells = distances[index]

            if distance_cells == math.inf:
                costmap[index] = original
                continue

            distance_m = (
                distance_cells * resolution
            )

            if distance_m > self.inflation_radius:
                costmap[index] = original
                continue

            ratio = (
                1.0
                -
                (
                    distance_m
                    / self.inflation_radius
                )
            )

            inflated_cost = int(
                1 + 98 * ratio
            )

            costmap[index] = max(
                original,
                inflated_cost
            )

        return costmap


def main(args=None):

    rclpy.init(args=args)

    node = CostmapManager()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

