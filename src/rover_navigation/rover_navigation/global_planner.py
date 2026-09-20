#!/usr/bin/env python3

import heapq
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Path
from std_msgs.msg import String
from tf2_ros import Buffer, TransformListener, TransformException


class GlobalPlanner(Node):

    def __init__(self):
        super().__init__('global_planner')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------
        self.declare_parameter('costmap_topic', '/map')
        self.declare_parameter('goal_topic', '/goal_pose')
        self.declare_parameter('path_topic', '/global_path')

        self.declare_parameter('robot_frame', 'base_footprint')
        self.declare_parameter('obstacle_threshold', 65)
        self.declare_parameter('allow_unknown', True)
        self.declare_parameter('cost_weight', 2.0)
        self.declare_parameter('replan_cooldown', 0.75)

        self.costmap_topic = (
            self.get_parameter('costmap_topic')
            .get_parameter_value().string_value
        )

        self.goal_topic = (
            self.get_parameter('goal_topic')
            .get_parameter_value().string_value
        )

        self.path_topic = (
            self.get_parameter('path_topic')
            .get_parameter_value().string_value
        )

        self.robot_frame = (
            self.get_parameter('robot_frame')
            .get_parameter_value().string_value
        )

        self.obstacle_threshold = (
            self.get_parameter('obstacle_threshold')
            .get_parameter_value().integer_value
        )

        self.allow_unknown = (
            self.get_parameter('allow_unknown')
            .get_parameter_value().bool_value
        )

        self.cost_weight = (
            self.get_parameter('cost_weight')
            .get_parameter_value().double_value
        )

        self.replan_cooldown = (
            self.get_parameter('replan_cooldown')
            .get_parameter_value()
            .double_value
        )

        # ---------------------------------------------------------
        # Stored data
        # ---------------------------------------------------------
        self.costmap = None

        # Original final goal is preserved during replanning.
        self.current_goal = None

        # True while perception reports a blocking obstacle.
        self.obstacle_blocked = False

        # Prevent excessive A* calls.
        self.last_replan_time_ns = 0

        # ---------------------------------------------------------
        # TF
        # ---------------------------------------------------------
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(
            self.tf_buffer,
            self
        )

        # ---------------------------------------------------------
        # ROS interfaces
        # ---------------------------------------------------------
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        self.costmap_sub = self.create_subscription(
            OccupancyGrid,
            self.costmap_topic,
            self.costmap_callback,
            map_qos
        )

        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.goal_topic,
            self.goal_callback,
            10
        )

        self.obstacle_sub = self.create_subscription(
            String,
            '/obstacle_status',
            self.obstacle_status_callback,
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            self.path_topic,
            10
        )

        self.replan_timer = self.create_timer(
            0.50,
            self.replan_timer_callback
        )

        self.get_logger().info(
            'Global Planner started'
        )

        self.get_logger().info(
            f'Costmap : {self.costmap_topic}'
        )

        self.get_logger().info(
            f'Goal    : {self.goal_topic}'
        )

        self.get_logger().info(
            f'Path    : {self.path_topic}'
        )

    # =============================================================
    # COSTMAP
    # =============================================================

    def costmap_callback(self, msg):
        first_map = self.costmap is None
        self.costmap = msg

        if first_map:
            self.get_logger().info(
                'Costmap received: '
                f'{msg.info.width} x {msg.info.height}, '
                f'resolution={msg.info.resolution:.3f} m'
            )

    # =============================================================
    # DYNAMIC OBSTACLE / A* REPLANNING
    # =============================================================

    def obstacle_status_callback(self, msg):

        blocked_now = msg.data.startswith('BLOCKED')

        if blocked_now and not self.obstacle_blocked:

            self.obstacle_blocked = True

            self.get_logger().warning(
                'Dynamic obstacle detected -> A* replan requested'
            )

            self.request_replan(
                reason='new obstacle',
                force=True
            )

        elif not blocked_now and self.obstacle_blocked:

            self.obstacle_blocked = False

            self.get_logger().info(
                'Dynamic obstacle cleared'
            )


    def replan_timer_callback(self):

        if (
            self.obstacle_blocked
            and
            self.current_goal is not None
        ):
            self.request_replan(
                reason='obstacle still blocking'
            )


    def request_replan(
        self,
        reason='',
        force=False
    ):

        if self.current_goal is None:
            return

        if self.costmap is None:
            return

        now_ns = self.get_clock().now().nanoseconds

        cooldown_ns = int(
            self.replan_cooldown * 1e9
        )

        if (
            not force
            and
            (
                now_ns -
                self.last_replan_time_ns
            ) < cooldown_ns
        ):
            return

        self.last_replan_time_ns = now_ns

        self.get_logger().info(
            f'A* REPLAN | {reason}'
        )

        # Same final goal.
        # New start position = current rover pose.
        self.goal_callback(
            self.current_goal
        )


    # =============================================================
    # GOAL
    # =============================================================

    def goal_callback(self, goal):

        # Keep the SAME original final goal during replanning.
        self.current_goal = goal

        if self.costmap is None:
            self.get_logger().warning(
                'Goal received, but no costmap is available yet.'
            )
            return

        map_frame = self.costmap.header.frame_id

        if not map_frame:
            map_frame = 'map'

        if goal.header.frame_id != map_frame:
            self.get_logger().warning(
                f'Goal frame is "{goal.header.frame_id}", '
                f'but planner expects "{map_frame}".'
            )
            return

        start_world = self.get_robot_position(map_frame)

        if start_world is None:
            return

        goal_world = (
            goal.pose.position.x,
            goal.pose.position.y
        )

        self.get_logger().info(
            'Planning request: '
            f'start=({start_world[0]:.2f}, '
            f'{start_world[1]:.2f}) '
            f'goal=({goal_world[0]:.2f}, '
            f'{goal_world[1]:.2f})'
        )

        start_cell = self.world_to_grid_start(
            start_world[0],
            start_world[1]
        )

        goal_cell = self.world_to_grid(
            goal_world[0],
            goal_world[1]
        )

        if start_cell is None:
            self.get_logger().error(
                'Robot start position is outside the costmap.'
            )
            return

        if goal_cell is None:
            self.get_logger().error(
                'Goal position is outside the costmap.'
            )
            return

        if not self.is_traversable(*start_cell):
            self.get_logger().error(
                f'Start cell {start_cell} is occupied.'
            )
            return

        if not self.is_traversable(*goal_cell):
            self.get_logger().error(
                f'Goal cell {goal_cell} is occupied or unknown.'
            )
            return

        grid_path = self.astar(
            start_cell,
            goal_cell
        )

        if not grid_path:
            self.get_logger().warning(
                'No collision-free path found.'
            )

            self.publish_empty_path(map_frame)
            return

        self.publish_path(
            grid_path,
            goal,
            map_frame
        )

        length = self.calculate_path_length(grid_path)

        self.get_logger().info(
            f'Path found: {len(grid_path)} cells, '
            f'length ≈ {length:.2f} m'
        )

    # =============================================================
    # TF / START POSE
    # =============================================================

    def get_robot_position(self, target_frame):
        try:
            transform = self.tf_buffer.lookup_transform(
                target_frame,
                self.robot_frame,
                Time()
            )

            return (
                transform.transform.translation.x,
                transform.transform.translation.y
            )

        except TransformException as exc:
            self.get_logger().warning(
                f'Cannot get {target_frame} -> '
                f'{self.robot_frame} transform: {exc}'
            )

            return None

    # =============================================================
    # WORLD <-> GRID
    # =============================================================

    def world_to_grid(self, wx, wy):
        info = self.costmap.info

        ox = info.origin.position.x
        oy = info.origin.position.y

        yaw = self.quaternion_to_yaw(
            info.origin.orientation
        )

        dx = wx - ox
        dy = wy - oy

        # Rotate world point into map-grid coordinates.
        gx_local = (
            math.cos(yaw) * dx +
            math.sin(yaw) * dy
        )

        gy_local = (
            -math.sin(yaw) * dx +
            math.cos(yaw) * dy
        )

        gx = int(
            math.floor(gx_local / info.resolution)
        )

        gy = int(
            math.floor(gy_local / info.resolution)
        )

        if not self.in_bounds(gx, gy):
            return None

        return gx, gy

    def grid_to_world(self, gx, gy):
        info = self.costmap.info

        # Use centre of cell.
        lx = (gx + 0.5) * info.resolution
        ly = (gy + 0.5) * info.resolution

        yaw = self.quaternion_to_yaw(
            info.origin.orientation
        )

        wx = (
            info.origin.position.x +
            math.cos(yaw) * lx -
            math.sin(yaw) * ly
        )

        wy = (
            info.origin.position.y +
            math.sin(yaw) * lx +
            math.cos(yaw) * ly
        )

        return wx, wy

    @staticmethod
    def quaternion_to_yaw(q):
        siny_cosp = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy_cosp = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z 
        )

        return math.atan2(
            siny_cosp,
            cosy_cosp
        )

    def world_to_grid_start(self, wx, wy):
        """
        Convert the rover start position to a grid cell.

        SLAM-generated maps can sometimes end a few millimetres
        before the robot pose. If the start is only slightly outside
        the costmap, snap it to the nearest boundary cell.

        Goals are NOT clamped.
        """

        info = self.costmap.info

        ox = info.origin.position.x
        oy = info.origin.position.y

        yaw = self.quaternion_to_yaw(
            info.origin.orientation
        )

        dx = wx - ox
        dy = wy - oy

        gx_local = (
            math.cos(yaw) * dx +
            math.sin(yaw) * dy
        )

        gy_local = (
            -math.sin(yaw) * dx +
            math.cos(yaw) * dy
        )

        gx_raw = gx_local / info.resolution
        gy_raw = gy_local / info.resolution

        gx = int(math.floor(gx_raw))
        gy = int(math.floor(gy_raw))

        # Normal case
        if self.in_bounds(gx, gy):
            return gx, gy

        # Allow the rover START to be slightly outside the
        # dynamically generated map boundary.
        tolerance_cells = 10

        if (
            gx < -tolerance_cells
            or gx >= info.width + tolerance_cells
            or gy < -tolerance_cells
            or gy >= info.height + tolerance_cells
        ):
            return None

        snapped_x = min(
            max(gx, 0),
            info.width - 1
        )

        snapped_y = min(
            max(gy, 0),
            info.height - 1
        )

        self.get_logger().warning(
            'Robot start is slightly outside the costmap; '
            f'snapping grid cell ({gx}, {gy}) -> '
            f'({snapped_x}, {snapped_y}).'
        )

        return snapped_x, snapped_y

        # =============================================================
        # GRID HELPERS
        # =============================================================

    def in_bounds(self, x, y):
        return (
            0 <= x < self.costmap.info.width
            and
            0 <= y < self.costmap.info.height
        )

    def cell_value(self, x, y):
        index = (
            y * self.costmap.info.width + x
        )

        return self.costmap.data[index]

    def is_traversable(self, x, y):
        if not self.in_bounds(x, y):
            return False

        value = self.cell_value(x, y)

        if value < 0:
            return self.allow_unknown

        return value < self.obstacle_threshold

    # =============================================================
    # A*
    # =============================================================

    def astar(self, start, goal):

        # x, y, movement cost
        neighbours = [
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),

            (1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (-1, -1, math.sqrt(2.0)),
        ]

        open_heap = []

        heapq.heappush(
            open_heap,
            (
                self.heuristic(start, goal),
                0.0,
                start
            )
        )

        came_from = {}
        g_score = {
            start: 0.0
        }

        closed = set()

        while open_heap:

            _, current_g, current = heapq.heappop(
                open_heap
            )

            if current in closed:
                continue

            if current == goal:
                return self.reconstruct_path(
                    came_from,
                    current
                )

            closed.add(current)

            cx, cy = current

            for dx, dy, movement_cost in neighbours:

                nx = cx + dx
                ny = cy + dy

                neighbour = (nx, ny)

                if not self.is_traversable(nx, ny):
                    continue

                # Prevent diagonal corner cutting.
                if dx != 0 and dy != 0:

                    if (
                        not self.is_traversable(
                            cx + dx,
                            cy
                        )
                        or
                        not self.is_traversable(
                            cx,
                            cy + dy
                        )
                    ):
                        continue

                value = self.cell_value(nx, ny)

                if value < 0:
                    cell_penalty = 0.0
                else:
                    cell_penalty = (
                        self.cost_weight *
                        (float(value) / 100.0)
                    )

                tentative_g = (
                    current_g +
                    movement_cost +
                    cell_penalty
                )

                if tentative_g < g_score.get(
                    neighbour,
                    float('inf')
                ):

                    came_from[neighbour] = current
                    g_score[neighbour] = tentative_g

                    f_score = (
                        tentative_g +
                        self.heuristic(
                            neighbour,
                            goal
                        )
                    )

                    heapq.heappush(
                        open_heap,
                        (
                            f_score,
                            tentative_g,
                            neighbour
                        )
                    )

        return None

    @staticmethod
    def heuristic(a, b):
        dx = a[0] - b[0]
        dy = a[1] - b[1]

        return math.hypot(dx, dy)

    @staticmethod
    def reconstruct_path(came_from, current):
        path = [current]

        while current in came_from:
            current = came_from[current]
            path.append(current)

        path.reverse()

        return path

    # =============================================================
    # PATH PUBLISHING
    # =============================================================

    def publish_path(
        self,
        grid_path,
        goal,
        frame_id
    ):
        path_msg = Path()

        path_msg.header.frame_id = frame_id
        path_msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        for index, cell in enumerate(grid_path):

            wx, wy = self.grid_to_world(
                cell[0],
                cell[1]
            )

            pose = PoseStamped()

            pose.header.frame_id = frame_id
            pose.header.stamp = path_msg.header.stamp

            pose.pose.position.x = wx
            pose.pose.position.y = wy
            pose.pose.position.z = 0.0

            pose.pose.orientation.w = 1.0

            path_msg.poses.append(pose)

        # Preserve the requested final goal orientation.
        if path_msg.poses:

            path_msg.poses[-1].pose.orientation = (
                goal.pose.orientation
            )

        self.path_pub.publish(path_msg)

    def publish_empty_path(self, frame_id):
        msg = Path()

        msg.header.frame_id = frame_id
        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )

        self.path_pub.publish(msg)

    def calculate_path_length(self, path):
        if len(path) < 2:
            return 0.0

        resolution = self.costmap.info.resolution

        length = 0.0

        for a, b in zip(
            path[:-1],
            path[1:]
        ):

            dx = b[0] - a[0]
            dy = b[1] - a[1]

            length += (
                math.hypot(dx, dy) *
                resolution
            )

        return length


def main(args=None):
    rclpy.init(args=args)

    node = GlobalPlanner()

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
