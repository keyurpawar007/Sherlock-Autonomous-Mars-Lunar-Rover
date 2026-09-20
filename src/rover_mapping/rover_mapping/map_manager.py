#!/usr/bin/env python3

import math
import os

import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
)

from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger


class MapManager(Node):

    def __init__(self):
        super().__init__('map_manager')

        # -------------------------------------------------
        # Parameters
        # -------------------------------------------------

        self.declare_parameter(
            'map_topic',
            '/map'
        )

        self.declare_parameter(
            'loaded_map_topic',
            '/loaded_map'
        )

        self.declare_parameter(
            'map_directory',
            os.path.expanduser('~/ros2_ws/maps')
        )

        self.declare_parameter(
            'map_name',
            'rover_map'
        )

        self.map_topic = (
            self.get_parameter('map_topic')
            .get_parameter_value()
            .string_value
        )

        self.loaded_map_topic = (
            self.get_parameter('loaded_map_topic')
            .get_parameter_value()
            .string_value
        )

        self.map_directory = os.path.expanduser(
            self.get_parameter('map_directory')
            .get_parameter_value()
            .string_value
        )

        self.map_name = (
            self.get_parameter('map_name')
            .get_parameter_value()
            .string_value
        )

        # Latest SLAM map received
        self.latest_map = None

        # -------------------------------------------------
        # QoS
        # -------------------------------------------------

        qos = QoSProfile(depth=1)

        qos.reliability = (
            ReliabilityPolicy.RELIABLE
        )

        qos.durability = (
            DurabilityPolicy.TRANSIENT_LOCAL
        )

        # -------------------------------------------------
        # Subscribe to SLAM map
        # -------------------------------------------------

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            qos
        )

        # -------------------------------------------------
        # Publisher used when loading a saved map
        # -------------------------------------------------

        self.loaded_map_pub = self.create_publisher(
            OccupancyGrid,
            self.loaded_map_topic,
            qos
        )

        # -------------------------------------------------
        # Services
        # -------------------------------------------------

        self.save_service = self.create_service(
            Trigger,
            '/rover_mapping/save_map',
            self.save_map_callback
        )

        self.load_service = self.create_service(
            Trigger,
            '/rover_mapping/load_map',
            self.load_map_callback
        )

        os.makedirs(
            self.map_directory,
            exist_ok=True
        )

        self.first_map = True

        self.get_logger().info(
            'Map manager started'
        )

        self.get_logger().info(
            f'Listening to: {self.map_topic}'
        )

        self.get_logger().info(
            f'Loaded map output: '
            f'{self.loaded_map_topic}'
        )

        self.get_logger().info(
            f'Map directory: '
            f'{self.map_directory}'
        )

        self.get_logger().info(
            f'Map name: {self.map_name}'
        )

    # =====================================================
    # MAP CALLBACK
    # =====================================================

    def map_callback(self, msg):

        self.latest_map = msg

        if self.first_map:

            self.first_map = False

            self.get_logger().info(
                'Map received successfully'
            )

            self.get_logger().info(
                f'Map size: '
                f'{msg.info.width} x '
                f'{msg.info.height}'
            )

            self.get_logger().info(
                f'Resolution: '
                f'{msg.info.resolution:.3f} m/cell'
            )

    # =====================================================
    # SAVE MAP SERVICE
    # =====================================================

    def save_map_callback(
        self,
        request,
        response
    ):

        if self.latest_map is None:

            response.success = False
            response.message = (
                'No /map message has been received yet'
            )

            return response

        try:

            yaml_path = os.path.join(
                self.map_directory,
                self.map_name + '.yaml'
            )

            pgm_path = os.path.join(
                self.map_directory,
                self.map_name + '.pgm'
            )

            self.write_pgm(
                pgm_path,
                self.latest_map
            )

            self.write_yaml(
                yaml_path,
                pgm_path,
                self.latest_map
            )

            response.success = True

            response.message = (
                f'Map saved successfully: '
                f'{yaml_path}'
            )

            self.get_logger().info(
                response.message
            )

        except Exception as exc:

            response.success = False

            response.message = (
                f'Failed to save map: {exc}'
            )

            self.get_logger().error(
                response.message
            )

        return response

    # =====================================================
    # WRITE PGM IMAGE
    # =====================================================

    def write_pgm(
        self,
        filename,
        map_msg
    ):

        width = map_msg.info.width
        height = map_msg.info.height

        data = map_msg.data

        with open(filename, 'wb') as file:

            header = (
                f'P5\n'
                f'# CREATOR: rover_mapping\n'
                f'{width} {height}\n'
                f'255\n'
            )

            file.write(
                header.encode('ascii')
            )

            # OccupancyGrid starts at the bottom-left.
            # PGM starts at the top-left.
            # Therefore rows are vertically reversed.

            for y in range(
                height - 1,
                -1,
                -1
            ):

                row = bytearray()

                for x in range(width):

                    index = (
                        y * width + x
                    )

                    value = data[index]

                    if value < 0:

                        pixel = 205

                    elif value >= 65:

                        pixel = 0

                    elif value <= 25:

                        pixel = 254

                    else:

                        pixel = 205

                    row.append(pixel)

                file.write(row)

    # =====================================================
    # WRITE YAML METADATA
    # =====================================================

    def write_yaml(
        self,
        yaml_path,
        pgm_path,
        map_msg
    ):

        origin = map_msg.info.origin

        yaw = self.quaternion_to_yaw(
            origin.orientation
        )

        metadata = {
            'image': os.path.basename(
                pgm_path
            ),
            'mode': 'trinary',
            'resolution': float(
                map_msg.info.resolution
            ),
            'origin': [
                float(origin.position.x),
                float(origin.position.y),
                float(yaw),
            ],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25,
        }

        with open(
            yaml_path,
            'w'
        ) as file:

            yaml.safe_dump(
                metadata,
                file,
                sort_keys=False
            )

    # =====================================================
    # LOAD MAP SERVICE
    # =====================================================

    def load_map_callback(
        self,
        request,
        response
    ):

        try:

            yaml_path = os.path.join(
                self.map_directory,
                self.map_name + '.yaml'
            )

            if not os.path.exists(
                yaml_path
            ):

                response.success = False

                response.message = (
                    f'Map file does not exist: '
                    f'{yaml_path}'
                )

                return response

            with open(
                yaml_path,
                'r'
            ) as file:

                metadata = yaml.safe_load(
                    file
                )

            image_name = metadata['image']

            if os.path.isabs(
                image_name
            ):

                pgm_path = image_name

            else:

                pgm_path = os.path.join(
                    os.path.dirname(
                        yaml_path
                    ),
                    image_name
                )

            width, height, pixels = (
                self.read_pgm(
                    pgm_path
                )
            )

            map_msg = OccupancyGrid()

            map_msg.header.stamp = (
                self.get_clock().now()
                .to_msg()
            )

            map_msg.header.frame_id = 'map'

            map_msg.info.resolution = float(
                metadata['resolution']
            )

            map_msg.info.width = width
            map_msg.info.height = height

            origin = metadata['origin']

            map_msg.info.origin.position.x = (
                float(origin[0])
            )

            map_msg.info.origin.position.y = (
                float(origin[1])
            )

            map_msg.info.origin.position.z = 0.0

            yaw = float(origin[2])

            map_msg.info.origin.orientation.z = (
                math.sin(yaw / 2.0)
            )

            map_msg.info.origin.orientation.w = (
                math.cos(yaw / 2.0)
            )

            occupied_threshold = float(
                metadata.get(
                    'occupied_thresh',
                    0.65
                )
            )

            free_threshold = float(
                metadata.get(
                    'free_thresh',
                    0.25
                )
            )

            negate = int(
                metadata.get(
                    'negate',
                    0
                )
            )

            occupancy_data = []

            # Convert PGM rows back into
            # OccupancyGrid bottom-left ordering.

            for map_y in range(height):

                image_y = (
                    height - 1 - map_y
                )

                for x in range(width):

                    image_index = (
                        image_y * width + x
                    )

                    pixel = pixels[
                        image_index
                    ]

                    if negate:

                        occupancy = (
                            pixel / 255.0
                        )

                    else:

                        occupancy = (
                            255 - pixel
                        ) / 255.0

                    if (
                        occupancy
                        > occupied_threshold
                    ):

                        value = 100

                    elif (
                        occupancy
                        < free_threshold
                    ):

                        value = 0

                    else:

                        value = -1

                    occupancy_data.append(
                        value
                    )

            map_msg.data = occupancy_data

            self.loaded_map_pub.publish(
                map_msg
            )

            response.success = True

            response.message = (
                f'Map loaded successfully '
                f'from {yaml_path}'
            )

            self.get_logger().info(
                response.message
            )

        except Exception as exc:

            response.success = False

            response.message = (
                f'Failed to load map: {exc}'
            )

            self.get_logger().error(
                response.message
            )

        return response

    # =====================================================
    # READ PGM
    # =====================================================

    def read_pgm(self, filename):

        with open(filename, 'rb') as file:

            magic = self.read_token(file)

            if magic != b'P5':

                raise RuntimeError(
                    'Only binary P5 PGM maps '
                    'are supported'
                )

            width = int(
                self.read_token(file)
            )

            height = int(
                self.read_token(file)
            )

            max_value = int(
                self.read_token(file)
            )

            if max_value != 255:

                raise RuntimeError(
                    'PGM max value must be 255'
                )

            pixels = file.read(
                width * height
            )

            if len(pixels) != (
                width * height
            ):

                raise RuntimeError(
                    'PGM image data is incomplete'
                )

            return (
                width,
                height,
                pixels
            )

    def read_token(self, file):

        token = bytearray()

        # Skip spaces and comments
        while True:

            character = file.read(1)

            if not character:

                raise RuntimeError(
                    'Unexpected end of PGM file'
                )

            if character == b'#':

                file.readline()
                continue

            if not character.isspace():

                token.extend(
                    character
                )
                break

        while True:

            character = file.read(1)

            if (
                not character
                or character.isspace()
            ):

                break

            token.extend(
                character
            )

        return bytes(token)

    # =====================================================
    # QUATERNION → YAW
    # =====================================================

    def quaternion_to_yaw(
        self,
        quaternion
    ):

        return math.atan2(
            2.0 * (
                quaternion.w
                * quaternion.z
                +
                quaternion.x
                * quaternion.y
            ),
            1.0
            -
            2.0
            * (
                quaternion.y
                * quaternion.y
                +
                quaternion.z
                * quaternion.z
            )
        )


def main(args=None):

    rclpy.init(args=args)

    node = MapManager()

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
