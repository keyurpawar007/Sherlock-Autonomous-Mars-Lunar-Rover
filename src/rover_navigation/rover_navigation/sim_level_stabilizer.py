#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node

from gazebo_msgs.msg import ModelStates
from gazebo_msgs.srv import SetEntityState


class SimLevelStabilizer(Node):

    def __init__(self):
        super().__init__('sim_level_stabilizer')

        self.model_name = 'rover'

        self.pose = None
        self.twist = None
        self.pending = None

        self.create_subscription(
            ModelStates,
            '/gazebo/model_states',
            self.model_states_callback,
            10
        )

        self.client = self.create_client(
            SetEntityState,
            '/gazebo/set_entity_state'
        )

        self.create_timer(
            0.05,       # 20 Hz
            self.update
        )

        self.get_logger().info(
            'Simulation level stabilizer started'
        )
        self.get_logger().info(
            'Keeping rover roll/pitch level; yaw remains free'
        )

    def model_states_callback(self, msg):

        try:
            index = msg.name.index(self.model_name)
        except ValueError:
            return

        self.pose = msg.pose[index]
        self.twist = msg.twist[index]

    def get_rpy(self, q):

        # roll
        sinr = 2.0 * (
            q.w * q.x +
            q.y * q.z
        )

        cosr = 1.0 - 2.0 * (
            q.x * q.x +
            q.y * q.y
        )

        roll = math.atan2(sinr, cosr)

        # pitch
        sinp = 2.0 * (
            q.w * q.y -
            q.z * q.x
        )

        if abs(sinp) >= 1.0:
            pitch = math.copysign(
                math.pi / 2.0,
                sinp
            )
        else:
            pitch = math.asin(sinp)

        # yaw
        siny = 2.0 * (
            q.w * q.z +
            q.x * q.y
        )

        cosy = 1.0 - 2.0 * (
            q.y * q.y +
            q.z * q.z
        )

        yaw = math.atan2(siny, cosy)

        return roll, pitch, yaw

    def update(self):

        if self.pose is None or self.twist is None:
            return

        if (
            self.pending is not None
            and not self.pending.done()
        ):
            return

        roll, pitch, yaw = self.get_rpy(
            self.pose.orientation
        )

        # Avoid unnecessary Gazebo service calls when level.
        tolerance = math.radians(0.25)

        if (
            abs(roll) < tolerance
            and abs(pitch) < tolerance
        ):
            return

        request = SetEntityState.Request()

        request.state.name = self.model_name
        request.state.reference_frame = 'world'

        # Preserve current XYZ position.
        request.state.pose.position.x = (
            self.pose.position.x
        )
        request.state.pose.position.y = (
            self.pose.position.y
        )
        request.state.pose.position.z = (
            self.pose.position.z
        )

        # Preserve yaw, remove roll/pitch.
        request.state.pose.orientation.x = 0.0
        request.state.pose.orientation.y = 0.0
        request.state.pose.orientation.z = (
            math.sin(yaw / 2.0)
        )
        request.state.pose.orientation.w = (
            math.cos(yaw / 2.0)
        )

        # Preserve translational motion.
        request.state.twist.linear.x = (
            self.twist.linear.x
        )
        request.state.twist.linear.y = (
            self.twist.linear.y
        )
        request.state.twist.linear.z = (
            self.twist.linear.z
        )

        # Remove roll/pitch angular velocity.
        request.state.twist.angular.x = 0.0
        request.state.twist.angular.y = 0.0
        request.state.twist.angular.z = (
            self.twist.angular.z
        )

        self.pending = self.client.call_async(
            request
        )


def main(args=None):

    rclpy.init(args=args)

    node = SimLevelStabilizer()

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
