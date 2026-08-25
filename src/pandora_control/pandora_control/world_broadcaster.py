#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Pose


class WorldBroadcaster(Node):
    def __init__(self):
        super().__init__('world_broadcaster')

        self.br = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Pose, 'pose', self.handle_pose, 1)

    def handle_pose(self, pose):
        t = geometry_msgs.msg.TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "odom"
        t.child_frame_id = "dummy"
        t.transform.translation.x = pose.position.x
        t.transform.translation.y = pose.position.y
        t.transform.translation.z = pose.position.z

        t.transform.rotation.x = pose.orientation.x
        t.transform.rotation.y = pose.orientation.y
        t.transform.rotation.z = pose.orientation.z
        t.transform.rotation.w = pose.orientation.w

        self.br.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = WorldBroadcaster()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
