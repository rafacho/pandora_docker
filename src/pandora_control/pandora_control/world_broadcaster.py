#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS2 port note (2026-08-04): the ROS1 version polled the Gazebo Classic
# "/gazebo/get_link_state" service (gazebo_msgs/GetLinkState), which has no
# Gazebo Sim / ros_gz equivalent -- there is no such service in ros_gz_bridge.
# Redesigned to subscribe to a topic instead: pandora_gazebo/urdf/pandora.gazebo
# adds a gz-sim-pose-publisher-system plugin that publishes the "pandora"
# model's root-link ("dummy") pose on a gz-transport topic, which
# pandora_launch bridges to ROS2 as geometry_msgs/msg/Pose on /model/pandora/pose.
# This node just republishes that as the "odom" -> "dummy" TF the rest of the
# stack expects (robot_state_publisher fills in "dummy" -> "base_link" and
# below from the URDF's fixed/revolute joints).

import rclpy
from rclpy.node import Node

import tf2_ros
import geometry_msgs.msg
from geometry_msgs.msg import Pose


class WorldBroadcaster(Node):
    def __init__(self):
        super().__init__('world_broadcaster')

        self.br = tf2_ros.TransformBroadcaster(self)
        self.create_subscription(Pose, '/model/pandora/pose', self.handle_pose, 1)

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
