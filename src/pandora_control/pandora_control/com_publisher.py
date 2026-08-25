#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import xml.etree.ElementTree as ET

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from std_msgs.msg import String
from geometry_msgs.msg import PointStamped
from visualization_msgs.msg import Marker

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  needed to register PointStamped conversions


def _parse_link_masses(urdf_xml):
    # Reads mass + inertial origin straight from the processed URDF (as
    # published by robot_state_publisher on /robot_description, i.e. after
    # xacro substitution -- no macros left to expand). Only the translation
    # part of <inertial><origin> matters here: it locates the link's center
    # of mass in the link's own frame, its rotation only affects the inertia
    # tensor, not where the mass point sits.
    links = []
    root = ET.fromstring(urdf_xml)
    for link in root.findall('link'):
        inertial = link.find('inertial')
        if inertial is None:
            continue
        mass_el = inertial.find('mass')
        if mass_el is None:
            continue
        mass = float(mass_el.get('value', '0'))
        if mass <= 0.0:
            continue
        origin_el = inertial.find('origin')
        if origin_el is not None:
            xyz = [float(v) for v in origin_el.get('xyz', '0 0 0').split()]
        else:
            xyz = [0.0, 0.0, 0.0]
        links.append((link.get('name'), mass, xyz))
    return links


class ComPublisher(Node):
    def __init__(self):
        super().__init__('com_publisher')

        self.links = None

        # robot_state_publisher publishes /robot_description latched
        # (transient_local); match that durability so we get it even if
        # this node starts after robot_state_publisher.
        robot_description_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            String, 'robot_description', self.handle_robot_description, robot_description_qos)

        self.pub = self.create_publisher(Marker, 'com', 1)
        self.get_logger().info("publisher com is ready")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.timer = self.create_timer(0.1, self.publish_com)

    def handle_robot_description(self, msg):
        self.links = _parse_link_masses(msg.data)
        self.get_logger().info(
            "parsed %d links with mass from /robot_description" % len(self.links))

    def publish_com(self):
        if not self.links:
            return

        totalMass = 0.0
        weighted = [0.0, 0.0, 0.0]

        for name, mass, xyz in self.links:
            local = PointStamped()
            local.header.frame_id = name
            local.point.x, local.point.y, local.point.z = xyz

            try:
                transform = self.tfBuffer.lookup_transform('odom', name, rclpy.time.Time()) # type: ignore
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, # type: ignore
                    tf2_ros.ExtrapolationException): # type: ignore
                # Link not (yet) in the TF tree -- e.g. the detachable-joint
                # links before the loop closure attaches. Skip it for this
                # tick rather than spamming an error at 10 Hz.
                continue

            worldPoint = tf2_geometry_msgs.do_transform_point(local, transform)
            weighted[0] += mass * worldPoint.point.x
            weighted[1] += mass * worldPoint.point.y
            weighted[2] += mass * worldPoint.point.z
            totalMass += mass

        if totalMass == 0.0:
            return

        com = Marker()
        com.header.frame_id = "odom"
        com.header.stamp = self.get_clock().now().to_msg()
        com.ns = "com"
        com.id = 0
        com.type = Marker.SPHERE
        com.action = Marker.ADD
        com.pose.position.x = weighted[0] / totalMass
        com.pose.position.y = weighted[1] / totalMass
        com.pose.position.z = weighted[2] / totalMass
        com.pose.orientation.w = 1.0
        com.scale.x = 0.05
        com.scale.y = 0.05
        com.scale.z = 0.05
        com.color.r = 1.0
        com.color.g = 1.0
        com.color.b = 0.0
        com.color.a = 1.0

        self.pub.publish(com)


def main(args=None):
    rclpy.init(args=args)
    node = ComPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
