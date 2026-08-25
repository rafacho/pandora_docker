#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float64
from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import PointStamped

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  needed to register PoseStamped conversions

from visualization_msgs.msg import Marker


class StaticStability(Node):
    def __init__(self):
        super().__init__('static_stability')

        self.valueCom = PointStamped()
        self.valueStability = 0.0

        self.lastSupportPolygon = None
        self.lastCom = None

        self.pubComProjection = self.create_publisher(PointStamped, 'com_projection', 1)
        self.get_logger().info("publisher com_projection is ready")
        self.pubStaticStability = self.create_publisher(Float64, 'static_stability', 1)
        self.get_logger().info("publisher static_stability is ready")

        self.create_subscription(
            PolygonStamped, 'support_polygon', self.support_polygon_callback, 1)
        self.create_subscription(Marker, 'com', self.com_callback, 1)

    def support_polygon_callback(self, support_polygon):
        self.lastSupportPolygon = support_polygon
        if self.lastCom is not None:
            self.callback(self.lastSupportPolygon, self.lastCom)

    def com_callback(self, com):
        self.lastCom = com
        if self.lastSupportPolygon is not None:
            self.callback(self.lastSupportPolygon, self.lastCom)

    def callback(self, support_polygon, com):
        try:
            comProjectionPoint = PointStamped()
            comProjectionPoint.header.stamp = self.get_clock().now().to_msg()
            comProjectionPoint.header.frame_id = "odom"
            comProjectionPoint.point.x = com.pose.position.x
            comProjectionPoint.point.y = com.pose.position.y
            comProjectionPoint.point.z = 0.0

            comProjection = np.array([com.pose.position.x, com.pose.position.y, 0])

            minDist = 100  # initialize minDist with a large number to enter the if condition minDist > d[2]
            if len(support_polygon.polygon.points) < 3:
                minDist = 0
            else:
                for i in range(len(support_polygon.polygon.points)):
                    p1 = np.array([support_polygon.polygon.points[i].x, support_polygon.polygon.points[i].y, 0])

                    if (i + 1) >= len(support_polygon.polygon.points):
                        p2 = np.array([support_polygon.polygon.points[0].x, support_polygon.polygon.points[0].y, 0])
                    else:
                        p2 = np.array([support_polygon.polygon.points[i+1].x, support_polygon.polygon.points[i+1].y, 0])

                    d = np.cross(p2-p1, comProjection-p1) / np.linalg.norm(p2-p1)

                    if minDist > d[2]:
                        minDist = d[2]

            self.valueCom = comProjectionPoint
            self.valueStability = minDist
            self.run()  # runs the publisher

        except Exception:
            self.get_logger().error("Can not calculate static stability! ")

    def run(self):
        self.pubComProjection.publish(self.valueCom)
        self.pubStaticStability.publish(Float64(data=float(self.valueStability)))


def main(args=None):
    rclpy.init(args=args)
    node = StaticStability()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
