#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import message_filters

# ROS2 port note (2026-08-04): gazebo_msgs/ContactsState (ROS1) -> the closest
# analog under Gazebo Sim / ros_gz is ros_gz_interfaces/Contacts. This script
# only ever checked whether a contact list was non-empty (never read force/
# position fields from it), so the port is a straight rename: `.states` ->
# `.contacts`.
from ros_gz_interfaces.msg import Contacts

from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import PointStamped

import tf2_ros


class SupportPolygon(Node):
    def __init__(self):
        super().__init__('support_polygon')

        self.value = PolygonStamped()
        self.valueCentroid = PointStamped()

        self.polygon_pub = self.create_publisher(PolygonStamped, '/pandora/support_polygon', 1)
        self.get_logger().info("publisher /pandora/support_polygon is ready")

        self.pubCentroid = self.create_publisher(PointStamped, '/pandora/centroid', 1)
        self.get_logger().info("publisher /pandora/centroid is ready")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        wheelSub1 = message_filters.Subscriber(self, Contacts, '/pandora/contacts_1')
        wheelSub2 = message_filters.Subscriber(self, Contacts, '/pandora/contacts_2')
        wheelSub3 = message_filters.Subscriber(self, Contacts, '/pandora/contacts_3')
        wheelSub4 = message_filters.Subscriber(self, Contacts, '/pandora/contacts_4')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [wheelSub1, wheelSub2, wheelSub3, wheelSub4], 1, 0.5)
        self.ts.registerCallback(self.drawPolygon)

    def drawPolygon(self, w1, w2, w3, w4):
        try:
            supportPolygon = PolygonStamped()
            supportPolygon.header.frame_id = "odom"
            supportPolygon.header.stamp = self.get_clock().now().to_msg()

            contacts = [w1, w2, w3, w4]  # wheel contacts
            for i in range(4):
                if contacts[i].contacts:
                    wheel = 'wheel_' + str(i + 1)
                    trans = self.tfBuffer.lookup_transform(
                        'odom', wheel, rclpy.time.Time(),
                        timeout=rclpy.duration.Duration(seconds=1.0))
                    positionWheel = trans.transform.translation
                    positionWheel.z = 0.0
                    supportPolygon.polygon.points.append(positionWheel)

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as err:
            self.get_logger().error("TF error: %s" % err)

        # NOTE(ros2-port): faithfully preserved -- on a TF error above, this
        # still runs and publishes whatever partial `supportPolygon` was built
        # before the exception, exactly like the ROS1 version (no early
        # return). See code review 2026-08-04.
        self.value = supportPolygon
        self.valueCentroid = self.centroid()

        self.run()

    def centroid(self):
        _x_list = []
        _y_list = []
        vertexes = self.value.polygon.points

        centroidPoint = PointStamped()
        centroidPoint.header.stamp = self.get_clock().now().to_msg()
        centroidPoint.header.frame_id = "odom"

        if len(vertexes) != 0:
            for i in range(len(vertexes)):
                _x_list.append(vertexes[i].x)
                _y_list.append(vertexes[i].y)

            _len = len(vertexes)
            _x = sum(_x_list) / _len
            _y = sum(_y_list) / _len

            centroidPoint.point.x = _x
            centroidPoint.point.y = _y
            centroidPoint.point.z = 0.0

        return centroidPoint

    def run(self):
        self.polygon_pub.publish(self.value)
        self.pubCentroid.publish(self.valueCentroid)


def main(args=None):
    rclpy.init(args=args)
    node = SupportPolygon()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
