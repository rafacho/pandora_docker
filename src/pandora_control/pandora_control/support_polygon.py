#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.time import Time

# ROS2 port note (2026-08-04): gazebo_msgs/ContactsState (ROS1) -> the closest
# analog under Gazebo Sim / ros_gz is ros_gz_interfaces/Contacts. This script
# only ever checked whether a contact list was non-empty (never read force/
# position fields from it), so the port is a straight rename: `.states` ->
# `.contacts`.
from ros_gz_interfaces.msg import Contacts

from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import PointStamped

import tf2_ros

# Mirrors the old message_filters.ApproximateTimeSynchronizer's slop=0.5 --
# without it, a wheel whose contact topic goes silent would keep its last
# cached message reused forever instead of the whole update simply not
# firing, like the synchronizer used to do.
MAX_CONTACT_AGE = 0.5  # seconds


class realSupportPolygon(Node):
    def __init__(self):
        super().__init__('support_polygon')

        self.value = PolygonStamped()
        self.valueCentroid = PointStamped()

        self.polygon_pub = self.create_publisher(PolygonStamped, '/pandora/real_support_polygon', 1)
        self.get_logger().info("publisher /pandora/real_support_polygon is ready")

        self.pubCentroid = self.create_publisher(
            PointStamped, '/pandora/real_support_polygon/centroid', 1)
        self.get_logger().info("publisher /pandora/real_support_polygon/centroid is ready")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.lastContacts = [None, None, None, None]

        self.create_subscription(
            Contacts, '/pandora/contacts_1', self.make_contact_callback(0), 1)
        self.create_subscription(
            Contacts, '/pandora/contacts_2', self.make_contact_callback(1), 1)
        self.create_subscription(
            Contacts, '/pandora/contacts_3', self.make_contact_callback(2), 1)
        self.create_subscription(
            Contacts, '/pandora/contacts_4', self.make_contact_callback(3), 1)

    def make_contact_callback(self, index):
        def contact_callback(msg):
            self.lastContacts[index] = msg
            if all(c is not None for c in self.lastContacts):
                self.drawPolygon(*self.lastContacts)
        return contact_callback

    def drawPolygon(self, w1, w2, w3, w4):
        now = self.get_clock().now()
        for msg in (w1, w2, w3, w4):
            age = (now - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
            if age > MAX_CONTACT_AGE:
                self.get_logger().error(
                    "Stale wheel contact data (%.2fs old) -- skipping support polygon update"
                    % age)
                return

        try:
            supportPolygon = PolygonStamped()
            supportPolygon.header.frame_id = "odom"
            supportPolygon.header.stamp = self.get_clock().now().to_msg()

            contacts = [w1, w2, w3, w4]  # wheel contacts
            for i in range(4):
                if contacts[i].contacts:
                    wheel = 'wheel_' + str(i + 1)
                    trans = self.tfBuffer.lookup_transform(
                        'odom', wheel, rclpy.time.Time(), # type: ignore
                        timeout=rclpy.duration.Duration(seconds=1.0)) # type: ignore
                    positionWheel = trans.transform.translation
                    positionWheel.z = 0.0
                    supportPolygon.polygon.points.append(positionWheel) # type: ignore

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, # type: ignore
                tf2_ros.ExtrapolationException) as err: # type: ignore
            self.get_logger().error("TF error: %s" % err)

        # NOTE(ros2-port): faithfully preserved -- on a TF error above, this
        # still runs and publishes whatever partial `supportPolygon` was built
        # before the exception, exactly like the ROS1 version (no early
        # return). See code review 2026-08-04.
        self.value = supportPolygon # type: ignore
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
                _x_list.append(vertexes[i].x) # type: ignore
                _y_list.append(vertexes[i].y) # type: ignore

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
    node = realSupportPolygon()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
