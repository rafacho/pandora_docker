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
from geometry_msgs.msg import Point32

import tf2_ros

# Mirrors the old message_filters.ApproximateTimeSynchronizer's slop=0.5 --
# without it, a wheel whose contact topic goes silent would keep its last
# cached message reused forever instead of the whole update simply not
# firing, like the synchronizer used to do.
MAX_CONTACT_AGE = 0.1  # seconds


class realSupportPolygon(Node):
    def __init__(self):
        super().__init__('real_support_polygon')

        self.value = PolygonStamped()
        self.valueCentroid = PointStamped()

        self.polygon_pub = self.create_publisher(PolygonStamped, 'real_support_polygon', 1)
        self.get_logger().info("publisher real_support_polygon is ready")

        self.pubCentroid = self.create_publisher(
            PointStamped, 'real_support_polygon/centroid', 1)
        self.get_logger().info("publisher real_support_polygon/centroid is ready")

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.lastContacts = [None, None, None, None]

        self.create_subscription(
            Contacts, 'contacts_1', self.make_contact_callback(0), 1)
        self.create_subscription(
            Contacts, 'contacts_2', self.make_contact_callback(1), 1)
        self.create_subscription(
            Contacts, 'contacts_3', self.make_contact_callback(2), 1)
        self.create_subscription(
            Contacts, 'contacts_4', self.make_contact_callback(3), 1)

    def make_contact_callback(self, index):
        def contact_callback(msg):
            self.lastContacts[index] = msg
            if all(c is not None for c in self.lastContacts):
                self.drawPolygon(*self.lastContacts)
        return contact_callback

    def drawPolygon(self, w1, w2, w3, w4):
        # Guard the whole body: this runs inside a subscription callback on the
        # single-threaded executor, and an exception escaping here doesn't just
        # log -- it tears the process down with SIGABRT. Every expected failure
        # (stale data, missing TF) is handled below; this catch-all is only for
        # genuinely unexpected bugs, and skips the tick rather than crashing.
        try:
            now = self.get_clock().now()

            supportPolygon = PolygonStamped()
            supportPolygon.header.frame_id = "odom"
            supportPolygon.header.stamp = now.to_msg()

            contacts = [w1, w2, w3, w4]  # per-wheel contact messages
            for i in range(4):
                msg = contacts[i]

                # A wheel is a support-polygon vertex only while it is actually
                # on the ground, i.e. its contact sensor reported at least one
                # contact *recently*. Two ways that fails:
                #   - msg.contacts is empty: sensor published, nothing touching;
                #   - msg is stale: the gz-sim contact sensor stops publishing
                #     altogether while the wheel is airborne, so the last cached
                #     message would otherwise be reused forever.
                # Either way, drop just this wheel and keep building the polygon
                # from the others (this was a global `return` before -- the
                # whole polygon froze the moment any single wheel lifted off).
                age = (now - Time.from_msg(msg.header.stamp)).nanoseconds / 1e9
                if age > MAX_CONTACT_AGE or not msg.contacts:
                    continue

                wheel = 'wheel_' + str(i + 1)
                try:
                    # No timeout= here: a blocking wait can't be satisfied
                    # from inside a callback on the single-threaded executor
                    # (the TransformListener needs that same thread to drain
                    # /tf), so it would just stall the full duration and
                    # then fail anyway. Time() = latest transform available.
                    trans = self.tfBuffer.lookup_transform(
                        'odom', wheel, rclpy.time.Time())  # type: ignore
                except tf2_ros.TransformException as err:  # type: ignore
                    # wheel_N not in the TF tree yet (robot_state_publisher
                    # / joint_state_broadcaster still coming up, or the leg
                    # loop-closure links not attached). Skip this wheel this
                    # tick, like com_publisher does per-link.
                    self.get_logger().warning(
                        "TF error looking up odom->%s: %s" % (wheel, err))
                    continue

                point = Point32()
                point.x = trans.transform.translation.x
                point.y = trans.transform.translation.y
                point.z = 0.0
                supportPolygon.polygon.points.append(point)  # type: ignore

        except Exception as err:  # noqa: BLE001 -- see comment above
            self.get_logger().error("drawPolygon failed, skipping tick: %s" % err)
            return

        # Publishes whatever polygon was built -- one vertex per grounded wheel,
        # so 4 points with all wheels down, fewer (a triangle / segment) while
        # one or more wheels are off the ground.
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
