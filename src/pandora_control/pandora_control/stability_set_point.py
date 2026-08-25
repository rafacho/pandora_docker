#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import PolygonStamped
from pandora_msgs.msg import HrefCommand # type: ignore

import tf2_ros
from tf_transformations import euler_from_quaternion # type: ignore


class StabilityClass(Node):
    def __init__(self):
        super().__init__('stability_set_point')

        self.create_subscription(
            PointStamped, 'support_polygon/centroid', self.stabilityCallback, 1)
        self.create_subscription(
            PolygonStamped, 'terrain_profile', self.terrainCallback, 1)

        self.pubPoseCommand = self.create_publisher(HrefCommand, 'pose_controller/command', 1)
        self.sp = HrefCommand()

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.pastTime = self.get_clock().now()
        self.pastPointX = 0
        self.pastPointY = 0

        self.terrainZ = float()

        self.create_timer(1.0 / 25.0, self.run)

    def terrainCallback(self, terrain):
        p1, p2, p3, p4 = terrain.polygon.points
        zMidPoint = (p1.z + p2.z + p3.z + p4.z) / 4

        self.terrainZ = zMidPoint

    def stabilityCallback(self, polygonCentroid):
        now = self.get_clock().now()
        dt = (now - self.pastTime).nanoseconds / 1e9
        if dt == 0:
            dt = 1 / 25

        dx = polygonCentroid.point.x - self.pastPointX
        dy = polygonCentroid.point.y - self.pastPointY

        try:
            trans = self.tfBuffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time(), # type: ignore
                timeout=rclpy.duration.Duration(seconds=1.0)) # type: ignore
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, # type: ignore
                tf2_ros.ExtrapolationException) as err: # type: ignore
            self.get_logger().error("TF error: %s" % err)
            return

        orientation_q = trans.transform.rotation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        roll = 0
        pitch = 0

        x = polygonCentroid.point.x + round(dx, 2) / dt
        y = polygonCentroid.point.y + round(dy, 2) / dt
        z = 0.3

        self.sp.position = [
            float(roll),
            float(pitch),
            float(yaw),
            float(x),
            float(y),
            float(z),
        ]

        self.pastPointX = polygonCentroid.point.x
        self.pastPointY = polygonCentroid.point.y

        self.pastTime = now

    def run(self):
        self.pubPoseCommand.publish(self.sp)


def main(args=None):
    rclpy.init(args=args)
    node = StabilityClass()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
