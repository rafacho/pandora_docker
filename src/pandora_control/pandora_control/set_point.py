#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from pandora_msgs.msg import HrefCommand #type: ignore
import numpy as np

import tf2_ros
from tf_transformations import euler_from_quaternion


def set_point():
    rclpy.init()
    node = Node('pose_sp')

    Href_command = node.create_publisher(HrefCommand, '/pose_controller/command', 1)
    rate_hz = 3.0  # dar tempo do controlador ler

    tfBuffer = tf2_ros.Buffer()
    listener = tf2_ros.TransformListener(tfBuffer, node)

    sp = HrefCommand()

    roll_values = [0, np.pi/12, np.pi/6, np.pi/4, np.pi/6, np.pi/12, 0, -np.pi/12, -np.pi/6, -np.pi/4, 0]
    for i in roll_values:

        try:
            trans = tfBuffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as err:
            node.get_logger().error("TF error: %s" % err)
            rclpy.spin_once(node, timeout_sec=1.0 / rate_hz)
            continue

        orientation_q = trans.transform.rotation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)

        roll = i
        pitch = 0

        t = trans.transform.translation
        x = round(t.x, 3)
        y = round(t.y, 3)
        z = 0.3

        sp.position = [
            float(roll),
            float(pitch),
            float(yaw),
            float(x),
            float(y),
            float(z),
        ]

        node.get_logger().info("desiredBasePose: %s" % sp.position)
        Href_command.publish(sp)

        rclpy.spin_once(node, timeout_sec=1.0 / rate_hz)

    node.destroy_node()
    rclpy.shutdown()


def main(args=None):
    set_point()


if __name__ == '__main__':
    main()
