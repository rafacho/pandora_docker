#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped

PI = 3.1415926535897


class WheelControl(Node):
    def __init__(self):
        super().__init__('move_wheels')
        self.wheel_command = self.create_publisher(TwistStamped, '/velocity_controller/cmd_vel', 1)
        self.create_timer(1.0 / 25.0, self.run)

    def run(self):
        b = TwistStamped()
        b.header.stamp = self.get_clock().now().to_msg()
        b.header.frame_id = 'base_link'
        b.twist.linear.x = -0.5  # -0.01 * time_sec
        b.twist.linear.y = 0.0
        b.twist.linear.z = 0.0
        b.twist.angular.x = 0.0
        b.twist.angular.y = 0.0
        b.twist.angular.z = 0.0  # -0.1 * time_sec

        self.wheel_command.publish(b)
        print(b)


def main(args=None):
    rclpy.init(args=args)
    node = WheelControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
