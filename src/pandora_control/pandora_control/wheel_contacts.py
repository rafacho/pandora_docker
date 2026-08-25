#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import message_filters
from pyquaternion import Quaternion

# ROS2 port note (2026-08-04): gazebo_msgs/ContactsState -> ros_gz_interfaces/Contacts.
# This is a real field-layout change, not a rename:
#   ContactsState.states[]                -> Contacts.contacts[]
#   ContactState.total_wrench (Wrench)    -> no equivalent; the pre-aggregated
#                                             total force ros_gz_interfaces
#                                             doesn't compute for us, so the
#                                             first contact point's wrench
#                                             (contacts[0].wrenches[0].body_1_wrench)
#                                             is used as the closest analog.
#   ContactState.wrenches[] (Wrench[])    -> Contact.wrenches[] (JointWrench[],
#                                             each with body_1_wrench/body_2_wrench)
#   ContactState.contact_positions/normals -> Contact.positions/normals (same shape)
from ros_gz_interfaces.msg import Contacts, Contact, JointWrench
from geometry_msgs.msg import Vector3
from pandora_msgs.msg import WheelContacts

import tf2_ros
import tf2_geometry_msgs  # noqa: F401  needed to register PoseStamped conversions


class WheelContact(Node):
    def __init__(self):
        super().__init__('wheel_contacts')

        self.value = WheelContacts()

        self.pub = self.create_publisher(WheelContacts, 'wheel_contacts', 1)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        wheel1_sub = message_filters.Subscriber(self, Contacts, 'contacts_1')
        wheel2_sub = message_filters.Subscriber(self, Contacts, 'contacts_2')
        wheel3_sub = message_filters.Subscriber(self, Contacts, 'contacts_3')
        wheel4_sub = message_filters.Subscriber(self, Contacts, 'contacts_4')

        self.ts = message_filters.ApproximateTimeSynchronizer(
            [wheel1_sub, wheel2_sub, wheel3_sub, wheel4_sub], 1, 0.5)
        self.ts.registerCallback(self.callback)

    def to_quaternion(self, trans):
        # return a normalized quaternion from tf2 transform
        qx = trans.transform.rotation.x
        qy = trans.transform.rotation.y
        qz = trans.transform.rotation.z
        qw = trans.transform.rotation.w

        q = Quaternion(qw, qx, qy, qz)
        qn = q.unit
        return qn

    def _first_wrench(self, contact):
        # NOTE(ros2-port): see the field-mapping note at the top of this file.
        if contact.contacts and contact.contacts[0].wrenches:
            return contact.contacts[0].wrenches[0]
        return None

    def callback(self, f1, f2, f3, f4):
        try:
            trans1 = self.tfBuffer.lookup_transform(
                'world', 'wheel_1', f1.header.stamp,
                timeout=rclpy.duration.Duration(seconds=1.0))
            trans2 = self.tfBuffer.lookup_transform(
                'world', 'wheel_2', f2.header.stamp,
                timeout=rclpy.duration.Duration(seconds=1.0))
            trans3 = self.tfBuffer.lookup_transform(
                'world', 'wheel_3', f3.header.stamp,
                timeout=rclpy.duration.Duration(seconds=1.0))
            trans4 = self.tfBuffer.lookup_transform(
                'world', 'wheel_4', f4.header.stamp,
                timeout=rclpy.duration.Duration(seconds=1.0))

            if (trans1.header.stamp == f1.header.stamp) and (trans2.header.stamp == f2.header.stamp) \
                    and (trans3.header.stamp == f3.header.stamp) and (trans4.header.stamp == f4.header.stamp):

                q1n = self.to_quaternion(trans1)
                q2n = self.to_quaternion(trans2)
                q3n = self.to_quaternion(trans3)
                q4n = self.to_quaternion(trans4)

                # initialize f transformed
                f1t = f1
                f2t = f2
                f3t = f3
                f4t = f4

                # initialize wheelContacts
                wheelContacts = WheelContacts()
                for wheel in (wheelContacts.wheel1, wheelContacts.wheel2,
                              wheelContacts.wheel3, wheelContacts.wheel4):
                    wheel.wrenches.append(JointWrench())
                    wheel.positions.append(Vector3())
                    wheel.normals.append(Vector3())

                # Rotates all contact forces to world base
                w1 = self._first_wrench(f1)
                if w1 is not None:
                    force = w1.body_1_wrench.force
                    f1Rotated = q1n.rotate((force.x, force.y, force.z))
                    w1.body_1_wrench.force.x = f1Rotated[0]
                    w1.body_1_wrench.force.y = f1Rotated[1]
                    w1.body_1_wrench.force.z = f1Rotated[2]

                    wheelContacts.header = f1.header
                    wheelContacts.header.frame_id = ""
                    wheelContacts.wheel1 = f1t.contacts[0]

                w2 = self._first_wrench(f2)
                if w2 is not None:
                    force = w2.body_1_wrench.force
                    f2Rotated = q2n.rotate((force.x, force.y, force.z))
                    w2.body_1_wrench.force.x = f2Rotated[0]
                    w2.body_1_wrench.force.y = f2Rotated[1]
                    w2.body_1_wrench.force.z = f2Rotated[2]

                    wheelContacts.wheel2 = f2t.contacts[0]

                w3 = self._first_wrench(f3)
                if w3 is not None:
                    force = w3.body_1_wrench.force
                    f3Rotated = q3n.rotate((force.x, force.y, force.z))
                    w3.body_1_wrench.force.x = f3Rotated[0]
                    w3.body_1_wrench.force.y = f3Rotated[1]
                    w3.body_1_wrench.force.z = f3Rotated[2]

                    wheelContacts.wheel3 = f3t.contacts[0]

                w4 = self._first_wrench(f4)
                if w4 is not None:
                    force = w4.body_1_wrench.force
                    f4Rotated = q4n.rotate((force.x, force.y, force.z))
                    w4.body_1_wrench.force.x = f4Rotated[0]
                    w4.body_1_wrench.force.y = f4Rotated[1]
                    w4.body_1_wrench.force.z = f4Rotated[2]

                    wheelContacts.wheel4 = f4t.contacts[0]

            else:
                self.get_logger().error("state not found")

            # FIXME(ros2-port): `wheelContacts` is only ever assigned inside the
            # `if` branch above; if the stamps don't line up (the `else`
            # branch), this raises UnboundLocalError, which is NOT one of the
            # exception types caught below, so it propagates out of the
            # callback uncaught. Ported unchanged per the "preserve
            # control-law bugs, fix later" decision -- see code review
            # 2026-08-04.
            self.value = wheelContacts
            self.run()  # runs the publisher

        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException):
            self.get_logger().error("Can not transform wheel contacts! ")

    def run(self):
        self.pub.publish(self.value)


def main(args=None):
    rclpy.init(args=args)
    node = WheelContact()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
