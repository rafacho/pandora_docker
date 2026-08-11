#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu, JointState

import tf2_ros
from tf_transformations import euler_from_quaternion

from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point32

from pandora_msgs.srv import IkSrv

from pandora_control import MDH


class KinematicsWrapper(Node):
    def __init__(self):
        super().__init__('kinematic_model')

        robotParam = MDH.initRobot()
        self.robot = MDH.robotModel(*robotParam)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.create_subscription(Imu, '/imu', self.handle_imu, 1)
        self.create_subscription(
            JointState, '/position_controller/state', self.handle_leg_angles, 1)

        self.create_service(IkSrv, '/inverse_kinematics', self.handle_inverse_kinematics)
        self.get_logger().info("handle_inverse_kinematics is Ready")

        self.valuePolygon = PolygonStamped()
        self.valueCentroid = PointStamped()
        self.arrayImu = Float64MultiArray()
        self.valueTerrain = PolygonStamped()

        self.pubPolygon = self.create_publisher(PolygonStamped, '/pandora/support_polygon', 1)
        self.get_logger().info("publisher /pandora/support_polygon is ready")

        self.pubTerrain = self.create_publisher(PolygonStamped, '/pandora/terrain_profile', 1)
        self.get_logger().info("publisher /pandora/terrain_profile is ready")

        self.pubCentroid = self.create_publisher(
            PointStamped, '/pandora/support_polygon/centroid', 1)
        self.get_logger().info("publisher /pandora/centroid is ready")

        self.pubImuPose = self.create_publisher(Float64MultiArray, '/pandora/imuPose', 1)
        self.get_logger().info("publisher /pandora/imuPose is ready")

        self.create_timer(1.0 / 25.0, self.run)

    def drawPolygon(self):
        supportPolygon = PolygonStamped()
        supportPolygon.polygon.points = []
        supportPolygon.header.frame_id = "odom"
        supportPolygon.header.stamp = self.get_clock().now().to_msg()

        terrainProfile = PolygonStamped()
        terrainProfile.polygon.points = []
        terrainProfile.header.frame_id = "odom"
        terrainProfile.header.stamp = self.get_clock().now().to_msg()

        currentlegAngles = {
            'teta11': self.robot.angles[0], 'teta21': self.robot.angles[1],
            'teta31': self.robot.angles[2], 'teta41': self.robot.angles[3]}
        currentBasePose = self.robot.imuPose
        pose = self.robot.setState(currentBasePose, currentlegAngles, self.robot.resetKinematicTree)
        contacts = self.robot.contacts(pose)

        for row in contacts:
            point = Point32(x=row[0], y=row[1], z=0.0)
            point2 = Point32(x=row[0], y=row[1], z=row[2])

            supportPolygon.polygon.points.append(point)
            terrainProfile.polygon.points.append(point2)

        self.valuePolygon = supportPolygon
        self.valueTerrain = terrainProfile
        self.valueCentroid = self.centroid()

    def centroid(self):
        _x_list = []
        _y_list = []
        vertexes = self.valuePolygon.polygon.points

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
        self.drawPolygon()
        self.pubPolygon.publish(self.valuePolygon)
        self.pubCentroid.publish(self.valueCentroid)
        self.pubImuPose.publish(self.arrayImu)
        self.pubTerrain.publish(self.valueTerrain)

    def handle_inverse_kinematics(self, request, response):
        roll = np.array(request.pose_set_point[0])
        pitch = np.array(request.pose_set_point[1])
        yaw = np.array(request.pose_set_point[2])
        x = np.array(request.pose_set_point[3])
        y = np.array(request.pose_set_point[4])
        z = np.array(request.pose_set_point[5])

        desiredBasePose = np.array([[roll, pitch, yaw], [x, y, z]])

        currentlegAngles = {
            'teta11': self.robot.angles[0], 'teta21': self.robot.angles[1],
            'teta31': self.robot.angles[2], 'teta41': self.robot.angles[3]}
        # FIXME(ros2-port): self.robot.imuPose is passed by reference and mutated
        # in place inside MDH.inverseKinematics (zeroes yaw/x/y), so this call
        # silently resets the stored IMU pose every time the service is called.
        # Ported unchanged per the "preserve control-law bugs, fix later"
        # decision -- see code review 2026-08-04.
        currentBasePose = self.robot.imuPose

        ik = self.robot.inverseKinematics(
            currentlegAngles, currentBasePose, desiredBasePose, self.robot.resetKinematicTree)

        response.success = ik['success']
        response.joint_cmd_angles = [float(a) for a in ik['joint_cmd_angles']]
        return response

    def handle_imu(self, imuMsg):
        self.arrayImu.data = []
        orientation_q = imuMsg.orientation
        orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
        (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
        roll = round(roll, 3)
        pitch = round(pitch, 3)
        yaw = round(yaw, 3)

        try:
            trans = self.tfBuffer.lookup_transform(
                'odom', 'base_link', rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=1.0))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as err:
            self.get_logger().error("TF error: %s" % err)
            return

        t = trans.transform.translation
        x = round(t.x, 3)
        y = round(t.y, 3)
        z = round(t.z, 3)

        self.robot.imuPose = np.array([[roll, pitch, yaw], [x, y, z]])

        self.arrayImu.data.append(roll)
        self.arrayImu.data.append(pitch)
        self.arrayImu.data.append(yaw)
        self.arrayImu.data.append(x)
        self.arrayImu.data.append(y)
        self.arrayImu.data.append(z)

    def handle_leg_angles(self, jointStateMsg):
        self.robot.angles = jointStateMsg.position


def main(args=None):
    rclpy.init(args=args)
    node = KinematicsWrapper()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
