#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import Imu, JointState

import tf2_ros
from tf_transformations import euler_from_quaternion # type: ignore

from geometry_msgs.msg import PolygonStamped
from geometry_msgs.msg import PointStamped
from geometry_msgs.msg import Point32

from pandora_msgs.srv import IkSrv # type: ignore

from pandora_control import MDH # type: ignore


class KinematicsWrapper(Node):
    def __init__(self):
        super().__init__('kinematic_model')

        robotParam = MDH.init_robot()
        self.robot = MDH.RobotModel(*robotParam)

        self.tfBuffer = tf2_ros.Buffer()
        self.listener = tf2_ros.TransformListener(self.tfBuffer, self)

        self.create_subscription(Imu, 'imu', self.handle_imu, 1)
        self.create_subscription(
            JointState, 'leg_controller/state', self.handle_leg_angles, 1)

        self.create_service(IkSrv, 'inverse_kinematics', self.handle_inverse_kinematics)
        self.get_logger().info("handle_inverse_kinematics is Ready")

        self.valuePolygon = PolygonStamped()
        self.valueCentroid = PointStamped()
        self.arrayImu = Float64MultiArray()
        self.valueTerrain = PolygonStamped()

        self.pubPolygon = self.create_publisher(PolygonStamped, 'support_polygon', 1)
        self.get_logger().info("publisher support_polygon is ready")

        self.pubTerrain = self.create_publisher(PolygonStamped, 'terrain_profile', 1)
        self.get_logger().info("publisher terrain_profile is ready")

        self.pubCentroid = self.create_publisher(
            PointStamped, 'support_polygon/centroid', 1)
        self.get_logger().info("publisher support_polygon/centroid is ready")

        self.pubImuPose = self.create_publisher(Float64MultiArray, 'imuPose', 1)
        self.get_logger().info("publisher imuPose is ready")

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
            'theta11': self.robot.angles[0], 'theta21': self.robot.angles[1],
            'theta31': self.robot.angles[2], 'theta41': self.robot.angles[3]}
        currentBasePose = self.robot.imu_pose
        pose = self.robot.set_state(currentBasePose, currentlegAngles, self.robot.reset_kinematic_tree)
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
            'theta11': self.robot.angles[0], 'theta21': self.robot.angles[1],
            'theta31': self.robot.angles[2], 'theta41': self.robot.angles[3]}
        currentBasePose = self.robot.imu_pose

        ik = self.robot.inverse_kinematics(
            currentlegAngles, currentBasePose, desiredBasePose, self.robot.reset_kinematic_tree)

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
                'odom', 'base_link', rclpy.time.Time(), # type: ignore
                timeout=rclpy.duration.Duration(seconds=1.0)) # type: ignore
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, # type: ignore
                tf2_ros.ExtrapolationException) as err: # type: ignore
            self.get_logger().error("TF error: %s" % err)
            return

        t = trans.transform.translation
        x = round(t.x, 3)
        y = round(t.y, 3)
        z = round(t.z, 3)

        self.robot.imu_pose = np.array([[roll, pitch, yaw], [x, y, z]])

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
