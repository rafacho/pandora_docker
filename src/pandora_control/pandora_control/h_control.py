#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import numpy as np

from std_msgs.msg import Float64MultiArray
from custom_messages.msg import CustomCmnd
from pandora_msgs.srv import IkSrv
from pandora_msgs.msg import HrefCommand


class HandleControl(Node):
    def __init__(self):
        super().__init__('pose_control')

        callback_group = ReentrantCallbackGroup()

        self.pubJointCommand = self.create_publisher(CustomCmnd, '/position_controller/command', 1)
        self.get_logger().info("publisher /position_controller/command is ready")

        self.create_subscription(
            HrefCommand, '/pose_controller/command', self.handlePoseCommand, 1,
            callback_group=callback_group)

        self.create_subscription(Float64MultiArray, '/pandora/imuPose', self.handleImu, 1)

        self.ikClient = self.create_client(IkSrv, '/inverse_kinematics')

        self.commandedPose = np.asarray([])
        self.currentState = []
        self.jointSuccess = False

        self.pastTime = self.get_clock().now()
        self.errorAnglesIntegral = np.asarray([0, 0, 0], dtype='float64')
        self.pastErrorAngles = np.asarray([0, 0, 0], dtype='float64')

        self.errorPositionsIntegral = np.asarray([0, 0, 0], dtype='float64')
        self.pastErrorPositions = np.asarray([0, 0, 0], dtype='float64')

        self.create_timer(1.0 / 25.0, self.run, callback_group=callback_group)

    def handleImu(self, imuMsg):
        self.currentState = np.asarray(imuMsg.data)

    def handlePoseCommand(self, spValue):
        now = self.get_clock().now()
        dt = (now - self.pastTime).nanoseconds / 1e9

        currentState = np.asarray(self.currentState)
        if currentState.size == 0:
            currentState = np.array([0, 0, 0, 0, 0, 0])

        currentAngles = np.split(currentState, 2)[0]
        currentPosition = np.split(currentState, 2)[1]

        # Estrutura do setPoint = (roll, pitch, yaw, x, y, z)
        baseSetPoint = np.asarray(spValue.position)
        if baseSetPoint.size == 0:
            baseSetPoint = np.array([0, 0, 0, 0, 0, 0.35])

        error = baseSetPoint - currentState
        errorAngles = np.split(error, 2)[0]
        errorAngles[2] = 0 # força erro em yaw ser zero pois não está sendo controlado

        errorPositions = np.split(error, 2)[1]
        errorPositions[0] = 0 # força erro em x ser zero pois não está sendo controlado
        errorPositions[1] = 0 # força erro em y ser zero pois não está sendo controlado

        #*********************************** PID ************************************************
        #****************************************************************************************
        Pkp = 0.85
        Pki = 0 #0.01
        Pkd = 0.015 #0.05

        kpRoll = 1.2 #2
        kiRoll = 0.01
        kdRoll = 0.1

        kpPitch = 0.7 #1
        kiPitch = 0.05 #0.01
        kdPitch = 0.03

        # FIXME(ros2-port): dt isn't guarded against 0 (two callbacks landing on
        # the same clock tick produces inf/nan here, unlike
        # stability_set_point.py which guards this exact pattern). Ported
        # unchanged per the "preserve control-law bugs, fix later" decision --
        # see code review 2026-08-04.
        PIDz = Pkp*errorPositions[2] + Pki*self.errorPositionsIntegral[2]*dt + Pkd*(errorPositions[2] - self.pastErrorPositions[2])/dt
        commandedZ = currentPosition[2] + PIDz

        PIDroll = kpRoll*errorAngles[0] + kiRoll*self.errorAnglesIntegral[0]*dt + kdRoll*(errorAngles[0] - self.pastErrorAngles[0])/dt
        commandedRoll = currentAngles[0] + PIDroll

        PIDpitch = kpPitch*errorAngles[1] + kiPitch*self.errorAnglesIntegral[1]*dt + kdPitch*(errorAngles[1] - self.pastErrorAngles[1])/dt
        commandedPitch = currentAngles[1] + PIDpitch

        commandedAngles = np.concatenate((commandedRoll, commandedPitch, currentAngles[2]), axis=None)
        commandedPosition = np.concatenate((currentPosition[0], currentPosition[1], commandedZ), axis=None)

        #****************************************************************************************************

        self.commandedPose = np.concatenate((commandedAngles, commandedPosition), axis=None)

        self.pastTime = now
        toleranceAngles = 0.02
        tolerancePositions = 0.005

        # FIXME(ros2-port): ".any" is a bound-method reference, not a call --
        # always truthy, so this anti-windup gate never actually gates. Ported
        # unchanged per the "preserve control-law bugs, fix later" decision --
        # see code review 2026-08-04.
        if (errorAngles > toleranceAngles).any  and self.jointSuccess:
            self.errorAnglesIntegral += errorAngles

        if (errorPositions > tolerancePositions).any and self.jointSuccess:
            self.errorPositionsIntegral += errorPositions

        self.pastErrorAngles = errorAngles
        self.pastErrorPositions = errorPositions

    def inverse_kinematics_client(self, set_point):
        if not self.ikClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().error("Service call failed: /inverse_kinematics not available")
            return None

        request = IkSrv.Request()
        request.pose_set_point = [float(v) for v in set_point]
        future = self.ikClient.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        if not future.done() or future.exception() is not None:
            self.get_logger().error("Service call failed: %s" % future.exception())
            return None
        return future.result()

    def jointControl(self, baseSetPoint):
        jointSetPoint = CustomCmnd()

        baseSetPoint = np.asarray(baseSetPoint)
        if baseSetPoint.size == 0:
            cmdAngles = IkSrv.Response()
            cmdAngles.success = False
            cmdAngles.joint_cmd_angles = [-0.2454, -0.2454, -0.2454, -0.2454]

        else:
            cmdAngles = self.inverse_kinematics_client(baseSetPoint)

        # FIXME(ros2-port): cmdAngles is None whenever the IK service call
        # fails/times out, and that's never checked here -- unhandled
        # AttributeError. Ported unchanged per the "preserve control-law bugs,
        # fix later" decision -- see code review 2026-08-04.
        jointSetPoint.position = cmdAngles.joint_cmd_angles
        self.jointSuccess = cmdAngles.success

        return jointSetPoint

    def run(self):
        jointSetPoint = self.jointControl(self.commandedPose)
        self.pubJointCommand.publish(jointSetPoint)


def main(args=None):
    rclpy.init(args=args)
    node = HandleControl()
    print('*****************************************************************')
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
