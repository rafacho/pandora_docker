#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import rclpy
from rclpy.node import Node
import numpy as np

from std_msgs.msg import Float64MultiArray
from custom_messages.msg import CustomCmnd #type: ignore
from pandora_msgs.srv import IkSrv #type: ignore
from pandora_msgs.msg import HrefCommand #type: ignore

# Piso para dt no termo derivativo do PID. Guardar so contra dt<=0 nao basta:
# chamadas de handlePoseCommand em sequencia rapida (sem pausa real) medem
# um dt positivo mas minusculo (us) no relogio de parede, que ainda explode
# Pkd*(erro-erroAnterior)/dt. Ver validation_tests/handle_pose_command_convergence.py,
# caso "stress: sem pausa entre chamadas" -- reproduz a divergencia com o
# guard antigo (dt<=0) e converge com este piso.
MIN_DT = 1.0 / 25.0


class HandleControl(Node):
    def __init__(self):
        super().__init__('pose_control')

        self.pubJointCommand = self.create_publisher(CustomCmnd, 'leg_controller/command', 1)
        self.get_logger().info("publisher leg_controller/command is ready")

        self.create_subscription(
            HrefCommand, 'pose_controller/command', self.handlePoseCommand, 1)

        self.create_subscription(Float64MultiArray, 'imuPose', self.handleImu, 1)

        self.ikClient = self.create_client(IkSrv, 'inverse_kinematics')

        self.commandedPose = np.asarray([])
        self.currentState = []
        self.jointSuccess = False
        # Cache for the last IK result -- see inverse_kinematics_client()'s
        # note on why this is filled asynchronously instead of blocking.
        self.lastJointAngles = [-0.2454, -0.2454, -0.2454, -0.2454]
        self.ikRequestPending = False

        self.pastTime = self.get_clock().now()
        self.errorAnglesIntegral = np.asarray([0, 0, 0], dtype='float64')
        self.pastErrorAngles = np.asarray([0, 0, 0], dtype='float64')

        self.errorPositionsIntegral = np.asarray([0, 0, 0], dtype='float64')
        self.pastErrorPositions = np.asarray([0, 0, 0], dtype='float64')

        self.create_timer(1.0 / 25.0, self.run)

        self.get_logger().info("h_control init complete")

    def handleImu(self, imuMsg):
        self.currentState = np.asarray(imuMsg.data)

    def handlePoseCommand(self, spValue):
        now = self.get_clock().now()
        dt = (now - self.pastTime).nanoseconds / 1e9
        if dt < MIN_DT:
            dt = MIN_DT

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

        if (errorAngles > toleranceAngles).any() and self.jointSuccess:
            self.errorAnglesIntegral += errorAngles

        if (errorPositions > tolerancePositions).any() and self.jointSuccess:
            self.errorPositionsIntegral += errorPositions

        self.pastErrorAngles = errorAngles
        self.pastErrorPositions = errorPositions

    def inverse_kinematics_client(self, set_point):
        
        if self.ikRequestPending or not self.ikClient.service_is_ready():
            return

        self.ikRequestPending = True
        request = IkSrv.Request()
        request.pose_set_point = [float(v) for v in set_point]
        future = self.ikClient.call_async(request)
        future.add_done_callback(self.handleIkResponse)

    def handleIkResponse(self, future):
        self.ikRequestPending = False
        if future.exception() is not None:
            self.get_logger().error("Service call failed: %s" % future.exception())
            return
        response = future.result()
        self.lastJointAngles = response.joint_cmd_angles
        self.jointSuccess = response.success

    def jointControl(self, baseSetPoint):
        jointSetPoint = CustomCmnd()

        baseSetPoint = np.asarray(baseSetPoint)
        if baseSetPoint.size != 0:
            # Kicks off an async request when none is in flight; the result
            # (if any) lands in self.lastJointAngles via handleIkResponse()
            # some future cycle, not this one -- see inverse_kinematics_client().
            self.inverse_kinematics_client(baseSetPoint)

        jointSetPoint.position = self.lastJointAngles

        # ActuatorPositionController::setCommand() (custom_controller) replaces
        # its whole command_struct_ with whatever arrives on this topic, then
        # indexes velocity/effort/online_gain1/online_gain2 with operator[]
        # (unchecked) in update() -- leaving these at CustomCmnd's default
        # empty array is an out-of-bounds read on the controller side, not a
        # no-op. Must be sized to match position on every publish.
        num_joints = len(jointSetPoint.position)
        jointSetPoint.velocity = [0.0] * num_joints
        jointSetPoint.effort = [0.0] * num_joints
        jointSetPoint.online_gain1 = [0.0] * num_joints
        jointSetPoint.online_gain2 = [0.0] * num_joints

        return jointSetPoint

    def run(self):
        jointSetPoint = self.jointControl(self.commandedPose)
        self.pubJointCommand.publish(jointSetPoint)


def main(args=None):
    rclpy.init(args=args)
    node = HandleControl()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
