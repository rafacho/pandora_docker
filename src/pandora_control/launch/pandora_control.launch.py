#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS2 port note (2026-08-04): replaces the ROS1 pandora_control/launch/
# pandora_control.launch. controller_manager itself is now started by the
# gz_ros2_control Gazebo plugin (see pandora_gazebo/urdf/pandora.gazebo),
# already loaded with pandora_controllers.yaml -- so there's no separate
# "load the YAML" step here (ROS1's <rosparam file=... command="load"/>
# equivalent), only spawner actions to activate each controller against the
# already-running controller_manager, plus the ik_server node.
#
# Lyrical Luth note (2026-08-07): under Lyrical, gz_ros2_control's
# controller_manager only forwards "--param use_sim_time:=true" to each
# spawned controller node -- unlike the loaded controller_manager node
# itself, the per-controller top-level YAML sections (leg_controller's
# "dof"/"joint1"/"actuators", wheel_controller's "wheel_separation" etc.)
# never reach the individual controller nodes, so leg_controller fails
# to find "dof" and wheel_controller's wheel_separation fails its
# built-in floating-point-range validation against its unset default.
# Fixed by passing pandora_controllers.yaml explicitly via each spawner's
# "-p/--param-file" flag, which loads it directly into the controller node
# before configure -- this is the standard ros2_control mechanism for this
# exact situation, not a workaround specific to this project.

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    controllers_yaml = PathJoinSubstitution(
        [FindPackageShare('pandora_control'), 'config', 'pandora_controllers.yaml'])

    # '-c controller_manager' (relative) is explicit on purpose: this file is
    # always included from bringup.launch.py inside a PushRosNamespace group,
    # and pandora.gazebo's gz_ros2_control plugin puts controller_manager in
    # that same namespace (see its <ros><namespace> fix note) -- a relative
    # name here resolves to it correctly under any namespace, so the
    # spawners don't need to know what that namespace actually is.
    joint_state_broadcaster_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['joint_state_broadcaster', '-c', 'controller_manager', '-p', controllers_yaml],
        output='screen',
    )

    leg_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['leg_controller', '-c', 'controller_manager', '-p', controllers_yaml],
        output='screen',
    )

    wheel_controller_spawner = Node(
        package='controller_manager',
        executable='spawner',
        arguments=['wheel_controller', '-c', 'controller_manager', '-p', controllers_yaml],
        output='screen',
    )

    ik_server = Node(
        package='pandora_control',
        executable='ik_server',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # Static-stability pipeline: real_support_polygon needs wheel_N->odom TF
    # (from joint_state_broadcaster + robot_state_publisher) and com_publisher
    # needs /robot_description (from robot_state_publisher); both are already
    # up by the time bringup.launch.py includes this file.
    real_support_polygon = Node(
        package='pandora_control',
        executable='real_support_polygon',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    com_publisher = Node(
        package='pandora_control',
        executable='com_publisher',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    static_stability = Node(
        package='pandora_control',
        executable='static_stability',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    # stability_set_point = Node(
    #     package='pandora_control',
    #     executable='stability_set_point',
    #     output='screen',
    #     parameters=[{'use_sim_time': True}],
    # )
   
    # h_control = Node(
    #     package='pandora_control', 
    #     executable='h_control', 
    #     output='screen'
    # )

    return LaunchDescription([
        joint_state_broadcaster_spawner,
        leg_controller_spawner,
        wheel_controller_spawner,
        ik_server,
        real_support_polygon,
        com_publisher,
        static_stability,
        # stability_set_point,
        # h_control,
    ])
