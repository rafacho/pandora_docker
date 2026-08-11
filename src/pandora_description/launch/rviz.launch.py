#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS2 port note (2026-08-04): replaces the ROS1 rviz.launch. Like the
# original (which only ever had the rviz node active -- robot_state_publisher
# and joint_state_publisher were commented out there too), this only launches
# rviz2; /robot_description is expected to already be published by
# pandora_launch/bringup.launch.py running in the same session.

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    rviz_config = PathJoinSubstitution(
        [FindPackageShare('pandora_description'), 'launch', 'config.rviz'])

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz',
        arguments=['-d', rviz_config],
    )

    return LaunchDescription([rviz])
