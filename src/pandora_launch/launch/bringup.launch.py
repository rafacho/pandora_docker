#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS2 port note (2026-08-04): replaces the ROS1 pandora_launch/bringup.launch
# (roslaunch XML). Brings up Gazebo Sim, publishes robot_description from
# pandora.xacro, spawns the robot (ros_gz_sim create instead of
# gazebo_ros/spawn_model), bridges the sensor/pose topics declared in
# pandora_gazebo's plugins to ROS2, and starts world_broadcaster (which turns
# the bridged pose into the "odom"->"dummy" TF). Controller loading itself
# lives in pandora_control's launch file, included at the bottom, mirroring
# the ROS1 package split.
#
# DEBUG (2026-08-06): world_file now points at pandora_gazebo's
# test_world.world (PGS solver + ODE collision detector configured, per the
# DetachableJoint/four-bar-loop investigation) instead of pandora.world or
# gz-sim's bundled empty.sdf.
#
# Config note (2026-08-10): pandora_controllers.yaml is (re)generated from
# pandora_control/generate_controllers_yaml.py right below, before gz_sim is
# included -- gz_ros2_control reads that file's resolved path straight from
# pandora.gazebo's <parameters> tag as soon as gzserver starts, so it has to
# exist with up-to-date content before that point. See that module for why
# (ROS2's parameter YAML parser doesn't support anchors/aliases, so the
# per-joint gain/SEA-constant duplication can't be avoided in the YAML
# itself).

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

from pandora_control.generate_controllers_yaml import generate as generate_controllers_yaml


def generate_launch_description():
    pandora_description_share = FindPackageShare('pandora_description')
    pandora_gazebo_share = FindPackageShare('pandora_gazebo')
    pandora_control_share = FindPackageShare('pandora_control')

    generate_controllers_yaml(
        os.path.join(
            get_package_share_directory('pandora_control'), 'config',
            'pandora_controllers.yaml'))

    # DEBUG: see the note at the top of this file.
    world_file = PathJoinSubstitution([pandora_gazebo_share, 'worlds', 'test_world.world'])
    xacro_file = PathJoinSubstitution([pandora_description_share, 'urdf', 'pandora.xacro'])
    bridge_config = PathJoinSubstitution([pandora_gazebo_share, 'config', 'ros_gz_bridge.yaml'])

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    gui = LaunchConfiguration('gui')
    paused = LaunchConfiguration('paused')

    declare_args = [
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        # "ramp" starting height, matches the ROS1 default.
        DeclareLaunchArgument('z', default_value='0.4'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
    ]

    # "model://heightmap" (pandora.world) resolves via this env var (Gazebo
    # Sim's replacement for Gazebo Classic's GAZEBO_MODEL_PATH). It also has to
    # include pandora_description's share *parent* dir, or the GUI can't
    # resolve the "package://pandora_description/meshes/..." mesh URIs used by
    # leg.xacro's <visual> elements (gz-sim has no ROS ament_index-aware
    # package:// resolver of its own, it falls back to searching each
    # GZ_SIM_RESOURCE_PATH entry for a "pandora_description/..." subpath,
    # exactly like it does for "model://" URIs). Still needed with
    # test_world.world, since the robot model itself uses those mesh URIs.
    
    set_resource_path = SetEnvironmentVariable(
        'GZ_SIM_RESOURCE_PATH',
        [
            PathJoinSubstitution([pandora_gazebo_share, 'worlds']),
            ':',
            PathJoinSubstitution([pandora_description_share, '..']),
        ])

    # -s (server-only/headless) is appended when gui:=false, matching the
    # ROS1 launch's "gui" arg (which toggled Gazebo Classic's GUI client).
    # -r (run) is appended unless paused:=true -- gz-sim starts paused by
    # default when -r is omitted.
    gz_args = [
        world_file,
        PythonExpression(["'' if '", paused, "' == 'true' else ' -r'"]),
        PythonExpression(["'' if '", gui, "' == 'true' else ' -s'"]),
    ]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # ParameterValue(..., value_type=str) is required here -- without it,
    # launch_ros tries to parse the URDF/XML text as YAML and fails.
    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'publish_frequency': 25.0,
            'use_sim_time': True,
        }],
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=[
            # DEBUG: test_world.world declares <world name="paragrip_robot_mdam">
            # -- revert to 'pandora_world' when world_file is switched back to
            # pandora.world.
            '-world', 'paragrip_robot_mdam',
            '-topic', 'robot_description',
            '-name', 'pandora',
            '-x', x, '-y', y, '-z', z, '-Y', yaw,
        ],
    )

    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
    )

    world_broadcaster = Node(
        package='pandora_control',
        executable='world_broadcaster',
        output='screen',
        parameters=[{'use_sim_time': True}],
    )

    pandora_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pandora_control_share, 'launch', 'pandora_control.launch.py'])),
    )

    return LaunchDescription(declare_args + [
        set_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        world_broadcaster,
        pandora_control_launch,
    ])
