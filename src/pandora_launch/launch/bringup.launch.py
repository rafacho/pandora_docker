#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
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

from pandora_control.generate_controllers_yaml import generate as generate_controllers_yaml #type: ignore

# world launch arg -> (world file relative to pandora_gazebo/worlds,
# SDF <world name=...>)
WORLDS = {
    'flat': (['flat', 'flat.world'], 'flat'),
    'sinusoidal': (['sinusoidal', 'sinusoidal.world'], 'sinusoidal'),
}


def launch_setup(context, pandora_description_share, pandora_gazebo_share, pandora_control_share):
    world_arg = LaunchConfiguration('world').perform(context)
    world_path_parts, gz_world_name = WORLDS[world_arg]

    world_file = PathJoinSubstitution([pandora_gazebo_share, 'worlds', *world_path_parts])
    xacro_file = PathJoinSubstitution([pandora_description_share, 'urdf', 'pandora.xacro'])

    # ros_gz_bridge.yaml is a template (see the file's header note): the
    # contact sensors' gz_topic_name contains a __WORLD__ placeholder because
    # gz-sim's Contact system always publishes on the world-scoped default
    # topic regardless of the <topic> declared in the SDF, and that topic
    # embeds whichever world was actually loaded. Substitute it here, once
    # gz_world_name is known, into a generated file -- never edit the
    # template in place, that would make the substitution one-shot and leave
    # a stale world name behind on the next run with a different world.
    bridge_template_path = os.path.join(
        get_package_share_directory('pandora_gazebo'), 'config', 'ros_gz_bridge.yaml')
    bridge_config = os.path.join(
        get_package_share_directory('pandora_gazebo'), 'config', 'ros_gz_bridge.generated.yaml')
    with open(bridge_template_path) as f:
        bridge_yaml_text = f.read()
    with open(bridge_config, 'w') as f:
        f.write(bridge_yaml_text.replace('__WORLD__', gz_world_name))

    x = LaunchConfiguration('x')
    y = LaunchConfiguration('y')
    z = LaunchConfiguration('z')
    yaw = LaunchConfiguration('yaw')
    gui = LaunchConfiguration('gui')
    paused = LaunchConfiguration('paused')

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
        # --headless-rendering: EGL-based offscreen rendering for the
        # Sensors system (imu/contact/... sensors), instead of GLX/X11.
        # Without it, in this container the Sensors render thread hangs
        # forever at "Waiting for init" (confirmed with -v 4) regardless of
        # GUI vs headless, LIBGL_ALWAYS_SOFTWARE, or a local Xvfb with GLX --
        # so every sensor topic gets advertised but never actually publishes.
        # See https://gazebosim.org/api/sim/9/headless_rendering.html.
        ' --headless-rendering',
        # TEMP DEBUG (2026-08-20): max verbosity while tracking down why the
        # Sensors system plugin isn't producing any /imu or /pandora/contacts_N
        # data -- remove once resolved.
        ' -v 4',
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
            '-world', gz_world_name,
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

    return [
        set_resource_path,
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        world_broadcaster,
        pandora_control_launch,
    ]


def generate_launch_description():
    pandora_description_share = FindPackageShare('pandora_description')
    pandora_gazebo_share = FindPackageShare('pandora_gazebo')
    pandora_control_share = FindPackageShare('pandora_control')

    generate_controllers_yaml(
        os.path.join(
            get_package_share_directory('pandora_control'), 'config',
            'pandora_controllers.yaml'))

    declare_args = [
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='0.4'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument(
            'world', default_value='flat',
            choices=list(WORLDS.keys()),
            description="'flat' (flat ground, default) or 'sinusoidal' (sinusoidal terrain)"),
    ]

    return LaunchDescription(declare_args + [
        OpaqueFunction(
            function=launch_setup,
            args=[pandora_description_share, 
                  pandora_gazebo_share, 
                  pandora_control_share]),
    ])
