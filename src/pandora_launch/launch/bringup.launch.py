#!/usr/bin/env python3
# -*- coding: utf-8 -*-

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    Command,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace
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
    namespace = LaunchConfiguration('namespace').perform(context)

    # Fix (2026-08-25): must run after 'namespace' is resolved above, not in
    # generate_launch_description() -- pandora_controllers.yaml's top-level
    # keys need the actual namespace value baked in (see that module's fix
    # note: bare/wildcard keys don't work once controller_manager and its
    # controllers run under /pandora instead of the global namespace).
    generate_controllers_yaml(
        os.path.join(
            get_package_share_directory('pandora_control'), 'config',
            'pandora_controllers.yaml'),
        namespace)

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
        # ' --headless-rendering',
        # TEMP DEBUG (2026-08-20): max verbosity while tracking down why the
        # Sensors system plugin isn't producing any /imu or /pandora/contacts_N
        # data -- remove once resolved.
        # ' -v 4',
    ]

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([FindPackageShare('ros_gz_sim'), 'launch', 'gz_sim.launch.py'])),
        launch_arguments={'gz_args': gz_args}.items(),
    )

    # ParameterValue(..., value_type=str) is required here -- without it,
    # launch_ros tries to parse the URDF/XML text as YAML and fails.
    # namespace:=<namespace> feeds pandora.gazebo's xacro:arg of the same
    # name, which sets the ros2_control plugin's controller_manager
    # namespace -- see the fix note there. Keep this in sync with the
    # PushRosNamespace below; both need to agree for the ROS graph created by
    # ros2_control (controller_manager, its controllers) and the ROS graph
    # created by our own launch actions to land in the same namespace.
    robot_description = ParameterValue(
        Command(['xacro ', xacro_file, ' namespace:=', namespace]), value_type=str)

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

    # Fix (2026-08-25): PushRosNamespace's effect doesn't reliably reach
    # actions added later via an event handler's on_exit callback, even when
    # the RegisterEventHandler itself sits inside the namespaced GroupAction
    # below -- on_exit actions are visited when the event actually fires
    # (after spawn_robot exits), not when the group is first built, and by
    # then the outer group's pushed namespace context is gone. Confirmed
    # empirically: com_publisher, ik_server, real_support_polygon,
    # static_stability, stability_set_point, h_control and the 3 controller
    # spawners all came up in the global namespace instead of /pandora.
    # Re-pushing the namespace in its own GroupAction right at the point of
    # inclusion, rather than relying on inheriting it from the outer group,
    # fixes this regardless of when on_exit actually fires.
    pandora_control_launch = GroupAction([
        PushRosNamespace(namespace),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                PathJoinSubstitution([pandora_control_share, 'launch', 'pandora_control.launch.py'])),
        ),
    ])

    # spawn_robot ('ros_gz_sim create') is a one-shot process that exits once
    # the robot is spawned -- gate the controller spawners/ik_server/etc.
    # behind its exit instead of launching them in parallel, since they all
    # need the robot (and its controller_manager, started by gz_ros2_control
    # as part of spawning) to already exist in the running simulation.
    controllers_after_spawn = RegisterEventHandler(
        event_handler=OnProcessExit(
            target_action=spawn_robot,
            on_exit=[pandora_control_launch],
        )
    )

    # PushRosNamespace namespaces every Node/IncludeLaunchDescription created
    # inside this group (relative topic/service/action names only -- '/clock'
    # and other absolute names, plus tf2's own /tf and /tf_static, are
    # unaffected by design). set_resource_path is a plain env var, not a ROS
    # node, so it's left outside the group; gz_sim just launches the Gazebo
    # process and has no ROS graph of its own, but including it costs
    # nothing.
    namespaced_group = GroupAction([
        PushRosNamespace(namespace),
        gz_sim,
        robot_state_publisher,
        spawn_robot,
        ros_gz_bridge,
        world_broadcaster,
        controllers_after_spawn,
    ])

    return [
        set_resource_path,
        namespaced_group,
    ]


def generate_launch_description():
    pandora_description_share = FindPackageShare('pandora_description')
    pandora_gazebo_share = FindPackageShare('pandora_gazebo')
    pandora_control_share = FindPackageShare('pandora_control')

    declare_args = [
        DeclareLaunchArgument('x', default_value='0'),
        DeclareLaunchArgument('y', default_value='0'),
        DeclareLaunchArgument('z', default_value='0.4'),
        DeclareLaunchArgument('yaw', default_value='1.5708'),
        # DeclareLaunchArgument('yaw', default_value='0'),
        DeclareLaunchArgument('gui', default_value='true'),
        DeclareLaunchArgument('paused', default_value='false'),
        DeclareLaunchArgument(
            'world', default_value='flat',
            choices=list(WORLDS.keys()),
            description="'flat' (flat ground, default) or 'sinusoidal' (sinusoidal terrain)"),
        DeclareLaunchArgument(
            'namespace', default_value='pandora',
            description='ROS namespace for every sensor/actuator/controller topic and '
                         'service in this bringup (robot_state_publisher, ros_gz_bridge, '
                         'controller_manager and its controllers, and every pandora_control '
                         'node). Change this if running more than one Pandora instance.'),
    ]

    return LaunchDescription(declare_args + [
        OpaqueFunction(
            function=launch_setup,
            args=[pandora_description_share, 
                  pandora_gazebo_share, 
                  pandora_control_share]),
    ])
