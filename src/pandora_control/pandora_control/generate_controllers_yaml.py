#!/usr/bin/env python3
# -*- coding: utf-8 -*-

# ROS2 config note (2026-08-10): ROS2's own parameter YAML parser
# (rcl_yaml_param_parser) does not support YAML anchors/aliases/merge keys
# ("Will not support aliasing", confirmed empirically) -- so
# pandora_controllers.yaml cannot declare a_Kp/a_Kd/j_Kp/j_Kd/Ki or the SEA
# constants (Ks/D/Bm/phim/nu/Torque_limit) once and reuse them across the 4
# joints inside the YAML file itself. This module is the single source of
# truth for those shared values instead: bringup.launch.py calls generate()
# before starting gz_sim, which (re)writes pandora_controllers.yaml from the
# dicts below. The per-joint blocks in the generated file are a flattened
# expansion, not something meant to be hand-edited -- change values here.

import yaml

# Shared PID gains for leg_controller's 4 actuated joints. See
# pandora_controllers.yaml's original gain-bump note (2026-08-06): a_Kp
# 500->5000, a_Kd 10->20 was needed for the SEA spring (Ks=50000) to actually
# move the joint toward a 0.3 rad setpoint instead of stalling a few
# thousandths of a radian in. j_Kp/j_Kd are read by the controller but never
# used in its control law (see actuator_position_controller.h) -- kept only
# for config compatibility with the original ROS1 file.
DEFAULT_GAINS = {
    'a_Kp': 500.0,
    'a_Kd': 20.0, 
    'j_Kp': 0.0,
    'j_Kd': 0.0,
    'Ki': 0.15, 
}

# Shared SEA (series elastic actuator) physical constants, identical for all
# 4 legs on this robot.
DEFAULT_SEA = {
    'type': 'SEA',
    'Ks': 3500.0,  # Nm/rad
    'D': 2.0,  # Nms/rad
    'Bm': 0.0000252,
    'phim': 0.0000512,
    'nu': 160,
    'Torque_limit': 304.0,  # Nm
}

JOINT_NAMES = [
    'joint_frame_crank_1',
    'joint_frame_crank_2',
    'joint_frame_crank_3',
    'joint_frame_crank_4',
]

# Per-joint overrides, keyed by joint name -- merged on top of DEFAULT_GAINS/
# DEFAULT_SEA for that joint only. Leave empty to keep every joint identical;
# e.g. {'joint_frame_crank_1': {'a_Kd': 0.0}} would give only joint 1 a
# different a_Kd while keeping everything else shared.
GAIN_OVERRIDES = {}
SEA_OVERRIDES = {}


def _joint_gains(name):
    gains = dict(DEFAULT_GAINS)
    gains.update(GAIN_OVERRIDES.get(name, {}))
    return gains


def _joint_sea(name):
    sea = dict(DEFAULT_SEA)
    sea.update(SEA_OVERRIDES.get(name, {}))
    return sea


def build_config(namespace):
    # Fix (2026-08-25): bare keys like 'controller_manager' only match a node
    # of that name in the GLOBAL namespace. Since bringup.launch.py runs
    # controller_manager (and every controller it hosts) under /<namespace>
    # (see pandora.gazebo's <ros><namespace> fix note), the spawners failed
    # with "Missing namespace : /pandora or wildcard in parameter file" and
    # controller_manager itself never learned the controllers' 'type' --
    # confirmed empirically. The documented fix (controller_manager's own
    # userdoc) is to key each node's block with its full namespaced path,
    # '/<namespace>/<node_name>', instead of the bare name or a blanket '/**'
    # wildcard (which can't disambiguate 4 different nodes' distinct
    # ros__parameters sharing one file the way a per-node wildcard could).
    ns = f'/{namespace}'
    leg_controller_params = {
        'dof': len(JOINT_NAMES),
        'controller_update': 1,
    }
    for i, name in enumerate(JOINT_NAMES, start=1):
        leg_controller_params[f'joint{i}'] = {'name': name, **_joint_gains(name)} # type: ignore
    leg_controller_params['actuators'] = { # type: ignore
        name: _joint_sea(name) for name in JOINT_NAMES
    }

    return {
        f'{ns}/controller_manager': {
            'ros__parameters': {
                'update_rate': 4000,
                'joint_state_broadcaster': {
                    'type': 'joint_state_broadcaster/JointStateBroadcaster',
                },
                'leg_controller': {
                    'type': 'custom_controller/ActuatorPositionController',
                },
                'wheel_controller': {
                    'type': 'diff_drive_controller/DiffDriveController',
                },
            },
        },
        f'{ns}/joint_state_broadcaster': {
            'ros__parameters': {
                'use_local_topics': False,
            },
        },
        f'{ns}/leg_controller': {
            'ros__parameters': leg_controller_params,
        },
        f'{ns}/wheel_controller': {
            'ros__parameters': {
                'left_wheel_names': ['joint_fork_wheel_2', 'joint_fork_wheel_3'],
                'right_wheel_names': ['joint_fork_wheel_1', 'joint_fork_wheel_4'],
                'publish_rate': 25.0,
                'pose_covariance_diagonal':
                    [0.001, 0.001, 1000000.0, 1000000.0, 1000000.0, 0.03],
                'twist_covariance_diagonal':
                    [0.001, 0.001, 0.001, 1000000.0, 1000000.0, 0.03],
                'cmd_vel_timeout': 0.25,
                'enable_odom_tf': False,
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_link',
                'wheel_separation': 0.2934375,
                'wheel_radius': 0.0625,
                'wheel_separation_multiplier': 1.0,
                'left_wheel_radius_multiplier': 1.0,
                'right_wheel_radius_multiplier': 1.0,
                'linear.x.has_velocity_limits': True,
                'linear.x.max_velocity': 2.0,
                'linear.x.has_acceleration_limits': True,
                'linear.x.max_acceleration': 20.0,
                'angular.z.has_velocity_limits': True,
                'angular.z.max_velocity': 4.0,
                'angular.z.has_acceleration_limits': True,
                'angular.z.max_acceleration': 25.0,
            },
        },
    }


HEADER = (
    '# AUTO-GENERATED by pandora_control/generate_controllers_yaml.py -- do not\n'
    '# edit this file by hand, it is overwritten on every bringup.launch.py run.\n'
    '# Edit the DEFAULT_GAINS/DEFAULT_SEA/GAIN_OVERRIDES/SEA_OVERRIDES dicts in\n'
    '# that module instead.\n'
)


def generate(output_path, namespace='pandora'):
    with open(output_path, 'w') as f:
        f.write(HEADER)
        yaml.dump(build_config(namespace), f, default_flow_style=False, sort_keys=False)


if __name__ == '__main__':
    import sys
    generate(sys.argv[1], sys.argv[2] if len(sys.argv) > 2 else 'pandora')
