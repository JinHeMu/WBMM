#!/usr/bin/env python3
"""MuJoCo + REMANI navigation + WipePlanner + OCS2 MPC."""

import os

from ament_index_python.packages import (
    get_package_prefix,
    get_package_share_directory,
)
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _prepare_contact_task(context, *args, **kwargs):
    source = os.path.join(
        get_package_share_directory('tracer_jaka_ocs2'), 'config', 'task.info')
    target = context.perform_substitution(LaunchConfiguration('contact_task_file'))
    os.makedirs(os.path.dirname(target), exist_ok=True)
    with open(source, encoding='utf-8') as stream:
        contents = stream.read()
    old = 'environmentCollision\n{\n  activate            true'
    new = 'environmentCollision\n{\n  activate            false'
    if old not in contents:
        raise RuntimeError('Could not locate environmentCollision in task.info')
    contents = contents.replace(old, new, 1)
    # REMANI and WipePlanner both validate the complete arm path with the same
    # conservative sphere model.  Keeping OCS2's independent FCL self-collision
    # barrier enabled here creates contradictory gradients at the handoff (the
    # measured state can be valid for the planners but already inside OCS2's
    # activation distance), so MPC moves a joint away from the validated path
    # and the adaptive progress gate correctly stops.  The generic OCS2 task is
    # left untouched; only this generated wipe-pipeline task delegates collision
    # ownership to the two planners.
    old_self_collision = 'selfCollision\n{\n  activate true'
    new_self_collision = 'selfCollision\n{\n  activate false'
    if old_self_collision not in contents:
        raise RuntimeError('Could not locate selfCollision in task.info')
    contents = contents.replace(old_self_collision, new_self_collision, 1)
    # The generic navigation task uses a very cheap arm input (R=0.01), which
    # makes the short-horizon policy overly aggressive during the long Cartesian
    # approach. The wipe task uses R=0.02, stronger state tracking, and an
    # explicit 0.55 rad/s arm limit.
    old_arm_cost = '    arm\n    {\n      scaling 1e-2'
    new_arm_cost = '    arm\n    {\n      scaling 2e-2'
    if old_arm_cost not in contents:
        raise RuntimeError('Could not locate arm input cost in task.info')
    contents = contents.replace(old_arm_cost, new_arm_cost, 1)
    for joint in range(1, 7):
        old_weight = f'({joint - 1},{joint - 1})  2.0   ; joint_{joint}'
        new_weight = f'({joint - 1},{joint - 1})  50.0  ; joint_{joint}'
        if old_weight not in contents:
            raise RuntimeError(f'Could not locate tracking weight for joint_{joint}')
        contents = contents.replace(old_weight, new_weight, 1)
    old_velocity = 'jointVelocityLimits\n{\n  mu'
    new_velocity = 'jointVelocityLimits\n{\n  activate  true\n  mu'
    if old_velocity not in contents:
        raise RuntimeError('Could not locate jointVelocityLimits in task.info')
    contents = contents.replace(old_velocity, new_velocity, 1)
    contents = contents.replace('(0,0)  -2.0\n      (1,0)  -2.0\n      (2,0)  -2.0\n      (3,0)  -2.0\n      (4,0)  -2.0\n      (5,0)  -2.0',
                                '(0,0)  -0.55\n      (1,0)  -0.55\n      (2,0)  -0.55\n      (3,0)  -0.55\n      (4,0)  -0.55\n      (5,0)  -0.55', 1)
    contents = contents.replace('(0,0)  2.0\n      (1,0)  2.0\n      (2,0)  2.0\n      (3,0)  2.0\n      (4,0)  2.0\n      (5,0)  2.0',
                                '(0,0)  0.55\n      (1,0)  0.55\n      (2,0)  0.55\n      (3,0)  0.55\n      (4,0)  0.55\n      (5,0)  0.55', 1)
    with open(target, 'w', encoding='utf-8') as stream:
        stream.write(contents)
    return [LogInfo(msg=f'[wipe_planner] prepared contact MPC task: {target}')]


def generate_launch_description():
    planner_share = get_package_share_directory('wipe_planner')
    # Resolve REMANI from the same colcon install root as wipe_planner. Using
    # get_package_share_directory('remani_planner') directly is unsafe when a
    # shell still contains an older REMANI overlay earlier in AMENT_PREFIX_PATH.
    install_root = os.path.dirname(get_package_prefix('wipe_planner'))
    remani_launch = os.path.join(
        install_root,
        'remani_planner',
        'share',
        'remani_planner',
        'launch',
        'remani_mpc_tracking.launch.py')
    if not os.path.isfile(remani_launch):
        raise RuntimeError(
            'The REMANI launch file co-installed with wipe_planner is missing: '
            f'{remani_launch}. Build remani_planner in this workspace first.')
    # The included REMANI launch contains Node(package='remani_planner', ...)
    # and also loads its parameter helper through the ament index. Put the
    # matching package prefix first so both lookups select the same build as
    # the launch file, even if the terminal has sourced an older overlay.
    remani_prefix = os.path.join(install_root, 'remani_planner')
    ament_prefixes = os.environ.get('AMENT_PREFIX_PATH', '').split(os.pathsep)
    os.environ['AMENT_PREFIX_PATH'] = os.pathsep.join(
        [remani_prefix] + [
            prefix for prefix in ament_prefixes
            if prefix and os.path.normpath(prefix) != os.path.normpath(remani_prefix)
        ])
    mujoco_share = get_package_share_directory('tracer_jaka_mujoco')
    ocs2_share = get_package_share_directory('tracer_jaka_ocs2')
    base_launch = os.path.join(ocs2_share, 'launch', 'ocs2_sim.launch.py')

    return LaunchDescription([
        DeclareLaunchArgument('viewer', default_value='true'),
        DeclareLaunchArgument('use_rviz', default_value='true'),
        DeclareLaunchArgument('start_slam', default_value='false'),
        DeclareLaunchArgument('auto_goal', default_value='true'),
        DeclareLaunchArgument(
            'contact_task_file',
            default_value='/tmp/ocs2_tracer_jaka/wipe_task.info'),
        OpaqueFunction(function=_prepare_contact_task),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                'viewer': LaunchConfiguration('viewer'),
                'use_rviz': LaunchConfiguration('use_rviz'),
                'start_slam': LaunchConfiguration('start_slam'),
                'start_remani': 'true',
                'remani_launch_file': remani_launch,
                # Exactly one MPC reference owner: WipePlanner relays REMANI
                # during navigation, then latches it off before contact planning.
                'start_remani_bridge': 'false',
                # The wall-normal pre-contact frame (base + all six joints) is
                # sent to REMANI. WipePlanner owns final alignment and approach.
                'remani_freeze_manipulator': 'false',
                # These are REMANI's *planning* (trajectory-feasibility) limits
                # toward the whole-body goal, NOT the actuation limits: the real
                # These are REMANI's collision-aware navigation feasibility
                # limits.  The global reference allocator now reserves margin
                # and scales against both joint velocity and acceleration, so
                # the optimized path passes the post-check without repeated
                # replanning.  Slow physical alignment is deliberately owned by
                # WipePlanner after handoff (0.05 rad/s), not by tightening the
                # coupled MINCO navigation problem until it becomes ill-conditioned.
                'remani_manipulator_max_vel': '0.55',
                'remani_manipulator_max_acc': '1.00',
                'remani_tracking_error_replan_enabled': 'false',
                'use_csv_target': 'false',
                # The arm controller accepts absolute joint positions.  A
                # 100-ms policy look-ahead gives it a useful position step while
                # the WipePlanner reference itself remains limited to 0.05 rad/s.
                'mrt_traj_horizon': '0.10',
                'task_file': LaunchConfiguration('contact_task_file'),
            }.items()),
        TimerAction(period=12.0, actions=[Node(
            package='wipe_planner',
            executable='wipe_planner_node',
            name='wipe_planner',
            output='screen',
            parameters=[
                os.path.join(planner_share, 'config', 'wipe_planner.yaml'),
                {
                    'use_sim_time': True,
                    'urdf_file': os.path.join(
                        mujoco_share, 'urdf', 'tracer_jaka_zu5.urdf'),
                    'task_file': os.path.join(
                        planner_share, 'config', 'wipe_task.yaml'),
                    'auto_navigation_goal': ParameterValue(
                        LaunchConfiguration('auto_goal'), value_type=bool),
                },
            ],
        )]),
    ])
