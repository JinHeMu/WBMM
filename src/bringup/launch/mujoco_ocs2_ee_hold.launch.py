#!/usr/bin/env python3
"""Run MuJoCo with OCS2 end-effector holding and external base teleop.

The intended command path for this test is:

    teleop_twist_keyboard -> /cmd_vel -> mujoco_bridge (base)
    OCS2 MRT              -> /arm_controller/commands -> mujoco_bridge (arm)

The OCS2 base command is deliberately remapped to an unused topic.  This
prevents the OCS2 whole-body policy from racing with the keyboard command on
the MuJoCo bridge while still allowing OCS2 to observe the resulting base
motion through wheel odometry and compensate with the arm.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    GroupAction,
    IncludeLaunchDescription,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import SetRemap


def _include(path, arguments):
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(path),
        launch_arguments=arguments.items(),
    )


def generate_launch_description():
    bringup_share = get_package_share_directory("tracer_jaka_bringup")
    mujoco_launch = os.path.join(
        bringup_share, "launch", "mujoco_hardware_interface.launch.py")
    ocs2_launch = os.path.join(
        bringup_share, "launch", "ocs2.launch.py")

    description_share = get_package_share_directory("tracer_jaka_description")
    urdf_file = os.path.join(
        description_share, "urdf", "tracer_jaka_zu5.urdf")

    # This is a simulation-only launch.  command_output_enabled is exposed so
    # that a dry run can still disable all MRT command publishers explicitly.
    ocs2_group = GroupAction(actions=[
        # sim/ocs2.yaml names the OCS2 base output
        # /base_controller/cmd_vel.  Keep it away from mujoco_bridge; the
        # keyboard owns /cmd_vel in this experiment.
        SetRemap(
            src="/base_controller/cmd_vel",
            dst="/ocs2/disabled_base_cmd",
        ),
        _include(ocs2_launch, {
            "base_config_file": os.path.join(
                bringup_share, "config", "common", "ocs2.yaml"),
            "config_file": os.path.join(
                bringup_share, "config", "sim", "ocs2.yaml"),
            "task_file": os.path.join(
                bringup_share, "config", "sim", "task.info"),
            "urdf_file": urdf_file,
            "lib_folder": LaunchConfiguration("lib_folder"),
            "use_sim_time": "true",
            "use_target": LaunchConfiguration("use_target"),
            "use_rviz": LaunchConfiguration("use_rviz"),
            "initial_task_phase": LaunchConfiguration("initial_task_phase"),
            "command_output_enabled": LaunchConfiguration(
                "command_output_enabled"),
            # mujoco_bridge publishes wheel odometry directly; no EKF is
            # started by this minimal test launch.
            "odom_topic": "/wheel/odometry",
        }),
    ])

    mujoco = _include(mujoco_launch, {
        "scene": LaunchConfiguration("scene"),
        "viewer": LaunchConfiguration("viewer"),
        "initial_pose": LaunchConfiguration("initial_pose"),
        "start_robot_state_publisher": "true",
        # OCS2 MRT and wbmm_target_node need odom -> base_footprint -> tool0.
        "publish_odom_tf": "true",
        "start_imu": "false",
        "start_lidar": "false",
        "start_camera": "false",
        "start_fts": "false",
        "wheel_odom_topic": "/wheel/odometry",
    })

    return LaunchDescription([
        DeclareLaunchArgument(
            "scene",
            default_value="empty",
            description="MuJoCo scene name."),
        DeclareLaunchArgument(
            "viewer",
            default_value="true",
            description="Show the MuJoCo viewer."),
        DeclareLaunchArgument(
            "initial_pose",
            default_value="low",
            description="MuJoCo initial keyframe: low, home or task_contact."),
        DeclareLaunchArgument(
            "use_target",
            default_value="true",
            description=(
                "Start wbmm_target_node so a target can be sent from RViz.")),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start the OCS2 RViz configuration."),
        DeclareLaunchArgument(
            "initial_task_phase",
            default_value="2",
            description=(
                "Initial TaskPhase; 2 is Execution in the simulation task.")),
        DeclareLaunchArgument(
            "command_output_enabled",
            default_value="true",
            description=(
                "Enable OCS2 MRT output. Only arm output reaches MuJoCo in "
                "this launch; the base output is remapped to a sink.")),
        DeclareLaunchArgument(
            "lib_folder",
            default_value="/tmp/wbmm_ocs2_mujoco_ee_hold",
            description="Directory for generated OCS2 MPC/MRT files."),
        mujoco,
        ocs2_group,
    ])
