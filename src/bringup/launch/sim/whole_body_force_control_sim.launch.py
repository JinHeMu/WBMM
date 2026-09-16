#!/usr/bin/env python3
"""Focused MuJoCo + OCS2 + whole-body force-control simulation."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


_PROFILES = {
    "infinite": {
        "mujoco_model": "scene_force_follow_infinite.xml",
        "force_overrides": {},
    },
    "20s": {
        "mujoco_model": "scene_force_follow_5m.xml",
        # The finite-travel regression shares the simulation configuration.
        # Override only the differences from the default velocity-follow test.
        "force_overrides": {
            "force_velocity_mode": False,
            "force_deadband": 0.0,
            "max_offset": 5.20,
            "max_base_delta": 5.10,
        },
    },
}


def _launch_nodes(context):
    """Resolve one named experiment profile and start its peer nodes."""
    profile_name = LaunchConfiguration("profile").perform(context)
    profile = _PROFILES[profile_name]

    description_share = get_package_share_directory(
        "tracer_jaka_description")
    force_share = get_package_share_directory("whole_body_force_control")
    mujoco_share = get_package_share_directory("tracer_jaka_mujoco")
    ocs2_share = get_package_share_directory("wbmm_ocs2_ros")
    bringup_share = get_package_share_directory("tracer_jaka_bringup")

    urdf_file = os.path.join(
        description_share, "urdf", "tracer_jaka_zu5.urdf")
    with open(urdf_file, "r", encoding="utf-8") as urdf_stream:
        robot_description = urdf_stream.read()

    task_file = os.path.join(ocs2_share, "config", "task_sim.info")
    ocs2_config = os.path.join(
        ocs2_share, "config", "force_control_sim.yaml")
    generated_library_root = "/tmp/wbmm_force_control/auto_generated"

    mujoco_config = os.path.join(
        mujoco_share, "config", "force_control_sim.yaml")
    mujoco_model = os.path.join(
        mujoco_share, "models", profile["mujoco_model"])
    force_config = os.path.join(
        force_share, "config", "force_follow_sim.yaml")
    rviz_config = os.path.join(
        bringup_share, "rviz", "whole_body_force_control_sim.rviz")

    viewer = ParameterValue(
        LaunchConfiguration("viewer"), value_type=bool)

    return [
        Node(
            package="tracer_jaka_mujoco",
            executable="mujoco_bridge",
            name="mujoco_bridge",
            output="screen",
            parameters=[
                os.path.join(mujoco_share, "config", "sensors.yaml"),
                mujoco_config,
                {
                    "model_path": mujoco_model,
                    "use_viewer": viewer,
                    "use_sim_time": False,
                },
            ],
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[
                {
                    "robot_description": ParameterValue(
                        robot_description, value_type=str),
                    "use_sim_time": True,
                }
            ],
        ),
        Node(
            package="wbmm_ocs2_ros",
            executable="wbmm_mpc_node",
            name="wbmm_mpc_node",
            output="screen",
            parameters=[
                ocs2_config,
                {
                    "taskFile": task_file,
                    "urdfFile": urdf_file,
                    "libFolder": os.path.join(
                        generated_library_root, "mpc"),
                },
            ],
        ),
        Node(
            package="wbmm_ocs2_ros",
            executable="wbmm_mrt_node",
            name="wbmm_mrt_node",
            output="screen",
            parameters=[
                ocs2_config,
                {
                    "taskFile": task_file,
                    "urdfFile": urdf_file,
                    "libFolder": os.path.join(
                        generated_library_root, "mrt"),
                },
            ],
        ),
        Node(
            package="whole_body_force_control",
            executable="whole_body_force_control_node",
            name="whole_body_force_control",
            output="screen",
            parameters=[
                force_config,
                profile["force_overrides"],
                {
                    "urdf_file": urdf_file,
                    "ee_frame": "tool0",
                    "robot_name": "mobile_manipulator",
                    "use_sim_time": True,
                },
            ],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="screen",
            arguments=["-d", rviz_config],
            parameters=[{"use_sim_time": True}],
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "viewer",
            default_value="true",
            choices=["true", "false"],
            description="Open the MuJoCo native viewer.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            choices=["true", "false"],
            description="Open the force-control RViz view.",
        ),
        DeclareLaunchArgument(
            "profile",
            default_value="infinite",
            choices=list(_PROFILES),
            description="Named simulation experiment: infinite or 20s.",
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
