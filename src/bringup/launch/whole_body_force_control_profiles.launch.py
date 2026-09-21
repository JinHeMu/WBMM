#!/usr/bin/env python3
"""Whole-body force-control simulation profiles (algorithm nodes only).

Hardware/MuJoCo must be provided by ``mujoco_hardware_interface.launch.py``
or a real backend.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


_PROFILES = {
    "sensor_z": {
        "mujoco_model": "scene_force_follow_infinite.xml",
        "init_keyframe": "low",
        "force_overrides": {},
    },
    # Example 1: three-axis translational admittance with elastic return.
    "three_axis_admittance": {
        "mujoco_model": "scene_force_follow_infinite.xml",
        "init_keyframe": "low",
        "force_overrides": {
            "admittance.selected_axes":
                [True, True, True, False, False, False],
            "admittance.mass":
                [3.0, 3.0, 3.0, 0.3, 0.3, 0.3],
            "admittance.damping":
                [45.0, 45.0, 45.0, 4.5, 4.5, 4.5],
            "admittance.stiffness":
                [150.0, 150.0, 150.0, 0.0, 0.0, 0.0],
            "admittance.max_velocity":
                [0.035, 0.035, 0.035, 0.15, 0.15, 0.15],
            "whole_body.base_share": 0.40,
        },
    },
    # Example 2: three-axis translational force following, K = 0.
    "three_axis_follow": {
        "mujoco_model": "scene_force_follow_infinite.xml",
        "init_keyframe": "low",
        "force_overrides": {
            "admittance.selected_axes":
                [True, True, True, False, False, False],
            "admittance.mass":
                [3.0, 3.0, 3.0, 0.3, 0.3, 0.3],
            "admittance.damping":
                [50.0, 50.0, 50.0, 4.5, 4.5, 4.5],
            "admittance.stiffness":
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "admittance.max_velocity":
                [0.25, 0.25, 0.10, 0.15, 0.15, 0.15],
            "whole_body.base_share": 0.80,
        },
    },
    # K = 0 force-following limit: F = M*a + D*v.  D is tuned so the steady
    # velocity of a 7 N push stays near max_velocity without an elastic stop.
    "infinite": {
        "mujoco_model": "scene_force_follow_infinite.xml",
        "force_overrides": {
            "admittance.selected_axes":
                [True, False, False, False, False, False],
            "admittance.mass":
                [1.0, 3.0, 3.0, 0.3, 0.3, 0.3],
            "admittance.damping":
                [28.0, 45.0, 45.0, 4.5, 4.5, 4.5],
            "admittance.stiffness":
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "admittance.max_velocity":
                [0.25, 0.035, 0.035, 0.15, 0.15, 0.15],
            "whole_body.base_share": 0.98,
        },
    },
    # Six-axis sequence: the axis_wrench_sequence.py script cycles through
    # +X/+Y/+Z forces and +Tx/+Ty/+Tz torques, with zero pauses in between.
    "six_axis_sequence": {
        "mujoco_model": "scene_force_follow_infinite.xml",
        "init_keyframe": "low",
        "force_overrides": {
            "admittance.selected_axes":
                [True, True, True, True, True, True],
            "admittance.mass":
                [3.0, 3.0, 3.0, 0.3, 0.3, 0.3],
            "admittance.damping":
                [45.0, 45.0, 45.0, 4.5, 4.5, 4.5],
            "admittance.stiffness":
                [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "admittance.max_velocity":
                [0.10, 0.10, 0.10, 0.15, 0.15, 0.15],
            "whole_body.base_share": 0.80,
        },
    },
    "20s": {
        "mujoco_model": "scene_force_follow_5m.xml",
        # Finite-travel regression: K > 0 creates the elastic endpoint.
        "force_overrides": {
            "admittance.selected_axes":
                [True, False, False, False, False, False],
            "admittance.mass":
                [1.0, 3.0, 3.0, 0.3, 0.3, 0.3],
            "admittance.damping":
                [2.0, 45.0, 45.0, 4.5, 4.5, 4.5],
            "admittance.stiffness":
                [1.0, 0.0, 0.0, 0.0, 0.0, 0.0],
            "admittance.max_velocity":
                [0.25, 0.035, 0.035, 0.15, 0.15, 0.15],
            "whole_body.base_share": 0.98,
        },
    },
}


def _launch_nodes(context):
    """Resolve one named experiment profile and start its peer nodes."""
    profile_name = LaunchConfiguration("profile").perform(context)
    profile = _PROFILES[profile_name]

    description_share = get_package_share_directory(
        "tracer_jaka_description")
    bringup_share = get_package_share_directory("tracer_jaka_bringup")

    urdf_file = os.path.join(
        description_share, "urdf", "tracer_jaka_zu5.urdf")
    task_file = os.path.join(bringup_share, "config", "sim", "task.info")
    ocs2_base = os.path.join(bringup_share, "config", "common", "ocs2.yaml")
    ocs2_config = os.path.join(bringup_share, "config", "sim", "ocs2.yaml")
    force_base = os.path.join(
        bringup_share, "config", "common", "force_control.yaml")
    force_config = os.path.join(
        bringup_share, "config", "sim", "force_control.yaml")
    generated_library_root = "/tmp/wbmm_force_control/auto_generated"
    rviz_config = os.path.join(
        bringup_share, "rviz", "whole_body_force_control_sim.rviz")

    return [
        Node(
            package="wbmm_ocs2_ros",
            executable="wbmm_mpc_node",
            name="wbmm_mpc_node",
            output="screen",
            parameters=[
                ocs2_base,
                ocs2_config,
                {
                    "taskFile": task_file,
                    "urdfFile": urdf_file,
                    "libFolder": os.path.join(
                        generated_library_root, "mpc"),
                    "initial_task_phase": 2,
                },
            ],
        ),
        Node(
            package="wbmm_ocs2_ros",
            executable="wbmm_mrt_node",
            name="wbmm_mrt_node",
            output="screen",
            parameters=[
                ocs2_base,
                ocs2_config,
                {
                    "taskFile": task_file,
                    "urdfFile": urdf_file,
                    "libFolder": os.path.join(
                        generated_library_root, "mrt"),
                    "odom_topic": "/wheel/odometry",
                    "initial_task_phase": 2,
                },
            ],
        ),
        Node(
            package="whole_body_force_control",
            executable="whole_body_force_control_node",
            name="whole_body_force_control",
            output="screen",
            parameters=[
                force_base,
                force_config,
                profile["force_overrides"],
                {
                    "urdf_file": urdf_file,
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
            "use_rviz",
            default_value="true",
            choices=["true", "false"],
            description="Open the force-control RViz view.",
        ),
        DeclareLaunchArgument(
            "profile",
            default_value="sensor_z",
            choices=list(_PROFILES),
            description="Sensor Z compliance, or legacy infinite/20s following.",
        ),
        OpaqueFunction(function=_launch_nodes),
    ])
