#!/usr/bin/env python3
"""Compose MuJoCo state/action source with the shared MoveIt algorithms.

The MuJoCo bridge owns ``/joint_states`` and the canonical
``/arm_trajectory_controller/follow_joint_trajectory`` action. This launch adds
the MoveIt move_group / RViz / optional Servo nodes on top of that simulation
contract.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare("tracer_jaka_bringup")

    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_servo", default_value="false"),
        DeclareLaunchArgument("use_joy", default_value="false"),
        DeclareLaunchArgument(
            "model",
            default_value=os.path.join(
                get_package_share_directory("tracer_jaka_mujoco"),
                "models", "scene.xml")),
        DeclareLaunchArgument("init_keyframe", default_value="home"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(
                get_package_share_directory("tracer_jaka_bringup"),
                "launch", "sim.launch.py")),
            launch_arguments={
                "viewer": LaunchConfiguration("viewer"),
                "camera": "false",
                "model": LaunchConfiguration("model"),
                "init_keyframe": LaunchConfiguration("init_keyframe"),
            }.items(),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup_share, "launch", "moveit.launch.py",
            ])),
            launch_arguments={
                "use_sim_time": "true",
                "use_rviz": LaunchConfiguration("use_rviz"),
                "use_servo": LaunchConfiguration("use_servo"),
                "use_joy": LaunchConfiguration("use_joy"),
                # Simulation has no physical JAKA; mark motion allowed so the
                # shared MoveIt launch enables its command owner.
                "hardware_write": "true",
            }.items(),
        ),
    ])
