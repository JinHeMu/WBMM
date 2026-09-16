#!/usr/bin/env python3
"""Single-entry REMANI + OCS2 + MuJoCo simulation.

The functional composition is delegated to ``ocs2_sim.launch.py``. This file
only selects the odom-only REMANI profile and optionally publishes one demo
goal.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _compose(context):
    bringup_share = FindPackageShare("tracer_jaka_bringup")
    publish_goal = _as_bool(
        context.perform_substitution(LaunchConfiguration("publish_demo_goal")))

    actions = [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                bringup_share, "launch", "ocs2_sim.launch.py",
            ])),
            launch_arguments={
                # REMANI plans in odom; no SLAM/AMCL map frame is needed.
                "start_slam": "false",
                "viewer": LaunchConfiguration("viewer"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "start_ocs2": "true",
                "start_remani": "true",
                "start_remani_bridge": "true",
                "planner_frame": "odom",
                "target_frame": "odom",
                "use_tf_transform": "false",
            }.items(),
        )
    ]

    if publish_goal:
        actions.append(TimerAction(
            period=LaunchConfiguration("goal_delay"),
            actions=[Node(
                package="tracer_jaka_mujoco",
                executable="demo_goal_publisher",
                name="remani_mpc_sim_demo_goal",
                output="screen",
                parameters=[{
                    "goal_x": ParameterValue(
                        LaunchConfiguration("goal_x"), value_type=float),
                    "goal_y": ParameterValue(
                        LaunchConfiguration("goal_y"), value_type=float),
                    "goal_yaw": ParameterValue(
                        LaunchConfiguration("goal_yaw"), value_type=float),
                    "frame_id": "odom",
                    "use_sim_time": True,
                }],
            )],
        ))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("publish_demo_goal", default_value="true"),
        DeclareLaunchArgument("goal_x", default_value="0.0"),
        DeclareLaunchArgument("goal_y", default_value="1.2"),
        DeclareLaunchArgument("goal_yaw", default_value="0.0"),
        DeclareLaunchArgument("goal_delay", default_value="30.0"),
        OpaqueFunction(function=_compose),
    ])
