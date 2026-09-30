#!/usr/bin/env python3
"""Start the WBMM whole-body planner and its OCS2 reference bridge.

This is the WBMM-native replacement for ``remani.launch.py``. It starts only
algorithms and consumes the hardware interface contract:

    /goal_pose + odom + /joint_states
            -> wbmm_planner_ros/WholeBodyTrajectory      (wbmm_planner_ros)
            -> ocs2_msgs/MpcTargetTrajectories             (wbmm_trajectory_to_mpc)

Neither node owns hardware, TF or the map: the deployment launch supplies the
state sources and the ESDF path, exactly like the other algorithm launches.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.conditions import IfCondition


def _as_bool(value):
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _value(context, name):
    return context.perform_substitution(LaunchConfiguration(name)).strip()


def _make_nodes(context):
    bringup = get_package_share_directory("tracer_jaka_bringup")

    urdf_file = _value(context, "urdf_file")
    esdf_file = _value(context, "esdf_file")
    world_frame = _value(context, "world_frame")

    for label, path in (
        ("planner urdf_file", urdf_file),
        ("planner esdf_file", esdf_file),
    ):
        if not path:
            raise RuntimeError(f"{label} must be provided by the deployment launch")
        if not os.path.isfile(path):
            raise RuntimeError(f"{label} does not exist: {path!r}")

    use_sim_time = _as_bool(_value(context, "use_sim_time"))
    robot_name = _value(context, "robot_name")
    joint_names = _value(context, "joint_names").split()

    planner_parameters = {
        "use_sim_time": use_sim_time,
        "urdf_file": urdf_file,
        "esdf_file": esdf_file,
        "world_frame": world_frame,
        "base_collision_link": _value(context, "base_collision_link"),
        "joint_names": joint_names,
        "goal_topic": _value(context, "goal_topic"),
        "odom_topic": _value(context, "odom_topic"),
        "joint_state_topic": _value(context, "joint_state_topic"),
        "trajectory_topic": _value(context, "trajectory_topic"),
        "sample_rrt_max_time": float(_value(context, "sample_rrt_max_time")),
        "whole_body_rrt_max_time": float(_value(context, "whole_body_rrt_max_time")),
        "rrt_max_nodes": int(_value(context, "rrt_max_nodes")),
        "rrt_random_seed": int(_value(context, "rrt_random_seed")),
        "enable_whole_body_rrt": _as_bool(_value(context, "enable_whole_body_rrt")),
        "enable_primitive_fallback": _as_bool(_value(context, "enable_primitive_fallback")),
        "enable_optimization": _as_bool(_value(context, "enable_optimization")),
        "cruise_speed": float(_value(context, "cruise_speed")),
        "max_linear_velocity": float(_value(context, "max_linear_velocity")),
        "max_yaw_rate": float(_value(context, "max_yaw_rate")),
        "max_joint_velocity": float(_value(context, "max_joint_velocity")),
        "max_base_speed": float(_value(context, "max_base_speed")),
        "max_base_yaw_rate": float(_value(context, "max_base_yaw_rate")),
        "collision_safety_margin": float(
            _value(context, "collision_safety_margin")),
        "treat_unknown_as_occupied": _as_bool(
            _value(context, "treat_unknown_as_occupied")),
        "minco_waypoint_stride": int(_value(context, "minco_waypoint_stride")),
        "tangent_chord_length": float(_value(context, "tangent_chord_length")),
        "max_trajectory_duration": float(
            _value(context, "max_trajectory_duration")),
    }

    bridge_parameters = {
        "use_sim_time": use_sim_time,
        "robot_name": robot_name,
        "world_frame": _value(context, "bridge_world_frame"),
        "joint_names": joint_names,
        "trajectory_topic": _value(context, "trajectory_topic"),
        "target_topic": _value(context, "target_topic"),
        "observation_topic": _value(context, "observation_topic"),
        "publish_rate": float(_value(context, "bridge_publish_rate")),
        "reference_horizon": float(_value(context, "bridge_reference_horizon")),
        "sample_dt": float(_value(context, "bridge_sample_dt")),
    }

    actions = [
        Node(
            package="wbmm_planner_ros",
            executable="wbmm_planner_ros_node",
            name="wbmm_planner_node",
            output="screen",
            parameters=[planner_parameters],
        ),
        Node(
            condition=IfCondition(LaunchConfiguration("start_bridge")),
            package="wbmm_trajectory_to_mpc",
            executable="wbmm_trajectory_to_mpc_node",
            name="wbmm_trajectory_to_mpc",
            output="screen",
            parameters=[bridge_parameters],
        ),
    ]
    if _as_bool(_value(context, "wait_for_localization")):
        from wbmm_localization_launch.launch_support import gate_until_ready
        return gate_until_ready(actions, use_sim_time,
            _value(context, "localization_backend"),
            float(_value(context, "localization_timeout")),
            _value(context, "localization_status_topic"))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        DeclareLaunchArgument("urdf_file", default_value=""),
        DeclareLaunchArgument("esdf_file", default_value=""),
        DeclareLaunchArgument(
            "world_frame", default_value="",
            description=(
                "Planning frame. Empty adopts the ESDF's own frame_id, which "
                "is the only value that can be correct.")),
        DeclareLaunchArgument("bridge_world_frame", default_value="odom",
            description="Must match the OCS2 controller world_frame; TF transforms the planning frame."),
        DeclareLaunchArgument("start_bridge", default_value="true"),
        DeclareLaunchArgument("wait_for_localization", default_value="false"),
        DeclareLaunchArgument("localization_backend", default_value=""),
        DeclareLaunchArgument("localization_timeout", default_value="30.0"),
        DeclareLaunchArgument("localization_status_topic", default_value="/localization/status"),
        DeclareLaunchArgument("base_collision_link", default_value="base_link"),
        DeclareLaunchArgument(
            "joint_names",
            default_value="joint_1 joint_2 joint_3 joint_4 joint_5 joint_6"),
        DeclareLaunchArgument("goal_topic", default_value="/goal_pose"),
        DeclareLaunchArgument("odom_topic", default_value="/wheel/odometry"),
        DeclareLaunchArgument("joint_state_topic", default_value="/joint_states"),
        DeclareLaunchArgument(
            "trajectory_topic", default_value="/wbmm/whole_body_trajectory"),
        DeclareLaunchArgument("robot_name", default_value="mobile_manipulator"),
        DeclareLaunchArgument("target_topic", default_value=""),
        DeclareLaunchArgument("observation_topic", default_value=""),

        DeclareLaunchArgument("sample_rrt_max_time", default_value="2.0"),
        DeclareLaunchArgument("whole_body_rrt_max_time", default_value="3.0"),
        DeclareLaunchArgument("rrt_max_nodes", default_value="12000"),
        DeclareLaunchArgument("rrt_random_seed", default_value="1"),
        DeclareLaunchArgument("enable_whole_body_rrt", default_value="true"),
        DeclareLaunchArgument("enable_primitive_fallback", default_value="true"),
        DeclareLaunchArgument("enable_optimization", default_value="true"),
        DeclareLaunchArgument("cruise_speed", default_value="0.35"),
        DeclareLaunchArgument("max_linear_velocity", default_value="0.5"),
        DeclareLaunchArgument("max_yaw_rate", default_value="1.0"),
        DeclareLaunchArgument("max_joint_velocity", default_value="2.0"),
        DeclareLaunchArgument("max_base_speed", default_value="0.5"),
        DeclareLaunchArgument("max_base_yaw_rate", default_value="1.0"),
        DeclareLaunchArgument("max_trajectory_duration", default_value="60.0"),
        DeclareLaunchArgument("minco_waypoint_stride", default_value="3"),
        DeclareLaunchArgument("tangent_chord_length", default_value="0.15"),
        DeclareLaunchArgument("collision_safety_margin", default_value="0.0"),
        DeclareLaunchArgument(
            "treat_unknown_as_occupied", default_value="false",
            description=(
                "Reject any query touching unobserved space. The deployed map1 "
                "records unknown_is_occupied=false, so the default trusts it.")),

        DeclareLaunchArgument("bridge_publish_rate", default_value="20.0"),
        DeclareLaunchArgument("bridge_reference_horizon", default_value="3.0"),
        DeclareLaunchArgument("bridge_sample_dt", default_value="0.04"),

        OpaqueFunction(function=_make_nodes),
    ])
