#!/usr/bin/env python3
"""WBMM whole-body navigation + OCS2 end-effector tracking demo.

The WBMM-native successor to ``remani_tracking.launch.py``. Same composition,
no REMANI involvement:

    MuJoCo empty scene (odom aliased to map by an identity map->odom TF)
      -> map1 static ESDF, shared by the planner and OCS2
      -> wbmm_planner_node: 2D goal -> wbmm_planning_msgs/WholeBodyTrajectory
      -> wbmm_reference_bridge: that trajectory -> ocs2 MpcTargetTrajectories
      -> wbmm_visualization: planner/MPC whole-robot ghosts in the same RViz
      -> WbmmTargetNode interactive 3D marker for the EE pose
      -> phase bridge: new goal => Navigation, odom arrival stays Navigation,
         fresh EE target => Execution directly

Typical use, from the workspace root:

    source /opt/ros/humble/setup.bash
    source install/setup.bash
    ros2 launch tracer_jaka_bringup wbmm_tracking.launch.py

Then click "2D Goal Pose" in RViz. For an unattended check, set auto_goal:

    ros2 launch tracer_jaka_bringup wbmm_tracking.launch.py \\
      auto_goal:=1.0,0.0 viewer:=false use_rviz:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

DEFAULT_ESDF = "/home/a/WBMM/maps/map1/site_remani.npz"


def _value(context, name):
    return context.perform_substitution(LaunchConfiguration(name)).strip()


def _make_actions(context):
    bringup = get_package_share_directory("tracer_jaka_bringup")

    task_file = _value(context, "task_file")
    if not task_file:
        task_file = os.path.join(
            bringup, "config", "sim", "task_esdf_tracking.info")

    esdf_file = _value(context, "esdf_file") or DEFAULT_ESDF

    for label, path in (
        ("task_esdf_tracking file", task_file),
        ("planner and OCS2 ESDF file", esdf_file),
    ):
        if not os.path.isfile(path):
            raise RuntimeError(f"{label} does not exist: {path!r}")

    world_frame = _value(context, "world_frame")
    rviz_config = _value(context, "rviz_config")
    if rviz_config and not os.path.isfile(rviz_config):
        raise RuntimeError(f"RViz config does not exist: {rviz_config!r}")

    # The planner adopts the ESDF's own frame when world_frame is empty; the
    # bridge must be told the same frame or it will reject every trajectory.
    # Both are therefore pinned to world_frame here rather than left to their
    # independent defaults.
    wbmm_args = {
        "hardware_backend": "mujoco",
        "config_profile": "sim",
        "scene": _value(context, "scene"),
        "viewer": _value(context, "viewer"),
        "publish_odom_tf": "true",
        "use_rviz": _value(context, "use_rviz"),
        "hardware_write": _value(context, "hardware_write"),
        "start_localization": "false",
        "start_ocs2": "true",
        # WBMM-native planner; the REMANI upstream is no longer used.
        "start_planning": "true",
        "start_force_control": "false",
        "start_moveit": "false",
        "use_target": "true",
        "task_file": task_file,
        "urdf_file": _value(context, "urdf_file"),
        "static_esdf_file": esdf_file,
        "esdf_file": esdf_file,
        "world_frame": world_frame,
        "remani_planner_frame": world_frame,
        "goal_topic": _value(context, "goal_topic"),
        "planner_cruise_speed": _value(context, "planner_cruise_speed"),
        "planner_max_linear_velocity": _value(
            context, "planner_max_linear_velocity"),
        "planner_max_yaw_rate": _value(context, "planner_max_yaw_rate"),
        "planner_max_joint_velocity": _value(
            context, "planner_max_joint_velocity"),
        "planner_max_base_speed": _value(context, "planner_max_base_speed"),
        "planner_max_base_yaw_rate": _value(
            context, "planner_max_base_yaw_rate"),
        "planner_collision_safety_margin": _value(
            context, "planner_collision_safety_margin"),
        "planner_treat_unknown_as_occupied": _value(
            context, "planner_treat_unknown_as_occupied"),
        "odom_topic": _value(context, "odom_topic"),
        "joint_state_topic": _value(context, "joint_state_topic"),
        "lib_folder": _value(context, "lib_folder"),
    }
    if rviz_config:
        wbmm_args["rviz_config"] = rviz_config

    actions = [
        # Display only. Reuse the tracking frame/model/clock and the existing
        # RViz process; this node never publishes control commands.
        Node(
            package="wbmm_visualization",
            executable="trajectory_visualizer",
            name="wbmm_visualization",
            output="screen",
            parameters=[
                _value(context, "trajectory_visualization_config"),
                {
                    "urdf_file": _value(context, "urdf_file"),
                    "use_sim_time": True,
                    "mpc_frame": world_frame,
                },
            ],
            condition=IfCondition(LaunchConfiguration("use_trajectory_visualization")),
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(PathJoinSubstitution([
                FindPackageShare("tracer_jaka_bringup"),
                "launch", "wbmm.launch.py",
            ])),
            launch_arguments=wbmm_args.items(),
        ),
        Node(
            package="grid_map",
            executable="esdf_rviz_publisher",
            name="esdf_rviz_publisher",
            output="screen",
            parameters=[{
                "esdf_file": esdf_file,
                "frame_id": world_frame,
                "z_slice": 0.5,
                "max_distance": 0.8,
                "stride": 2,
                "publish_period": 0.5,
            }],
        ),
        Node(
            package="tracer_jaka_bringup",
            executable="remani_phase_bridge.py",
            name="remani_phase_bridge",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "phase_service": "/mobile_manipulator_set_task_phase",
                "goal_topic": _value(context, "goal_topic"),
                # WBMM publishes measured arrival after the trajectory finishes.
                "finish_topic": "/planning/finish",
                "enable_odom_fallback": False,
                "odom_topic": _value(context, "odom_topic"),
                "goal_position_tolerance": 0.15,
                "goal_yaw_tolerance": 0.30,
                "goal_hold_time": 1.0,
            }],
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="map_to_odom_tf",
            output="screen",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            condition=IfCondition(
                LaunchConfiguration("publish_map_odom_tf")),
        ),
    ]

    # Unattended check: publish the 2D goal so the planner can be exercised
    # without clicking in RViz.
    #
    # The goal is repeated because the planner rejects one that arrives before
    # odom and joint states are fresh (state_timeout, default 0.5 s) or while the
    # robot is still settling. A single shot races with startup; repeating it
    # until one is accepted is what makes this usable as a smoke test. The
    # planner replans on each accepted goal, which is harmless here.
    auto_goal = _value(context, "auto_goal")
    if auto_goal:
        try:
            goal_x, goal_y = (float(part) for part in auto_goal.split(","))
        except ValueError as error:
            raise RuntimeError(
                f"auto_goal must be 'x,y', got {auto_goal!r}") from error

        goal_message = (
            "{header: {frame_id: " + world_frame + "},"
            " pose: {position: {x: " + repr(goal_x) +
            ", y: " + repr(goal_y) + "},"
            " orientation: {z: 0.0, w: 1.0}}}")

        first_delay = float(_value(context, "auto_goal_delay"))
        period = float(_value(context, "auto_goal_retry_period"))
        attempts = int(_value(context, "auto_goal_attempts"))
        for attempt in range(max(1, attempts)):
            actions.append(TimerAction(
                period=first_delay + attempt * period,
                actions=[ExecuteProcess(
                    cmd=[
                        "ros2", "topic", "pub", "--once",
                        _value(context, "goal_topic"),
                        "geometry_msgs/msg/PoseStamped",
                        goal_message,
                    ],
                    output="screen",
                )],
            ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "scene", default_value="esdf_validation",
            description=(
                "MuJoCo scene. Default is robot+floor only; the ESDF supplies "
                "the virtual obstacles.")),
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "use_trajectory_visualization", default_value="true",
            description="Publish planner/MPC whole-robot ghosts, even with use_rviz=false."),
        DeclareLaunchArgument(
            "trajectory_visualization_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("wbmm_visualization"), "config", "visualization.yaml"]),
            description="Whole-robot trajectory display colors, sampling and topic settings."),
        DeclareLaunchArgument(
            "urdf_file",
            default_value=PathJoinSubstitution([
                FindPackageShare("tracer_jaka_description"), "urdf", "tracer_jaka_zu5.urdf"]),
            description="Shared URDF for the planner, OCS2 and trajectory display."),
        DeclareLaunchArgument(
            "hardware_write", default_value="true",
            description=(
                "MuJoCo command output. Keep true for full navigation; set "
                "false for a dry-run.")),
        DeclareLaunchArgument(
            "world_frame", default_value="map",
            description=(
                "Planning frame. Must match the ESDF NPZ frame_id; it is also "
                "pinned as the bridge frame so the two cannot disagree.")),
        DeclareLaunchArgument(
            "publish_map_odom_tf", default_value="true",
            description=(
                "Publish an identity map->odom TF for this odom-aliased "
                "MuJoCo demo.")),
        DeclareLaunchArgument(
            "task_file", default_value="",
            description=(
                "Defaults to installed config/sim/task_esdf_tracking.info.")),
        DeclareLaunchArgument(
            "esdf_file", default_value=DEFAULT_ESDF,
            description="Static ESDF shared by the planner and OCS2."),
        DeclareLaunchArgument(
            "rviz_config",
            default_value=PathJoinSubstitution([
                FindPackageShare("tracer_jaka_bringup"),
                "rviz", "wbmm_tracking_map1.rviz"]),
            description="Map1 layout with ESDF, goal/EE tools and planner/MPC robot ghosts."),
        DeclareLaunchArgument("odom_topic", default_value="/wheel/odometry"),
        DeclareLaunchArgument(
            "joint_state_topic", default_value="/joint_states"),
        DeclareLaunchArgument("goal_topic", default_value="/goal_pose"),
        DeclareLaunchArgument(
            "lib_folder", default_value="/tmp/wbmm_ocs2_esdf_tracking"),

        DeclareLaunchArgument("planner_cruise_speed", default_value="0.35"),
        DeclareLaunchArgument(
            "planner_max_linear_velocity", default_value="0.5",
            description=(
                "Controller envelope the planner must stay inside. Must match "
                "jointVelocityLimits in the task file.")),
        DeclareLaunchArgument("planner_max_yaw_rate", default_value="1.0"),
        DeclareLaunchArgument("planner_max_joint_velocity", default_value="2.0"),
        DeclareLaunchArgument("planner_max_base_speed", default_value="0.5"),
        DeclareLaunchArgument("planner_max_base_yaw_rate", default_value="1.0"),
        DeclareLaunchArgument(
            "planner_collision_safety_margin", default_value="0.0"),
        DeclareLaunchArgument(
            "planner_treat_unknown_as_occupied", default_value="false",
            description=(
                "Reject ESDF queries touching unobserved space. map1 records "
                "unknown_is_occupied=false, so the default trusts it.")),

        DeclareLaunchArgument(
            "auto_goal", default_value="",
            description=(
                "Optional 'x,y' goal published for unattended checks. Empty "
                "disables it.")),
        DeclareLaunchArgument(
            "auto_goal_delay", default_value="15.0",
            description="Seconds before the first goal attempt."),
        DeclareLaunchArgument(
            "auto_goal_retry_period", default_value="3.0",
            description="Seconds between goal attempts."),
        DeclareLaunchArgument(
            "auto_goal_attempts", default_value="6",
            description=(
                "Goal attempts. The planner drops a goal that arrives before "
                "odom and joint states are fresh, so one shot is unreliable.")),

        OpaqueFunction(function=_make_actions),
    ])
