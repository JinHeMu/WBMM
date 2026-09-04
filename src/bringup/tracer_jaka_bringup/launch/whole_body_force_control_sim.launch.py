#!/usr/bin/env python3
"""Independent MuJoCo + OCS2 + whole-body force-follow bringup."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup = FindPackageShare("tracer_jaka_bringup")
    description = FindPackageShare("tracer_jaka_description")
    force_control = FindPackageShare("whole_body_force_control")
    mujoco = FindPackageShare("tracer_jaka_mujoco")

    viewer = LaunchConfiguration("viewer")
    use_rviz = LaunchConfiguration("use_rviz")
    force_params_file = LaunchConfiguration("force_params_file")
    fake_wrench_topic = LaunchConfiguration("fake_wrench_topic")
    mujoco_model = LaunchConfiguration("mujoco_model")
    armed = LaunchConfiguration("force_control_armed")
    reference_output = LaunchConfiguration("force_reference_output_enabled")
    urdf = PathJoinSubstitution(
        [description, "urdf", "tracer_jaka_zu5.urdf"])

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([bringup, "launch", "ocs2_sim.launch.py"])
        ),
        launch_arguments={
            "viewer": viewer,
            "use_rviz": use_rviz,
            "start_slam": "false",
            "start_remani": "false",
            "start_remani_bridge": "false",
            "use_joy": "false",
            "use_csv_target": "false",
            "mujoco_model": mujoco_model,
            "init_keyframe": "home",
            "task_file": PathJoinSubstitution([
                FindPackageShare("tracer_jaka_ocs2"), "config", "task.info"]),
            "mrt_traj_horizon": "0.05",
            "arm_use_velocity_integrator": "true",
            "arm_max_command_velocity": "0.15",
            "arm_max_delta_per_step": "0.10",
        }.items(),
    )

    controller = Node(
        package="whole_body_force_control",
        executable="whole_body_force_control_node",
        name="whole_body_force_control",
        output="screen",
        parameters=[
            force_params_file,
            {
                "urdf_file": urdf,
                "ee_frame": "tool0",
                "robot_name": "mobile_manipulator",
                "wrench_topic": fake_wrench_topic,
                "armed": ParameterValue(armed, value_type=bool),
                "reference_output_enabled": ParameterValue(
                    reference_output, value_type=bool),
                "use_sim_time": True,
            },
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument(
            "force_params_file",
            default_value=PathJoinSubstitution([
                force_control, "config", "force_follow_infinite_sim.yaml"])),
        DeclareLaunchArgument(
            "fake_wrench_topic",
            default_value="/whole_body_force_control/fake_wrench"),
        DeclareLaunchArgument(
            "mujoco_model",
            default_value=PathJoinSubstitution(
                [mujoco, "models", "scene_force_follow_infinite.xml"])),
        DeclareLaunchArgument("force_control_armed", default_value="true"),
        DeclareLaunchArgument(
            "force_reference_output_enabled", default_value="true"),
        simulation,
        TimerAction(period=10.0, actions=[controller]),
    ])
