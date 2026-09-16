#!/usr/bin/env python3
"""Compose MuJoCo, localization, OCS2 and optional REMANI algorithms."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    bringup = get_package_share_directory("tracer_jaka_bringup")
    description = get_package_share_directory("tracer_jaka_description")
    mujoco = get_package_share_directory("tracer_jaka_mujoco")
    ocs2 = get_package_share_directory("wbmm_ocs2_ros")
    grid_map = get_package_share_directory("grid_map")

    def source(name):
        return PythonLaunchDescriptionSource(os.path.join(
            bringup, "launch", name))

    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("use_rviz", default_value="true"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("start_slam", default_value="true"),
        DeclareLaunchArgument("start_ocs2", default_value="true"),
        DeclareLaunchArgument("start_remani", default_value="true"),
        DeclareLaunchArgument("start_remani_bridge", default_value="true"),
        DeclareLaunchArgument("use_target", default_value="false"),
        DeclareLaunchArgument("command_output_enabled", default_value="true"),
        DeclareLaunchArgument("odom_topic", default_value="/odometry/filtered"),
        DeclareLaunchArgument("joint_state_topic", default_value="/joint_states"),
        DeclareLaunchArgument("task_file", default_value=os.path.join(
            ocs2, "config", "task_sim.info")),
        DeclareLaunchArgument("urdf_file", default_value=os.path.join(
            description, "urdf", "tracer_jaka_zu5.urdf")),
        DeclareLaunchArgument("lib_folder", default_value="/tmp/wbmm_ocs2/auto_generated"),
        DeclareLaunchArgument("ocs2_config", default_value=os.path.join(
            ocs2, "config", "ocs2_sim.yaml")),
        DeclareLaunchArgument("remani_config", default_value=os.path.join(
            bringup, "config", "sim", "remani_sim.yaml")),
        DeclareLaunchArgument("static_esdf_file", default_value=os.path.join(
            grid_map, "maps", "tracer_jaka_zu5_scene_esdf.npz")),
        DeclareLaunchArgument("planner_frame", default_value="odom"),
        DeclareLaunchArgument("target_frame", default_value="odom"),
        DeclareLaunchArgument("use_tf_transform", default_value="false"),
        DeclareLaunchArgument("rviz_config", default_value=os.path.join(
            ocs2, "rviz", "wbmm_ocs2_ros.rviz")),
        DeclareLaunchArgument("mujoco_model", default_value=os.path.join(
            mujoco, "models", "scene.xml")),
        DeclareLaunchArgument("init_keyframe", default_value="home"),
        DeclareLaunchArgument("sim_camera", default_value="false"),
        IncludeLaunchDescription(source("sim.launch.py"), launch_arguments={
            "viewer": LaunchConfiguration("viewer"),
            "camera": LaunchConfiguration("sim_camera"),
            "model": LaunchConfiguration("mujoco_model"),
            "init_keyframe": LaunchConfiguration("init_keyframe"),
        }.items()),
        IncludeLaunchDescription(source("localization.launch.py"), launch_arguments={
            "start_ekf": "true",
            "start_slam": LaunchConfiguration("start_slam"),
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "wheel_odom_topic": "/wheel/odometry",
            "imu_topic": "/imu/data",
            "ekf_config": os.path.join(bringup, "config", "sim", "ekf_sim.yaml"),
            "slam_config": os.path.join(bringup, "config", "sim", "slam_toolbox_sim.yaml"),
        }.items()),
        Node(
            package="tf2_ros", executable="static_transform_publisher",
            name="map_to_odom_static_tf", output="screen",
            arguments=["0", "0", "0", "0", "0", "0", "map", "odom"],
            condition=UnlessCondition(LaunchConfiguration("start_slam"))),
        TimerAction(period=8.0, actions=[IncludeLaunchDescription(
            source("ocs2.launch.py"),
            condition=IfCondition(LaunchConfiguration("start_ocs2")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "use_rviz": LaunchConfiguration("use_rviz"),
                "use_target": LaunchConfiguration("use_target"),
                "config_file": LaunchConfiguration("ocs2_config"),
                "task_file": LaunchConfiguration("task_file"),
                "urdf_file": LaunchConfiguration("urdf_file"),
                "lib_folder": LaunchConfiguration("lib_folder"),
                "command_output_enabled": LaunchConfiguration("command_output_enabled"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "rviz_config": LaunchConfiguration("rviz_config"),
            }.items())]),
        TimerAction(period=12.0, actions=[IncludeLaunchDescription(
            source("remani.launch.py"),
            condition=IfCondition(LaunchConfiguration("start_remani")),
            launch_arguments={
                "use_sim_time": LaunchConfiguration("use_sim_time"),
                "start_bridge": LaunchConfiguration("start_remani_bridge"),
                "config_file": LaunchConfiguration("remani_config"),
                "urdf_file": LaunchConfiguration("urdf_file"),
                "static_esdf_file": LaunchConfiguration("static_esdf_file"),
                "odom_topic": LaunchConfiguration("odom_topic"),
                "joint_state_topic": LaunchConfiguration("joint_state_topic"),
                "planner_frame": LaunchConfiguration("planner_frame"),
                "target_frame": LaunchConfiguration("target_frame"),
                "use_tf_transform": LaunchConfiguration("use_tf_transform"),
            }.items())]),
    ])
