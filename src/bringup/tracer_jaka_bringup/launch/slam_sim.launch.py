#!/usr/bin/env python3
"""MuJoCo + wheel/IMU EKF + slam_toolbox mapping pipeline."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    mujoco_share = get_package_share_directory("tracer_jaka_mujoco")
    bringup_share = get_package_share_directory("tracer_jaka_bringup")
    bridge_launch = os.path.join(
        mujoco_share, "launch", "bridge.launch.py")
    ekf_config = os.path.join(bringup_share, "config", "ekf_sim.yaml")
    slam_config = os.path.join(
        bringup_share, "config", "slam_toolbox_sim.yaml")
    rviz_config = os.path.join(bringup_share, "rviz", "slam.rviz")

    viewer = LaunchConfiguration("viewer")
    rviz = LaunchConfiguration("rviz")
    camera = LaunchConfiguration("camera")
    camera_rate = LaunchConfiguration("camera_rate")
    camera_width = LaunchConfiguration("camera_width")
    camera_height = LaunchConfiguration("camera_height")
    model = LaunchConfiguration("model")
    init_keyframe = LaunchConfiguration("init_keyframe")

    return LaunchDescription([
        DeclareLaunchArgument("viewer", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("camera", default_value="false"),
        DeclareLaunchArgument("camera_rate", default_value="30.0"),
        DeclareLaunchArgument("camera_width", default_value="640"),
        DeclareLaunchArgument("camera_height", default_value="480"),
        DeclareLaunchArgument(
            "model",
            default_value=os.path.join(mujoco_share, "models", "scene.xml")),
        DeclareLaunchArgument("init_keyframe", default_value="home"),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(bridge_launch),
            launch_arguments={
                "viewer": viewer,
                "camera": camera,
                "camera_rate": camera_rate,
                "camera_width": camera_width,
                "camera_height": camera_height,
                "model": model,
                "init_keyframe": init_keyframe,
            }.items(),
        ),

        Node(
            package="robot_localization",
            executable="ekf_node",
            name="ekf_filter_node",
            output="screen",
            parameters=[ekf_config, {"use_sim_time": True}],
            remappings=[("odometry/filtered", "/odometry/filtered")],
        ),

        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[slam_config],
        ),

        # Let the bridge, EKF and slam_toolbox establish the complete TF tree
        # before RViz starts requesting transforms at interpolated sim time.
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="rviz2",
                    executable="rviz2",
                    name="slam_rviz",
                    output="screen",
                    arguments=["-d", rviz_config],
                    parameters=[{"use_sim_time": True}],
                    condition=IfCondition(rviz),
                ),
            ],
        ),
    ])
